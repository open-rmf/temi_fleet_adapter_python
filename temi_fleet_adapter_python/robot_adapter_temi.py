# Copyright 2026 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
import time
import rmf_adapter.easy_full_control as rmf_easy
import rclpy
import rclpy.node as Node
from rclpy.duration import Duration
from .temi_mqtt_api import RobotAPI
from threading import Lock


# Parallel processing solution derived from
# https://stackoverflow.com/a/59385935
def parallel(f):
    def run_in_parallel(*args, **kwargs):

        return args[0].event_loop.run_in_executor(
            None, f, *args, **kwargs
        )

    return run_in_parallel


class ActionContext:
    def __init__(self,
                 node: Node,
                 name: str,
                 api: RobotAPI,
                 update_handle,  # rmf_fleet_adapter.RobotUpdateHandle
                 fleet_config: rmf_easy.FleetConfiguration,
                 action_config: dict):
        self.node = node
        self.name = name
        self.api = api
        self.update_handle = update_handle
        self.fleet_config = fleet_config
        self.action_config = action_config


class RobotAdapterTemi:
    def __init__(
        self,
        name: str,
        rmf_config: rmf_easy.RobotConfiguration,
        temi_config: dict,
        conversions: dict,
        fleet_handle,
        fleet_config,
        node: Node,
        event_loop,
        debug=False
    ):
        self.name = name
        self.node = node
        self.event_loop = event_loop
        self.debug = debug

        self.disconnect = False
        self.disconnect_mutex = Lock()
        self.replan_after_disconnect = False

        reference_coordinates = conversions["reference_coordinates"]
        map_name = next(iter(reference_coordinates))
        self.rmf_to_robot = conversions["positions"]

        self.api: RobotAPI = RobotAPI(
            temi_config['base_url'],
            temi_config['user'],
            temi_config['password'],
            name,
            temi_config['serial'],
            map_name,
        )
        robot_state = self.api.get_state().robot_state

        self.fleet_config = fleet_config
        self.fleet_handle = fleet_handle
        self.update_handle = fleet_handle.add_robot(
            self.name,
            robot_state,
            rmf_config,
            self._make_callbacks(),
        )
        if not self.update_handle:
            error_message = \
                f'Failed to add robot [{self.name}] to fleet ' \
                f'[{self.fleet_handle.more().fleet_name}], this is most ' \
                'likely due to a configuration error.'
            self.node.get_logger().error(error_message)
            raise RuntimeError(error_message)
        
        self.execution = None


    async def update_loop(self, period):
        while rclpy.ok():
            now = self.node.get_clock().now()
            next_wakeup = now + Duration(nanoseconds=period * 1e9)
            await self.request_update()
            while self.node.get_clock().now() < next_wakeup:
                time.sleep(0.01)

    @parallel
    def request_update(self):
        status = self.api.get_state()
        execution = self.execution
        activity = None

        if execution is not None:
            activity = execution.identifier
        self.update_handle.update(status.robot_state, activity)

        if status.navigation_completed():
            execution.finished()
            execution = None

    def update_rmf_finished(self):
        pass

    def _make_callbacks(self):
        callbacks = rmf_easy.RobotCallbacks(
            lambda destination, execution: self.navigate(
                destination, execution
            ),
            lambda activity: self.stop(activity),
            lambda category, description, execution: self.perform_action(
                category, description, execution
            )
        )

        callbacks.localize = lambda estimate, execution: self.localize(
            estimate, execution
        )

        return callbacks

    def navigate(self, destination, execution):
        current_execution = self.execution
        if current_execution is not None:
            self.stop(current_execution.identifier)

        self.node.get_logger().info(
            f'New navigation command received for [{self.name}]'
        )
        self.execution = execution

        # If the destination is to the charger, then dock the robot
        if destination.dock is not None:
            dock_name = destination.dock
            if dock_name is not None and dock_name in self.rmf_to_robot:
                robot_pos = self.rmf_to_robot[dock_name]
                self.node.get_logger().info(
                    f'Received command to dock into [{dock_name}] ([{robot_pos}] on robot) for [{self.name}]'
                )
                self.api.dock(robot_pos)
                return

        pos = (destination.position[0], destination.position[1], destination.position[2])
        self.api.navigate(pos)

    def stop(self, activity):
        self.api.stop()
        self.execution = None

    def perform_action(self, category, description, execution):
        pass

    def dist(self, A, B):
        assert (len(A) > 1)
        assert (len(B) > 1)
        return math.sqrt((A[0] - B[0])**2 + (A[1] - B[1])**2)
