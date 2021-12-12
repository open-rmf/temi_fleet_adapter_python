# Copyright 2021 Open Source Robotics Foundation, Inc.
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


import socketio
import json
import math
import time
import copy
from rmf_fleet_msgs.msg import Location
from rclpy.time import Time
from rclpy.node import Node
import temi_fleet_adapter_python.msg_definitions as msg


class State:
    # Save robot state in local coordinate frame ( not RMF global frame )
    def __init__(self, vehicle_traits):
        self.initialized = False
        self.current_loc = Location()
        self.target_loc = Location()
        self.vehicle_traits = vehicle_traits
        self.battery_level = 100.0
        self.is_charging = False
        self.teleop_mode = False
        self.last_teleop_msg_time = Time()

    def duration_to_target(self):
        return int(self.disp()/self.vehicle_traits.linear.nominal_velocity) + \
            int(abs(abs(self.current_loc.yaw) - abs(self.target_loc.yaw)) /
                self.vehicle_traits.rotational.nominal_velocity)

    def disp(self):
        return self._disp(self.target_loc, self.current_loc)

    def _disp(self, A: Location, B: Location):
        return math.sqrt((A.x-B.x)**2 + (A.y-B.y)**2)


class RobotAPI(Node):
    def __init__(
            self, server_endpoint: str, user: str, password: str,
            robot_name: str, config: dict, vehicle_traits: dict):
        super().__init__(f"{robot_name}_robot_api")
        # API connection to robot
        self.server_endpoint = server_endpoint
        self.user = user
        self.password = password
        self.connected = False
        self.robot_name = robot_name
        self.config = config
        self.sio = socketio.Client()
        self.state = State(vehicle_traits)
        self.state.last_teleop_msg_time = self.get_clock().now()

        # Task housekeeping
        self.task_id = -1

        # Initialize
        self.fleet_name = self.config["fleet_name"]

        @self.sio.event
        def robot_state(data):
            json_msg = json.loads(data["data"])
            self.state.current_loc.x = json_msg["x"]
            self.state.current_loc.y = json_msg["y"]
            self.state.current_loc.yaw = json_msg["yaw"]
            if not self.state.initialized:
                self.state.target_loc = self.state.current_loc
                self.state.initialized = True

        self.sio.connect(
            self.server_endpoint,
            headers={"robot_name": robot_name})
        while True:
            json_msg = copy.deepcopy(msg.SKIDJOY_DEFINITION)
            json_msg["x"] = 0.1
            self.sio.emit("skidJoy", json_msg)
            if self.state.initialized:
                self.connected = True
                break
            else:
                self.get_logger().info("Initializing robot state.")
                self.get_logger().info("You might observe the robot driving.")
                time.sleep(1)

        @self.sio.event
        def battery_status(data):
            json_msg = json.loads(data["data"])
            self.state.battery_level = float(json_msg['level'])
            self.state.is_charging = json_msg['isCharging']

        @self.sio.event
        def skidJoy(data):
            self.state.last_teleop_msg_time = self.get_clock().now()
            if not self.state.teleop_mode:
                self.state.teleop_mode = True
                self.get_logger().info("SHIFTING TO TELEOP MODE")

        @self.sio.event
        def turnBy(data):
            self.state.last_teleop_msg_time = self.get_clock().now()
            if not self.state.teleop_mode:
                self.get_logger().info("SHIFTING TO TELEOP MODE")
                self.state.teleop_mode = True

    def position(self):
        return(self.state.current_loc.x,
               self.state.current_loc.y,
               self.state.current_loc.yaw)

    def navigate(self, pose, map_name: str):
        target_loc = Location()
        target_loc.x = pose[0]
        target_loc.y = pose[1]
        target_loc.yaw = pose[2]
        self.state.target_loc = target_loc

        t = self.get_clock().now()
        # Check if we should be out of teleop mode (5 seconds)
        if (t - self.state.last_teleop_msg_time).nanoseconds > 5000000000:
            if self.state.teleop_mode:
                self.get_logger().info("RESUMING RMF CONTROL")
                self.state.teleop_mode = False

        duration_to_target_loc = self.state.duration_to_target()

        if not self.navigation_completed():
            json_msg = copy.deepcopy(msg.GOTOPOSITION_DEFINITION)
            json_msg["x"] = target_loc.x
            json_msg["y"] = target_loc.y
            json_msg["yaw"] = target_loc.yaw
            json_msg["tiltAngle"] = 50
            if not self.state.teleop_mode:
                self.sio.emit("goToPosition", json_msg)

        return True

    def start_process(self, process: str, map_name: str):
        raise NotImplementedError

    def stop(self):
        self.sio.emit("stopMovement", msg.STOPMOVEMENT_DEFINITION)
        return True

    def navigation_remaining_duration(self):
        return self.state.duration_to_target()

    def navigation_completed(self):
        # BH(WARN): Arbitrary threshold
        if (self.state.duration_to_target() < 0.05):
            return True
        else:
            return False

    def process_completed(self):
        return False

    def battery_soc(self):
        return float(self.state.battery_level / 100.0)
