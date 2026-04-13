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

import asyncio
import sys
import yaml
import json
import nudged
import argparse
import time
import threading
from pprint import pprint
from functools import partial

import rclpy
import rclpy.node
from rclpy.parameter import Parameter

from functools import partial

import rmf_adapter
import rmf_adapter.easy_full_control as rmf_easy
import rmf_adapter.fleet_update_handle as rmf_fleet
from rmf_adapter import Transformation
from .robot_adapter_temi import RobotAdapterTemi

# ------------------------------------------------------------------------------
# Helper functions
# ------------------------------------------------------------------------------

def sanitise_dict(dictionary, inplace=False, recursive=False):
    """Remove dictionary Nones."""
    if type(dictionary) is not dict:
        return dictionary

    output = {}

    if inplace:
        del_keys = set()

        for key, val in dictionary.items():
            if val is None:
                del_keys.add(key)
            elif recursive:
                sanitise_dict(dictionary[key], True, True)

        for key in del_keys:
            del dictionary[key]

        output = dictionary
    else:
        for key, val in dictionary.items():
            if val is None:
                continue
            else:
                if recursive:
                    val = sanitise_dict(val, False, True)
                output[key] = val

    return output


def compute_transforms(level, coords, node=None):
    """Get transforms between RMF and robot coordinates."""
    rotation = None
    scale = None
    translation = None
    if 'rmf' in coords:
        rmf_coords = coords['rmf']
        robot_coords = coords['robot']
        tf = nudged.estimate(rmf_coords, robot_coords)
        if node:
            mse = nudged.estimate_error(tf, rmf_coords, robot_coords)
            node.get_logger().info(
                f"Transformation error estimate for {level}: {mse}"
            )
        rotation = tf.get_rotation()
        scale = tf.get_scale()
        translation = tf.get_translation()
    elif 'rotation' in coords:
        rotation = coords['rotation']
        scale = coords['scale']
        translation = coords['translation']
    else:
        return None

    return Transformation(
        rotation,
        scale,
        translation
    )


class FleetAdapterTemi:
    def __init__(
        self,
        cmd_node,
        adapter,
        fleet_handle,
        robot_handles: list[RobotAdapterTemi],
        frequency,
        event_loop,
    ):
        self.event_loop = event_loop
        self.adapter = adapter
        self.fleet_handle = fleet_handle
        self.robot_handles: list[RobotAdapterTemi] = robot_handles
        self.node = cmd_node
        if frequency > 0.0:
            self.period = 1.0/frequency
        else:
            raise Exception(f'Invalid robot update frequency: {frequency}')
        self.robot_update_jobs = {}

    async def state_updates(self):
        robot_updaters = []
        for robot in self.robot_handles:
            robot_updaters.append(robot.update_loop(self.period))
        await asyncio.gather(*robot_updaters)

    def update_loop(self):
        asyncio.set_event_loop(self.event_loop)
        self.event_loop.run_until_complete(self.state_updates())

    def start(self):
        update_thread = threading.Thread(target=self.update_loop)
        update_thread.start()

        # Create the executor for the logger node
        rclpy_executor = rclpy.executors.SingleThreadedExecutor()
        rclpy_executor.add_node(self.node)

        self.adapter.start()
        rclpy_executor.spin()

        self.node.destroy_node()
        rclpy_executor.shutdown()
        rclpy.shutdown()


def create_fleet(
        fleet_config, config_yaml, cmd_node, server_uri) -> FleetAdapterTemi:
    """Create RMF Adapter and fleet handle"""
    for level, coords in \
            config_yaml['conversions']['reference_coordinates'].items():
        tf = compute_transforms(level, coords, cmd_node)
        fleet_config.add_robot_coordinates_transformation(level, tf)

    # RMF_CORE Fleet Adapter: Manages delivery or loop requests
    adapter = rmf_adapter.Adapter.make(
        config_yaml['node_names']['rmf_fleet_adapter'])
    assert adapter, ("Adapter could not be init! "
                     "Ensure a ROS2 scheduler node is running")

    cmd_node.declare_parameter('server_uri', '')
    server_uri = cmd_node.get_parameter(
        'server_uri'
    ).get_parameter_value().string_value
    if server_uri == '':
        server_uri = None

    fleet_config.server_uri = server_uri
    fleet_handle = adapter.add_easy_fleet(fleet_config)

    conversions = config_yaml['conversions']
    update_frequency = config_yaml['rmf_fleet']['robot_state_update_frequency']
    debug = config_yaml['rmf_fleet']['debug']


    event_loop = asyncio.new_event_loop()

    robots_handles = []
    for robot_name, rmf_config in \
            fleet_config.known_robot_configurations.items():
        temi_config = \
            config_yaml['rmf_fleet']['robots'][robot_name]['temi_config']
        robots_handles.append(RobotAdapterTemi(
            robot_name,
            rmf_config,
            temi_config,
            conversions,
            fleet_handle,
            fleet_config,
            cmd_node,
            event_loop,
            debug,
        ))

    return FleetAdapterTemi(
        cmd_node, adapter, fleet_handle, robots_handles, update_frequency,
        event_loop)


# ------------------------------------------------------------------------------
# Main
# ------------------------------------------------------------------------------
def main(argv=sys.argv):

    # INIT RCL ================================================================
    rclpy.init(args=argv)
    rmf_adapter.init_rclcpp()
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog="fleet_adapter_temi",
        description="Configure and spin up fleet adapters for Temi robots")
    parser.add_argument("-c", "--config_file", type=str, required=True,
                        help="Input config.yaml file to process")
    parser.add_argument("-n", "--nav_graph", type=str, required=True,
                        help="Path to the nav_graph for this fleet adapter")
    parser.add_argument("-r", "--server_uri", type=str,
                        required=False, default='',
                        help="Server URI for rmf-web")
    args = parser.parse_args(args_without_ros[1:])
    print(f"Starting temi fleet adapter...")

    config_path = args.config_file
    nav_graph_path = args.nav_graph
    server_uri = args.server_uri

    fleet_config = rmf_easy.FleetConfiguration.from_config_files(
        config_path, nav_graph_path
    )
    assert fleet_config, f'Failed to parse config file [{config_path}]'

    with open(config_path, 'r') as f:
        config_yaml = yaml.safe_load(f)

    sanitise_dict(config_yaml, inplace=True, recursive=True)

    print("\n== Temi Adapter Configuration ==")
    pprint(config_yaml)
    print()

    # Create a node to use inside of the Python code for logging
    cmd_node = rclpy.node.Node(
        config_yaml['node_names']['robot_command_handle'])

    # Create the fleet, including the robots that are in the config file
    fleet = create_fleet(fleet_config, config_yaml, cmd_node, server_uri)

    # GO!
    fleet.start()


if __name__ == "__main__":
    main(sys.argv)
