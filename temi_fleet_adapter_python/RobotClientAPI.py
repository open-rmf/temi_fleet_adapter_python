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
import numpy as np
import time
import copy
from rmf_fleet_msgs.msg import Location
from rclpy.time import Time
from rclpy.node import Node
import temi_fleet_adapter_python.msg_definitions as msg
import paho.mqtt.client as mqtt


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
        yaw_delta = self.current_loc.yaw - self.target_loc.yaw
        if yaw_delta > np.pi:
            yaw_delta = yaw_delta - (2 * np.pi)
            if yaw_delta < -np.pi:
                yaw_delta = (2 * np.pi) + yaw_delta

        return float(self.disp()/self.vehicle_traits.linear.nominal_velocity) + \
            float(abs(yaw_delta)) / \
            float(self.vehicle_traits.rotational.nominal_velocity)

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
        self.mqtt_client = mqtt.Client(f"{self.robot_name}_temi_publisher")
        self.sio = socketio.Client()
        self.state = State(vehicle_traits)
        self.state.last_teleop_msg_time = self.get_clock().now()
        self.video_join_topic = f"/teleoperation/{self.robot_name}/video/join"
        self.video_leave_topic = f"/teleoperation/{self.robot_name}/video/leave"
        self.move_topic = f"/teleoperation/{self.robot_name}/move"

        # Task housekeeping
        self.task_id = -1

        # Initialize
        self.fleet_name = self.config["fleet_name"]
        self.robot_serial = self.config["serial"]
        self.mqtt_server_url = self.config["base_url"]

        def _on_robot_state(client, userstate, msg):
            json_msg = json.loads(msg.payload)
            self.state.current_loc.x = float(json_msg["position_x"])
            self.state.current_loc.y = float(json_msg["position_y"])
            self.state.current_loc.yaw = float(json_msg["yaw"])
            if not self.state.initialized:
                self.state.target_loc = self.state.current_loc
                self.state.initialized = True
            # print(self.state.current_loc)

        def _on_battery_status(client, userstate, msg):
            json_msg = json.loads(msg.payload)
            self.state.battery_level = float(json_msg['percentage'])
            self.state.is_charging = json_msg['is_charging']
            print(self.state.battery_level)

        @self.sio.event
        def _connect():
            print('connection established')

        @self.sio.on(self.video_join_topic)
        def _video_join(data):
            t = self.get_clock().now()
            self.state.last_teleop_msg_time = t
            self.state.teleop_mode = True

            print("VIDEO_JOIN")
            print(self.video_join_topic)
            print(data["id"])
            print(data["data"]["url"])
            self.mqtt_client.publish(
                f"temi/{self.robot_serial}/command/media/join", json.dumps({}))
        
        @self.sio.on(self.video_leave_topic)
        def _video_leave(data):
            t = self.get_clock().now()
            self.state.last_teleop_msg_time = t
            self.state.teleop_mode = True

            print("VIDEO_LEAVE")
            print(data["id"])
            self.mqtt_client.publish(
                f"temi/{self.robot_serial}/command/media/leave", json.dumps({}))

        @self.sio.on(self.move_topic)
        def _move(data):
            t = self.get_clock().now()
            self.state.last_teleop_msg_time = t
            self.state.teleop_mode = True

            print("MOVE")
            print(data["id"])
            if data["data"]["name"] == "camera":
                print("MOVE CAMERA")
                print(data["data"])
                json_msg = {}
                pitch = data['data']['pitch_deg']

                if pitch != 0.0:
                    if pitch > 0:
                        json_msg['angle'] = '20'  # default set to 20 deg for now. value has to be int
                    else:
                        json_msg['angle'] = '-20'
                    self.mqtt_client.publish(
                        f"temi/{self.robot_serial}/command/move/tilt_by", json.dumps(json_msg))

            elif data["data"]["name"] == self.robot_name:
                print("MOVE ROBOT")
                print(data["data"])
                yaw = data['data']['yaw_deg']
                json_msg = {}

                if yaw != 0.0:  # if turning left/right
                    if yaw > 0:
                        json_msg['angle'] = '20'  # default set to 20 deg for now. value has to be int
                    else:
                        json_msg['angle'] = '-20'
                    self.mqtt_client.publish(
                        f"temi/{self.robot_serial}/command/move/turn_by", json.dumps(json_msg))

                else:  # if moving robot                
                    json_msg['x'] = data['data']['x_m']
                    json_msg['y'] = data['data']['y_m']
                    self.mqtt_client.publish(
                        f"temi/{self.robot_serial}/command/move/joystick", json.dumps(json_msg))
            else:
                print("unknown thing to move!")

        @self.sio.event
        def _subscribe(data):
            t = self.get_clock().now()
            self.state.last_teleop_msg_time = t
            self.state.teleop_mode = True
            print('message received with ', data)

        @self.sio.event
        def _disconnect():
            print('disconnected from server')

        # def skidJoy(data):
        #     self.state.last_teleop_msg_time = self.get_clock().now()
        #     if not self.state.teleop_mode:
        #         self.state.teleop_mode = True
        #         self.get_logger().info("SHIFTING TO TELEOP MODE")

        # def turnBy(data):
        #     self.state.last_teleop_msg_time = self.get_clock().now()
        #     if not self.state.teleop_mode:
        #         self.get_logger().info("SHIFTING TO TELEOP MODE")
        #         self.state.teleop_mode = True

        def on_message(client, userdata, message):
            print(f"Received message with unknown pattern: {message}")

        self.mqtt_client.connect(self.mqtt_server_url)
        self.mqtt_client.message_callback_add(
            f"temi/{self.robot_serial}/status/position",
            _on_robot_state)
        self.mqtt_client.message_callback_add(
            f"temi/{self.robot_serial}/status/utils/battery",
            _on_battery_status)
        self.mqtt_client.subscribe(f"temi/{self.robot_serial}/status/position")
        self.mqtt_client.subscribe(
            f"temi/{self.robot_serial}/status/utils/battery")

        self.sio.connect('http://localhost:8000/socket.io')
        self.sio.emit('subscribe', {'room': self.video_join_topic})
        self.sio.emit('subscribe', {'room': self.video_leave_topic})
        self.sio.emit('subscribe', {'room': self.move_topic})
        # self.sio.wait()
        self.mqtt_client.loop_start()

        while True:
            json_msg = copy.deepcopy(msg.SKIDJOY_DEFINITION)
            json_msg["x"] = '0.1'
            json_msg["y"] = '0.0'
            self.mqtt_client.publish(
                f"temi/{self.robot_serial}/command/move/joystick", json.dumps(json_msg))
            if self.state.initialized:
                self.connected = True
                self.get_logger().info(f'Connected!')
                break
            else:
                self.get_logger().info("Initializing robot state.")
                self.get_logger().info("You might observe the robot driving.")
                time.sleep(10)

    def position(self):
        # print("GETTING POSITION")
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
            json_msg = copy.deepcopy(msg.POSITION_DEFINITION)
            json_msg["x"] = target_loc.x
            json_msg["y"] = target_loc.y
            json_msg["yaw"] = target_loc.yaw
            json_msg["angle"] = 50
            if not self.state.teleop_mode:
                self.mqtt_client.publish(
                    f"temi/{self.robot_serial}/command/move/position", json.dumps(json_msg))
                pass
                # self.sio.emit("goToPosition", json_msg)

        return True

    def start_process(self, process: str, map_name: str):
        raise NotImplementedError

    def stop(self):
        # self.sio.emit("stopMovement", msg.STOPMOVEMENT_DEFINITION)
        return True

    def navigation_remaining_duration(self):
        return self.state.duration_to_target()

    def navigation_completed(self):
        # BH(WARN): Arbitrary threshold
        print(f"DURATION TO TARGET: {self.state.duration_to_target()}")
        print(f"DISTANCE TO TARGET: {self.state.disp()}")
        # if (self.state.duration_to_target() < 0.25):
        if (self.state.disp() < 0.5):
            return True
        else:
            return False

    def process_completed(self):
        return False

    def battery_soc(self):
        # print("BATTERY")
        return float(self.state.battery_level / 100.0)

    def on_teleop(self):
        return self.state.teleop_mode