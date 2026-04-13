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
from rclpy.node import Node
import temi_fleet_adapter_python.msg_definitions as msg
import paho.mqtt.client as mqtt

from rmf_adapter.easy_full_control import RobotState


class State:
    # Save robot state in local coordinate frame ( not RMF global frame )
    def __init__(self, map_name: str):
        self.map_name = map_name
        self.current_loc = Location()
        self.target_loc = Location()
        self.battery_level = 100.0
        self.is_charging = False
        # TODO(@xiyuoh) add teleop mode state
        # self.teleop_mode = False
        # self.last_teleop_msg_time = Time()
        self.robot_state = None

    def update_position(self, x, y, yaw):
        self.current_loc.x = x
        self.current_loc.y = y
        self.current_loc.yaw = yaw
        self.update_robot_state()

    def update_battery(self, battery_soc, is_charging=False):
        self.battery_level = battery_soc
        self.is_charging = is_charging
        self.update_robot_state()

    def update_robot_state(self):
        self.robot_state = RobotState(
            self.map_name,
            [self.current_loc.x, self.current_loc.y, self.current_loc.yaw],
            self.battery_level/100.0,
        )

    def new_target(self, x, y, yaw) -> bool:
        if (self.target_loc.x == x and
                self.target_loc.y == y and
                self.target_loc.yaw == yaw):
            return False

        self.target_loc.x = x
        self.target_loc.y = y
        self.target_loc.yaw = yaw
        return True

    def navigation_completed(self):
        # TODO(@xiyuoh) check if there is a Temi API for this
        if self._disp(self.target_loc, self.current_loc) < 0.5:
            return True
        else:
            return False

    def _disp(self, A: Location, B: Location):
        return math.sqrt((A.x-B.x)**2 + (A.y-B.y)**2)


class RobotAPI(Node):
    def __init__(
            self,
            mqtt_broker: str,
            mqtt_user: str,
            mqtt_password: str,
            robot_name: str,
            serial: str,
            map_name: str):
        super().__init__(f"{robot_name}_robot_api")
        # API connection to robot
        self.mqtt_broker = mqtt_broker
        self.robot_name = robot_name
        self.robot_serial = serial
        self.mqtt_client = mqtt.Client(f"{self.robot_name}_temi_publisher")
        # TODO(@xiyuoh)
        # self.sio = socketio.Client()
        self.state = State(map_name)
        # self.state.last_teleop_msg_time = self.get_clock().now()
        # self.video_join_topic = f"/teleoperation/{self.robot_name}/video/join"
        # self.video_leave_topic = f"/teleoperation/{self.robot_name}/video/leave"
        # self.move_topic = f"/teleoperation/{self.robot_name}/move"

        # Task housekeeping
        self.task_id = -1

        def _on_connect(client, userstate, msg):
            json_msg = json.loads(msg.payload)
            print(f"Connected to MQTT server with message: {json_msg}")
            # Update position once with status/info and stop subscription
            # /status/info is published everything 3 seconds, while
            # /status/position is only published when the robot is moving,
            # and therefore position update is relevant.
            self.state.update_position(
                float(json_msg["x"]),
                float(json_msg["y"]),
                float(json_msg["yaw"])
            )
            if self.state.robot_state:
                # Unsubscribe once robot state is populated
                self.mqtt_client.message_callback_remove(
                    f"temi/{self.robot_serial}/status/info")

        def _on_robot_state(client, userstate, msg):
            json_msg = json.loads(msg.payload)
            self.state.update_position(
                float(json_msg["x"]),
                float(json_msg["y"]),
                float(json_msg["yaw"])
            )

        def _on_battery_status(client, userstate, msg):
            json_msg = json.loads(msg.payload)
            self.state.update_battery(
                float(json_msg['percentage']),
                json_msg['is_charging']
            )

        # @self.sio.event
        # def _connect():
        #     print('connection established')

        # @self.sio.on(self.video_join_topic)
        # def _video_join(data):
        #     t = self.get_clock().now()
        #     self.state.last_teleop_msg_time = t
        #     self.state.teleop_mode = True

        #     print("VIDEO_JOIN")
        #     print(self.video_join_topic)
        #     print(data["id"])
        #     print(data["data"]["url"])
        #     self.mqtt_client.publish(
        #         f"temi/{self.robot_serial}/command/media/join", json.dumps({}))
        
        # @self.sio.on(self.video_leave_topic)
        # def _video_leave(data):
        #     t = self.get_clock().now()
        #     self.state.last_teleop_msg_time = t
        #     self.state.teleop_mode = True

        #     print("VIDEO_LEAVE")
        #     print(data["id"])
        #     self.mqtt_client.publish(
        #         f"temi/{self.robot_serial}/command/media/leave", json.dumps({}))

        # @self.sio.on(self.move_topic)
        # def _move(data):
        #     t = self.get_clock().now()
        #     self.state.last_teleop_msg_time = t
        #     self.state.teleop_mode = True

        #     print("MOVE")
        #     print(data["id"])
        #     if data["data"]["name"] == "camera":
        #         print("MOVE CAMERA")
        #         print(data["data"])
        #         json_msg = {}
        #         pitch = data['data']['pitch_deg']

        #         if pitch != 0.0:
        #             if pitch > 0:
        #                 json_msg['angle'] = '20'  # default set to 20 deg for now. value has to be int
        #             else:
        #                 json_msg['angle'] = '-20'
        #             self.mqtt_client.publish(
        #                 f"temi/{self.robot_serial}/command/move/tilt_by", json.dumps(json_msg))

        #     elif data["data"]["name"] == self.robot_name:
        #         print("MOVE ROBOT")
        #         print(data["data"])
        #         yaw = data['data']['yaw_deg']
        #         json_msg = {}

        #         if yaw != 0.0:  # if turning left/right
        #             if yaw > 0:
        #                 json_msg['angle'] = '20'  # default set to 20 deg for now. value has to be int
        #             else:
        #                 json_msg['angle'] = '-20'
        #             self.mqtt_client.publish(
        #                 f"temi/{self.robot_serial}/command/move/turn_by", json.dumps(json_msg))

        #         else:  # if moving robot                
        #             json_msg['x'] = data['data']['x_m']
        #             json_msg['y'] = data['data']['y_m']
        #             self.mqtt_client.publish(
        #                 f"temi/{self.robot_serial}/command/move/joystick", json.dumps(json_msg))
        #     else:
        #         print("unknown thing to move!")

        # @self.sio.event
        # def _subscribe(data):
        #     t = self.get_clock().now()
        #     self.state.last_teleop_msg_time = t
        #     self.state.teleop_mode = True
        #     print('message received with ', data)

        # @self.sio.event
        # def _disconnect():
        #     print('disconnected from server')

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

        self.mqtt_client.username_pw_set(mqtt_user, mqtt_password)
        self.mqtt_client.connect(self.mqtt_broker)
        self.mqtt_client.message_callback_add(
            f"temi/{self.robot_serial}/status/info",
            _on_connect)
        self.mqtt_client.message_callback_add(
            f"temi/{self.robot_serial}/status/position",
            _on_robot_state)
        self.mqtt_client.message_callback_add(
            f"temi/{self.robot_serial}/status/utils/battery",
            _on_battery_status)
        self.mqtt_client.subscribe([
            (f"temi/{self.robot_serial}/status/info", 0),
            (f"temi/{self.robot_serial}/status/position", 0),
            (f"temi/{self.robot_serial}/status/utils/battery", 0)
        ])

        # self.sio.connect('http://localhost:8000/socket.io')
        # self.sio.emit('subscribe', {'room': self.video_join_topic})
        # self.sio.emit('subscribe', {'room': self.video_leave_topic})
        # self.sio.emit('subscribe', {'room': self.move_topic})
        # self.sio.wait()
        self.mqtt_client.loop_start()

        while True:
            json_msg = copy.deepcopy(msg.SKIDJOY_DEFINITION)
            json_msg["x"] = '0.1'
            json_msg["y"] = '0.0'
            self.mqtt_client.publish(
                f"temi/{self.robot_serial}/command/move/joystick", json.dumps(json_msg))
            if self.state.robot_state:
                self.get_logger().info(f'Connected!')
                break
            else:
                self.get_logger().info("Initializing robot state.")
                self.get_logger().info("You might observe the robot driving.")
                time.sleep(10)

    def get_state(self):
        return self.state

    def position(self):
        # print("GETTING POSITION")
        return (self.state.current_loc.x,
                self.state.current_loc.y,
                self.state.current_loc.yaw)

    def navigate(self, pose):
        if not self.state.new_target(pose[0], pose[1], pose[2]):
            return

        t = self.get_clock().now()
        # Check if we should be out of teleop mode (5 seconds)
        # if (t - self.state.last_teleop_msg_time).nanoseconds > 5000000000:
        #     if self.state.teleop_mode:
        #         self.get_logger().info("RESUMING RMF CONTROL")
        #         self.state.teleop_mode = False

        if not self.state.navigation_completed():
            json_msg = copy.deepcopy(msg.POSITION_DEFINITION)
            json_msg["x"] = self.state.target_loc.x
            json_msg["y"] = self.state.target_loc.y
            json_msg["yaw"] = self.state.target_loc.yaw
            json_msg["angle"] = 50
            # if not self.state.teleop_mode:
            self.mqtt_client.publish(
                f"temi/{self.robot_serial}/command/move/position",
                json.dumps(json_msg)
            )
        return True

    def dock(self, position):
        json_msg = copy.deepcopy(msg.GOTO_DEFINITION)
        json_msg["location"] = position
        self.mqtt_client.publish(
            f"temi/{self.robot_serial}/command/waypoint",
            json.dumps(json_msg)
        )

    def stop(self):
        self.mqtt_client.publish(
            f"temi/{self.robot_serial}/command/move/stop",
            json.dumps(copy.deepcopy(msg.STOPMOVEMENT_DEFINITION))
        )
        return True

    def battery_soc(self):
        return float(self.state.battery_level / 100.0)

    # def on_teleop(self):
    #     return self.state.teleop_mode