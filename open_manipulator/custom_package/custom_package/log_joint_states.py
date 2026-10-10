#!/usr/bin/env python3
#
# Copyright 2026 ROBOTIS CO., LTD.
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
#

import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateLogger(Node):
    """
    Node to continuously log and print OpenManipulator-X joint states
    in real-time formatted in both Radians and Degrees.
    """

    def __init__(self):
        super().__init__('joint_state_logger')

        self.arm_joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        self.gripper_joint_name = 'gripper_left_joint'

        # Declare parameter for logging frequency (Hz)
        self.declare_parameter('log_rate_hz', 5.0)
        self.log_rate_hz = self.get_parameter('log_rate_hz').value

        self.latest_msg = None
        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        # Timer to throttle log printing at desired Hz
        period = 1.0 / self.log_rate_hz if self.log_rate_hz > 0 else 0.2
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            f'Joint State Logger Initialized. Streaming at {self.log_rate_hz} Hz...'
        )
        print('\n' + '=' * 100)
        print(' 📊 OPENMANIPULATOR-X LIVE JOINT STATE LOGGER')
        print('=' * 100)

    def joint_state_callback(self, msg: JointState):
        self.latest_msg = msg

    def timer_callback(self):
        if self.latest_msg is None:
            return

        positions = {}
        for name, pos in zip(self.latest_msg.name, self.latest_msg.position):
            positions[name] = pos

        j1 = positions.get('joint1', 0.0)
        j2 = positions.get('joint2', 0.0)
        j3 = positions.get('joint3', 0.0)
        j4 = positions.get('joint4', 0.0)
        gripper = positions.get('gripper_left_joint', 0.0)

        j1_deg = math.degrees(j1)
        j2_deg = math.degrees(j2)
        j3_deg = math.degrees(j3)
        j4_deg = math.degrees(j4)

        now = time.localtime()
        time_str = time.strftime('%H:%M:%S', now)
        ms = int((time.time() % 1) * 1000)

        log_line = (
            f"[{time_str}.{ms:03d}] "
            f"j1: {j1:+7.4f} rad ({j1_deg:+6.1f}°) | "
            f"j2: {j2:+7.4f} rad ({j2_deg:+6.1f}°) | "
            f"j3: {j3:+7.4f} rad ({j3_deg:+6.1f}°) | "
            f"j4: {j4:+7.4f} rad ({j4_deg:+6.1f}°) | "
            f"grip: {gripper:6.4f} m"
        )
        print(log_line, flush=True)


def main(args=None):
    rclpy.init(args=args)
    logger_node = JointStateLogger()

    try:
        rclpy.spin(logger_node)
    except KeyboardInterrupt:
        print('\nStopping Joint State Logger...')
    finally:
        logger_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
