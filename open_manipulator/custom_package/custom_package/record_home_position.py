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

import os
import sys
import threading
import time
import yaml

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool


class HomePositionRecorder(Node):
    """
    Node to teach and record a custom home position for OpenManipulator-X.
    Disables torque, listens to /joint_states, prompts user to save on ENTER,
    and updates configured_home_position.yaml.
    """

    def __init__(self):
        super().__init__('home_position_recorder')

        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        self.current_positions = None
        self.lock = threading.Lock()

        # Service client to control motor torque
        self.torque_client = self.create_client(
            SetBool, '/dynamixel_hardware_interface/set_dxl_torque'
        )

        # Subscriber for joint states
        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        self.get_logger().info('Home Position Recorder Node Initialized.')

    def joint_state_callback(self, msg: JointState):
        if set(self.joint_names).issubset(set(msg.name)):
            with self.lock:
                self.current_positions = [
                    msg.position[msg.name.index(j)] for j in self.joint_names
                ]

    def set_torque(self, enable: bool) -> bool:
        if not self.torque_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn(
                'Torque service /dynamixel_hardware_interface/set_dxl_torque not available.'
            )
            return False

        request = SetBool.Request()
        request.data = enable
        future = self.torque_client.call_async(request)
        
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > 3.0:
                self.get_logger().error('Timeout waiting for torque service response.')
                return False
            time.sleep(0.05)

        if future.result() is not None:
            self.get_logger().info(f"Torque enable={enable} response: {future.result().message}")
            return future.result().success
        else:
            self.get_logger().error('Failed to call torque service.')
            return False

    def save_home_position(self, positions):
        rounded_positions = [round(float(p), 4) for p in positions]

        yaml_content = {
            'joint_trajectory_executor': {
                'ros__parameters': {
                    'joint_names': self.joint_names,
                    'step_names': ['home'],
                    'home': rounded_positions,
                    'duration': 3.0,
                    'epsilon': 0.01,
                    'action_topic': '/arm_controller/follow_joint_trajectory',
                    'joint_states_topic': '/joint_states',
                }
            }
        }

        # Resolve paths for both package share and source workspace inside container
        paths_to_write = []
        try:
            share_dir = get_package_share_directory('custom_package')
            paths_to_write.append(
                os.path.join(share_dir, 'config', 'configured_home_position.yaml')
            )
        except Exception:
            pass

        src_yaml_path = os.path.abspath(
            os.path.join(
                os.path.dirname(__file__),
                '..',
                'config',
                'configured_home_position.yaml',
            )
        )
        if src_yaml_path not in paths_to_write:
            paths_to_write.append(src_yaml_path)

        for filepath in paths_to_write:
            os.makedirs(os.path.dirname(filepath), exist_ok=True)
            with open(filepath, 'w') as f:
                yaml.dump(yaml_content, f, default_flow_style=False, sort_keys=False)
            self.get_logger().info(f'Saved configured home position to: {filepath}')

        print('\n' + '=' * 60)
        print('✅ CONFIGURED HOME POSITION RECORDED SUCCESSFULLY!')
        print(f'Joint Target Positions (rad): {rounded_positions}')
        print('=' * 60 + '\n')


def main(args=None):
    rclpy.init(args=args)
    recorder = HomePositionRecorder()

    spin_thread = threading.Thread(target=rclpy.spin, args=(recorder,), daemon=True)
    spin_thread.start()

    print('Waiting for /joint_states from robot arm...')
    while recorder.current_positions is None:
        time.sleep(0.2)

    print('\n' + '=' * 60)
    print('🦾 OPENMANIPULATOR-X HOME POSITION TEACHING MODE')
    print('=' * 60)
    print('Disabling motor torque to allow moving the arm by hand...')
    recorder.set_torque(enable=False)

    try:
        while True:
            with recorder.lock:
                pos = list(recorder.current_positions)
            pos_str = ', '.join([f'{p:+.4f}' for p in pos])
            print(f'\rCurrent Joint Angles [j1, j2, j3, j4] (rad): [{pos_str}]', end='', flush=True)

            print('\n------------------------------------------------------------')
            user_input = input('Move robot by hand to desired HOME position and press ENTER to record (or type "q" to cancel): ')

            if user_input.strip().lower() == 'q':
                print('\nOperation cancelled. Restoring torque...')
                recorder.set_torque(enable=True)
                break
            else:
                with recorder.lock:
                    target_pos = list(recorder.current_positions)
                recorder.save_home_position(target_pos)
                print('Restoring motor torque...')
                recorder.set_torque(enable=True)
                break
    except KeyboardInterrupt:
        print('\nInterrupted. Restoring torque...')
        recorder.set_torque(enable=True)
    finally:
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
