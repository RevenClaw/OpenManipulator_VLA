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

import select
import sys
import termios
import threading
import time
import tty

from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import SwitchController
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class InteractiveHomingNode(Node):
    """
    Interactive Homing Node:
    - Provides interactive startup menu (Option 1: Custom Manual Pose, Option 2: MoveIt Default Pose).
    - Deactivates arm_controller while the arm is moved by hand and re-activates it after torque enable, so it
      holds the new manual position instead of pulling back to its old hold target.
    - Saves chosen pose as session_home_position.
    - Monitors terminal for 'z' keypress to move to the minimum-stress exit position, then asks to
      either return to session_home_position (and keep listening) or disable torque and quit.
    """

    def __init__(self):
        super().__init__('interactive_homing_node')

        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        self.moveit_default_home = [0.6090, -1.3622, 1.3898, -0.0154]
        self.zero_pose = [0.0, 0.0, 0.0, 0.0]
        # Resting pose the arm settles into with torque off (measured), so cutting torque here
        # barely drops it. joint2 rests at -1.8392, just past the servo Min Position Limit, so it
        # is commanded to -1.8190 (just inside the limit).
        self.min_stress_exit_position = [0.4633, -1.8190, 1.4972, 0.4019]
        self.session_home_position = None
        self.current_positions = None
        self.lock = threading.Lock()

        self.quit_trigger_received = False
        self.running = True

        # Service client to control motor torque
        self.torque_client = self.create_client(
            SetBool, '/dynamixel_hardware_interface/set_dxl_torque'
        )

        # Service client to activate/deactivate arm_controller around manual positioning
        self.switch_controller_client = self.create_client(
            SwitchController, '/controller_manager/switch_controller'
        )

        # Action client to send trajectory commands
        self.action_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory'
        )

        # Service server
        self.quit_service = self.create_service(
            Trigger, '/go_home_and_quit', self.go_home_and_quit_service_callback
        )

        # Subscriber for joint states
        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        self.get_logger().info('Interactive Homing & Session Home Manager Initialized.')

    def go_home_and_quit_service_callback(self, request, response):
        self.quit_trigger_received = True
        response.success = True
        response.message = 'Go-home and quit sequence triggered.'
        return response

    def joint_state_callback(self, msg: JointState):
        if set(self.joint_names).issubset(set(msg.name)):
            with self.lock:
                self.current_positions = [
                    msg.position[msg.name.index(j)] for j in self.joint_names
                ]

    def set_torque(self, enable: bool) -> bool:
        if not self.torque_client.wait_for_service(timeout_sec=5.0):
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

        res = future.result()
        if res is not None:
            self.get_logger().info(f"Torque enable={enable} response: {res.message}")
            return res.success
        else:
            self.get_logger().error('Failed to call torque service.')
            return False

    def set_arm_controller_active(self, active: bool) -> bool:
        """
        Activates or deactivates arm_controller. On activation the JointTrajectoryController
        holds the joints at their measured position.
        """
        if not self.switch_controller_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                'Service /controller_manager/switch_controller not available.'
            )
            return False

        request = SwitchController.Request()
        if active:
            request.activate_controllers = ['arm_controller']
        else:
            request.deactivate_controllers = ['arm_controller']
        request.strictness = SwitchController.Request.STRICT
        request.timeout.sec = 3
        future = self.switch_controller_client.call_async(request)

        start_time = time.time()
        while not future.done():
            if time.time() - start_time > 5.0:
                self.get_logger().error('Timeout waiting for switch_controller response.')
                return False
            time.sleep(0.05)

        res = future.result()
        if res is None or not res.ok:
            self.get_logger().error(
                f"Failed to {'activate' if active else 'deactivate'} arm_controller"
                + (f': {res.message}' if res is not None and res.message else '.')
            )
            return False
        self.get_logger().info(f"arm_controller {'activated' if active else 'deactivated'}.")
        return True

    def sync_controller_goal_to_current(self, positions) -> bool:
        """
        Sends a single trajectory point at the current positions with short duration
        to overwrite arm_controller's internal goal target, preventing it from snapping back
        to its startup default position.
        """
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server /arm_controller/follow_joint_trajectory not available.')
            return False

        goal_msg = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * 4
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 200000000  # 200ms

        traj.points.append(point)
        goal_msg.trajectory = traj

        future = self.action_client.send_goal_async(goal_msg)

        start_time = time.time()
        while not future.done():
            if time.time() - start_time > 3.0:
                return False
            time.sleep(0.05)

        return True

    def send_safe_trajectory(self, target_positions, duration_sec=5.0) -> bool:
        """
        Sends a smooth 2-stage trajectory to prevent motor overload (high gravitational torque on Joint 2).
        Stage 1: Lifts joint3/joint4 up slightly to reduce lever arm load.
        Stage 2: Moves smoothly to final target positions.
        """
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server /arm_controller/follow_joint_trajectory not available.')
            return False

        with self.lock:
            start_pos = list(self.current_positions) if self.current_positions else [0.0]*4

        goal_msg = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        # Intermediate waypoint to prevent sudden gravity strain on joint 2 (shoulder)
        mid_j2 = (start_pos[1] + target_positions[1]) / 2.0
        mid_j3 = max(start_pos[2], target_positions[2])
        intermediate_pos = [start_pos[0], mid_j2, mid_j3, target_positions[3]]

        # Waypoint 1 (Midpoint at half duration)
        pt1 = JointTrajectoryPoint()
        pt1.positions = intermediate_pos
        pt1.velocities = [0.0] * 4
        pt1.time_from_start.sec = int(duration_sec / 2.0)
        pt1.time_from_start.nanosec = int(((duration_sec / 2.0) % 1) * 1e9)
        traj.points.append(pt1)

        # Waypoint 2 (Final target at full duration)
        pt2 = JointTrajectoryPoint()
        pt2.positions = target_positions
        pt2.velocities = [0.0] * 4
        pt2.time_from_start.sec = int(duration_sec)
        pt2.time_from_start.nanosec = int((duration_sec % 1) * 1e9)
        traj.points.append(pt2)

        goal_msg.trajectory = traj
        future = self.action_client.send_goal_async(goal_msg)

        start_time = time.time()
        while not future.done():
            if time.time() - start_time > 5.0:
                self.get_logger().error('Timeout sending trajectory goal.')
                return False
            time.sleep(0.05)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Trajectory goal rejected by controller.')
            return False

        result_future = goal_handle.get_result_async()
        start_time = time.time()
        while not result_future.done():
            if time.time() - start_time > (duration_sec + 5.0):
                self.get_logger().error('Timeout waiting for trajectory execution.')
                return False
            time.sleep(0.05)

        return True


def read_tty_choice(prompt, valid_choices=('1', '2', '3')):
    sys.stdout.write(prompt)
    sys.stdout.flush()
    try:
        with open('/dev/tty', 'r') as tty_file:
            while True:
                line = tty_file.readline().strip()
                if line in valid_choices:
                    return line
                sys.stdout.write(f"\nInvalid choice '{line}'. Please enter {', '.join(valid_choices)}: ")
                sys.stdout.flush()
    except Exception:
        while True:
            line = input().strip()
            if line in valid_choices:
                return line
            print(f"Invalid choice. Please enter {', '.join(valid_choices)}: ")


def wait_for_enter_keypress():
    sys.stdout.write('\n👉 Move the robot arm by hand to your desired INITIAL / HOME position.\n')
    sys.stdout.write('👉 Press ENTER in this terminal when ready to LOCK torque...\n')
    sys.stdout.flush()
    try:
        with open('/dev/tty', 'r') as tty_file:
            tty_file.readline()
    except Exception:
        input()


def listen_for_z_key(node):
    """Background listener for 'z' keypress to trigger the move to the minimum-stress exit position."""
    def key_listener():
        try:
            with open('/dev/tty', 'r') as tty_file:
                fd = tty_file.fileno()
                old_settings = termios.tcgetattr(fd)
                try:
                    tty.setcbreak(fd)
                    while node.running and not node.quit_trigger_received:
                        rlist, _, _ = select.select([tty_file], [], [], 0.1)
                        if rlist:
                            ch = tty_file.read(1)
                            if ch.lower() == 'z':
                                sys.stdout.write("\n\n🏠 'z' key detected! Moving to Minimum Stress Exit Position...\n")
                                sys.stdout.flush()
                                node.quit_trigger_received = True
                                break
                finally:
                    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        except Exception:
            pass

    t = threading.Thread(target=key_listener, daemon=True)
    t.start()
    return t


def main(args=None):
    rclpy.init(args=args)
    node = InteractiveHomingNode()

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print('\n' + '=' * 65)
    print(' 🦾 OPENMANIPULATOR-X HOMING SETUP')
    print('=' * 65)
    print('Waiting for /joint_states from robot arm...')
    while node.current_positions is None:
        time.sleep(0.2)

    print('Disabling motor torque to allow moving the arm freely by hand...')
    node.set_torque(enable=False)

    menu_prompt = (
        "\nSelect Homing Mode for this Session:\n"
        "  [1] Custom Home Position (Move arm by hand & press ENTER)\n"
        "  [2] MoveIt Default Home Position [0.6090, -1.3622, 1.3898, -0.0154]\n"
        "  [3] Zero Pose [0.0, 0.0, 0.0, 0.0] (All joints to zero)\n\n"
        "Enter choice (1, 2, or 3): "
    )

    choice = read_tty_choice(menu_prompt, valid_choices=('1', '2', '3'))

    if choice == '1':
        print('\n[Option 1 Selected] Motor torque disabled. Position arm by hand.')
        # Without an active arm_controller there is no stale hold target to pull back to
        controller_deactivated = node.set_arm_controller_active(False)
        if not controller_deactivated:
            print('⚠️ Could not deactivate arm_controller; falling back to syncing its goal before torque enable.')
        wait_for_enter_keypress()

        with node.lock:
            node.session_home_position = list(node.current_positions)

        pos_str = ', '.join([f'{p:+.4f}' for p in node.session_home_position])
        print(f'\nCapturing Custom Home Position [j1, j2, j3, j4]: [{pos_str}]')
        if controller_deactivated:
            print('Enabling motor torque and re-activating arm_controller...')
            node.set_torque(enable=True)
            # Re-activated controller holds the current (manual) position
            node.set_arm_controller_active(True)
        else:
            print('Syncing controller target and enabling motor torque...')
            node.sync_controller_goal_to_current(node.session_home_position)
            time.sleep(0.3)
            node.set_torque(enable=True)

    elif choice == '2':
        print('\n[Option 2 Selected] Smoothly moving to MoveIt Home Position [0.6090, -1.3622, 1.3898, -0.0154]...')
        node.session_home_position = list(node.moveit_default_home)
        print('Enabling motor torque...')
        node.set_torque(enable=True)
        node.send_safe_trajectory(node.session_home_position, duration_sec=5.0)

    elif choice == '3':
        print('\n[Option 3 Selected] Smoothly moving to Zero Pose [0.0, 0.0, 0.0, 0.0]...')
        node.session_home_position = list(node.zero_pose)
        print('Enabling motor torque...')
        node.set_torque(enable=True)
        node.send_safe_trajectory(node.session_home_position, duration_sec=5.0)

    session_pos_str = ', '.join([f'{p:+.4f}' for p in node.session_home_position])
    exit_pos_str = ', '.join([f'{p:+.4f}' for p in node.min_stress_exit_position])

    print('\n' + '=' * 65)
    print(f'🔒 TORQUE LOCKED! Session Home Position: [{session_pos_str}]')
    print('=' * 65)

    exit_menu_prompt = (
        "\nAt Minimum Stress Exit Position. Select next step:\n"
        f"  [1] Return to Session Home Position [{session_pos_str}]\n"
        "  [2] Disable torque on all motors and quit (the arm relaxes under gravity)\n\n"
        "Enter choice (1 or 2): "
    )

    while True:
        print(f"👉 Press 'z' in this terminal at ANY TIME to move to Minimum Stress Exit Position [{exit_pos_str}].")
        print("   (Or run in another terminal: ros2 service call /go_home_and_quit std_srvs/srv/Trigger)")
        print('=' * 65 + '\n')

        # Background key listener for 'z'
        listener = listen_for_z_key(node)

        while not node.quit_trigger_received:
            time.sleep(0.1)
        # The listener must restore the terminal before the menu reads from it
        listener.join(timeout=1.0)

        # Move to the Minimum Stress Exit Position
        print(f'\n🏠 MOVING TO MINIMUM STRESS EXIT POSITION: [{exit_pos_str}]...')
        if node.send_safe_trajectory(node.min_stress_exit_position, duration_sec=5.0):
            print('✅ Reached Minimum Stress Exit Position.')
        else:
            print('⚠️ Trajectory completed or timed out.')

        choice = read_tty_choice(exit_menu_prompt, valid_choices=('1', '2'))
        if choice == '2':
            break

        print(f'\n🔁 RETURNING TO SESSION HOME POSITION: [{session_pos_str}]...')
        if node.send_safe_trajectory(node.session_home_position, duration_sec=5.0):
            print('✅ Reached Session Home Position.')
        else:
            print('⚠️ Trajectory completed or timed out.')
        node.quit_trigger_received = False
        print('\n' + '=' * 65)

    print('\nDisabling motor torque...')
    if not node.set_torque(enable=False):
        print('⚠️ Torque disable failed! Motors may still be powered.')

    print('👋 Exiting interactive homing node...')
    node.running = False
    # Stop the spin thread before shutting rclpy down (avoids the abort on exit)
    executor.shutdown()
    spin_thread.join(timeout=2.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
