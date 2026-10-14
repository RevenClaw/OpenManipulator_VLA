#!/usr/bin/env python3
#
# joint_commander.py
#
# Interactive terminal script to send joint angle commands to the
# OpenManipulator-X arm and gripper directly via ros2_control action servers.
#
# Usage:
#   ros2 run custom_package joint_commander
#
# Input format (space-separated):
#   <j1_deg> <j2_deg> <j3_deg> <j4_deg> <gripper>
#
#   j1..j4 : Joint angles in DEGREES
#   gripper : 0 = close,  1 = open
#

import math
import sys
import threading
import time

from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# ---------------------------------------------------------------------------
# Joint limits (from open_manipulator_x_arm.urdf.xacro)
# ---------------------------------------------------------------------------
JOINT_LIMITS = {
    'joint1': (-math.pi,        math.pi),        # -180.0° to +180.0°
    'joint2': (-1.5,            1.5),             # -85.94° to +85.94°
    'joint3': (-1.5,            1.4),             # -85.94° to +80.21°
    'joint4': (-1.7,            1.97),            # -97.40° to +112.87°
}

GRIPPER_OPEN_M  =  0.019   # m  (upper limit of gripper_left_joint = 0.020)
GRIPPER_CLOSE_M = -0.010   # m  (matches initial_value in ros2_control.xacro)

MOVE_DURATION_SEC = 4.0    # seconds for joint trajectory execution


# ---------------------------------------------------------------------------
# ROS 2 Commander Node
# ---------------------------------------------------------------------------
class JointCommanderNode(Node):

    def __init__(self):
        super().__init__('joint_commander_node')

        self.arm_client = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        self.gripper_client = ActionClient(
            self, FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory'
        )

        self.get_logger().info('Joint Commander Node initialized.')

    # ------------------------------------------------------------------
    def _send_trajectory(self, action_client, joint_names, positions,
                         duration_sec, label='trajectory') -> bool:
        """Generic helper to send a single-point trajectory and wait for result."""
        if not action_client.wait_for_server(timeout_sec=5.0):
            print(f'  ❌  Action server not available for {label}.')
            return False

        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = joint_names

        pt = JointTrajectoryPoint()
        pt.positions   = list(positions)
        pt.velocities  = [0.0] * len(positions)
        pt.time_from_start.sec     = int(duration_sec)
        pt.time_from_start.nanosec = int((duration_sec % 1) * 1e9)
        traj.points.append(pt)
        goal.trajectory = traj

        future = action_client.send_goal_async(goal)
        start = time.time()
        while not future.done():
            if time.time() - start > 5.0:
                print(f'  ❌  Timeout sending {label} goal.')
                return False
            time.sleep(0.05)

        goal_handle = future.result()
        if not goal_handle.accepted:
            print(f'  ❌  {label} goal rejected by controller.')
            return False

        result_future = goal_handle.get_result_async()
        start = time.time()
        while not result_future.done():
            if time.time() - start > (duration_sec + 5.0):
                print(f'  ❌  Timeout waiting for {label} result.')
                return False
            time.sleep(0.05)

        return True

    # ------------------------------------------------------------------
    def move_arm(self, j1_rad, j2_rad, j3_rad, j4_rad) -> bool:
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        positions   = [j1_rad, j2_rad, j3_rad, j4_rad]
        return self._send_trajectory(
            self.arm_client, joint_names, positions,
            MOVE_DURATION_SEC, label='arm'
        )

    # ------------------------------------------------------------------
    def move_gripper(self, open_flag: bool) -> bool:
        pos = GRIPPER_OPEN_M if open_flag else GRIPPER_CLOSE_M
        return self._send_trajectory(
            self.gripper_client,
            ['gripper_left_joint'], [pos],
            1.5, label='gripper'
        )


# ---------------------------------------------------------------------------
# Input parsing & validation
# ---------------------------------------------------------------------------
def parse_input(line: str):
    """
    Parse a line of 5 space-separated values.
    Returns (j1_rad, j2_rad, j3_rad, j4_rad, gripper_open: bool)
    or raises ValueError with a descriptive message.
    """
    parts = line.strip().split()
    if len(parts) != 5:
        raise ValueError(
            f'Expected 5 values, got {len(parts)}. '
            'Format: <j1°> <j2°> <j3°> <j4°> <gripper 0|1>'
        )

    try:
        degrees = [float(p) for p in parts[:4]]
        gripper_flag = int(parts[4])
    except ValueError:
        raise ValueError('All 5 values must be numeric (degrees for joints, 0 or 1 for gripper).')

    if gripper_flag not in (0, 1):
        raise ValueError(f'Gripper value must be 0 (close) or 1 (open), got {gripper_flag}.')

    # Convert to radians
    radians = [math.radians(d) for d in degrees]

    # Validate limits
    joint_names = list(JOINT_LIMITS.keys())
    violations = []
    for i, (name, rad) in enumerate(zip(joint_names, radians)):
        lo, hi = JOINT_LIMITS[name]
        if not (lo <= rad <= hi):
            lo_deg = math.degrees(lo)
            hi_deg = math.degrees(hi)
            violations.append(
                f'  ⚠️  {name}: {degrees[i]:+.2f}° is out of range '
                f'[{lo_deg:.2f}°, {hi_deg:.2f}°]'
            )

    if violations:
        raise ValueError('Joint limit violations:\n' + '\n'.join(violations))

    return radians[0], radians[1], radians[2], radians[3], bool(gripper_flag)


# ---------------------------------------------------------------------------
# Helpers for display
# ---------------------------------------------------------------------------
def print_limits_table():
    print('\n  Joint Limits Reference:')
    print(f'  {"Joint":<10} {"Min (°)":>10} {"Max (°)":>10}')
    print(f'  {"-"*33}')
    for name, (lo, hi) in JOINT_LIMITS.items():
        print(f'  {name:<10} {math.degrees(lo):>+10.2f} {math.degrees(hi):>+10.2f}')
    print(f'  {"Gripper":<10} {"0 = close":>10} {"1 = open":>10}')
    print()


def print_header():
    print()
    print('=' * 60)
    print('  🦾  OpenManipulator-X  |  Joint Commander')
    print('=' * 60)
    print_limits_table()
    print('  Enter 5 values:  <j1°> <j2°> <j3°> <j4°> <gripper 0|1>')
    print('  Type  "quit" or "q"  to exit.')
    print('=' * 60)


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = JointCommanderNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print_header()

    try:
        while True:
            print()
            try:
                line = input('  >> ').strip()
            except (EOFError, KeyboardInterrupt):
                print('\n  👋  Interrupted. Exiting...')
                break

            if line.lower() in ('q', 'quit', 'exit'):
                print('  👋  Exiting joint commander.')
                break

            if not line:
                continue

            # --- Parse & validate ---
            try:
                j1, j2, j3, j4, gripper_open = parse_input(line)
            except ValueError as e:
                print(f'\n  ❌  Invalid input:\n{e}')
                print('  ↩️   Please enter new coordinates.')
                continue

            # --- Echo confirmed command ---
            print(
                f'\n  ✅  Sending command:\n'
                f'      joint1  = {math.degrees(j1):+.2f}°  ({j1:+.4f} rad)\n'
                f'      joint2  = {math.degrees(j2):+.2f}°  ({j2:+.4f} rad)\n'
                f'      joint3  = {math.degrees(j3):+.2f}°  ({j3:+.4f} rad)\n'
                f'      joint4  = {math.degrees(j4):+.2f}°  ({j4:+.4f} rad)\n'
                f'      gripper = {"OPEN  (1)" if gripper_open else "CLOSE (0)"}\n'
            )
            print(f'  ⏳  Moving arm... ({MOVE_DURATION_SEC:.1f}s)')

            # --- Send arm trajectory ---
            arm_ok = node.move_arm(j1, j2, j3, j4)

            # --- Send gripper command ---
            gripper_ok = node.move_gripper(gripper_open)

            # --- Result ---
            if arm_ok and gripper_ok:
                print('  🎯  Goal reached!')
            else:
                print('  ⚠️   Movement completed with errors (see above).')

    finally:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)


if __name__ == '__main__':
    main()
