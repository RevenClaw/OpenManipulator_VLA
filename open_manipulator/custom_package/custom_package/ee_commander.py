#!/usr/bin/env python3
#
# ee_commander.py
#
# Moves the OpenManipulator-X end effector to a Cartesian goal.
#   1. This node solves the closed-form 4-DOF inverse kinematics (x, y, z, pitch -> joint1..joint4).
#      MoveIt's KDL solver runs position-only for this arm, so it cannot honour pitch.
#   2. MoveIt move_group (/move_action) plans a collision-free path to those joint angles
#      and executes it on arm_controller.
#   3. The gripper is driven through gripper_controller's GripperCommand action.
#
# Requires (each in its own terminal):
#   ros2 launch custom_package open_manipulator_x_interactive.launch.py port_name:=/dev/ttyUSB1
#   ros2 launch open_manipulator_moveit_config open_manipulator_x_moveit.launch.py
#
# Usage:
#   ros2 run custom_package ee_commander
#
# Terminal input (space-separated):
#   <x_m> <y_m> <z_m> <pitch_deg> [<gripper 0|1>]
#   p  -> print the current end-effector pose,  q -> quit
#
# Topic input:  /ee_goal  (std_msgs/Float64MultiArray)
#   data = [x_m, y_m, z_m, pitch_rad, gripper]     gripper: 1 = open, 0 = close, -1 = unchanged
#   A new topic goal replaces any goal still waiting; the goal being executed is not interrupted.
#
# Frame: world (= link1, the robot base). x forward, y left, z up, in metres.
#   The pose is that of end_effector_link (between the fingertips).
#   pitch: gripper tilt about its horizontal axis. 0 = horizontal, positive = pointing down
#   (+90 deg = straight down). Yaw always follows the base (atan2(y, x)); roll is always 0.
#

import math
import threading
import time

from control_msgs.action import GripperCommand
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import JointConstraint
from moveit_msgs.msg import MoveItErrorCodes
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from custom_package.joint_commander import GRIPPER_CLOSE_M
from custom_package.joint_commander import GRIPPER_OPEN_M
from custom_package.joint_commander import JOINT_LIMITS


# ---------------------------------------------------------------------------
# Kinematic constants (from open_manipulator_x_arm.urdf.xacro, world = link1)
# ---------------------------------------------------------------------------
JOINT1_X = 0.012                     # joint1 axis offset along x
JOINT2_Z = 0.0595                    # joint2 axis height above the base
UPPER_ARM = (0.024, 0.128)           # joint2 -> joint3 (x, z) at joint2 = 0
FOREARM = 0.124                      # joint3 -> joint4
GRIPPER = 0.126                      # joint4 -> end_effector_link

UPPER_ARM_LEN = math.hypot(*UPPER_ARM)
# Downward pitch of the upper arm at joint2 = 0 is -UPPER_ARM_ANGLE (it points almost straight up)
UPPER_ARM_ANGLE = math.atan2(UPPER_ARM[1], UPPER_ARM[0])

JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4']


def forward_kinematics(joints):
    """Returns (x, y, z, pitch) of end_effector_link for [j1, j2, j3, j4]."""
    j1, j2, j3, j4 = joints
    # Planar chain: r = horizontal distance from joint1 axis, h = height above joint2 axis.
    # A link at downward pitch phi contributes (L cos phi, -L sin phi).
    phi_upper = j2 - UPPER_ARM_ANGLE
    phi_fore = j2 + j3
    pitch = j2 + j3 + j4
    r = (UPPER_ARM_LEN * math.cos(phi_upper) + FOREARM * math.cos(phi_fore)
         + GRIPPER * math.cos(pitch))
    h = -(UPPER_ARM_LEN * math.sin(phi_upper) + FOREARM * math.sin(phi_fore)
          + GRIPPER * math.sin(pitch))
    return (JOINT1_X + r * math.cos(j1), r * math.sin(j1), JOINT2_Z + h, pitch)


def inverse_kinematics(x, y, z, pitch):
    """
    Closed-form IK. Returns (solutions, reason): solutions is a list of [j1, j2, j3, j4]
    (elbow-up first) that lie within JOINT_LIMITS; reason explains an empty list.
    """
    dx = x - JOINT1_X
    r = math.hypot(dx, y)
    j1 = math.atan2(y, dx) if r > 1e-9 else 0.0

    # Wrist (joint4) position in the arm plane, relative to the joint2 axis
    r_w = r - GRIPPER * math.cos(pitch)
    h_w = (z - JOINT2_Z) + GRIPPER * math.sin(pitch)
    dist_sq = r_w * r_w + h_w * h_w

    cos_elbow = (dist_sq - UPPER_ARM_LEN ** 2 - FOREARM ** 2) / (2.0 * UPPER_ARM_LEN * FOREARM)
    if abs(cos_elbow) > 1.0:
        reach = UPPER_ARM_LEN + FOREARM
        return [], (
            f'unreachable: wrist would be {math.sqrt(dist_sq):.3f} m from the shoulder '
            f'(reachable {abs(UPPER_ARM_LEN - FOREARM):.3f} to {reach:.3f} m). '
            'Try a different pitch or move the target closer.'
        )

    wrist_pitch = math.atan2(-h_w, r_w)
    candidates = []
    for elbow in (math.acos(cos_elbow), -math.acos(cos_elbow)):  # elbow-up, elbow-down
        offset = math.atan2(FOREARM * math.sin(elbow), UPPER_ARM_LEN + FOREARM * math.cos(elbow))
        j2 = wrist_pitch - offset + UPPER_ARM_ANGLE
        j3 = elbow - UPPER_ARM_ANGLE
        j4 = pitch - j2 - j3
        candidates.append([_wrap(j1), _wrap(j2), _wrap(j3), _wrap(j4)])

    solutions = [c for c in candidates if not limit_violations(c)]
    if not solutions:
        return [], 'outside joint limits:\n' + '\n'.join(limit_violations(candidates[0]))
    return solutions, ''


def limit_violations(joints):
    out = []
    for name, value in zip(JOINT_NAMES, joints):
        lo, hi = JOINT_LIMITS[name]
        if not lo <= value <= hi:
            out.append(
                f'  ⚠️  {name}: {math.degrees(value):+.2f}° is out of range '
                f'[{math.degrees(lo):.2f}°, {math.degrees(hi):.2f}°]'
            )
    return out


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


# ---------------------------------------------------------------------------
# ROS 2 node
# ---------------------------------------------------------------------------
class EECommanderNode(Node):

    def __init__(self):
        super().__init__('ee_commander')

        self.declare_parameter('velocity_scaling', 0.3)
        self.declare_parameter('acceleration_scaling', 0.3)
        self.declare_parameter('planning_time', 5.0)
        self.declare_parameter('min_z', 0.0)  # reject targets below this height (m)

        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        self.gripper_client = ActionClient(
            self, GripperCommand, '/gripper_controller/gripper_cmd'
        )

        self.current_joints = None
        self.lock = threading.Lock()
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # Latest pending goal (topic and terminal both write here; the worker consumes it)
        self.pending_goal = None
        self.goal_ready = threading.Condition()
        self.running = True
        self.create_subscription(Float64MultiArray, '/ee_goal', self.ee_goal_callback, 10)

        self.get_logger().info('EE Commander initialized. Listening on /ee_goal.')

    def joint_state_callback(self, msg: JointState):
        if set(JOINT_NAMES).issubset(msg.name):
            with self.lock:
                self.current_joints = [msg.position[msg.name.index(j)] for j in JOINT_NAMES]

    def ee_goal_callback(self, msg: Float64MultiArray):
        data = list(msg.data)
        if len(data) not in (4, 5):
            self.get_logger().error(
                f'/ee_goal needs [x, y, z, pitch_rad, gripper], got {len(data)} values.'
            )
            return
        gripper = None
        if len(data) == 5 and data[4] >= 0.0:
            gripper = data[4] >= 0.5
        self.submit_goal(data[0], data[1], data[2], data[3], gripper, source='/ee_goal')

    def submit_goal(self, x, y, z, pitch, gripper, source):
        with self.goal_ready:
            if self.pending_goal is not None:
                self.get_logger().warn('Replacing a goal that had not started yet.')
            self.pending_goal = (x, y, z, pitch, gripper, source)
            self.goal_ready.notify()

    # ------------------------------------------------------------------
    def worker(self):
        while True:
            with self.goal_ready:
                while self.running and self.pending_goal is None:
                    self.goal_ready.wait()
                if not self.running:
                    return
                goal, self.pending_goal = self.pending_goal, None
            self.execute_goal(*goal)

    def execute_goal(self, x, y, z, pitch, gripper, source):
        print(
            f'\n  📥  Goal from {source}: x={x:+.3f} y={y:+.3f} z={z:+.3f} m, '
            f'pitch={math.degrees(pitch):+.1f}°'
            + ('' if gripper is None else f', gripper={"OPEN" if gripper else "CLOSE"}')
        )
        min_z = self.get_parameter('min_z').value
        if z < min_z:
            print(f'  ❌  z={z:.3f} m is below min_z={min_z:.3f} m. Goal rejected.')
            return

        solutions, reason = inverse_kinematics(x, y, z, pitch)
        if not solutions:
            print(f'  ❌  No IK solution, {reason}')
            return
        joints = solutions[0]
        print('  🧮  IK: ' + '  '.join(
            f'{n}={math.degrees(v):+.2f}°' for n, v in zip(JOINT_NAMES, joints)
        ))

        if self.plan_and_execute(joints):
            reached = self.current_pose()
            if reached is not None:
                print(f'  🎯  Reached: x={reached[0]:+.3f} y={reached[1]:+.3f} z={reached[2]:+.3f} m, '
                      f'pitch={math.degrees(reached[3]):+.1f}°')
        else:
            return

        if gripper is not None:
            if self.move_gripper(gripper):
                print(f'  ✋  Gripper {"opened" if gripper else "closed"}.')

    def current_pose(self):
        with self.lock:
            joints = list(self.current_joints) if self.current_joints else None
        return forward_kinematics(joints) if joints else None

    # ------------------------------------------------------------------
    def plan_and_execute(self, joints) -> bool:
        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
            print('  ❌  MoveIt /move_action not available. Is the MoveIt launch running?')
            return False

        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = 'arm'
        req.num_planning_attempts = 5
        req.allowed_planning_time = float(self.get_parameter('planning_time').value)
        req.max_velocity_scaling_factor = float(self.get_parameter('velocity_scaling').value)
        req.max_acceleration_scaling_factor = float(
            self.get_parameter('acceleration_scaling').value
        )
        req.start_state.is_diff = True
        constraints = Constraints()
        for name, value in zip(JOINT_NAMES, joints):
            constraints.joint_constraints.append(JointConstraint(
                joint_name=name, position=value,
                tolerance_above=0.001, tolerance_below=0.001, weight=1.0,
            ))
        req.goal_constraints.append(constraints)
        goal.planning_options.plan_only = False

        print('  ⏳  Planning and executing with MoveIt...')
        result = self._wait_for_action(self.move_group_client, goal, timeout_sec=60.0)
        if result is None:
            return False
        code = result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            print(f'  ❌  MoveIt failed: {_error_code_name(code)} ({code})')
            return False
        return True

    def move_gripper(self, open_flag: bool) -> bool:
        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            print('  ❌  /gripper_controller/gripper_cmd not available.')
            return False
        goal = GripperCommand.Goal()
        goal.command.position = GRIPPER_OPEN_M if open_flag else GRIPPER_CLOSE_M
        goal.command.max_effort = 1.0
        return self._wait_for_action(self.gripper_client, goal, timeout_sec=10.0) is not None

    def _wait_for_action(self, client, goal, timeout_sec):
        """Sends a goal and blocks until its result arrives. Returns the result or None."""
        future = client.send_goal_async(goal)
        start = time.time()
        while not future.done():
            if time.time() - start > 5.0:
                print('  ❌  Timeout sending goal.')
                return None
            time.sleep(0.05)
        goal_handle = future.result()
        if not goal_handle.accepted:
            print('  ❌  Goal rejected.')
            return None
        result_future = goal_handle.get_result_async()
        start = time.time()
        while not result_future.done():
            if time.time() - start > timeout_sec:
                print('  ❌  Timeout waiting for result.')
                return None
            time.sleep(0.05)
        return result_future.result().result


def _error_code_name(code):
    for name in dir(MoveItErrorCodes):
        if name.isupper() and getattr(MoveItErrorCodes, name) == code:
            return name
    return 'UNKNOWN'


# ---------------------------------------------------------------------------
# Terminal input
# ---------------------------------------------------------------------------
def parse_input(line):
    """Returns (x, y, z, pitch_rad, gripper or None) or raises ValueError."""
    parts = line.split()
    if len(parts) not in (4, 5):
        raise ValueError(
            f'Expected 4 or 5 values, got {len(parts)}. '
            'Format: <x_m> <y_m> <z_m> <pitch°> [<gripper 0|1>]'
        )
    try:
        x, y, z, pitch_deg = (float(p) for p in parts[:4])
        gripper = int(parts[4]) if len(parts) == 5 else None
    except ValueError:
        raise ValueError('Values must be numeric (metres, degrees, gripper 0 or 1).')
    if gripper not in (None, 0, 1):
        raise ValueError(f'Gripper value must be 0 (close) or 1 (open), got {gripper}.')
    return x, y, z, math.radians(pitch_deg), None if gripper is None else bool(gripper)


def print_header():
    print()
    print('=' * 66)
    print('  🦾  OpenManipulator-X  |  End-Effector Commander (MoveIt)')
    print('=' * 66)
    print('  Enter:  <x_m> <y_m> <z_m> <pitch°> [<gripper 0|1>]')
    print('          frame = world (robot base), x forward, z up')
    print('          pitch 0 = horizontal, +90 = pointing straight down')
    print('          e.g.  0.20 0.00 0.15 0 1')
    print('  "p" = print current pose,  "q" = quit')
    print('  Other nodes can publish goals on /ee_goal (std_msgs/Float64MultiArray):')
    print('          [x_m, y_m, z_m, pitch_rad, gripper (1 open, 0 close, -1 unchanged)]')
    print('=' * 66)


def main(args=None):
    rclpy.init(args=args)
    node = EECommanderNode()

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    worker_thread = threading.Thread(target=node.worker, daemon=True)
    worker_thread.start()

    print_header()
    try:
        while True:
            try:
                line = input('\n  >> ').strip()
            except (EOFError, KeyboardInterrupt):
                print('\n  👋  Interrupted. Exiting...')
                break
            if not line:
                continue
            if line.lower() in ('q', 'quit', 'exit'):
                print('  👋  Exiting EE commander.')
                break
            if line.lower() == 'p':
                pose = node.current_pose()
                if pose is None:
                    print('  ⚠️  No /joint_states received yet.')
                else:
                    print(f'  📍  x={pose[0]:+.3f} y={pose[1]:+.3f} z={pose[2]:+.3f} m, '
                          f'pitch={math.degrees(pose[3]):+.1f}°')
                continue
            try:
                goal = parse_input(line)
            except ValueError as e:
                print(f'  ❌  Invalid input: {e}')
                continue
            node.submit_goal(*goal, source='terminal')
    finally:
        with node.goal_ready:
            node.running = False
            node.goal_ready.notify()
        worker_thread.join(timeout=1.0)
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
