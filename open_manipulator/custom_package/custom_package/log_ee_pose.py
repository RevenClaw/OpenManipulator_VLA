#!/usr/bin/env python3
#
# log_ee_pose.py
#
# Prints the live end-effector position of the OpenManipulator-X: x, y, z (m) of `end_effector_link` in the robot's
# `world` frame (= link1: x forward, y left, z up) and the gripper pitch (0 = horizontal, +90° = pointing down),
# computed from /joint_states with the same forward kinematics as ee_commander and calibrate_camera.
# The printed values can be typed straight into ee_commander (`x y z pitch°`).
#
# Usage:
#   ros2 run custom_package log_ee_pose                                   # 5 Hz
#   ros2 run custom_package log_ee_pose --ros-args -p log_rate_hz:=10.0
#   ros2 run custom_package log_ee_pose --ros-args -p once:=true          # one line, then exit
#

import math
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from custom_package.ee_commander import forward_kinematics

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4']


class EePoseLogger(Node):

    def __init__(self):
        super().__init__('ee_pose_logger')
        self.declare_parameter('log_rate_hz', 5.0)
        self.declare_parameter('once', False)
        rate = float(self.get_parameter('log_rate_hz').value)
        self.once = bool(self.get_parameter('once').value)
        self.done = threading.Event()
        self.latest = None
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.create_timer(1.0 / rate if rate > 0 else 0.2, self.timer_callback)

    def joint_state_callback(self, msg):
        self.latest = msg

    def timer_callback(self):
        msg = self.latest
        if msg is None or self.done.is_set():
            return
        pos = dict(zip(msg.name, msg.position))
        if not all(j in pos for j in ARM_JOINTS):
            return
        x, y, z, pitch = forward_kinematics([pos[j] for j in ARM_JOINTS])
        grip = pos.get('gripper_left_joint', float('nan'))
        t = time.time()
        stamp = time.strftime('%H:%M:%S', time.localtime(t)) + f'.{int((t % 1) * 1000):03d}'
        print(f'[{stamp}] 📍 x: {x:+.4f} m | y: {y:+.4f} m | z: {z:+.4f} m | pitch: {math.degrees(pitch):+6.1f}° | '
              f'grip: {grip:+.4f} m   (ee_commander: {x:.3f} {y:.3f} {z:.3f} {math.degrees(pitch):.1f})', flush=True)
        if self.once:
            self.done.set()


def main(args=None):
    rclpy.init(args=args)
    node = EePoseLogger()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    print('\n' + '=' * 100)
    print('  📍  OpenManipulator-X  |  Live end-effector pose (world frame = base, x fwd, y left, z up)')
    print('=' * 100)
    try:
        while not node.done.wait(0.2):
            if node.latest is None:
                node.get_logger().info('waiting for /joint_states ...', throttle_duration_sec=5.0)
    except KeyboardInterrupt:
        print('\nStopping EE pose logger...')
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
