#!/usr/bin/env python3
#
# calibrate_camera.py
#
# Camera -> robot calibration for object_detector and scripted_demo: a rigid (Kabsch) fit between AprilTag centres measured
# by the camera (pixel + depth) and the same tag centres touched by the gripper (forward kinematics of /joint_states).
# Same method as yolo_pipeline's scratch/calibrate_kabsch.py, but the robot positions come from the OM-X's own kinematics
# instead of typed-in readings, any number of tags (>= 4 recommended, spread over the workspace) can be used, and the
# result is in metres.
#
#   1. Arm out of the camera's view (e.g. HOME). ENTER: the camera positions of all visible tags are averaged (2 s).
#   2. For each tag: put the fingertip centre (end_effector_link, gripper closed) on the tag's centre, ENTER records the
#      forward kinematics; s skips the tag. Move the arm by hand with torque off (homing menu option 1) or with
#      joint_commander / ee_commander.
#   3. The fit is computed, residuals printed per tag (aim: mean < 5 mm, max < 10 mm), and saved to calibration_file.
#      object_detector reloads it automatically.
#
# Needs object_detector running (it measures the tags; no calibration needed for that) and the arm launch.
#
# Usage:
#   ros2 run custom_package calibrate_camera
#   ros2 run custom_package calibrate_camera --ros-args -p calibration_file:=/workspace/calibration/camera_calibration.yaml
#

import json
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from custom_package import perception_tools as P
from custom_package.ee_commander import forward_kinematics

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4']


class CameraCalibrator(Node):

    def __init__(self):
        super().__init__('calibrate_camera')
        self.declare_parameter('calibration_file', '/workspace/calibration/camera_calibration.yaml')
        self.declare_parameter('detections_topic', '/object_detector/detections')
        self.declare_parameter('capture_seconds', 2.0)
        self.declare_parameter('fingertip_z_offset', 0.0)  # m added to the FK height (e.g. if a finger pad sits above the tag)
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.calibration_file = p('calibration_file')
        self.capture_seconds = float(p('capture_seconds'))
        self.z_offset = float(p('fingertip_z_offset'))
        self.lock = threading.Lock()
        self.detections = []        # recent (time, tags dict)
        self.joints = None
        self.joints_time = 0.0
        self.info = {}              # latest detections message (for the intrinsics)
        self.create_subscription(String, p('detections_topic'), self.detections_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

    def detections_callback(self, msg):
        data = json.loads(msg.data)
        with self.lock:
            self.detections.append((time.monotonic(), data))
            self.detections = self.detections[-100:]
            self.info = data

    def joint_state_callback(self, msg):
        names = list(msg.name)
        if all(j in names for j in ARM_JOINTS):
            with self.lock:
                self.joints = [msg.position[names.index(j)] for j in ARM_JOINTS]
                self.joints_time = time.monotonic()

    def capture_tags(self):
        """{tag id: median camera-frame position} over the last capture_seconds of detections."""
        t0 = time.monotonic()
        time.sleep(self.capture_seconds)
        with self.lock:
            recent = [d for t, d in self.detections if t >= t0]
        samples = {}
        for d in recent:
            for tag_id, tag in d['tags'].items():
                if tag['xyz_camera'] is not None:
                    samples.setdefault(int(tag_id), []).append(tag['xyz_camera'])
        return {k: np.median(np.array(v), axis=0) for k, v in samples.items() if len(v) >= 3}, len(recent)

    def fingertip(self):
        with self.lock:
            joints, age = self.joints, time.monotonic() - self.joints_time
        if joints is None or age > 0.5:
            return None
        x, y, z, pitch = forward_kinematics(joints)
        return np.array([x, y, z + self.z_offset]), pitch


def ask(prompt):
    try:
        return input(prompt).strip().lower()
    except (EOFError, KeyboardInterrupt):
        return 'q'


def run(node):
    print('\n  Step 1: move the arm out of the camera view (e.g. HOME); every tag must be visible and flat on the table.')
    if ask('  ENTER = measure the tags with the camera, q = quit: ') == 'q':
        return
    camera_points, n = node.capture_tags()
    if n == 0:
        print('  ❌  No detections received: is object_detector running (and the camera launched with depth:=true)?')
        return
    if len(camera_points) < 3:
        print(f'  ❌  Only {len(camera_points)} tag(s) with depth found ({sorted(camera_points)}); need >= 3, better 4-6.')
        return
    print(f'  ✅  {len(camera_points)} tags measured: {sorted(camera_points)}')

    pairs = []
    print('\n  Step 2: for each tag, put the fingertip centre (gripper closed) on the tag centre and press ENTER.')
    print('          Move the arm by hand with torque off (homing menu option 1), or with joint_commander.')
    for tag_id in sorted(camera_points):
        while True:
            choice = ask(f'  📍  tag {tag_id}: ENTER = record, s = skip, q = quit: ')
            if choice == 'q':
                return
            if choice == 's':
                break
            tip = node.fingertip()
            if tip is None:
                print('  ❌  No fresh /joint_states.')
                continue
            xyz, pitch = tip
            pairs.append((tag_id, camera_points[tag_id], xyz))
            print(f'      recorded ({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f}) m, gripper pitch {np.degrees(pitch):.0f}°')
            break
    if len(pairs) < 3:
        print(f'  ❌  Only {len(pairs)} tag(s) recorded; need >= 3.')
        return

    cam = np.array([c for _, c, _ in pairs])
    world = np.array([w for _, _, w in pairs])
    R, t = P.kabsch(cam, world)
    errors = P.residuals(cam, world, R, t)
    print('\n  Step 3: fit')
    for (tag_id, _, w), e in zip(pairs, errors):
        print(f'      tag {tag_id}: world ({w[0]:.3f}, {w[1]:.3f}, {w[2]:.3f})  error {e * 1000:5.1f} mm {"✅" if e < 0.01 else "⚠️"}')
    print(f'      mean {errors.mean() * 1000:.1f} mm, max {errors.max() * 1000:.1f} mm. Camera at ({t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f}) m.')
    if errors.max() > 0.015:
        print('  ⚠️  Max error above 15 mm: re-touch the worst tags (fingertip centre on the tag centre) and run again.')
    with node.lock:
        k = node.info.get('intrinsics', {})  # the camera_info object_detector used for the tag positions
    cal = P.CameraCalibration(R=R, t=t, fx=k.get('fx', 0.0), fy=k.get('fy', 0.0), cx=k.get('cx', 0.0), cy=k.get('cy', 0.0),
                              width=k.get('width', 640), height=k.get('height', 480), errors_m=errors.tolist(),
                              points=[{'id': int(i), 'camera': c.tolist(), 'world': w.tolist()} for i, c, w in pairs])
    if ask(f'  💾  Save to {node.calibration_file}? (y/n): ') == 'y':
        cal.save(node.calibration_file)
        print('  ✅  Saved. object_detector picks it up automatically.')


def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibrator()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    print()
    print('=' * 66)
    print('  📐  OpenManipulator-X  |  Camera Calibration (camera -> world)')
    print('=' * 66)
    try:
        run(node)
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
