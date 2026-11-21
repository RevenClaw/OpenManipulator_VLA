#!/usr/bin/env python3
#
# object_detector.py
#
# Finds the cubes (YOLO) and the targets/bins (AprilTags) in the front camera and gives their 3D positions in the robot's
# `world` frame (metres), for scripted_demo and calibrate_camera. Inspired by yolo_pipeline (src/perception/yolo_detector.py,
# src/calibration/), but reads the camera through ROS topics (the RealSense can only be opened by one program, and the
# recorder and vla_bridge need it as a topic), uses a robust depth, and works in metres.
#
#   /camera/front/color/image_raw (rgb8) + aligned depth (16UC1, mm) + camera_info
#     -> YOLO boxes (class = colour, e.g. red/green/blue) and AprilTag 36h11 centres
#     -> depth = median of the box's inner half (tags: a patch at the centre)
#     -> camera 3D point -> world (camera_calibration.yaml from calibrate_camera)
#
# Publishes (rate_hz, default 3 Hz):
#   ~/detections  std_msgs/String JSON: {"stamp", "calibrated", "intrinsics", "objects": [{label, confidence, box, pixel, depth_m,
#                 xyz_camera, xyz}], "targets": [{id, name, pixel, xyz_camera, xyz}], "tags": {id: {pixel, xyz_camera, xyz}}}
#                 xyz = world (null without calibration). Objects: xyz is the cube CENTRE (surface point pushed back by
#                 half a cube along the viewing direction).
#   ~/annotated   sensor_msgs/Image (boxes, tags, positions) for RViz, at the same rate
#   ~/markers     visualization_msgs/MarkerArray in `world` (only when calibrated)
#
# Needs: camera_front.launch.py depth:=true, ultralytics (Dockerfile), a model (yolo_pipeline/YOLO_MODEL/YOLO.pt copied to
# /workspace/models/), and for world coordinates /workspace/calibration/camera_calibration.yaml.
#
# Usage:
#   ros2 run custom_package object_detector --ros-args --params-file \
#       $(ros2 pkg prefix custom_package)/share/custom_package/config/object_detector.yaml
#

import json
import pathlib
import threading
import time

from geometry_msgs.msg import Point
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from custom_package import perception_tools as P
from custom_package import vla_safety as S

COLORS_BGR = {'red': (40, 40, 230), 'green': (40, 180, 40), 'blue': (230, 90, 30)}


class ObjectDetector(Node):

    def __init__(self):
        super().__init__('object_detector')
        self.declare_parameter('color_topic', '/camera/front/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/front/aligned_depth_to_color/image_raw')
        self.declare_parameter('info_topic', '/camera/front/color/camera_info')
        self.declare_parameter('calibration_file', '/workspace/calibration/camera_calibration.yaml')
        self.declare_parameter('model_path', '/workspace/models/yolo_cubes.pt')
        self.declare_parameter('confidence', 0.5)
        self.declare_parameter('image_size', 640)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('target_tags', ['20:bin'])     # "<AprilTag id>:<name used in prompts>"
        self.declare_parameter('object_size', 0.035)           # m; the surface point is pushed back by half of this
        self.declare_parameter('rate_hz', 3.0)
        self.declare_parameter('max_frame_skew', 0.1)          # s between the color and depth frames used together

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.calibration_file = pathlib.Path(p('calibration_file'))
        self.model_path = p('model_path')
        self.confidence, self.image_size, self.device = float(p('confidence')), int(p('image_size')), p('device')
        self.object_size = float(p('object_size'))
        self.max_frame_skew = float(p('max_frame_skew'))
        self.targets = {}
        for item in p('target_tags'):
            if item:
                tag_id, _, name = item.partition(':')
                self.targets[int(tag_id)] = name.strip() or f'tag {tag_id}'

        self.lock = threading.Lock()
        self.color = self.depth = self.info = None
        self.model = None
        self.model_error = ''
        self.calibration = None
        self.calibration_mtime = None
        self.detect_tags = P.make_tag_detector()

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)  # big frames: see vla_bridge
        self.create_subscription(Image, p('color_topic'), lambda m: self._store('color', m), qos)
        self.create_subscription(Image, p('depth_topic'), lambda m: self._store('depth', m), qos)
        self.create_subscription(CameraInfo, p('info_topic'), lambda m: self._store('info', m), 10)
        self.det_pub = self.create_publisher(String, '~/detections', 10)
        self.img_pub = self.create_publisher(Image, '~/annotated', 1)
        self.marker_pub = self.create_publisher(MarkerArray, '~/markers', 1)
        self.busy = False
        self.create_timer(1.0 / float(p('rate_hz')), self.tick)
        self._load_calibration()
        self.get_logger().info(f'object_detector: model {self.model_path}, targets {self.targets}, '
                               f'calibration {"loaded" if self.calibration else "MISSING (camera frame only)"}')

    # ------------------------------------------------------------------ inputs
    def _store(self, key, msg):
        with self.lock:
            setattr(self, key, msg)

    def _load_calibration(self):
        """(Re)loads the calibration when the file appears or changes, so calibrate_camera's result is used at once."""
        try:
            mtime = self.calibration_file.stat().st_mtime
        except OSError:
            return
        if mtime != self.calibration_mtime:
            self.calibration = P.CameraCalibration.load(self.calibration_file)
            self.calibration_mtime = mtime
            err = max(self.calibration.errors_m) if self.calibration.errors_m else float('nan')
            self.get_logger().info(f'calibration loaded ({self.calibration_file}, max error {err * 1000:.1f} mm)')

    def _yolo(self):
        if self.model is None and not self.model_error:
            try:
                from ultralytics import YOLO

                self.model = YOLO(self.model_path)
                self.get_logger().info(f'YOLO model loaded: classes {self.model.names}')
            except Exception as e:  # noqa: BLE001 - keep publishing tags without a model
                self.model_error = str(e)
                self.get_logger().error(f'YOLO unavailable ({e}); publishing AprilTags only.')
        return self.model

    # ------------------------------------------------------------------ processing
    def tick(self):
        if self.busy:
            return
        with self.lock:
            color, depth, info = self.color, self.depth, self.info
        if color is None or depth is None or info is None:
            return
        skew = abs((color.header.stamp.sec - depth.header.stamp.sec) + (color.header.stamp.nanosec - depth.header.stamp.nanosec) * 1e-9)
        if skew > self.max_frame_skew:
            return
        self.busy = True
        try:
            self._load_calibration()
            self.process(color, depth, info)
        finally:
            self.busy = False

    def process(self, color, depth, info):
        rgb = S.ros_image_to_rgb(color.encoding, color.height, color.width, color.step, color.data)
        depth_m = P.depth_image_to_metres(depth.encoding, depth.height, depth.width, depth.step, depth.data)
        if depth_m.shape != rgb.shape[:2]:
            self.get_logger().warn('depth and color sizes differ: launch the camera with depth:=true (aligned depth)',
                                   throttle_duration_sec=10.0)
            return
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        cal = self.calibration
        bgr = np.ascontiguousarray(rgb[..., ::-1])
        to_world = (lambda p: cal.camera_to_world(p).tolist()) if cal is not None else (lambda p: None)

        objects = []
        model = self._yolo()
        if model is not None:
            results = model.predict(bgr, imgsz=self.image_size, conf=self.confidence, device=self.device, verbose=False)
            for r in results:
                for box in r.boxes:
                    x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])
                    b = (int(x1), int(y1), int(x2 - x1), int(y2 - y1))
                    u, v = (x1 + x2) / 2, (y1 + y2) / 2
                    d = P.robust_depth(depth_m, u, v, box=b)
                    p_cam = P.pixel_to_camera(u, v, d, fx, fy, cx, cy) if d else None
                    xyz = None
                    if p_cam is not None and cal is not None:
                        xyz = P.push_back_along_view(cal.camera_to_world(p_cam), cal.t, self.object_size / 2).tolist()
                    objects.append({'label': r.names[int(box.cls[0])], 'confidence': round(float(box.conf[0]), 3),
                                    'box': list(b), 'pixel': [round(u, 1), round(v, 1)], 'depth_m': d,
                                    'xyz_camera': None if p_cam is None else p_cam.tolist(), 'xyz': xyz})

        tags = {}
        for tag_id, (u, v, side) in P.detect_tags(np.ascontiguousarray(bgr.mean(axis=2).astype(np.uint8)), self.detect_tags).items():
            d = P.robust_depth(depth_m, u, v, half=max(2, int(side / 6)))
            p_cam = P.pixel_to_camera(u, v, d, fx, fy, cx, cy) if d else None
            tags[tag_id] = {'pixel': [round(u, 1), round(v, 1)], 'side_px': round(side, 1), 'depth_m': d,
                            'xyz_camera': None if p_cam is None else p_cam.tolist(),
                            'xyz': None if p_cam is None else to_world(p_cam)}
        targets = [{'id': i, 'name': self.targets[i], **tags[i]} for i in sorted(tags) if i in self.targets]

        stamp = color.header.stamp.sec + color.header.stamp.nanosec * 1e-9
        out = {'stamp': stamp, 'wall_time': time.time(), 'calibrated': cal is not None,
               'intrinsics': {'fx': fx, 'fy': fy, 'cx': cx, 'cy': cy, 'width': rgb.shape[1], 'height': rgb.shape[0]},
               'model': None if model is None else self.model_path, 'objects': objects, 'targets': targets,
               'tags': {str(k): v for k, v in tags.items()}}
        self.det_pub.publish(String(data=json.dumps(out)))
        self.publish_annotated(bgr, objects, tags, color.header)
        if cal is not None:
            self.publish_markers(objects, targets)

    # ------------------------------------------------------------------ visualization
    def publish_annotated(self, bgr, objects, tags, header):
        import cv2

        img = bgr.copy()
        for o in objects:
            x, y, w, h = o['box']
            c = COLORS_BGR.get(o['label'], (255, 255, 255))
            cv2.rectangle(img, (x, y), (x + w, y + h), c, 2)
            pos = '' if o['xyz'] is None else ' ({:.3f}, {:.3f})'.format(*o['xyz'][:2])
            cv2.putText(img, f"{o['label']} {o['confidence']:.2f}{pos}", (x, max(y - 6, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, c, 1)
        for tag_id, t in tags.items():
            u, v = (int(a) for a in t['pixel'])
            name = self.targets.get(tag_id, f'tag {tag_id}')
            cv2.circle(img, (u, v), 5, (0, 255, 255), -1)
            cv2.putText(img, name, (u + 8, v - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
        rgb = np.ascontiguousarray(img[..., ::-1])
        h, w = rgb.shape[:2]
        self.img_pub.publish(Image(header=header, height=h, width=w, encoding='rgb8', is_bigendian=0, step=w * 3,
                                   data=rgb.tobytes()))

    def publish_markers(self, objects, targets):
        markers = MarkerArray()
        items = [(o['label'], o['xyz'], COLORS_BGR.get(o['label'], (255, 255, 255))) for o in objects if o['xyz']]
        items += [(t['name'], t['xyz'], (0, 220, 220)) for t in targets if t['xyz']]
        for i, (name, xyz, (b, g, r)) in enumerate(items):
            m = Marker(type=Marker.SPHERE, action=Marker.ADD, ns='object_detector', id=i)
            m.header.frame_id = 'world'
            m.pose.position = Point(x=xyz[0], y=xyz[1], z=xyz[2])
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.02
            m.color.r, m.color.g, m.color.b, m.color.a = r / 255, g / 255, b / 255, 0.9
            m.lifetime.sec = 1
            markers.markers.append(m)
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
