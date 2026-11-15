#!/usr/bin/env python3
#
# perception_tools.py
#
# Pure helpers (numpy only; OpenCV imported lazily for AprilTags) for object_detector and calibrate_camera, unit-tested in
# test/test_perception_tools.py. Everything is in METRES and the robot's `world` frame (= link1, x forward, y left, z up).
#
#   camera pixel (u, v) + depth  --pixel_to_camera-->  camera 3D point  --CameraCalibration (R, t)-->  world 3D point
#
# The camera-to-world transform is a rigid Kabsch fit between points seen by the camera (AprilTag centres + depth) and the
# same points touched by the gripper (forward kinematics). Ported from yolo_pipeline (src/calibration/kabsch.py,
# coordinate_mapper.py), with metres instead of mm, a robust depth (median over a patch instead of one pixel), and the
# calibration's residuals stored with it.
#

import dataclasses
import datetime
import pathlib

import numpy as np
import yaml

APRILTAG_DICT = 'DICT_APRILTAG_36h11'  # same tag family as yolo_pipeline (data/tags/)


# ---------------------------------------------------------------------------------------------------- calibration
def kabsch(camera_points, world_points):
    """Rotation R and translation t with world ≈ R @ camera + t (least squares, no scaling). Needs ≥ 3 non-collinear points."""
    P = np.asarray(camera_points, dtype=np.float64)
    Q = np.asarray(world_points, dtype=np.float64)
    if P.shape != Q.shape or P.ndim != 2 or P.shape[1] != 3:
        raise ValueError(f'need two (N, 3) arrays of equal shape, got {P.shape} and {Q.shape}')
    if len(P) < 3:
        raise ValueError('Kabsch needs at least 3 point pairs')
    pc, qc = P.mean(axis=0), Q.mean(axis=0)
    H = (P - pc).T @ (Q - qc)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))  # avoid a reflection
    R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    return R, qc - R @ pc


def residuals(camera_points, world_points, R, t):
    """Per-point distance (m) between the transformed camera points and the world points."""
    pred = (R @ np.asarray(camera_points, dtype=np.float64).T).T + t
    return np.linalg.norm(pred - np.asarray(world_points, dtype=np.float64), axis=1)


@dataclasses.dataclass
class CameraCalibration:
    R: np.ndarray                     # (3, 3) camera optical frame -> world
    t: np.ndarray                     # (3,) metres
    fx: float
    fy: float
    cx: float
    cy: float
    width: int = 640
    height: int = 480
    errors_m: list = dataclasses.field(default_factory=list)
    points: list = dataclasses.field(default_factory=list)  # [{'id', 'camera', 'world'}] used for the fit
    created: str = ''

    def camera_to_world(self, p_camera):
        return self.R @ np.asarray(p_camera, dtype=np.float64) + self.t

    def pixel_to_world(self, u, v, depth_m):
        return self.camera_to_world(pixel_to_camera(u, v, depth_m, self.fx, self.fy, self.cx, self.cy))

    def save(self, path):
        data = {
            'R': np.asarray(self.R).tolist(), 't': np.asarray(self.t).tolist(),
            'fx': float(self.fx), 'fy': float(self.fy), 'cx': float(self.cx), 'cy': float(self.cy),
            'width': int(self.width), 'height': int(self.height),
            'errors_m': [float(e) for e in self.errors_m],
            'mean_error_m': float(np.mean(self.errors_m)) if self.errors_m else None,
            'max_error_m': float(np.max(self.errors_m)) if self.errors_m else None,
            'points': self.points,
            'created': self.created or datetime.datetime.now().isoformat(timespec='seconds'),
        }
        path = pathlib.Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('# camera -> robot world frame (metres), written by calibrate_camera\n' + yaml.safe_dump(data, sort_keys=False))

    @classmethod
    def load(cls, path):
        d = yaml.safe_load(pathlib.Path(path).read_text())
        return cls(R=np.array(d['R'], dtype=np.float64), t=np.array(d['t'], dtype=np.float64),
                   fx=d['fx'], fy=d['fy'], cx=d['cx'], cy=d['cy'], width=d.get('width', 640), height=d.get('height', 480),
                   errors_m=d.get('errors_m', []), points=d.get('points', []), created=d.get('created', ''))


# ----------------------------------------------------------------------------------------------------- projection
def pixel_to_camera(u, v, depth_m, fx, fy, cx, cy):
    """Pinhole back-projection into the camera optical frame (x right, y down, z forward), metres."""
    return np.array([(u - cx) * depth_m / fx, (v - cy) * depth_m / fy, depth_m], dtype=np.float64)


def push_back_along_view(p_world, camera_world, distance):
    """Moves a visible surface point `distance` further from the camera, horizontally. The depth camera sees a cube's
    near face/top, not its centre; for a cube of edge a, distance = a / 2 gives roughly the centre's x, y."""
    p = np.asarray(p_world, dtype=np.float64).copy()
    d = p[:2] - np.asarray(camera_world, dtype=np.float64)[:2]
    n = np.linalg.norm(d)
    if n > 1e-9:
        p[:2] += d / n * distance
    return p


def robust_depth(depth_m, cx, cy, half=None, box=None, min_valid=0.15, max_valid=2.0, min_fraction=0.2):
    """Median depth (m) of the valid pixels in a patch: the inner half of `box` (x, y, w, h) if given, otherwise a square
    of +-half pixels around (cx, cy). None if fewer than min_fraction of the pixels are valid. A single pixel (as in
    yolo_pipeline) is often 0 (no return) or on an edge; the median of the centre patch is not."""
    h_img, w_img = depth_m.shape[:2]
    if box is not None:
        x, y, w, h = box
        x0, x1, y0, y1 = x + w // 4, x + 3 * w // 4 + 1, y + h // 4, y + 3 * h // 4 + 1
    else:
        half = 3 if half is None else half
        x0, x1, y0, y1 = cx - half, cx + half + 1, cy - half, cy + half + 1
    patch = depth_m[max(int(y0), 0):min(int(y1), h_img), max(int(x0), 0):min(int(x1), w_img)]
    if patch.size == 0:
        return None
    valid = patch[(patch > min_valid) & (patch < max_valid) & np.isfinite(patch)]
    if valid.size < max(1, min_fraction * patch.size):
        return None
    return float(np.median(valid))


def depth_image_to_metres(encoding, height, width, step, data):
    """sensor_msgs/Image depth (16UC1 in mm, as realsense2_camera publishes, or 32FC1 in m) -> float32 (H, W) metres."""
    if encoding in ('16UC1', 'mono16'):
        rows = np.frombuffer(memoryview(data), dtype=np.uint16).reshape(height, step // 2)
        return rows[:, :width].astype(np.float32) / 1000.0
    if encoding == '32FC1':
        rows = np.frombuffer(memoryview(data), dtype=np.float32).reshape(height, step // 4)
        return np.array(rows[:, :width], dtype=np.float32)
    raise ValueError(f'unsupported depth encoding {encoding!r} (16UC1 or 32FC1)')


# ------------------------------------------------------------------------------------------------------- AprilTags
def make_tag_detector():
    """OpenCV AprilTag 36h11 detector for both the old (4.6, apt) and new (>= 4.7, ArucoDetector) cv2.aruco APIs."""
    import cv2

    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, APRILTAG_DICT))
    if hasattr(cv2.aruco, 'ArucoDetector'):  # OpenCV >= 4.7
        detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
        return lambda gray: detector.detectMarkers(gray)
    # OpenCV 4.6 (Ubuntu 24.04 apt): parameters must come from DetectorParameters_create(); a directly constructed
    # DetectorParameters() is uninitialized there and detectMarkers segfaults.
    params = cv2.aruco.DetectorParameters_create()
    return lambda gray: cv2.aruco.detectMarkers(gray, dictionary, parameters=params)


def detect_tags(gray, detect):
    """{tag id: (centre u, centre v, side length px)} for every AprilTag found in a grayscale image."""
    corners, ids, _ = detect(gray)
    tags = {}
    if ids is not None:
        for c, tag_id in zip(corners, np.asarray(ids).flatten()):
            pts = np.asarray(c).reshape(4, 2)
            side = float(np.mean(np.linalg.norm(pts - np.roll(pts, 1, axis=0), axis=1)))
            tags[int(tag_id)] = (float(pts[:, 0].mean()), float(pts[:, 1].mean()), side)
    return tags
