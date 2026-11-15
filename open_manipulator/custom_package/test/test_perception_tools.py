"""Unit tests for perception_tools (no ROS needed): python3 -m pytest test/test_perception_tools.py"""

import importlib.util
import math
import pathlib

import numpy as np
import pytest

_spec = importlib.util.spec_from_file_location(
    'perception_tools', pathlib.Path(__file__).resolve().parent.parent / 'custom_package' / 'perception_tools.py'
)
P = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(P)


def _rotation(rx, ry, rz):
    cx, sx, cy, sy, cz, sz = math.cos(rx), math.sin(rx), math.cos(ry), math.sin(ry), math.cos(rz), math.sin(rz)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def test_kabsch_recovers_a_known_transform():
    rng = np.random.default_rng(0)
    R_true, t_true = _rotation(2.4, 0.1, -0.3), np.array([0.35, -0.02, 0.62])  # camera looking down at the table
    world = np.column_stack([rng.uniform(0.1, 0.3, 6), rng.uniform(-0.15, 0.15, 6), np.zeros(6)])  # coplanar tags
    camera = (R_true.T @ (world - t_true).T).T
    R, t = P.kabsch(camera, world)
    np.testing.assert_allclose(R, R_true, atol=1e-9)
    np.testing.assert_allclose(t, t_true, atol=1e-9)
    assert np.linalg.det(R) > 0
    assert P.residuals(camera, world, R, t).max() < 1e-9


def test_kabsch_with_noise_reports_residuals():
    rng = np.random.default_rng(1)
    R_true, t_true = _rotation(2.5, 0.0, 0.2), np.array([0.3, 0.0, 0.5])
    world = np.column_stack([rng.uniform(0.1, 0.3, 8), rng.uniform(-0.15, 0.15, 8), rng.uniform(0.0, 0.05, 8)])
    camera = (R_true.T @ (world - t_true).T).T + rng.normal(0, 0.002, (8, 3))
    R, t = P.kabsch(camera, world)
    err = P.residuals(camera, world, R, t)
    assert 0.0005 < err.mean() < 0.006


def test_kabsch_rejects_bad_input():
    with pytest.raises(ValueError):
        P.kabsch(np.zeros((2, 3)), np.zeros((2, 3)))
    with pytest.raises(ValueError):
        P.kabsch(np.zeros((4, 3)), np.zeros((3, 3)))


def test_pixel_to_camera_and_world(tmp_path):
    p = P.pixel_to_camera(420.0, 300.0, 0.5, 600.0, 600.0, 320.0, 240.0)
    np.testing.assert_allclose(p, [100 * 0.5 / 600, 60 * 0.5 / 600, 0.5])
    cal = P.CameraCalibration(R=np.eye(3), t=np.array([0.1, 0.0, 0.0]), fx=600, fy=600, cx=320, cy=240, errors_m=[0.003, 0.005])
    np.testing.assert_allclose(cal.pixel_to_world(320, 240, 0.4), [0.1, 0.0, 0.4])
    path = tmp_path / 'cal' / 'camera_calibration.yaml'
    cal.save(path)
    loaded = P.CameraCalibration.load(path)
    np.testing.assert_allclose(loaded.R, cal.R)
    np.testing.assert_allclose(loaded.t, cal.t)
    assert loaded.fx == 600 and loaded.errors_m == [0.003, 0.005]
    assert 'max_error_m: 0.005' in path.read_text()


def test_robust_depth_ignores_holes_and_edges():
    depth = np.full((480, 640), 0.6, dtype=np.float32)
    depth[200:260, 300:360] = 0.55            # the cube's top is 5 cm closer
    depth[225:235, 325:335] = 0.0             # holes (no depth return)
    assert P.robust_depth(depth, 330, 230, box=(300, 200, 60, 60)) == pytest.approx(0.55)
    assert P.robust_depth(depth, 330, 230, half=3) is None          # patch is all holes
    depth[:] = 0.0
    assert P.robust_depth(depth, 330, 230, half=5) is None


def test_depth_image_decoding():
    mm = np.array([[500, 0], [1000, 65535]], dtype=np.uint16)
    out = P.depth_image_to_metres('16UC1', 2, 2, 4, mm.tobytes())
    np.testing.assert_allclose(out, [[0.5, 0.0], [1.0, 65.535]])
    f = np.array([[0.5, 0.25]], dtype=np.float32)
    np.testing.assert_allclose(P.depth_image_to_metres('32FC1', 1, 2, 8, f.tobytes()), [[0.5, 0.25]])
    with pytest.raises(ValueError):
        P.depth_image_to_metres('rgb8', 1, 1, 3, b'\0\0\0')


def test_apriltags_are_found():
    cv2 = pytest.importorskip('cv2')
    tag_png = pathlib.Path('/yolo_pipeline/data/tags/tag_36h11_20.png')
    if tag_png.exists():
        tag = cv2.imread(str(tag_png), cv2.IMREAD_GRAYSCALE)
    else:  # draw one
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        draw = cv2.aruco.generateImageMarker if hasattr(cv2.aruco, 'generateImageMarker') else cv2.aruco.drawMarker
        tag = draw(dictionary, 20, 200)
    tag = cv2.resize(tag, (120, 120), interpolation=cv2.INTER_NEAREST)
    img = np.full((480, 640), 255, dtype=np.uint8)
    img[100:220, 300:420] = tag
    tags = P.detect_tags(img, P.make_tag_detector())
    assert 20 in tags
    u, v, side = tags[20]
    assert abs(u - 360) < 3 and abs(v - 160) < 3 and side > 40
