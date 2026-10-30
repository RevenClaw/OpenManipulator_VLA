"""Unit tests for vla_safety (no ROS needed): python3 -m pytest test/test_vla_safety.py"""

import importlib.util
import math
import pathlib

import numpy as np
import pytest

# Load the module by path so these tests run without building the ROS package.
_spec = importlib.util.spec_from_file_location(
    'vla_safety', pathlib.Path(__file__).resolve().parent.parent / 'custom_package' / 'vla_safety.py'
)
S = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(S)

LOWER = [-math.pi, -1.82, -1.5, -1.7]  # joint_commander.JOINT_LIMITS
UPPER = [math.pi, 1.5, 1.55, 1.97]
HOME = np.array([0.6090, -1.3622, 1.3898, -0.0154])


def test_gripper_convention():
    assert S.gripper_metres_to_norm(0.019) == 0.0  # open
    assert S.gripper_metres_to_norm(-0.010) == 1.0  # closed
    assert S.gripper_metres_to_norm(0.020) == 0.0 and S.gripper_metres_to_norm(-0.011) == 1.0  # clipped
    assert S.gripper_norm_to_metres(1.7) == pytest.approx(-0.010)
    for q in np.linspace(-0.010, 0.019, 11):
        assert S.gripper_norm_to_metres(S.gripper_metres_to_norm(q)) == pytest.approx(q)


def test_validate_chunk_rejects_bad_output():
    assert S.validate_chunk(np.zeros((10, 5))).shape == (10, 5)
    bad = np.zeros((10, 5))
    bad[3, 1] = np.nan
    for chunk in (bad, np.zeros((10, 4)), np.zeros((0, 5)), np.zeros(5), np.full((10, 5), np.inf)):
        with pytest.raises(ValueError):
            S.validate_chunk(chunk)


def test_stale_steps_are_dropped():
    fps = 15.0
    # Chunk computed for t_obs = 0, arriving 150 ms later: steps at 66.7 and 133.3 ms are past.
    assert S.first_future_step(0.0, fps, 10, now=0.150) == 2
    assert S.first_future_step(0.0, fps, 10, now=0.0) == 0
    assert S.first_future_step(0.0, fps, 10, now=5.0) == 10  # whole chunk stale


def test_clamp_keeps_margin_inside_limits():
    targets = np.array([[4.0, -3.0, 2.0, -2.0], [0.0, 0.0, 0.0, 0.0]])
    out = S.clamp_joints(targets, LOWER, UPPER, 0.05)
    np.testing.assert_allclose(out[0], [math.pi - 0.05, -1.77, 1.50, -1.65])
    np.testing.assert_allclose(out[1], 0.0)


def test_rate_limit_caps_speed_from_measured_position():
    max_vel, dts = 0.3, np.full(10, 1 / 15)
    targets = np.tile(HOME + 1.0, (10, 1))  # a 1 rad jump requested
    out = S.rate_limit(HOME, targets, max_vel, dts)
    speeds = np.abs(np.diff(np.vstack([HOME, out]), axis=0)) / dts[:, None]
    assert np.all(speeds <= max_vel + 1e-9)
    np.testing.assert_allclose(out[-1], HOME + 10 * max_vel / 15)
    small = HOME + np.array([0.005, 0, 0, 0])
    np.testing.assert_allclose(S.rate_limit(HOME, small[None], max_vel, [1 / 15])[0], small)  # small moves pass


def test_rate_limit_respects_a_first_point_due_almost_now():
    # After dropping stale steps the first point can be due in 5 ms: it may only move 0.3 * 0.005 rad.
    out = S.rate_limit(HOME, np.tile(HOME + 1.0, (3, 1)), 0.3, [0.005, 1 / 15, 1 / 15])
    np.testing.assert_allclose(out[0] - HOME, 0.3 * 0.005)


def test_start_jump():
    assert S.start_jump(HOME, HOME + [0, 0.2, 0, 0]) == pytest.approx(0.2)


def test_truncate_below_table():
    def fk(q):
        return (0.0, 0.0, q[0])  # fake FK: height = first joint value

    targets = np.array([[0.10, 0, 0, 0], [0.05, 0, 0, 0], [0.01, 0, 0, 0], [0.08, 0, 0, 0]])
    assert len(S.truncate_below_z(targets, fk, 0.02)) == 2
    assert len(S.truncate_below_z(targets[:2], fk, 0.02)) == 2


def test_request_threshold():
    assert S.request_threshold_steps(0.150, 15.0) == 4
    assert S.request_threshold_steps(0.0, 15.0) == 1


def test_gripper_hysteresis_no_chatter():
    h = S.GripperHysteresis(close_above=0.6, open_below=0.4, closed=False)
    events = [h.update(g) for g in (0.5, 0.55, 0.61, 0.59, 0.5, 0.45, 0.39, 0.5, 0.41)]
    assert events == [None, None, True, None, None, None, False, None, None]
    with pytest.raises(ValueError):
        S.GripperHysteresis(close_above=0.4, open_below=0.6)


def test_ros_image_conversion():
    h, w = 2, 3
    rgb = np.arange(h * w * 3, dtype=np.uint8).reshape(h, w, 3)
    padded = np.concatenate([rgb.reshape(h, w * 3), np.zeros((h, 4), np.uint8)], axis=1)  # row padding
    np.testing.assert_array_equal(S.ros_image_to_rgb('rgb8', h, w, w * 3 + 4, padded.tobytes()), rgb)
    bgr = rgb[..., ::-1].copy()
    np.testing.assert_array_equal(S.ros_image_to_rgb('bgr8', h, w, w * 3, bgr.tobytes()), rgb)
    with pytest.raises(ValueError):
        S.ros_image_to_rgb('mono8', h, w, w, bytes(h * w))
