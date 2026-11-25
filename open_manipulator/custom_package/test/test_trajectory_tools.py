import numpy as np
import pytest

from custom_package import trajectory_tools as T


def straight(n=50, offset=0.0):
    s = np.linspace(0, 1, n)
    return np.stack([0.1 + 0.1 * s, offset + 0 * s, 0.1 - 0.15 * s], axis=1)


def test_motion_range_trims_idle_ends():
    q = np.zeros((20, 4))
    q[5:15, 0] = np.linspace(0, 1, 10)
    q[15:, 0] = 1.0
    assert T.motion_range(q) == (5, 14)   # one sample of margin each side
    assert T.motion_range(np.zeros((5, 4))) == (0, 4)


def test_phases_grasp_and_release():
    g = np.array([0, 0, 0.2, 0.7, 0.75, 0.7, 0.1, 0, 0])
    assert T.phases(g) == (3, 6)
    assert T.phases(np.zeros(5)) == (None, None)
    assert T.phases(np.array([0, 0.9, 0.9])) == (1, None)


def test_path_deviation_is_speed_independent():
    ref = straight(50)
    assert T.path_deviation(straight(200), ref).max() < 0.002   # same path, other speed
    assert T.path_deviation(straight(50, offset=0.03), ref).mean() == pytest.approx(0.03, abs=1e-6)


def test_dtw_distance():
    ref = straight(40)
    assert T.dtw_distance(ref, ref) == pytest.approx(0.0)
    assert T.dtw_distance(straight(40, offset=0.02), ref) == pytest.approx(0.02, abs=1e-6)


def test_compare_reports_grasp_error_and_order():
    t = np.linspace(0, 5, 100)
    g = np.where((t > 2) & (t < 4), 0.7, 0.0)
    ref = {'t': t, 'path': straight(100), 'gripper': g}
    run = {'t': t, 'path': straight(100, offset=0.01), 'gripper': g}
    m = T.compare(run, ref)
    assert m['grasp_err_m'] == pytest.approx(0.01, abs=1e-6) and m['release_err_m'] == pytest.approx(0.01, abs=1e-6)
    assert m['mean_dev_m'] == pytest.approx(0.01, abs=1e-4)
    assert sorted(['ckpt25000', 'yolo', 'ckpt5000', 'ckpt15000'], key=T.label_order) == ['yolo', 'ckpt5000', 'ckpt15000', 'ckpt25000']
    assert sorted(['ckpt15000_2', 'ckpt5000_2', 'ckpt25000'], key=T.label_order) == ['ckpt5000_2', 'ckpt15000_2', 'ckpt25000']
