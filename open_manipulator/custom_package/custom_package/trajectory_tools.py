"""Pure helpers for trajectory_compare: trimming, task phases and divergence from a reference run. No ROS (unit-tested).

A run is an array of samples (N, 7) = [t, j1, j2, j3, j4, gripper_norm, ...] plus the fingertip path (N, 3) in metres.
Gripper: 0 = open .. 1 = closed (vla_safety.gripper_metres_to_norm).
"""
import math
import re

import numpy as np

CLOSE_ABOVE = 0.3   # gripper value above which it counts as closed/closing (a held 3.5 cm cube measures ~0.6-0.8)
OPEN_BELOW = 0.3


def motion_range(joints, tol=0.01):
    """(first, last) sample index of the movement: first sample that differs from the start by > tol rad, last sample that
    differs from the end by > tol. Returns (0, len-1) if the arm never moved."""
    joints = np.asarray(joints, dtype=float)
    moved = np.flatnonzero((np.abs(joints - joints[0]) > tol).any(axis=1))
    if len(moved) == 0:
        return 0, len(joints) - 1
    before_end = np.flatnonzero((np.abs(joints - joints[-1]) > tol).any(axis=1))
    return max(int(moved[0]) - 1, 0), min(int(before_end[-1]) + 1, len(joints) - 1)


def phases(gripper):
    """(grasp, release) sample indices: first sample where the gripper closes (> CLOSE_ABOVE), first sample after that where
    it is open again (< OPEN_BELOW). None for a phase that never happens."""
    g = np.asarray(gripper, dtype=float)
    closed = np.flatnonzero(g > CLOSE_ABOVE)
    if len(closed) == 0:
        return None, None
    grasp = int(closed[0])
    reopened = np.flatnonzero(g[grasp:] < OPEN_BELOW)
    return grasp, (grasp + int(reopened[0])) if len(reopened) else None


def task_segment(t, joints, gripper, after_release=0.5):
    """Index slice of the task: from the start of the movement to `after_release` s after the release (or the end of the
    movement if the gripper never released). Leaves out the idle start and the return to the start pose."""
    # The gripper counts as movement too: a close while the arm stands still belongs to the task.
    first, last = motion_range(np.column_stack([np.asarray(joints, float), np.asarray(gripper, float)]))
    _, release = phases(gripper)
    if release is not None:
        end_t = t[release] + after_release
        last = min(last, int(np.searchsorted(t, end_t)))
    return slice(first, max(last, first) + 1)


def resample(t, values, rate=20.0):
    """Linear resampling of (N, D) values onto a uniform grid at `rate` Hz from t[0] to t[-1]."""
    t = np.asarray(t, dtype=float)
    values = np.asarray(values, dtype=float)
    if len(t) < 2:
        return t.copy(), values.copy()
    grid = np.arange(t[0], t[-1] + 1e-9, 1.0 / rate)
    return grid, np.stack([np.interp(grid, t, values[:, k]) for k in range(values.shape[1])], axis=1)


def path_deviation(path, reference):
    """For every point of `path`, the distance (m) to the closest point of `reference` (both (N, 3)). Speed-independent:
    measures how far the run strays from the reference PATH, not whether it is early or late."""
    path, reference = np.asarray(path, float), np.asarray(reference, float)
    d = np.linalg.norm(path[:, None, :] - reference[None, :, :], axis=2)
    return d.min(axis=1)


def dtw_distance(a, b):
    """Mean distance (m) along the dynamic-time-warping alignment of two paths (N, 3) and (M, 3): compares them point by
    point in the same order, allowing different speeds."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    n, m = len(a), len(b)
    cost = np.linalg.norm(a[:, None, :] - b[None, :, :], axis=2)
    acc = np.full((n + 1, m + 1), np.inf)
    steps = np.zeros((n + 1, m + 1), dtype=int)
    acc[0, 0] = 0.0
    for i in range(1, n + 1):
        for j in range(1, m + 1):
            k = int(np.argmin((acc[i - 1, j - 1], acc[i - 1, j], acc[i, j - 1])))
            pi, pj = ((i - 1, j - 1), (i - 1, j), (i, j - 1))[k]
            acc[i, j] = cost[i - 1, j - 1] + acc[pi, pj]
            steps[i, j] = steps[pi, pj] + 1
    return float(acc[n, m] / steps[n, m])


def rms_jerk(t, path, rate=20.0):
    """RMS jerk (m/s^3) of a fingertip path, after resampling to `rate` Hz: lower = smoother."""
    grid, p = resample(t, path, rate)
    if len(grid) < 4:
        return float('nan')
    jerk = np.diff(p, n=3, axis=0) * rate ** 3
    return float(np.sqrt(np.mean(np.sum(jerk ** 2, axis=1))))


def label_order(label):
    """Sort key: the reference ('yolo', 'scripted', ...) first, then checkpoints by their number (ckpt5000 < ckpt15000)."""
    numbers = re.findall(r'\d+', label)
    return (1, int(numbers[0]), label) if numbers else (0, 0, label)   # ckpt5000_2: 5000 (the repeat suffix only breaks ties)


def compare(run, ref):
    """Divergence metrics of `run` against `ref`. Each is a dict with 't', 'path' (N, 3) and 'gripper' (N,), already cut
    to the task segment. Distances in metres, times in seconds."""
    _, rp = resample(ref['t'], ref['path'])
    _, pp = resample(run['t'], run['path'])
    dev = path_deviation(pp, rp)
    out = {
        'mean_dev_m': float(dev.mean()),
        'max_dev_m': float(dev.max()),
        'dtw_m': dtw_distance(pp, rp),
        'duration_s': float(run['t'][-1] - run['t'][0]),
        'rms_jerk': rms_jerk(run['t'], run['path']),
        'grasp_err_m': math.nan,
        'release_err_m': math.nan,
    }
    g_run, r_run = phases(run['gripper'])
    g_ref, r_ref = phases(ref['gripper'])
    if g_run is not None and g_ref is not None:
        out['grasp_err_m'] = float(np.linalg.norm(run['path'][g_run] - ref['path'][g_ref]))
    if r_run is not None and r_ref is not None:
        out['release_err_m'] = float(np.linalg.norm(run['path'][r_run] - ref['path'][r_ref]))
    return out
