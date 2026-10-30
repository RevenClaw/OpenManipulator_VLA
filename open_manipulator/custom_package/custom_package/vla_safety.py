#!/usr/bin/env python3
#
# vla_safety.py
#
# Pure safety and timing functions for vla_bridge (numpy only, no ROS), so they can be unit-tested
# without a robot: python3 -m pytest custom_package/test/test_vla_safety.py
#
# Observation contract with the policy server (vla_layer/omx_pi05, plan decisions D5–D8).
# vla_layer/tests/test_bridge_contract.py checks these values against omx_pi05.
#

import math

import numpy as np

OBS_FRONT = 'observation/front_image'
OBS_WRIST = 'observation/wrist_image'
OBS_STATE = 'observation/state'
PROMPT = 'prompt'
IMAGE_SIZE = 224
ACTION_DIM = 5  # [j1, j2, j3, j4, gripper_norm]

# Gripper convention: metres on gripper_left_joint <-> [0, 1] in the model, 0 = open, 1 = closed.
# Same values as joint_commander.GRIPPER_OPEN_M / GRIPPER_CLOSE_M.
GRIPPER_OPEN_M = 0.019
GRIPPER_CLOSE_M = -0.010


def gripper_metres_to_norm(q):
    return float(np.clip((GRIPPER_OPEN_M - q) / (GRIPPER_OPEN_M - GRIPPER_CLOSE_M), 0.0, 1.0))


def gripper_norm_to_metres(g):
    return float(GRIPPER_OPEN_M - np.clip(g, 0.0, 1.0) * (GRIPPER_OPEN_M - GRIPPER_CLOSE_M))


def validate_chunk(chunk):
    """Returns the chunk as a float (N, 5) array, or raises ValueError (wrong shape, NaN or inf)."""
    chunk = np.asarray(chunk, dtype=np.float64)
    if chunk.ndim != 2 or chunk.shape[1] != ACTION_DIM or chunk.shape[0] == 0:
        raise ValueError(f'expected an (N, {ACTION_DIM}) chunk, got shape {chunk.shape}')
    if not np.all(np.isfinite(chunk)):
        raise ValueError('chunk contains NaN or inf')
    return chunk


def step_times(t_obs, fps, n):
    """Wall time each chunk step is meant for: step k targets t_obs + (k + 1) / fps."""
    return t_obs + (np.arange(n) + 1) / fps


def first_future_step(t_obs, fps, n, now):
    """Index of the first step not yet in the past (n if the whole chunk is stale)."""
    return int(np.searchsorted(step_times(t_obs, fps, n), now, side='right'))


def clamp_joints(targets, lower, upper, margin):
    """Clamps (N, 4) joint targets to [lower + margin, upper - margin]."""
    return np.clip(targets, np.asarray(lower) + margin, np.asarray(upper) - margin)


def rate_limit(q_start, targets, max_vel, dts):
    """Caps every joint's speed at max_vel (rad/s), starting from q_start (the measured position).

    dts[k] is the time from the previous point (from now, for k = 0) to point k. Capping by time, not per step, matters:
    after stale steps are dropped, the first point can be due only milliseconds from now.
    """
    out = np.empty_like(targets)
    q = np.asarray(q_start, dtype=np.float64)
    for k, target in enumerate(targets):
        max_move = max_vel * max(float(dts[k]), 0.0)
        q = q + np.clip(target - q, -max_move, max_move)
        out[k] = q
    return out


def start_jump(q_now, first_target):
    """Largest joint distance between the measured position and the chunk's first step (rad)."""
    return float(np.max(np.abs(np.asarray(first_target) - np.asarray(q_now))))


def truncate_below_z(targets, fk, min_z):
    """Keeps the targets up to (not including) the first one whose fingertip height fk(q)[2] < min_z."""
    for k, q in enumerate(targets):
        if fk(q)[2] < min_z:
            return targets[:k]
    return targets


def request_threshold_steps(latency_s, fps):
    """Request the next chunk when this many (or fewer) steps remain, so it arrives before the plan runs out."""
    return int(math.ceil(latency_s * fps)) + 1


class GripperHysteresis:
    """Turns the model's continuous gripper value into open/close commands, without chatter around 0.5."""

    def __init__(self, close_above=0.6, open_below=0.4, closed=False):
        if not open_below < close_above:
            raise ValueError('open_below must be lower than close_above')
        self.close_above = close_above
        self.open_below = open_below
        self.closed = closed

    def update(self, g):
        """Returns True (close) or False (open) when the state changes, else None."""
        if not self.closed and g > self.close_above:
            self.closed = True
            return True
        if self.closed and g < self.open_below:
            self.closed = False
            return False
        return None


def ros_image_to_rgb(encoding, height, width, step, data):
    """sensor_msgs/Image fields (rgb8 or bgr8) -> uint8 (H, W, 3) RGB array, without cv_bridge."""
    if encoding not in ('rgb8', 'bgr8'):
        raise ValueError(f'unsupported image encoding {encoding!r} (use rgb8 or bgr8)')
    rows = np.frombuffer(bytes(data), dtype=np.uint8).reshape(height, step)
    image = rows[:, : width * 3].reshape(height, width, 3)
    return np.ascontiguousarray(image[..., ::-1] if encoding == 'bgr8' else image)
