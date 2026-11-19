#!/usr/bin/env python3
#
# demo_planning.py
#
# Motion planning for scripted pick-and-place demonstrations (scripted_demo), without ROS calls so it can be unit-tested
# (test/test_demo_planning.py). Uses ee_commander's closed-form IK/FK. All positions in metres in the `world` frame.
#
# One pick-and-place = waypoints in (x, y, z, pitch), each solved with IK:
#   READY -> pregrasp (above the cube) -> grasp -> [close] -> lift -> [via] -> above target -> release -> [open] -> retreat
#   -> READY   (READY: gripper pointing 70° down above the workspace centre, episode_tools.READY; HOME is only the session's
#   storage pose)
# Short vertical moves (descend, lift, lower, retreat) are Cartesian (straight line, IK at every sample); transfers are
# joint-space. Every move is minimum-jerk, timed so no joint exceeds its cap in max_joint_vel (per joint: the shoulder,
# joint2, overloads and stays slow; elbow and wrist may move faster). Deterministic for a given variation, so
# similar situations give similar motions (unlike a sampling planner such as OMPL), plus small random variations per episode
# (vla_layer/docs/DATASET_PLAN.md §4.4) so the policy sees slightly-off states and corrections.
#
# Gripper tilt: a 4-DOF arm reaches the table pointing straight down only up to ~0.25 m from the base (~0.32 m at 60°), so
# the steepest pitch that works for all of a pick's (or a place's) waypoints is chosen, from max_pitch down to min_pitch.
#

import dataclasses
import math

import numpy as np

from custom_package.episode_tools import HOME  # noqa: F401 (re-exported)
from custom_package.episode_tools import READY
from custom_package.ee_commander import forward_kinematics
from custom_package.ee_commander import inverse_kinematics


@dataclasses.dataclass
class PlanConfig:
    table_z: float = 0.0              # m, table surface in the world frame (the base stands on it)
    cube_size: float = 0.025          # m, edge of the cubes
    bin_height: float = 0.04          # m, rim height of the bins above the table
    release_clearance: float = 0.03   # m, fingertips above the rim when opening
    # rad/s per joint (j1..j4). joint2 (shoulder) overloads: kept at 0.3. vla_bridge must allow at least these
    # (max_joint_vel_per_joint), or it clips the motions the policy learned.
    max_joint_vel: tuple = (0.5, 0.3, 0.6, 0.8)
    max_pitch_deg: float = 90.0       # steepest gripper tilt tried first (90 = pointing down)
    min_pitch_deg: float = 30.0
    pitch_step_deg: float = 5.0
    min_segment_s: float = 0.6
    rate_hz: float = 30.0             # samples per second of the planned trajectory (and of /teleop/action)
    min_fingertip_z: float = 0.005    # m above the table, never lower


@dataclasses.dataclass
class Variation:
    """Per-episode random variation. Defaults = no variation."""
    speed_scale: float = 1.0          # x max_joint_vel (<= 1 keeps the cap)
    approach_height: float = 0.06     # m above grasp/release for pregrasp, above-target and retreat
    lift_height: float = 0.07         # m above grasp after closing
    grasp_dz: float = 0.0             # m, grasp height offset
    pitch_offset_deg: float = 0.0     # tried first: max_pitch - this
    via_offset: tuple = (0.0, 0.0, 0.0)  # m, added to the midpoint between lift and above-target (0 = no via point)
    dwell_close: float = 0.6          # s waiting after "close"
    dwell_open: float = 0.5           # s waiting after "open"
    correction: tuple = (0.0, 0.0)    # m, pregrasp first goes this far off in x/y, then corrects (0 = none)
    start_offset: tuple = (0.0, 0.0, 0.0, 0.0)  # rad added to READY as the episode's start pose

    def asdict(self):
        return {k: (list(v) if isinstance(v, tuple) else v) for k, v in dataclasses.asdict(self).items()}


def sample_variation(rng, enabled=True, correction_prob=0.1):
    if not enabled:
        return Variation()
    correction = (0.0, 0.0)
    if rng.random() < correction_prob:
        angle, dist = rng.uniform(0, 2 * math.pi), rng.uniform(0.015, 0.025)
        correction = (dist * math.cos(angle), dist * math.sin(angle))
    via = (0.0, 0.0, 0.0)
    if rng.random() < 0.7:
        via = (rng.uniform(-0.03, 0.03), rng.uniform(-0.03, 0.03), rng.uniform(0.0, 0.03))
    return Variation(
        speed_scale=rng.uniform(0.7, 1.0), approach_height=rng.uniform(0.04, 0.06), lift_height=rng.uniform(0.05, 0.07),
        grasp_dz=rng.uniform(-0.004, 0.004), pitch_offset_deg=rng.uniform(0.0, 10.0), via_offset=via,
        dwell_close=rng.uniform(0.4, 0.9), dwell_open=rng.uniform(0.3, 0.7), correction=correction,
        start_offset=tuple(rng.uniform(-0.03, 0.03, 4)))


# ---------------------------------------------------------------------------------------------------------- IK
def solve(pose, near=None):
    """IK for (x, y, z, pitch); the solution closest to `near` (joints) if several. None if unreachable."""
    solutions, _ = inverse_kinematics(*pose)
    if not solutions:
        return None
    if near is None:
        return np.asarray(solutions[0], dtype=np.float64)
    return np.asarray(min(solutions, key=lambda q: float(np.max(np.abs(np.asarray(q) - near)))), dtype=np.float64)


def choose_pitch(points_xyz, cfg, offset_deg=0.0):
    """Steepest pitch (rad) at which every point is reachable, or None."""
    start = cfg.max_pitch_deg - offset_deg
    for deg in np.arange(start, cfg.min_pitch_deg - 1e-9, -cfg.pitch_step_deg):
        pitch = math.radians(deg)
        if all(solve((*p, pitch)) is not None for p in points_xyz):
            return pitch
    return None


# ---------------------------------------------------------------------------------------------------------- timing
def min_jerk(s):
    return 10 * s**3 - 15 * s**4 + 6 * s**5


def joint_segment(q0, q1, max_vel, cfg):
    """(times, joints) of a minimum-jerk joint-space move; peak speed 1.875·Δ/T <= max_vel (scalar or per joint)."""
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    duration = max(cfg.min_segment_s, 1.875 * float(np.max(np.abs(q1 - q0) / np.asarray(max_vel, float))))
    n = max(2, int(math.ceil(duration * cfg.rate_hz)) + 1)
    s = min_jerk(np.linspace(0.0, 1.0, n))[:, None]
    return np.linspace(0.0, duration, n), q0 + (q1 - q0) * s


def cartesian_segment(p0, p1, pitch, q_start, max_vel, cfg):
    """(times, joints) of a straight minimum-jerk line from p0 to p1 (xyz) at a fixed pitch, IK at every sample. The
    duration is stretched until no joint exceeds max_vel (scalar or per joint). None if a sample is unreachable."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    duration = max(cfg.min_segment_s, 1.875 * float(np.linalg.norm(p1 - p0)) / 0.05)  # start at 5 cm/s peak
    for _ in range(8):
        n = max(2, int(math.ceil(duration * cfg.rate_hz)) + 1)
        s = min_jerk(np.linspace(0.0, 1.0, n))
        q, joints = q_start, []
        for si in s:
            q = solve((*(p0 + (p1 - p0) * si), pitch), near=q)
            if q is None:
                return None
            joints.append(q)
        joints = np.array(joints)
        times = np.linspace(0.0, duration, n)
        ratio = float(np.max(np.abs(np.diff(joints, axis=0)) / np.diff(times)[:, None] / np.asarray(max_vel, float)))
        if ratio <= 1.001:
            return times, joints
        duration *= ratio * 1.05
    return times, joints


# ----------------------------------------------------------------------------------------------------------- plan
@dataclasses.dataclass
class Segment:
    """kind 'move': joints (N, 4) at times (N,) from the segment start, gripper target constant.
    kind 'gripper': the gripper target changes to `gripper`, then the arm holds still for `duration`."""
    kind: str
    name: str
    gripper: float                    # commanded gripper, 0 open .. 1 closed
    times: np.ndarray = None
    joints: np.ndarray = None
    duration: float = 0.0

    @property
    def length(self):
        return float(self.times[-1]) if self.kind == 'move' else self.duration


@dataclasses.dataclass
class Plan:
    segments: list
    pick_pitch: float
    place_pitch: float
    waypoints: dict                   # name -> (x, y, z, pitch), for metadata
    start: np.ndarray                 # start joints (READY + start_offset)

    @property
    def duration(self):
        return sum(s.length for s in self.segments)


class PlanningError(ValueError):
    pass


def plan_pick_place(cube_xy, target_xy, cfg=PlanConfig(), var=Variation(), start=None):
    """Plan one pick-and-place. cube_xy: cube centre (x, y); target_xy: bin centre (x, y). Raises PlanningError."""
    start = np.asarray(READY, float) + np.asarray(var.start_offset, float) if start is None else np.asarray(start, float)
    max_vel = np.asarray(cfg.max_joint_vel, float) * min(var.speed_scale, 1.0)
    z_grasp = cfg.table_z + cfg.cube_size / 2 + var.grasp_dz
    z_release = cfg.table_z + cfg.bin_height + cfg.release_clearance
    cx, cy = cube_xy
    bx, by = target_xy
    pre = (cx, cy, z_grasp + var.approach_height)
    grasp = (cx, cy, z_grasp)
    lift = (cx, cy, z_grasp + var.lift_height)
    above = (bx, by, z_release + var.approach_height)
    release = (bx, by, z_release)
    if z_grasp - cfg.cube_size / 2 < cfg.table_z + cfg.min_fingertip_z - 0.02:
        raise PlanningError('grasp height below the table')

    # Optional variations are planned only where reachable: dropped (not failed) otherwise.
    off = (pre[0] + var.correction[0], pre[1] + var.correction[1], pre[2]) if any(var.correction) else None
    pick_points = [pre, grasp, lift] + ([off] if off else [])
    pick_pitch = choose_pitch(pick_points, cfg, var.pitch_offset_deg) or choose_pitch(pick_points, cfg)
    if pick_pitch is None and off:
        off = None
        pick_pitch = choose_pitch([pre, grasp, lift], cfg, var.pitch_offset_deg) or choose_pitch([pre, grasp, lift], cfg)
    if pick_pitch is None:
        raise PlanningError(f'cube at ({cx:.3f}, {cy:.3f}) is not reachable at pitch {cfg.min_pitch_deg:g}..{cfg.max_pitch_deg:g}°')
    place_pitch = choose_pitch([above, release], cfg, var.pitch_offset_deg) or choose_pitch([above, release], cfg)
    if place_pitch is None:
        raise PlanningError(f'target at ({bx:.3f}, {by:.3f}) is not reachable')

    segments, q = [], start

    def move_joint(name, pose, grip):
        nonlocal q
        q1 = solve(pose, near=q)
        if q1 is None:
            raise PlanningError(f'{name}: no IK solution')
        t, j = joint_segment(q, q1, max_vel, cfg)
        segments.append(Segment('move', name, grip, t, j))
        q = j[-1]

    def move_line(name, p0, p1, pitch, grip):
        nonlocal q
        seg = cartesian_segment(p0, p1, pitch, q, max_vel, cfg)
        if seg is None:
            raise PlanningError(f'{name}: straight line not reachable')
        segments.append(Segment('move', name, grip, *seg))
        q = seg[1][-1]

    if off:  # deliberately approach a little off, then correct (a scripted "recovery")
        move_joint('approach_off', (*off, pick_pitch), 0.0)
        move_line('correct', off, pre, pick_pitch, 0.0)
    else:
        move_joint('approach', (*pre, pick_pitch), 0.0)
    move_line('descend', pre, grasp, pick_pitch, 0.0)
    segments.append(Segment('gripper', 'close', 1.0, duration=var.dwell_close))
    move_line('lift', grasp, lift, pick_pitch, 1.0)
    if any(var.via_offset):
        via = tuple((np.asarray(lift) + np.asarray(above)) / 2 + np.asarray(var.via_offset))
        via_pitch = choose_pitch([via], cfg, cfg.max_pitch_deg - math.degrees((pick_pitch + place_pitch) / 2))
        if via_pitch is not None:
            move_joint('via', (*via, via_pitch), 1.0)
    move_joint('transfer', (*above, place_pitch), 1.0)
    move_line('lower', above, release, place_pitch, 1.0)
    segments.append(Segment('gripper', 'open', 0.0, duration=var.dwell_open))
    move_line('retreat', release, above, place_pitch, 0.0)
    t, j = joint_segment(q, np.asarray(READY, float), max_vel, cfg)
    segments.append(Segment('move', 'ready', 0.0, t, j))

    waypoints = {'pregrasp': (*pre, pick_pitch), 'grasp': (*grasp, pick_pitch), 'lift': (*lift, pick_pitch),
                 'above_target': (*above, place_pitch), 'release': (*release, place_pitch)}
    return Plan(segments, pick_pitch, place_pitch, waypoints, start)


def plan_groups(plan):
    """Consecutive move segments merged into one trajectory each (one controller goal, no stop in between), split at the
    gripper events: [{'kind': 'move', 'names', 'times', 'joints', 'gripper'} | {'kind': 'gripper', 'name', 'gripper',
    'duration'}]."""
    groups, cur = [], None
    for seg in plan.segments:
        if seg.kind == 'move':
            if cur is None:
                cur = {'kind': 'move', 'names': [seg.name], 'times': seg.times.copy(), 'joints': seg.joints.copy(),
                       'gripper': seg.gripper}
            else:
                cur['names'].append(seg.name)
                cur['times'] = np.concatenate([cur['times'], cur['times'][-1] + seg.times[1:]])
                cur['joints'] = np.vstack([cur['joints'], seg.joints[1:]])
        else:
            if cur is not None:
                groups.append(cur)
                cur = None
            groups.append({'kind': 'gripper', 'name': seg.name, 'gripper': seg.gripper, 'duration': seg.duration})
    if cur is not None:
        groups.append(cur)
    return groups


def fingertip_heights(plan):
    """Lowest fingertip z of every move segment (for safety checks)."""
    return {s.name: min(forward_kinematics(q)[2] for q in s.joints) for s in plan.segments if s.kind == 'move'}


# --------------------------------------------------------------------------------------------- tasks and checks
def place_offset(offsets, index):
    """Drop-point offset (dx, dy) in m for the index-th placement, cycling through the flat list [dx0, dy0, dx1, dy1, ...].
    Lets several objects be set down around one marker without landing on each other. [0, 0] = always the marker centre."""
    pairs = [(float(offsets[i]), float(offsets[i + 1])) for i in range(0, len(offsets) - 1, 2)] or [(0.0, 0.0)]
    return pairs[index % len(pairs)]


def make_prompt(template, color, target):
    return template.format(color=color, target=target)


def verify_place(cube_label, cube_xy_before, target_xy, detections_after, target_radius=0.06, moved_min=0.04):
    """Success check from the scene after the episode. detections_after: [{'label', 'xyz'}] of cubes on the table.
    Success if a cube of that colour is within target_radius of the target, or no cube of that colour is left near its
    start position (a cube inside a bin is often hidden by the rim). Returns (success, reason)."""
    same = [d for d in detections_after if d['label'] == cube_label]
    for d in same:
        if math.dist(d['xyz'][:2], target_xy) <= target_radius:
            return True, 'cube detected at the target'
    for d in same:
        if math.dist(d['xyz'][:2], cube_xy_before) < moved_min:
            return False, 'cube still at its start position'
    if same:
        far = min(same, key=lambda d: math.dist(d['xyz'][:2], target_xy))
        return False, f'cube detected {math.dist(far["xyz"][:2], target_xy):.2f} m from the target'
    return True, 'cube no longer visible on the table (inside the bin)'


def sample_trajectory(plan, t):
    """Commanded (joints, gripper) at time t (s) since the start of the plan; holds the last target after the end."""
    elapsed, q = 0.0, plan.start
    for seg in plan.segments:
        if seg.kind == 'gripper':
            if t < elapsed + seg.duration:
                return q, seg.gripper
        else:
            if t < elapsed + seg.length:
                tau = t - elapsed
                i = int(np.searchsorted(seg.times, tau, side='right') - 1)
                i = min(max(i, 0), len(seg.times) - 2)
                a = (tau - seg.times[i]) / (seg.times[i + 1] - seg.times[i])
                return seg.joints[i] + (seg.joints[i + 1] - seg.joints[i]) * a, seg.gripper
            q = seg.joints[-1]
        elapsed += seg.length
    return q, plan.segments[-1].gripper
