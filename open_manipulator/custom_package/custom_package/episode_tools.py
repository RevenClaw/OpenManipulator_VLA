#!/usr/bin/env python3
#
# episode_tools.py
#
# Pure helpers (numpy only, no ROS) for record_episodes and check_episode, so they can be unit-tested
# (test/test_episode_tools.py): episode folder naming, the 2-stage path back to HOME, the per-topic quick check of a
# recorded episode, and the metadata.json written next to each bag.
#
# Episode layout (the input contract of vla_layer/scripts/convert_rosbag_to_lerobot.py, see vla_layer/docs/PHASE5_GUIDE.md):
#   <data_dir>/<task_slug>/episode_0000/   metadata.yaml + episode_0000_0.mcap (rosbag2)  +  metadata.json (this module)
#

import datetime
import pathlib
import re
import socket

import numpy as np

RECORDER_VERSION = 1
HOME = [0.6090, -1.3622, 1.3898, -0.0154]  # configured_home_position.yaml (known issue #5: keep all copies in sync)

# Minimum healthy rates for the standard streams (the camera and teleop rates are 15 Hz and ~30 Hz).
DEFAULT_MIN_RATES = {
    '/joint_states': 50.0,
    '/teleop/action': 20.0,
    '/camera/front/color/image_raw': 12.0,
    '/camera/wrist/image_raw': 12.0,
}


def slugify(task: str, max_len: int = 60) -> str:
    """'Pick up the red cube!' -> 'pick_up_the_red_cube' (folder name for a task)."""
    slug = re.sub(r'[^a-z0-9]+', '_', task.lower()).strip('_')
    return slug[:max_len].rstrip('_') or 'task'


def episode_name(index: int) -> str:
    return f'episode_{index:04d}'


def next_episode_index(task_dir: pathlib.Path) -> int:
    """One more than the highest existing episode_NNNN in task_dir (0 for a new task)."""
    numbers = [int(m.group(1)) for p in pathlib.Path(task_dir).glob('episode_*')
               if (m := re.fullmatch(r'episode_(\d+)', p.name))]
    return max(numbers) + 1 if numbers else 0


def grid_position(index: int, n_positions: int):
    """Object start position to suggest for an episode (1..n, cycling), or None if the grid is off."""
    return index % n_positions + 1 if n_positions > 0 else None


# ---------------------------------------------------------------------------------------------------------- HOME
def two_stage_waypoints(start, target):
    """Same path as interactive_homing_node.send_safe_trajectory: halfway, the elbow (joint3) is at the higher of its two
    values and joint2 halfway, so the arm is folded while the shoulder moves (joint2 overload, 0x20)."""
    start, target = list(start), list(target)
    mid = [start[0], (start[1] + target[1]) / 2.0, max(start[2], target[2]), target[3]]
    return mid, target


def home_duration(start, target, max_speed: float, min_s: float = 4.0, max_s: float = 10.0) -> float:
    """Seconds for the HOME move: the largest joint distance at max_speed, clipped to [min_s, max_s]."""
    distance = float(np.max(np.abs(np.asarray(target) - np.asarray(start))))
    return float(np.clip(distance / max_speed, min_s, max_s))


def at_pose(q, target, tolerance: float) -> bool:
    return bool(np.all(np.abs(np.asarray(q) - np.asarray(target)) <= tolerance))


# --------------------------------------------------------------------------------------------------- quick check
def analyze_stamps(stamps: dict, min_rates: dict, max_gap: float = 0.2, skip_start: float = 0.5):
    """Per-topic health of a recorded episode from message receive times (seconds).

    stamps: {topic: array of times}; min_rates: {topic: Hz} for the topics that must be there.
    The first skip_start seconds of the bag are ignored for gaps (rosbag2 often misses messages right after it starts;
    that lead-in is idle and trimmed by the converter). Returns (rows, problems): one dict per topic, and a list of strings.
    """
    if not any(len(v) for v in stamps.values()):
        return [], ['the bag is empty']
    t0 = min(float(np.min(v)) for v in stamps.values() if len(v))
    t1 = max(float(np.max(v)) for v in stamps.values() if len(v))
    rows, problems = [], []
    for topic in sorted(set(stamps) | set(min_rates)):
        t = np.sort(np.asarray(stamps.get(topic, []), dtype=float))
        row = {'topic': topic, 'messages': int(len(t)), 'rate_hz': 0.0, 'max_gap_s': None, 'ok': True}
        if len(t) > 1:
            row['rate_hz'] = round((len(t) - 1) / max(t[-1] - t[0], 1e-9), 1)
            checked = t[t >= t0 + skip_start]
            # Include the edges, so a stream that stops early (or starts late) shows up as a gap.
            edges = np.concatenate([[t0 + skip_start], checked, [t1]]) if len(checked) else np.array([t0 + skip_start, t1])
            row['max_gap_s'] = round(float(np.max(np.diff(edges))), 3)
        need = min_rates.get(topic)
        if need is not None:
            if len(t) < 2:
                problems.append(f'{topic}: no messages' if len(t) == 0 else f'{topic}: only 1 message')
                row['ok'] = False
            else:
                if row['rate_hz'] < need:
                    problems.append(f'{topic}: {row["rate_hz"]:g} Hz (min {need:g})')
                    row['ok'] = False
                if row['max_gap_s'] > max_gap:
                    problems.append(f'{topic}: {row["max_gap_s"]:.2f} s gap (max {max_gap:g})')
                    row['ok'] = False
        rows.append(row)
    return rows, problems


def format_check(rows, problems, duration_s: float, size_bytes: int) -> str:
    lines = [f'  {"topic":<34} {"msgs":>6} {"rate":>8} {"max gap":>9}']
    for r in rows:
        gap = '-' if r['max_gap_s'] is None else f'{r["max_gap_s"]:.3f} s'
        lines.append(f'  {r["topic"]:<34} {r["messages"]:>6} {r["rate_hz"]:>6.1f} Hz {gap:>9}  {"✅" if r["ok"] else "⚠️"}')
    lines.append(f'  duration {duration_s:.1f} s, {size_bytes / 1e6:.0f} MB on disk')
    lines.append('  ✅ Episode looks good.' if not problems else '  ⚠️  ' + '; '.join(problems))
    return '\n'.join(lines)


# ------------------------------------------------------------------------------------------------------ metadata
def build_metadata(*, task: str, success: bool, index: int, start_time: float, duration_s: float, topics,
                   operator: str = '', grid_pos=None, storage: str = '', check=None, notes: str = '', extra=None) -> dict:
    """metadata.json content. 'task' and 'success' are what the converter needs; the rest documents how it was made."""
    meta = {
        'task': task,
        'success': bool(success),
        'episode_id': int(index),
        'start_time': datetime.datetime.fromtimestamp(start_time).isoformat(timespec='seconds'),
        'duration_s': round(float(duration_s), 2),
        'operator': operator,
        'grid_position': grid_pos,
        'topics': list(topics),
        'storage': storage,
        'quick_check': check or {},
        'notes': notes,
        'host': socket.gethostname(),
        'recorder_version': RECORDER_VERSION,
    }
    meta.update(extra or {})
    return meta
