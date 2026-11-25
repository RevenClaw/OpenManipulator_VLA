#!/usr/bin/env python3
"""Divergence of the policy from the correct (scripted YOLO) trajectory, per checkpoint.

record  Logs one run from /joint_states (joints, gripper, fingertip path) while something else moves the arm: the YOLO pipeline
        (scripted_demo / guided_demo) for the reference, or vla_bridge with a checkpoint. ENTER = start, ENTER = stop.
            ros2 run custom_package trajectory_compare record --session blue_red_1 --label yolo
            ros2 run custom_package trajectory_compare record --session blue_red_1 --label ckpt5000
plot    Compares every run of a session with the reference run and writes plots + a metrics table into the session folder.
            ros2 run custom_package trajectory_compare plot --session blue_red_1

Same command, same cube position and start pose (READY) for every run of a session; otherwise the comparison is meaningless.
Files: /workspace/recordings/trajectory_compare/<session>/<label>.csv (+ .json), divergence.png, metrics.csv, metrics.md.
A second run with the same label is saved as <label>_2, <label>_3 ...
"""
import argparse
import csv
import datetime
import json
import math
import pathlib
import sys
import threading
import time

import numpy as np

from custom_package import trajectory_tools as T
from custom_package.ee_commander import forward_kinematics
from custom_package.vla_safety import gripper_metres_to_norm

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4']
GRIPPER_JOINT = 'gripper_left_joint'
DATA_DIR = '/workspace/recordings/trajectory_compare'
COLUMNS = ['t', *ARM_JOINTS, 'gripper', 'x', 'y', 'z', 'pitch']


# ------------------------------------------------------------------------------------------------------------- record
def record(args):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    session_dir = pathlib.Path(args.data_dir) / args.session
    session_dir.mkdir(parents=True, exist_ok=True)
    name, k = args.label, 1
    while (session_dir / f'{name}.csv').exists():
        k += 1
        name = f'{args.label}_{k}'

    rows, lock, active = [], threading.Lock(), threading.Event()
    start = None
    rclpy.init()
    node = Node('trajectory_compare')

    def on_joint_state(msg):
        names = list(msg.name)
        if not active.is_set() or not all(n in names for n in (*ARM_JOINTS, GRIPPER_JOINT)):
            return
        q = [msg.position[names.index(j)] for j in ARM_JOINTS]
        g = gripper_metres_to_norm(msg.position[names.index(GRIPPER_JOINT)])
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 or time.time()
        with lock:
            rows.append([t, *q, g, *forward_kinematics(q)])

    node.create_subscription(JointState, '/joint_states', on_joint_state, 50)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    print('=' * 66)
    print(f'  📈  Trajectory compare | session "{args.session}" | run "{name}"')
    print('=' * 66)
    print('  Same command, same cube position, arm at READY for every run of this session.')
    try:
        input('  ⏎  ENTER = start recording (then start the YOLO demo / enable the bridge): ')
        active.set()
        start = datetime.datetime.now().isoformat(timespec='seconds')
        print('  🔴  Recording. ENTER = stop (after the release, or when the run has clearly failed).')
        input()
    except (EOFError, KeyboardInterrupt):
        print()
    active.clear()
    executor.shutdown()
    node.destroy_node()
    rclpy.try_shutdown()
    spin.join(timeout=1.0)

    with lock:
        data = list(rows)
    if len(data) < 10:
        print(f'  ❌  Only {len(data)} samples: is /joint_states running? Nothing saved.')
        return 1
    with open(session_dir / f'{name}.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(COLUMNS)
        w.writerows(data)
    meta = {'label': name, 'session': args.session, 'start': start, 'samples': len(data),
            'duration_s': round(data[-1][0] - data[0][0], 2), 'prompt': args.prompt, 'cube_xy': args.cube,
            'notes': args.notes}
    (session_dir / f'{name}.json').write_text(json.dumps(meta, indent=2) + '\n')
    print(f'  💾  {len(data)} samples, {meta["duration_s"]} s → {session_dir / (name + ".csv")}')
    return 0


# --------------------------------------------------------------------------------------------------------------- plot
def load_run(path):
    data = np.loadtxt(path, delimiter=',', skiprows=1, ndmin=2)
    t, joints, gripper, xyz = data[:, 0], data[:, 1:5], data[:, 5], data[:, 6:9]
    seg = T.task_segment(t, joints, gripper)
    return {'label': path.stem, 't': t[seg] - t[seg][0], 'path': xyz[seg], 'gripper': gripper[seg]}


def plot(args):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    session_dir = pathlib.Path(args.data_dir) / args.session
    files = sorted(session_dir.glob('*.csv'))
    skip = {x.strip() for x in args.exclude.split(',') if x.strip()}
    files = [f for f in files if f.name != 'metrics.csv' and f.stem not in skip]
    runs = sorted((load_run(f) for f in files), key=lambda r: T.label_order(r['label']))
    refs = [r for r in runs if r['label'] == args.ref or r['label'].startswith(args.ref + '_')]
    if not refs:
        print(f'  ❌  No reference run "{args.ref}" in {session_dir} (runs: {", ".join(r["label"] for r in runs) or "none"}).')
        return 1
    ref = refs[0]
    others = [r for r in runs if r is not ref]
    metrics = [(r['label'], T.compare(r, ref)) for r in others]

    colors = plt.get_cmap('viridis')(np.linspace(0.1, 0.9, max(len(others), 1)))
    fig, ax = plt.subplots(2, 2, figsize=(13, 10))
    (top, side), (dev_ax, bar_ax) = ax

    def draw(axis, r, color, ref_run=False):
        p = r['path']
        radial = np.hypot(p[:, 0], p[:, 1])
        xs, ys = (p[:, 1], p[:, 0]) if axis is top else (radial, p[:, 2])
        kw = dict(color='black', lw=3, label=f'{r["label"]} (reference)') if ref_run else dict(color=color, lw=1.6, label=r['label'])
        axis.plot(xs, ys, **kw)
        g, rel = T.phases(r['gripper'])
        for idx, marker in ((g, 'o'), (rel, 's')):
            if idx is not None:
                axis.plot(xs[idx], ys[idx], marker, color=kw['color'], ms=8, mec='white')

    for axis in (top, side):
        draw(axis, ref, 'black', ref_run=True)
        for r, c in zip(others, colors):
            draw(axis, r, c)
    top.set(title='Top view (o = grasp, □ = release)', xlabel='y (m, left +)', ylabel='x (m, forward)')
    top.invert_xaxis()
    top.set_aspect('equal', adjustable='datalim')
    side.set(title='Side view', xlabel='distance from the base axis (m)', ylabel='z (m)')
    side.axhline(-0.056, color='grey', ls=':', lw=1)
    side.text(side.get_xlim()[0], -0.056, ' table', va='bottom', color='grey')
    top.legend(fontsize=8)

    _, rp = T.resample(ref['t'], ref['path'])
    for r, c in zip(others, colors):
        _, pp = T.resample(r['t'], r['path'])
        dev = T.path_deviation(pp, rp) * 100
        dev_ax.plot(np.linspace(0, 100, len(dev)), dev, color=c, label=r['label'])
    dev_ax.set(title=f'Distance from the reference path over the run', xlabel='progress of the run (%)', ylabel='deviation (cm)')
    dev_ax.legend(fontsize=8)

    labels = [m[0] for m in metrics]
    x = np.arange(len(labels))
    for k, (key, name) in enumerate((('mean_dev_m', 'mean path deviation'), ('dtw_m', 'DTW distance'),
                                     ('grasp_err_m', 'grasp point error'))):
        bar_ax.bar(x + (k - 1) * 0.27, [m[1][key] * 100 for m in metrics], 0.27, label=name)
    bar_ax.set_xticks(x, labels)
    bar_ax.set(title='Divergence from the reference per checkpoint', ylabel='cm')
    bar_ax.legend(fontsize=8)
    fig.suptitle(f'Session "{args.session}": runs vs. reference "{ref["label"]}"', fontsize=14)
    fig.tight_layout()
    out_png = session_dir / 'divergence.png'
    fig.savefig(out_png, dpi=130)

    keys = ['mean_dev_m', 'max_dev_m', 'dtw_m', 'grasp_err_m', 'release_err_m', 'duration_s', 'rms_jerk']
    ref_row = {'duration_s': float(ref['t'][-1]), 'rms_jerk': T.rms_jerk(ref['t'], ref['path'])}
    with open(session_dir / 'metrics.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['run', *keys])
        w.writerow([ref['label'], *(ref_row.get(k, 0.0) for k in keys)])
        for label, m in metrics:
            w.writerow([label, *(m[k] for k in keys)])

    def cm(v):
        return '–' if math.isnan(v) else f'{v * 100:.1f}'
    lines = [f'# Trajectory divergence: session `{args.session}`', '',
             f'Reference: `{ref["label"]}` ({ref_row["duration_s"]:.1f} s, RMS jerk {ref_row["rms_jerk"]:.2f} m/s³). '
             'Distances in cm; grasp/release error = distance between the fingertip at the run\'s grasp/release and the reference\'s.', '',
             '| Run | Mean deviation | Max deviation | DTW | Grasp error | Release error | Duration (s) | RMS jerk (m/s³) |',
             '|---|---|---|---|---|---|---|---|']
    for label, m in metrics:
        lines.append(f'| {label} | {cm(m["mean_dev_m"])} | {cm(m["max_dev_m"])} | {cm(m["dtw_m"])} | {cm(m["grasp_err_m"])} | '
                     f'{cm(m["release_err_m"])} | {m["duration_s"]:.1f} | {m["rms_jerk"]:.2f} |')
    lines += ['', '![divergence](divergence.png)', '']
    (session_dir / 'metrics.md').write_text('\n'.join(lines))
    print('\n'.join(lines[4:-3]))
    print(f'  📊  {out_png}\n  📝  {session_dir / "metrics.md"}')
    return 0


def main(args=None):
    from rclpy.utilities import remove_ros_args
    argv = remove_ros_args(sys.argv if args is None else ['trajectory_compare', *args])[1:]
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('mode', choices=['record', 'plot'])
    parser.add_argument('--session', required=True, help='one command + one cube position, e.g. blue_red_1')
    parser.add_argument('--label', help='record: yolo (reference), ckpt5000, ckpt15000, ckpt25000, ...')
    parser.add_argument('--ref', default='yolo', help='plot: label of the reference run (default yolo)')
    parser.add_argument('--exclude', default='', help='plot: comma-separated runs to leave out (false starts), e.g. yolo,ckpt5000')
    parser.add_argument('--prompt', default='', help='record: the command, for the notes')
    parser.add_argument('--cube', default='', help='record: cube position "x y" (m), for the notes')
    parser.add_argument('--notes', default='')
    parser.add_argument('--data-dir', default=DATA_DIR)
    a = parser.parse_args(argv)
    if a.mode == 'record':
        if not a.label:
            parser.error('record needs --label')
        return record(a)
    return plot(a)


if __name__ == '__main__':
    sys.exit(main())
