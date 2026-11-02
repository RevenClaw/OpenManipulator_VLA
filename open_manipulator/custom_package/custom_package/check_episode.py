#!/usr/bin/env python3
#
# check_episode.py
#
# Quick health check of recorded episodes (rosbag2 directories): message count, rate and largest gap per topic, and a
# verdict. record_episodes runs it after every episode; it also works on its own, e.g. on a whole session:
#
#   ros2 run custom_package check_episode /workspace/recordings/pick_up_the_red_cube/episode_0003
#   ros2 run custom_package check_episode /workspace/recordings/pick_up_the_red_cube      # every episode in the folder
#   ros2 run custom_package check_episode <dir> --max-gap 0.3 --min-rate /camera/wrist/image_raw=10
#
# Uses message receive times (no deserialization), so it is fast. The converter on the server repeats these checks on
# header stamps (vla_layer/omx_pi05/dataset.py).
#

import argparse
import json
import pathlib
import sys

import numpy as np

from custom_package import episode_tools as E


def read_receive_times(bag_dir):
    """{topic: receive times in seconds} for every topic in a rosbag2 directory."""
    import rosbag2_py

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=''), rosbag2_py.ConverterOptions('', ''))
    times = {t.name: [] for t in reader.get_all_topics_and_types()}
    while reader.has_next():
        topic, _, t_ns = reader.read_next()
        times[topic].append(t_ns * 1e-9)
    del reader  # closes the bag
    return {k: np.asarray(v) for k, v in times.items()}


def bag_size(bag_dir) -> int:
    return sum(p.stat().st_size for p in pathlib.Path(bag_dir).iterdir() if p.is_file())


def check_bag(bag_dir, min_rates=None, max_gap=0.2):
    """Returns (rows, problems, duration_s, size_bytes) for one episode."""
    stamps = read_receive_times(bag_dir)
    rates = {t: r for t, r in (min_rates or E.DEFAULT_MIN_RATES).items() if t in stamps or min_rates}
    rows, problems = E.analyze_stamps(stamps, rates, max_gap=max_gap)
    all_t = [v for v in stamps.values() if len(v)]
    duration = float(max(v.max() for v in all_t) - min(v.min() for v in all_t)) if all_t else 0.0
    return rows, problems, duration, bag_size(bag_dir)


def main(args=None):
    parser = argparse.ArgumentParser(description='Quick health check of recorded episodes (rosbag2 directories).')
    parser.add_argument('paths', nargs='+', type=pathlib.Path, help='episode directories, or folders containing them')
    parser.add_argument('--max-gap', type=float, default=0.2, help='seconds; a longer gap in a checked topic is a problem')
    parser.add_argument('--min-rate', action='append', default=[], metavar='TOPIC=HZ',
                        help='required topic and its minimum rate (repeatable); default: the standard recorder topics present')
    ns = parser.parse_args(args if args is not None else sys.argv[1:])
    min_rates = {t: float(hz) for t, hz in (s.rsplit('=', 1) for s in ns.min_rate)} or None

    episodes = []
    for p in ns.paths:
        episodes += [p] if (p / 'metadata.yaml').exists() else sorted(q.parent for q in p.rglob('metadata.yaml'))
    if not episodes:
        print('  ❌  No rosbag2 episodes (metadata.yaml) found.')
        return 1
    bad = 0
    for ep in episodes:
        meta_path = ep / 'metadata.json'
        label = ''
        if meta_path.exists():
            meta = json.loads(meta_path.read_text())
            label = f'  [{"success" if meta.get("success") else "failure"}] "{meta.get("task", "")}"'
        print(f'\n  📼 {ep}{label}')
        rows, problems, duration, size = check_bag(ep, min_rates, ns.max_gap)
        print(E.format_check(rows, problems, duration, size))
        bad += bool(problems)
    print(f'\n  {len(episodes) - bad}/{len(episodes)} episode(s) without problems.')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
