#!/usr/bin/env python3
"""Recorded episodes (rosbag2 from record_episodes / scripted_demo) -> video.mp4 next to each bag: front | wrist side by side.

    ros2 run custom_package export_episode_video /workspace/recordings/policy_runs            # every episode without a video
    ros2 run custom_package export_episode_video /workspace/recordings/policy_runs/<task>/episode_0003 --force

Frames are placed on the front camera's timeline (15 Hz); the wrist frame shown is the latest one at that time. A view that is
not in the bag is drawn black. Needs rosbag2_py (ROS 2) and OpenCV; runs in the robot container.
"""
import argparse
import pathlib
import sys

import cv2
import numpy as np

FRONT = '/camera/front/color/image_raw'
WRIST = '/camera/wrist/color/image_raw'
HEIGHT = 480  # output height; each view is scaled to it, keeping its aspect ratio


def read_frames(bag_dir):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[FRONT, WRIST]))
    frames = {FRONT: [], WRIST: []}
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        msg = deserialize_message(raw, Image)
        if msg.encoding not in ('rgb8', 'bgr8'):
            continue
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, : msg.width * 3]
        img = img.reshape(msg.height, msg.width, 3)
        bgr = img if msg.encoding == 'bgr8' else img[..., ::-1]
        frames[topic].append((t_ns * 1e-9, cv2.resize(bgr, (int(msg.width * HEIGHT / msg.height), HEIGHT))))
    return frames


def export(bag_dir, force=False):
    out = pathlib.Path(bag_dir) / 'video.mp4'
    if out.exists() and not force:
        print(f'  ⏭️  {out} exists (--force to redo)')
        return True
    frames = read_frames(bag_dir)
    front, wrist = frames[FRONT], frames[WRIST]
    if not front:
        print(f'  ❌  {bag_dir}: no front images')
        return False
    w_front = front[0][1].shape[1]
    w_wrist = wrist[0][1].shape[1] if wrist else int(HEIGHT * 4 / 3)
    writer = cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*'mp4v'), 15.0, (w_front + w_wrist, HEIGHT))
    wrist_times = np.array([t for t, _ in wrist])
    t0 = front[0][0]
    for t, img in front:
        right = np.zeros((HEIGHT, w_wrist, 3), np.uint8)
        if wrist:
            k = int(np.searchsorted(wrist_times, t, side='right')) - 1
            right = wrist[max(k, 0)][1]
        frame = np.hstack([img, right])
        cv2.putText(frame, f'{t - t0:5.1f} s', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        writer.write(frame)
    writer.release()
    print(f'  🎬  {out} ({len(front)} frames, {front[-1][0] - t0:.1f} s)')
    return True


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('path', help='an episode folder, or a folder searched for episodes (any depth)')
    parser.add_argument('--force', action='store_true', help='overwrite existing video.mp4')
    a = parser.parse_args(args if args is not None else sys.argv[1:])
    root = pathlib.Path(a.path)
    episodes = [root] if (root / 'metadata.yaml').exists() else sorted(p.parent for p in root.rglob('metadata.yaml'))
    if not episodes:
        print(f'  ❌  no recorded episodes under {root}')
        return 1
    ok = all([export(e, a.force) for e in episodes])
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
