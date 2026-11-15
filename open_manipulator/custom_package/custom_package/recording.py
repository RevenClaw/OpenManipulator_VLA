#!/usr/bin/env python3
#
# recording.py
#
# Shared pieces of the episode recorders (record_episodes: keyboard, scripted_demo: automatic):
#   StreamMonitor  raw subscriptions that track arrival times only (frames are never deserialized) -> rates, silence
#   BagRecorder    one `ros2 bag record` subprocess per episode, started/stopped cleanly
# Episode layout and metadata: episode_tools.py. Format contract: vla_layer/docs/PHASE5_GUIDE.md §2.
#

import collections
import os
import signal
import subprocess
import threading
import time

from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image


class StreamMonitor:
    """Arrival-time tracking for a set of topics. streams: [(topic, msg_type, min_rate_hz)]."""

    def __init__(self, node, streams, image_reliable_qos=True):
        self.lock = threading.Lock()
        self.min_rates = {topic: rate for topic, _, rate in streams}
        self.arrivals = {topic: collections.deque(maxlen=400) for topic in self.min_rates}
        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE) if image_reliable_qos else qos_profile_sensor_data
        for topic, msg_type, _ in streams:
            # raw=True: only arrival times are needed. Reliable for images, as in vla_bridge (best effort drops big frames).
            node.create_subscription(msg_type, topic, lambda _m, t=topic: self._arrived(t),
                                     image_qos if msg_type is Image else 10, raw=True)

    def _arrived(self, topic):
        with self.lock:
            self.arrivals[topic].append(time.monotonic())

    def status(self):
        """[(topic, age_s, rate_hz over the last 2 s, min_rate)]."""
        now = time.monotonic()
        rows = []
        with self.lock:
            for topic, need in self.min_rates.items():
                t = list(self.arrivals[topic])
                age = now - t[-1] if t else float('inf')
                recent = [x for x in t if x >= now - 2.0]
                rate = (len(recent) - 1) / (recent[-1] - recent[0]) if len(recent) > 1 and recent[-1] > recent[0] else 0.0
                rows.append((topic, age, rate, need))
        return rows

    def problems(self, stale_after):
        out = []
        for topic, age, rate, need in self.status():
            if age > stale_after:
                out.append(f'{topic}: no messages' if age == float('inf') else f'{topic}: silent for {age:.1f} s')
            elif rate < need:
                out.append(f'{topic}: {rate:.1f} Hz (min {need:g})')
        return out

    def stale(self, stale_after):
        return [topic for topic, age, _, _ in self.status() if age > stale_after]

    def print_status(self, stale_after):
        for topic, age, rate, need in self.status():
            ok = age <= stale_after and rate >= need
            age_s = 'never' if age == float('inf') else f'{age * 1000:.0f} ms ago'
            print(f'  {"✅" if ok else "❌"}  {topic:<34} {rate:5.1f} Hz (min {need:g}), last {age_s}')


class BagRecorder:
    """One `ros2 bag record` process per episode."""

    def __init__(self, storage='mcap', storage_preset='zstd_fast'):
        self.storage, self.storage_preset = storage, storage_preset
        self.process = None
        self.lines = []

    @property
    def storage_label(self):
        return f'{self.storage}/{self.storage_preset}' if self.storage == 'mcap' and self.storage_preset else self.storage

    def running(self):
        return self.process is not None and self.process.poll() is None

    def start(self, path, topics, timeout=10.0):
        """Starts recording `topics` into the rosbag2 directory `path` and waits until every topic is subscribed."""
        # --disable-keyboard-controls + no stdin: otherwise rosbag2 switches the terminal to raw mode for its own
        # pause/resume keys and steals the operator's ENTER (a recording then "stopped" right after GO).
        cmd = ['ros2', 'bag', 'record', '-o', str(path), '-s', self.storage, '--disable-keyboard-controls']
        if self.storage == 'mcap' and self.storage_preset:
            cmd += ['--storage-preset-profile', self.storage_preset]
        cmd += ['--topics', *topics]
        self.lines = []
        # Own process group, so Ctrl+C in the terminal doesn't kill the recording uncleanly; stopped with SIGINT in stop().
        self.process = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                        text=True, start_new_session=True)
        threading.Thread(target=self._read_output, daemon=True).start()
        deadline = time.time() + timeout
        while time.time() < deadline:
            if not self.missing(topics):
                return True
            if self.process.poll() is not None:
                break
            time.sleep(0.05)
        print(f'  ❌  ros2 bag record did not subscribe to: {", ".join(self.missing(topics))}')
        for line in self.lines[-5:]:
            print(f'      {line}')
        self.stop()
        return False

    def missing(self, topics):
        return [t for t in topics if not any(f"Subscribed to topic '{t}'" in line for line in self.lines)]

    def _read_output(self):
        for line in self.process.stdout:
            self.lines.append(line.rstrip())

    def stop(self):
        """Stops like Ctrl+C (rosbag2 flushes and closes the file). Returns False if it had to be killed."""
        if self.process is None:
            return True
        clean = True
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=20.0)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait()
                clean = False
                print('  ❌  ros2 bag record did not stop cleanly: the episode may be incomplete.')
        self.process = None
        return clean
