#!/usr/bin/env python3
#
# record_episodes.py
#
# Records teleoperated demonstrations, one rosbag2 episode at a time, for training the pi0.5 policy
# (vla_layer/docs/PHASE4_GUIDE.md T4.6; output format: vla_layer/docs/PHASE5_GUIDE.md §2).
#
#   ENTER  pre-flight check (every stream alive, task set, disk space, arm at HOME) -> suggested object position ->
#          ENTER starts `ros2 bag record` (separate process) -> GO after a 1 s lead-in -> ENTER stops
#          -> quick check (check_episode) -> label: s = success, f = failure (kept, not trained), d = discard (deleted)
#          -> metadata.json next to the bag -> back to HOME (gripper open) for the next episode
#   h = go HOME, c = check streams now, l = session summary, q = quit
#
# Output: <data_dir>/<task_slug>/episode_NNNN/ (rosbag2, mcap + zstd_fast by default) + metadata.json.
# data_dir defaults to /workspace/recordings: mounted from the host (open_manipulator/docker/workspace, git-ignored),
# so recordings survive a container rebuild.
#
# Teleop hook: if a std_srvs/SetBool service `teleop_enable_service` exists (the teleop node, PHASE4_GUIDE T4.4), teleop
# is paused before the arm is sent HOME and resumed before the next episode. Without it, pause teleop yourself before `h`.
#
# Usage:
#   ros2 run custom_package record_episodes --ros-args --params-file \
#       $(ros2 pkg prefix custom_package)/share/custom_package/config/record_episodes.yaml -p task:="pick up the red cube"
#

import collections
import json
import os
import pathlib
import shutil
import signal
import subprocess
import sys
import threading
import time

from control_msgs.action import FollowJointTrajectory
from control_msgs.action import GripperCommand
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from custom_package import check_episode
from custom_package import episode_tools as E

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4']
GRIPPER_OPEN_M = 0.019


def _duration(msg, seconds):
    msg.sec, msg.nanosec = int(seconds), int((seconds % 1.0) * 1e9)
    return msg


class EpisodeRecorder(Node):

    def __init__(self):
        super().__init__('record_episodes')
        self.declare_parameter('task', '')                     # the prompt; must match the run-time prompt exactly
        self.declare_parameter('data_dir', '/workspace/recordings')
        self.declare_parameter('operator', '')
        self.declare_parameter('state_topic', '/joint_states')
        self.declare_parameter('action_topic', '/teleop/action')
        self.declare_parameter('front_topic', '/camera/front/color/image_raw')
        self.declare_parameter('wrist_topic', '/camera/wrist/image_raw')  # '' = no wrist camera
        self.declare_parameter('extra_topics', [''])            # also recorded, not checked (e.g. /leader/joint_states)
        self.declare_parameter('min_rate_state', 50.0)          # Hz
        self.declare_parameter('min_rate_action', 20.0)
        self.declare_parameter('min_rate_camera', 12.0)
        self.declare_parameter('max_gap', 0.2)                  # s, quick-check limit (same as the converter)
        self.declare_parameter('stale_after', 0.3)              # s without a message = stream down (pre-flight, live)
        self.declare_parameter('lead_in', 1.0)                  # s recorded before GO (trimmed by the converter)
        self.declare_parameter('max_duration', 60.0)            # s, recording stops by itself after this
        self.declare_parameter('storage', 'mcap')
        self.declare_parameter('storage_preset', 'zstd_fast')   # 'none' if the PC can't keep up (dropped frames)
        self.declare_parameter('min_free_gb', 5.0)
        self.declare_parameter('require_home', True)            # episodes must start at HOME
        self.declare_parameter('home_tolerance', 0.05)          # rad
        self.declare_parameter('home_max_speed', 0.25)          # rad/s for the move back to HOME
        self.declare_parameter('auto_home', True)               # go HOME after every episode
        self.declare_parameter('grid_positions', 9)             # suggest object start positions 1..N (0 = off)
        self.declare_parameter('teleop_enable_service', '/teleop/enable')
        self.declare_parameter('image_reliable_qos', True)      # as vla_bridge: best effort drops large frames

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.task = p('task').strip()
        self.data_dir = pathlib.Path(p('data_dir'))
        self.task_dir = self.data_dir / E.slugify(self.task)
        self.operator = p('operator')
        self.max_gap = float(p('max_gap'))
        self.stale_after = float(p('stale_after'))
        self.lead_in = float(p('lead_in'))
        self.max_duration = float(p('max_duration'))
        self.storage, self.storage_preset = p('storage'), p('storage_preset')
        self.min_free_gb = float(p('min_free_gb'))
        self.require_home, self.auto_home = bool(p('require_home')), bool(p('auto_home'))
        self.home_tolerance, self.home_max_speed = float(p('home_tolerance')), float(p('home_max_speed'))
        self.grid_positions = int(p('grid_positions'))

        # Checked streams: (key, topic, type, min rate). Topics are recorded in this order, then the extras.
        streams = [('state', p('state_topic'), JointState, float(p('min_rate_state'))),
                   ('action', p('action_topic'), JointState, float(p('min_rate_action'))),
                   ('front', p('front_topic'), Image, float(p('min_rate_camera'))),
                   ('wrist', p('wrist_topic'), Image, float(p('min_rate_camera')))]
        self.streams = [s for s in streams if s[1]]
        self.min_rates = {topic: rate for _, topic, _, rate in self.streams}
        self.topics = [s[1] for s in self.streams] + [t for t in p('extra_topics') if t and t not in self.min_rates]

        self.lock = threading.Lock()
        self.arrivals = {topic: collections.deque(maxlen=400) for topic in self.min_rates}
        self.joints = None
        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE) if p('image_reliable_qos') \
            else qos_profile_sensor_data
        for _, topic, msg_type, _ in self.streams:
            # raw=True: only arrival times are needed, so frames are never deserialized.
            qos = image_qos if msg_type is Image else 10
            self.create_subscription(msg_type, topic, lambda _m, t=topic: self._arrived(t), qos, raw=True)
        self.create_subscription(JointState, p('state_topic'), self.joint_state_callback, 10)

        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')
        self.teleop_client = self.create_client(SetBool, p('teleop_enable_service')) if p('teleop_enable_service') else None

        self.bag = None             # subprocess.Popen of `ros2 bag record`
        self.bag_lines = []         # its output
        self.session = {'success': 0, 'failure': 0, 'discarded': 0, 'seconds': 0.0}

    # ------------------------------------------------------------------ inputs
    def _arrived(self, topic):
        with self.lock:
            self.arrivals[topic].append(time.monotonic())

    def joint_state_callback(self, msg: JointState):
        names = list(msg.name)
        if all(j in names for j in ARM_JOINTS):
            with self.lock:
                self.joints = [msg.position[names.index(j)] for j in ARM_JOINTS]

    def stream_status(self):
        """[(topic, age_s, rate_hz over the last 2 s, min_rate)]."""
        now = time.monotonic()
        rows = []
        with self.lock:
            for topic, rate_needed in self.min_rates.items():
                t = list(self.arrivals[topic])
                age = now - t[-1] if t else float('inf')
                recent = [x for x in t if x >= now - 2.0]
                rate = (len(recent) - 1) / (recent[-1] - recent[0]) if len(recent) > 1 and recent[-1] > recent[0] else 0.0
                rows.append((topic, age, rate, rate_needed))
        return rows

    # ------------------------------------------------------------------ checks
    def preflight(self):
        """Problems that block recording (empty list = ready)."""
        problems = []
        if not self.task:
            problems.append('no task: set -p task:="<prompt>" (exactly the prompt used at run time)')
        for topic, age, rate, need in self.stream_status():
            if age > self.stale_after:
                problems.append(f'{topic}: no messages' if age == float('inf') else f'{topic}: silent for {age:.1f} s')
            elif rate < need:
                problems.append(f'{topic}: {rate:.1f} Hz (min {need:g})')
        self.data_dir.mkdir(parents=True, exist_ok=True)
        free_gb = shutil.disk_usage(self.data_dir).free / 1e9
        if free_gb < self.min_free_gb:
            problems.append(f'only {free_gb:.1f} GB free in {self.data_dir} (min {self.min_free_gb:g})')
        if self.require_home:
            with self.lock:
                joints = self.joints
            if joints is None:
                problems.append('no arm joint positions yet')
            elif not E.at_pose(joints, E.HOME, self.home_tolerance):
                problems.append('arm is not at HOME (press h)')
        return problems

    def print_streams(self):
        for topic, age, rate, need in self.stream_status():
            ok = age <= self.stale_after and rate >= need
            age_s = 'never' if age == float('inf') else f'{age * 1000:.0f} ms ago'
            print(f'  {"✅" if ok else "❌"}  {topic:<34} {rate:5.1f} Hz (min {need:g}), last {age_s}')

    # ------------------------------------------------------------------ arm
    def set_teleop(self, enable):
        """Pauses/resumes the teleop node if it offers the service. Returns False only if the call failed."""
        if self.teleop_client is None or not self.teleop_client.service_is_ready():
            return True
        future = self.teleop_client.call_async(SetBool.Request(data=enable))
        start = time.time()
        while not future.done() and time.time() - start < 3.0:
            time.sleep(0.05)
        ok = future.done() and future.result() is not None and future.result().success
        print(f'  {"▶️  Teleop resumed" if enable else "⏸️  Teleop paused"}.' if ok else f'  ❌  Teleop {"resume" if enable else "pause"} failed.')
        return ok

    def go_home(self):
        """Gripper open, then the 2-stage move to HOME (joint2 overload: elbow folded while the shoulder moves)."""
        with self.lock:
            joints = self.joints
        if joints is None:
            print('  ❌  No /joint_states yet.')
            return False
        if not self.set_teleop(False):
            return False
        self._send_gripper_open()
        if E.at_pose(joints, E.HOME, self.home_tolerance):
            print('  🏠  Already at HOME.')
            return True
        if not self.arm_client.wait_for_server(timeout_sec=5.0):
            print('  ❌  /arm_controller/follow_joint_trajectory not available.')
            return False
        mid, target = E.two_stage_waypoints(joints, E.HOME)
        duration = E.home_duration(joints, E.HOME, self.home_max_speed)
        traj = JointTrajectory(joint_names=ARM_JOINTS)
        for q, t in ((mid, duration / 2.0), (target, duration)):
            point = JointTrajectoryPoint(positions=[float(v) for v in q], velocities=[0.0] * 4)
            _duration(point.time_from_start, t)
            traj.points.append(point)
        print(f'  🏠  Going HOME ({duration:.1f} s)...')
        future = self.arm_client.send_goal_async(FollowJointTrajectory.Goal(trajectory=traj))
        if not self._wait(future, 5.0) or not future.result().accepted:
            print('  ❌  HOME goal rejected.')
            return False
        result = future.result().get_result_async()
        if not self._wait(result, duration + 5.0):
            print('  ❌  Timeout going HOME.')
            return False
        print('  🏠  At HOME.')
        return True

    def _send_gripper_open(self):
        if self.gripper_client.wait_for_server(timeout_sec=2.0):
            goal = GripperCommand.Goal()
            goal.command.position, goal.command.max_effort = GRIPPER_OPEN_M, 1.0
            self.gripper_client.send_goal_async(goal)  # not awaited: it opens while the arm moves
        else:
            print('  ⚠️  /gripper_controller/gripper_cmd not available (gripper not opened).')

    @staticmethod
    def _wait(future, timeout):
        start = time.time()
        while not future.done():
            if time.time() - start > timeout:
                return False
            time.sleep(0.05)
        return True

    # ------------------------------------------------------------------ bag process
    def start_bag(self, path):
        """Starts `ros2 bag record` and waits until it has subscribed to every topic."""
        # --disable-keyboard-controls + no stdin: otherwise rosbag2 switches this terminal to raw mode for its own
        # pause/resume keys and steals the ENTER meant for us (a recording then "stopped" right after GO).
        cmd = ['ros2', 'bag', 'record', '-o', str(path), '-s', self.storage, '--disable-keyboard-controls']
        if self.storage == 'mcap' and self.storage_preset:
            cmd += ['--storage-preset-profile', self.storage_preset]
        cmd += ['--topics', *self.topics]
        self.bag_lines = []
        # Own process group, so Ctrl+C in this terminal doesn't kill the recording uncleanly; stopped with SIGINT below.
        self.bag = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    text=True, start_new_session=True)
        threading.Thread(target=self._read_bag_output, daemon=True).start()
        deadline = time.time() + 10.0
        while time.time() < deadline:
            subscribed = {t for t in self.topics if any(f"Subscribed to topic '{t}'" in line for line in self.bag_lines)}
            if subscribed == set(self.topics):
                return True
            if self.bag.poll() is not None:
                break
            time.sleep(0.05)
        missing = [t for t in self.topics if not any(f"Subscribed to topic '{t}'" in line for line in self.bag_lines)]
        print(f'  ❌  ros2 bag record did not subscribe to: {", ".join(missing)}')
        for line in self.bag_lines[-5:]:
            print(f'      {line}')
        self.stop_bag()
        return False

    def _read_bag_output(self):
        for line in self.bag.stdout:
            self.bag_lines.append(line.rstrip())

    def stop_bag(self):
        if self.bag is None:
            return True
        clean = True
        if self.bag.poll() is None:
            os.killpg(self.bag.pid, signal.SIGINT)  # like Ctrl+C: rosbag2 flushes and closes the file
            try:
                self.bag.wait(timeout=20.0)
            except subprocess.TimeoutExpired:
                os.killpg(self.bag.pid, signal.SIGKILL)
                self.bag.wait()
                clean = False
                print('  ❌  ros2 bag record did not stop cleanly: the episode may be incomplete.')
        self.bag = None
        return clean

    # ------------------------------------------------------------------ one episode
    def record_episode(self):
        self.set_teleop(True)
        time.sleep(0.5 if self.teleop_client is not None and self.teleop_client.service_is_ready() else 0.0)
        problems = self.preflight()
        if problems:
            print('  ❌  Not ready:')
            for problem in problems:
                print(f'      - {problem}')
            return
        index = E.next_episode_index(self.task_dir)
        name = E.episode_name(index)
        grid = E.grid_position(index, self.grid_positions)
        where = f'Place the object at grid position {grid} of {self.grid_positions}, then press' if grid else 'Press'
        if input(f'  📍  {name}: {where} ENTER to start (x = cancel): ').strip().lower() == 'x':
            return
        problems = self.preflight()
        if problems:
            print('  ❌  Not ready any more: ' + '; '.join(problems))
            return

        path = self.task_dir / name
        self.task_dir.mkdir(parents=True, exist_ok=True)
        start_wall = time.time()
        if not self.start_bag(path):
            shutil.rmtree(path, ignore_errors=True)
            return
        time.sleep(self.lead_in)
        t_go = time.monotonic()
        stop = threading.Event()
        print(f'  🟢  GO! Recording {name}. Press ENTER to stop (stops by itself after {self.max_duration:.0f} s).')
        monitor = threading.Thread(target=self._monitor, args=(stop, t_go), daemon=True)
        monitor.start()
        try:
            input()
        except (EOFError, KeyboardInterrupt):
            print()
        stop.set()
        duration = time.monotonic() - t_go
        clean = self.stop_bag()
        print(f'  ⏹️  Stopped after {duration:.1f} s.')

        rows, problems, bag_duration, size = check_episode.check_bag(path, self.min_rates, self.max_gap)
        print(E.format_check(rows, problems, bag_duration, size))
        if not clean:
            problems.append('recording did not stop cleanly')
        label = self._ask_label(recommend_discard=bool(problems))
        if label == 'd':
            shutil.rmtree(path, ignore_errors=True)
            self.session['discarded'] += 1
            print(f'  🗑️  {name} deleted.')
        else:
            meta = E.build_metadata(
                task=self.task, success=label == 's', index=index, start_time=start_wall, duration_s=duration,
                topics=self.topics, operator=self.operator, grid_pos=grid,
                storage=f'{self.storage}/{self.storage_preset}' if self.storage == 'mcap' else self.storage,
                check={'problems': problems, 'topics': rows})
            (path / 'metadata.json').write_text(json.dumps(meta, indent=2) + '\n')
            self.session['success' if label == 's' else 'failure'] += 1
            self.session['seconds'] += duration
            print(f'  💾  {name} saved as {"SUCCESS" if label == "s" else "FAILURE (kept, not trained on)"}: {path}')
        if self.auto_home:
            self.go_home()

    def _monitor(self, stop, t_go):
        warned = set()
        last_tick = 0
        while not stop.wait(0.25):
            elapsed = time.monotonic() - t_go
            for topic, age, _, _ in self.stream_status():
                if age > self.stale_after and topic not in warned:
                    warned.add(topic)
                    print(f'  ⚠️  {topic} stopped at {elapsed:.1f} s: this episode will likely fail the check (discard it).')
            if self.bag is not None and self.bag.poll() is not None:
                print('  ❌  ros2 bag record exited unexpectedly. Press ENTER.')
                return
            if elapsed >= self.max_duration:
                print(f'  ⏱️  {self.max_duration:.0f} s reached: press ENTER to stop.')
                return
            if int(elapsed) // 5 > last_tick:
                last_tick = int(elapsed) // 5
                print(f'  ⏺️  {last_tick * 5} s')

    @staticmethod
    def _ask_label(recommend_discard):
        hint = ' (problems found: d recommended)' if recommend_discard else ''
        while True:
            try:
                choice = input(f'  🏷️  s = success, f = failure (kept, not trained on), d = discard{hint}: ').strip().lower()
            except (EOFError, KeyboardInterrupt):
                choice = 'd'
            if choice in ('s', 'f', 'd'):
                return choice
            print('  ⚠️  Type s, f or d.')

    def print_summary(self):
        s = self.session
        used = sum(f.stat().st_size for f in self.task_dir.rglob('*') if f.is_file()) / 1e9 if self.task_dir.exists() else 0.0
        free = shutil.disk_usage(self.data_dir).free / 1e9 if self.data_dir.exists() else float('nan')
        total = len([p for p in self.task_dir.glob('episode_*') if (p / 'metadata.json').exists()]) if self.task_dir.exists() else 0
        print(f'  📊  This session: {s["success"]} success, {s["failure"]} failure, {s["discarded"]} discarded, '
              f'{s["seconds"] / 60:.1f} min recorded.')
        print(f'      {self.task_dir}: {total} episode(s), {used:.1f} GB. Free: {free:.0f} GB.')


def print_header(node):
    print()
    print('=' * 66)
    print('  🎬  OpenManipulator-X  |  Episode Recorder')
    print('=' * 66)
    print(f'  Task: "{node.task}"' if node.task else '  ⚠️  Task not set: -p task:="<prompt>"')
    print(f'  Saving to: {node.task_dir}')
    print(f'  Topics: {", ".join(node.topics)}')
    print('  ENTER = record an episode,  h = go HOME,  c = check streams,  l = summary,  q = quit')
    print('=' * 66)


def main(args=None):
    rclpy.init(args=args)
    node = EpisodeRecorder()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    print_header(node)
    try:
        time.sleep(1.0)  # let the subscriptions fill
        node.print_streams()
        while True:
            try:
                line = input('\n  >> ').strip().lower()
            except (EOFError, KeyboardInterrupt):
                print('\n  👋  Interrupted. Exiting...')
                break
            if line in ('q', 'quit', 'exit'):
                print('  👋  Exiting episode recorder.')
                break
            if line == '':
                node.record_episode()
            elif line == 'h':
                node.go_home()
            elif line == 'c':
                node.print_streams()
            elif line == 'l':
                node.print_summary()
            else:
                print('  ⚠️  Unknown command. ENTER / h / c / l / q')
    finally:
        node.stop_bag()
        node.print_summary()
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
