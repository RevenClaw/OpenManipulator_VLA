#!/usr/bin/env python3
#
# scripted_demo.py
#
# Records scripted pick-and-place demonstrations for fine-tuning the pi0.5 VLA (v1, vla_layer/docs/FINETUNING_STAGES.md;
# procedure: docs/finetuning_procedure.md). Every episode, automatically:
#   1. task     a reachable cube (object_detector, YOLO) + a target (AprilTag) -> template prompt, e.g.
#               "pick up the red cube and place it in the bin"
#   2. plan     demo_planning: waypoints -> IK -> smooth minimum-jerk motion, small random variation, speed <= max_joint_vel
#   3. start    move to the start pose (not recorded), pre-flight check of every stream, `ros2 bag record`, 1 s lead-in
#   4. execute  arm_controller (FollowJointTrajectory) + gripper_controller (GripperCommand); every commanded target is
#               published on /teleop/action at 30 Hz: that is the ACTION the policy learns
#   5. check    grasp (gripper stopped on the cube, not fully closed), placement (object_detector after the motion), and
#               the recording's quick check -> success / failure label; episodes with recording problems are deleted
#   6. save     metadata.json (prompt, success, source=scripted, task, positions, waypoints, variation, checks)
# Output = the recorder's format: <data_dir>/<dataset>/episode_NNNN/ (vla_layer/docs/PHASE5_GUIDE.md §2).
#
# Keys while running: p = pause after this episode, ENTER = continue, q = quit after this episode, Ctrl+C = abort now
# (the arm holds, the current episode is deleted).
#
# Usage:
#   ros2 run custom_package scripted_demo --ros-args --params-file \
#       $(ros2 pkg prefix custom_package)/share/custom_package/config/scripted_demo.yaml
#

import json
import pathlib
import queue
import shutil
import threading
import time

from control_msgs.action import FollowJointTrajectory
from control_msgs.action import GripperCommand
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from custom_package import check_episode
from custom_package import demo_planning as D
from custom_package import episode_tools as E
from custom_package.joint_commander import GRIPPER_CLOSE_M
from custom_package.joint_commander import GRIPPER_OPEN_M
from custom_package.recording import BagRecorder
from custom_package.recording import StreamMonitor

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4']
GRIPPER_JOINT = 'gripper_left_joint'


def _duration(msg, seconds):
    msg.sec, msg.nanosec = int(seconds), int(round((seconds % 1.0) * 1e9)) % 1_000_000_000
    return msg


class Aborted(Exception):
    pass


class ScriptedDemo(Node):

    def __init__(self):
        super().__init__('scripted_demo')
        dp = self.declare_parameter
        dp('dataset', 'scripted_pick_place')                   # folder under data_dir
        dp('data_dir', '/workspace/recordings')
        dp('prompt_template', 'pick up the {color} cube and place it in the {target}')
        dp('colors', ['red', 'green', 'blue'])                  # YOLO classes used as pick objects
        dp('color_targets', [''])                               # "red:red bin" fixes a target per colour; empty = random
        dp('episodes', 0)                                       # stop after this many saved episodes (0 = until q)
        dp('record', True)                                      # False: run the episodes without recording (trials)
        dp('seed', -1)                                          # -1 = random
        dp('detections_topic', '/object_detector/detections')
        dp('detection_max_age', 1.0)                            # s
        dp('action_topic', '/teleop/action')
        dp('action_rate', 30.0)
        dp('state_topic', '/joint_states')
        dp('front_topic', '/camera/front/color/image_raw')
        dp('wrist_topic', '')
        dp('extra_topics', [''])
        dp('min_rate_state', 50.0)
        dp('min_rate_action', 20.0)
        dp('min_rate_camera', 12.0)
        dp('stale_after', 0.3)
        dp('max_gap', 0.2)
        dp('lead_in', 1.0)
        dp('tail', 0.5)
        dp('storage', 'mcap')
        dp('storage_preset', 'zstd_fast')
        dp('min_free_gb', 5.0)
        dp('image_reliable_qos', True)
        # planning (demo_planning.PlanConfig)
        dp('table_z', 0.0)
        dp('cube_size', 0.025)
        dp('bin_height', 0.04)
        dp('release_clearance', 0.03)
        dp('max_joint_vel', [0.5, 0.3, 0.6, 0.8])               # rad/s per joint; vla_bridge must allow at least these
        dp('max_pitch_deg', 90.0)
        dp('min_pitch_deg', 30.0)
        dp('target_xy_offset', [0.0, 0.0])                      # m from the target tag to the drop point
        dp('place_offsets', [0.0, 0.0])                         # [dx0, dy0, dx1, dy1, ...] extra drop offsets, one per placement, cycling
        dp('remember_targets', True)                            # a tag covered by a placed object keeps its last seen position
        dp('confirm', False)                                    # print each plan and wait: ENTER = execute, s = skip, q = quit
        dp('workspace_x', [0.08, 0.32])                         # m, cubes outside are ignored
        dp('workspace_y', [-0.25, 0.25])
        dp('variation', True)
        dp('correction_prob', 0.1)
        # checks
        dp('check_grasp', True)                                 # False on mock hardware (the gripper always closes fully)
        dp('grasp_closed_threshold', -0.007)                    # m; at or below = closed on nothing
        dp('check_place', True)
        dp('target_radius', 0.06)
        dp('gripper_max_effort', 1.0)

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.p = p
        self.data_dir = pathlib.Path(p('data_dir')) / p('dataset')
        self.record = bool(p('record'))
        self.template = p('prompt_template')
        self.colors = list(p('colors'))
        self.color_targets = dict(s.split(':', 1) for s in p('color_targets') if ':' in s)
        self.target_offset = np.array(p('target_xy_offset'), float)
        self.known_targets = {}       # tag id -> last detected target (a placed object can cover its tag)
        self.attempts = 0             # episodes actually executed (selects the next place_offsets entry)
        self.answers = queue.Queue()  # keyboard lines, while a plan waits for confirmation
        self.awaiting = False
        self.cfg = D.PlanConfig(table_z=p('table_z'), cube_size=p('cube_size'), bin_height=p('bin_height'),
                                release_clearance=p('release_clearance'), max_joint_vel=tuple(p('max_joint_vel')),
                                max_pitch_deg=p('max_pitch_deg'), min_pitch_deg=p('min_pitch_deg'), rate_hz=p('action_rate'))
        seed = int(p('seed'))
        self.rng = np.random.default_rng(None if seed < 0 else seed)

        streams = [(p('state_topic'), JointState, p('min_rate_state')), (p('action_topic'), JointState, p('min_rate_action')),
                   (p('front_topic'), Image, p('min_rate_camera'))]
        if p('wrist_topic'):
            streams.append((p('wrist_topic'), Image, p('min_rate_camera')))
        self.min_rates = {t: r for t, _, r in streams}
        self.topics = [t for t, _, _ in streams] + [t for t in p('extra_topics') if t and t not in self.min_rates]
        self.monitor = StreamMonitor(self, streams, bool(p('image_reliable_qos')))
        self.bag = BagRecorder(p('storage'), p('storage_preset'))

        self.lock = threading.Lock()
        self.joints = None
        self.gripper_m = None
        self.joints_time = 0.0
        self.detections = None
        self.detections_time = 0.0
        self.cmd_q = None             # commanded arm target (published on /teleop/action)
        self.cmd_g = 0.0              # commanded gripper, 0 open .. 1 closed
        self.active = None            # (times, joints, t0) of the trajectory being executed
        self.abort = threading.Event()
        self.quit_requested = False
        self.recording_path = None    # episode being recorded (deleted if interrupted)
        self.session = {'success': 0, 'failure': 0, 'deleted': 0, 'skipped': 0, 'seconds': 0.0, 'trials': 0}

        self.create_subscription(JointState, p('state_topic'), self.joint_state_callback, 10)
        self.create_subscription(String, p('detections_topic'), self.detections_callback, 10)
        self.action_pub = self.create_publisher(JointState, p('action_topic'), 10)
        self.hold_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')
        self.create_timer(1.0 / float(p('action_rate')), self.publish_action)

    # ------------------------------------------------------------------ inputs
    def joint_state_callback(self, msg):
        names = list(msg.name)
        if all(j in names for j in ARM_JOINTS):
            with self.lock:
                self.joints = np.array([msg.position[names.index(j)] for j in ARM_JOINTS])
                if GRIPPER_JOINT in names:
                    self.gripper_m = msg.position[names.index(GRIPPER_JOINT)]
                self.joints_time = time.monotonic()
                if self.cmd_q is None:
                    self.cmd_q = self.joints.copy()

    def detections_callback(self, msg):
        data = json.loads(msg.data)
        with self.lock:
            self.detections = data
            self.detections_time = time.monotonic()
            for t in data.get('targets', []):
                if t.get('xyz'):
                    self.known_targets[t['id']] = t

    def current_joints(self):
        with self.lock:
            if self.joints is None or time.monotonic() - self.joints_time > 0.5:
                return None
            return self.joints.copy()

    def fresh_detections(self, newer_than=0.0, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.lock:
                d, t = self.detections, self.detections_time
            if d is not None and t > newer_than and time.monotonic() - t < self.p('detection_max_age'):
                return d
            time.sleep(0.05)
        return None

    # ------------------------------------------------------------------ the action stream
    def publish_action(self):
        """The commanded target at this instant, 30 Hz: interpolated along the trajectory being executed, else held."""
        with self.lock:
            if self.cmd_q is None:
                return
            q = self.cmd_q
            if self.active is not None:
                times, joints, t0 = self.active
                tau = time.monotonic() - t0
                i = int(np.clip(np.searchsorted(times, tau, side='right') - 1, 0, len(times) - 2))
                a = float(np.clip((tau - times[i]) / (times[i + 1] - times[i]), 0.0, 1.0))
                q = joints[i] + (joints[i + 1] - joints[i]) * a
            g = self.cmd_g
        msg = JointState(name=[*ARM_JOINTS, 'gripper'], position=[*map(float, q), float(g)])
        msg.header.stamp = self.get_clock().now().to_msg()
        self.action_pub.publish(msg)

    # ------------------------------------------------------------------ motion
    def execute(self, times, joints, timeout_extra=5.0):
        """Sends one dense trajectory (times from 0, first row = current target) and waits until it is done."""
        if not self.arm_client.wait_for_server(timeout_sec=3.0):
            raise Aborted('/arm_controller/follow_joint_trajectory not available')
        traj = JointTrajectory(joint_names=ARM_JOINTS)
        for t, q in zip(times[1:], joints[1:]):
            point = JointTrajectoryPoint(positions=[float(v) for v in q])
            _duration(point.time_from_start, float(t))
            traj.points.append(point)
        future = self.arm_client.send_goal_async(FollowJointTrajectory.Goal(trajectory=traj))
        with self.lock:
            self.active = (np.asarray(times), np.asarray(joints), time.monotonic())
        try:
            self._wait(future, 3.0, 'goal not accepted')
            handle = future.result()
            if not handle.accepted:
                raise Aborted('trajectory rejected by arm_controller')
            result = handle.get_result_async()
            self._wait(result, float(times[-1]) + timeout_extra, 'trajectory timeout')
            code = result.result().result.error_code
            if code != 0:
                raise Aborted(f'arm_controller error {code}')
        finally:
            with self.lock:
                self.active = None
                self.cmd_q = np.asarray(joints[-1]).copy()

    def _wait(self, future, timeout, what):
        deadline = time.monotonic() + timeout
        while not future.done():
            if self.abort.is_set():
                raise Aborted('aborted')
            if time.monotonic() > deadline:
                raise Aborted(what)
            time.sleep(0.01)

    def move_to(self, target):
        """Unrecorded move to `target` joints (2-stage, elbow folded while the shoulder moves, if far)."""
        q = self.current_joints()
        if q is None:
            raise Aborted('no fresh /joint_states')
        with self.lock:
            self.cmd_q = q.copy()
        if E.at_pose(q, target, 0.005):
            return
        points = [q]
        if np.max(np.abs(np.asarray(target) - q)) > 0.3:
            points.append(np.asarray(E.two_stage_waypoints(q, target)[0]))
        points.append(np.asarray(target, float))
        times, joints = [0.0], [q]
        for a, b in zip(points[:-1], points[1:]):
            t, j = D.joint_segment(a, b, self.cfg.max_joint_vel, self.cfg)
            times += list(times[-1] + t[1:])
            joints += list(j[1:])
        self.execute(np.array(times), np.array(joints))

    def gripper(self, value, dwell):
        """Commands the gripper (0 open .. 1 closed), waits `dwell` s, returns the measured gripper position (m)."""
        with self.lock:
            self.cmd_g = float(value)
        if self.gripper_client.wait_for_server(timeout_sec=2.0):
            goal = GripperCommand.Goal()
            goal.command.position = GRIPPER_CLOSE_M if value >= 0.5 else GRIPPER_OPEN_M
            goal.command.max_effort = float(self.p('gripper_max_effort'))
            self.gripper_client.send_goal_async(goal)
        else:
            raise Aborted('/gripper_controller/gripper_cmd not available')
        deadline = time.monotonic() + dwell
        while time.monotonic() < deadline:
            if self.abort.is_set():
                raise Aborted('aborted')
            time.sleep(0.02)
        with self.lock:
            return self.gripper_m

    def hold(self):
        """Stops the arm where it is (one-point trajectory on the topic: works even while a goal is running)."""
        q = self.current_joints()
        if q is None:
            return
        point = JointTrajectoryPoint(positions=[float(v) for v in q])
        _duration(point.time_from_start, 0.1)
        self.hold_pub.publish(JointTrajectory(joint_names=ARM_JOINTS, points=[point]))
        with self.lock:
            self.active = None
            self.cmd_q = q

    # ------------------------------------------------------------------ tasks
    def choose_task(self, det):
        """(cube, target, plan, variation, prompt) for a random reachable cube, or raises Aborted with the reason."""
        lo_x, hi_x = self.p('workspace_x')
        lo_y, hi_y = self.p('workspace_y')
        targets = [t for t in det['targets'] if t.get('xyz')]
        if self.p('remember_targets'):
            with self.lock:
                cached = [dict(t, cached=True) for i, t in self.known_targets.items() if i not in {x['id'] for x in targets}]
            targets += cached
        if not targets:
            raise Aborted('no target (AprilTag) visible')
        cubes = [o for o in det['objects'] if o['label'] in self.colors and o.get('xyz')
                 and lo_x <= o['xyz'][0] <= hi_x and lo_y <= o['xyz'][1] <= hi_y]
        reasons = []
        for i in self.rng.permutation(len(cubes)):
            cube = cubes[i]
            if cube['label'] in self.color_targets:
                options = [t for t in targets if t['name'] == self.color_targets[cube['label']]]
            else:
                options = list(targets)
            options = [t for t in options if np.linalg.norm(np.array(cube['xyz'][:2]) - t['xyz'][:2]) > self.p('target_radius')]
            if not options:
                continue
            target = options[int(self.rng.integers(len(options)))]
            var = D.sample_variation(self.rng, bool(self.p('variation')), float(self.p('correction_prob')))
            target_xy = np.array(target['xyz'][:2]) + self.target_offset + D.place_offset(self.p('place_offsets'), self.attempts)
            try:
                plan = D.plan_pick_place(cube['xyz'][:2], target_xy, self.cfg, var)
            except D.PlanningError as e:
                reasons.append(f'{cube["label"]}: {e}')
                continue
            return cube, target, target_xy, plan, var, D.make_prompt(self.template, cube['label'], target['name'])
        raise Aborted('no reachable cube' + (f' ({"; ".join(reasons)})' if reasons else f' among {len(cubes)} detected'))

    def confirmed(self, plan):
        """Prints the plan's waypoints and waits for the operator: ENTER = execute, s = skip (nothing moves), q = quit."""
        for name, w in plan.waypoints.items():
            print(f'        {name:13s} x {w[0]:.3f}  y {w[1]:.3f}  z {w[2]:.3f}  pitch {np.degrees(w[3]):.0f}°')
        print('  ❓  ENTER = execute this plan, s = skip it (nothing moves), q = quit')
        while not self.answers.empty():
            self.answers.get_nowait()
        self.awaiting = True
        try:
            while True:
                try:
                    answer = self.answers.get(timeout=0.2)
                except queue.Empty:
                    if self.abort.is_set():
                        raise Aborted('aborted')
                    continue
                if answer == 'q':
                    self.quit_requested = True
                    return False
                if answer in ('', 'y'):
                    return True
                if answer == 's':
                    return False
        finally:
            self.awaiting = False

    def preflight(self):
        problems = self.monitor.problems(self.p('stale_after'))
        self.data_dir.mkdir(parents=True, exist_ok=True)
        free_gb = shutil.disk_usage(self.data_dir).free / 1e9
        if free_gb < self.p('min_free_gb'):
            problems.append(f'only {free_gb:.1f} GB free in {self.data_dir}')
        return problems

    # ------------------------------------------------------------------ one episode
    def run_episode(self):
        """Returns 'saved', 'deleted' or raises Aborted (nothing recorded, or the recording was deleted)."""
        det = self.fresh_detections()
        if det is None:
            raise Aborted('no fresh detections (is object_detector running?)')
        cube, target, target_xy, plan, var, prompt = self.choose_task(det)
        index = E.next_episode_index(self.data_dir)
        name = E.episode_name(index)
        print(f'\n  🎯  {name}: "{prompt}"  cube ({cube["xyz"][0]:.3f}, {cube["xyz"][1]:.3f}) -> '
              f'{target["name"]} ({target_xy[0]:.3f}, {target_xy[1]:.3f}), pitch {np.degrees(plan.pick_pitch):.0f}°, '
              f'{plan.duration:.1f} s' + ('  [target tag covered: using its last seen position]' if target.get('cached') else ''))
        if self.p('confirm') and not self.confirmed(plan):
            return 'skipped'
        self.attempts += 1

        self.gripper(0.0, 0.3)
        self.move_to(plan.start)
        problems = self.preflight()
        if problems and self.record:
            raise Aborted('not ready: ' + '; '.join(problems))
        if not self.record:
            return self.trial(plan, cube, target_xy)

        path = self.data_dir / name
        start_wall = time.time()
        self.recording_path = path
        if not self.bag.start(path, self.topics):
            shutil.rmtree(path, ignore_errors=True)
            self.recording_path = None
            raise Aborted('ros2 bag record failed')
        grasp_m, error = None, ''
        t_go = time.monotonic()
        try:
            time.sleep(self.p('lead_in'))
            t_go = time.monotonic()
            for group in D.plan_groups(plan):
                if group['kind'] == 'move':
                    self.execute(group['times'], group['joints'])
                else:
                    measured = self.gripper(group['gripper'], group['duration'])
                    if group['name'] == 'close':
                        grasp_m = measured
                for topic in self.monitor.stale(self.p('stale_after')):
                    print(f'  ⚠️  {topic} stopped during the episode.')
            time.sleep(self.p('tail'))
        except Aborted as e:
            error = str(e)
            self.hold()
        duration = time.monotonic() - t_go
        clean = self.bag.stop()
        self.recording_path = None
        if error or not clean:
            shutil.rmtree(path, ignore_errors=True)
            self.session['deleted'] += 1
            raise Aborted(f'{name} deleted: {error or "recording did not stop cleanly"}')

        rows, problems, bag_duration, size = check_episode.check_bag(path, self.min_rates, self.p('max_gap'))
        print(E.format_check(rows, problems, bag_duration, size))
        if problems:
            shutil.rmtree(path, ignore_errors=True)
            self.session['deleted'] += 1
            print(f'  🗑️  {name} deleted (recording problems).')
            return 'deleted'

        checks = {}
        if self.p('check_grasp'):
            held = grasp_m is not None and grasp_m > self.p('grasp_closed_threshold')
            checks['grasp'] = {'ok': bool(held), 'gripper_m': grasp_m}
        if self.p('check_place'):
            after = self.fresh_detections(newer_than=time.monotonic() + 0.3, timeout=4.0)
            if after is None:
                checks['place'] = {'ok': False, 'reason': 'no detections after the episode'}
            else:
                ok, reason = D.verify_place(cube['label'], cube['xyz'][:2], target_xy,
                                            [o for o in after['objects'] if o.get('xyz')], self.p('target_radius'))
                checks['place'] = {'ok': ok, 'reason': reason}
        success = all(c['ok'] for c in checks.values())

        meta = E.build_metadata(
            task=prompt, success=success, index=index, start_time=start_wall, duration_s=duration, topics=self.topics,
            operator='scripted_demo', storage=self.bag.storage_label, check={'problems': problems, 'topics': rows},
            extra={'source': 'scripted', 'pick': cube['label'], 'place': target['name'],
                   'cube_xyz': cube['xyz'], 'cube_confidence': cube['confidence'], 'target_tag': target['id'],
                   'target_xyz': target['xyz'], 'drop_xy': target_xy.tolist(),
                   'pick_pitch_deg': round(float(np.degrees(plan.pick_pitch)), 1),
                   'place_pitch_deg': round(float(np.degrees(plan.place_pitch)), 1),
                   'waypoints': {k: [round(float(v), 4) for v in w] for k, w in plan.waypoints.items()},
                   'variation': var.asdict(), 'checks': checks,
                   'planner': {k: v for k, v in vars(self.cfg).items()}})
        (path / 'metadata.json').write_text(json.dumps(meta, indent=2) + '\n')
        self.session['success' if success else 'failure'] += 1
        self.session['seconds'] += duration
        verdict = '✅ SUCCESS' if success else '❌ FAILURE (kept, not trained on)'
        detail = '; '.join(f'{k}: {v.get("reason", "ok" if v["ok"] else "failed")}' for k, v in checks.items())
        print(f'  💾  {name} saved: {verdict}' + (f' [{detail}]' if detail else ''))
        return 'saved'

    def trial(self, plan, cube, target_xy):
        """The same motion and checks without recording (first runs on the real arm, tuning grasp_closed_threshold)."""
        grasp_m = None
        for group in D.plan_groups(plan):
            if group['kind'] == 'move':
                self.execute(group['times'], group['joints'])
            else:
                measured = self.gripper(group['gripper'], group['duration'])
                if group['name'] == 'close':
                    grasp_m = measured
        held = grasp_m is not None and grasp_m > self.p('grasp_closed_threshold')
        after = self.fresh_detections(newer_than=time.monotonic() + 0.3, timeout=4.0)
        place = D.verify_place(cube['label'], cube['xyz'][:2], target_xy, [o for o in after['objects'] if o.get('xyz')],
                               self.p('target_radius')) if after else (False, 'no detections')
        print(f'  🧪  trial (not recorded): gripper after closing {grasp_m if grasp_m is None else round(grasp_m, 4)} m '
              f'-> {"holding" if held else "nothing held"} (threshold {self.p("grasp_closed_threshold")}); '
              f'place: {"ok" if place[0] else "failed"} ({place[1]})')
        self.session['trials'] += 1
        return 'trial'

    def print_summary(self):
        s = self.session
        if not self.record:
            print(f'  📊  {s["trials"]} trial episode(s) run (nothing recorded), {s["skipped"]} skipped')
            return
        print(f'  📊  {s["success"]} success, {s["failure"]} failure, {s["deleted"]} deleted, {s["skipped"]} skipped, '
              f'{s["seconds"] / 60:.1f} min recorded -> {self.data_dir}')


def keyboard(state, node):
    """p = pause after this episode, ENTER = continue, q = quit after this episode. While a plan waits for confirmation
    (confirm:=true) the lines go to it instead."""
    while not state['quit']:
        try:
            line = input().strip().lower()
        except EOFError:  # no keyboard (started from a script or launch file): run until `episodes`, Ctrl+C to stop
            return
        except KeyboardInterrupt:
            state['quit'] = True
            break
        if node.awaiting:
            node.answers.put(line)
            continue
        if line == 'q':
            state['quit'] = True
        elif line == 'p':
            state['pause'] = True
            print('  ⏸️  Pausing after this episode (ENTER = continue).')
        elif line == '':
            state['pause'] = False
        state['event'].set()


def main(args=None):
    rclpy.init(args=args)
    node = ScriptedDemo()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    print()
    print('=' * 66)
    print('  🤖  OpenManipulator-X  |  Scripted Pick-and-Place Demonstrations')
    print('=' * 66)
    print(f'  Saving to: {node.data_dir}' if node.record else '  🧪  TRIAL MODE: episodes are executed but NOT recorded')
    print(f'  Prompt: "{node.template}"  colours {node.colors}, speed caps j1..j4 {list(node.cfg.max_joint_vel)} rad/s')
    print(f'  Checks: grasp {node.p("check_grasp")}, place {node.p("check_place")}. Topics: {", ".join(node.topics)}')
    print('  p = pause after this episode, ENTER = continue, q = quit after this episode, Ctrl+C = abort now')
    print('=' * 66)
    state = {'quit': False, 'pause': False, 'event': threading.Event()}
    threading.Thread(target=keyboard, args=(state, node), daemon=True).start()
    target = int(node.p('episodes'))
    try:
        time.sleep(1.5)  # subscriptions fill
        node.monitor.print_status(node.p('stale_after'))
        saved = 0
        while not state['quit'] and (target <= 0 or saved < target):
            if state['pause']:
                node.print_summary()
                print('  ⏸️  Paused. ENTER = continue, q = quit.')
                state['event'].clear()
                state['event'].wait()
                continue
            try:
                if node.run_episode() in ('saved', 'trial'):
                    saved += 1
                if node.quit_requested:
                    state['quit'] = True
            except Aborted as e:
                node.session['skipped'] += 1
                print(f'  ⚠️  {e}')
                node.hold()
                print('  ⏸️  Fix it (e.g. rearrange the cubes), then ENTER = continue, q = quit.')
                state['pause'] = True
    except KeyboardInterrupt:
        print('\n  🛑  Ctrl+C: stopping the arm.')
        node.abort.set()
        node.hold()
    finally:
        node.bag.stop()
        if node.recording_path is not None:  # interrupted mid-episode: never keep a partial recording
            shutil.rmtree(node.recording_path, ignore_errors=True)
            print(f'  🗑️  {node.recording_path.name} deleted (interrupted).')
        node.print_summary()
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
