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


class Finished(Exception):
    """Nothing is left to pick (all objects at the target, or the rest is unreachable / given up on)."""


def height_report(cube, target, plan, cfg):
    """One line comparing what the camera measured (z) with what the plan aims for. The plan's heights come only from the
    config (table_z, cube_size, bin_height, release_clearance); the camera's z is shown for comparison, not used."""
    cam_cube = cube['xyz'][2] if len(cube.get('xyz') or []) > 2 else float('nan')
    cam_tag = (target.get('xyz') or [0, 0, float('nan')])[2]
    return (f'z: cube centre camera {cam_cube:+.3f} / grasp aim {plan.waypoints["grasp"][2]:+.3f}   '
            f'tag camera {cam_tag:+.3f} / table_z {cfg.table_z:+.3f}   release aim {plan.waypoints["release"][2]:+.3f}  (m)')


class ScriptedDemo(Node):

    def __init__(self, node_name='scripted_demo'):
        super().__init__(node_name)
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
        dp('wrist_topic', '')                  # '/camera/wrist/color/image_raw' = record the wrist D435 too
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
        dp('cube_size', 0.035)
        dp('bin_height', 0.04)
        dp('release_clearance', 0.015)
        dp('max_joint_vel', [0.5, 0.3, 0.6, 0.8])               # rad/s per joint; vla_bridge must allow at least these
        dp('max_pitch_deg', 90.0)
        dp('min_pitch_deg', 30.0)
        dp('target_xy_offset', [0.0, 0.0])                      # m from the target tag to the drop point
        dp('place_offsets', [0.0, 0.0])                         # [dx0, dy0, dx1, dy1, ...] extra drop offsets, one per placement, cycling
        dp('remember_targets', True)                            # a tag covered by a placed object keeps its last seen position
        dp('confirm', False)                                    # print each plan and wait: ENTER = execute, s = skip, q = quit
        dp('until_clear', False)                                # count the objects, keep going until all are at the target
        dp('max_attempts_per_object', 3)                        # until_clear: give up on an object after this many failed picks
        dp('min_radius', 0.08)                                  # m from the base axis; closer detections are ignored
        dp('cube_max_above', 0.03)                              # m: detections whose centre is higher than a cube's by this are ignored
        dp('variation', True)
        dp('correction_prob', 0.1)
        # checks
        dp('check_gripper_open', True)                          # before every pick: the gripper must measure open, else pause
        dp('gripper_open_tolerance', 0.004)                     # m from GRIPPER_OPEN_M that still counts as open
        dp('check_grasp', True)                                 # False on mock hardware (the gripper always closes fully)
        dp('grasp_closed_threshold', -0.007)                    # m; at or below = closed on nothing (closed end = -0.010)
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
        self.attempts = 0             # episodes actually executed
        self.placed = 0               # objects successfully placed
        self.placed_at = {}           # target tag id -> objects placed there (selects the next place_offsets entry per target)
        self.warned_no_target = set()
        self.until_clear = bool(p('until_clear'))
        self.max_attempts = int(p('max_attempts_per_object'))
        self.initial_count = None     # until_clear: objects to place, counted at the first look
        self.fail_log = []            # (label, x, y) of every failed pick, to give up on an object after max_attempts
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

    def ensure_gripper_open(self):
        """Check before every pick: the gripper must measure open (within gripper_open_tolerance of GRIPPER_OPEN_M). One retry
        with a longer wait, then Aborted, which pauses the loop until ENTER: nothing moves toward the object with closed fingers."""
        if not self.p('check_gripper_open'):
            return
        tol = float(self.p('gripper_open_tolerance'))
        with self.lock:
            measured = self.gripper_m
        if D.gripper_is_open(measured, GRIPPER_OPEN_M, tol):
            print(f'  ✅  gripper open ({measured:.4f} m)')
            return
        print(f'  ⚠️  gripper not open ({measured} m, expected {GRIPPER_OPEN_M} ± {tol} m): opening again')
        measured = self.gripper(0.0, 1.5)
        if D.gripper_is_open(measured, GRIPPER_OPEN_M, tol):
            print(f'  ✅  gripper open ({measured:.4f} m)')
            return
        raise Aborted(f'gripper did not open ({measured} m, expected {GRIPPER_OPEN_M} ± {tol} m): remove anything held, '
                      'check the gripper servo (ID 15) and its polarity (known issue #15)')

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
    def screen(self, det):
        """(pickable cubes, [(ignored detection, reason)]): demo_planning.screen_cubes with this node's parameters."""
        return D.screen_cubes(det['objects'], self.colors, self.cfg.table_z, self.cfg.cube_size, det.get('tags'),
                              lambda xy: D.cube_reachable(xy, self.cfg), float(self.p('cube_max_above')),
                              float(self.p('min_radius')))

    def choose_task(self, det):
        """(cube, target, plan, variation, prompt) for a random reachable cube, or raises Aborted with the reason."""
        if not det.get('calibrated', True):
            raise Aborted('the camera is not calibrated (no camera_calibration.yaml): run calibrate_camera first')
        targets = [t for t in det['targets'] if t.get('xyz')]
        if self.p('remember_targets'):
            with self.lock:
                cached = [dict(t, cached=True) for i, t in self.known_targets.items() if i not in {x['id'] for x in targets}]
            targets += cached
        if not targets:
            raise Aborted('no target (AprilTag) visible')
        cubes, _ = self.screen(det)
        if not cubes:
            if self.until_clear and self.attempts:
                raise Finished('no reachable objects left')
            raise Aborted('no reachable cube among 0 detected')
        items, at_target, given_up, no_target = [], 0, 0, set()
        for cube in cubes:
            if cube['label'] in self.color_targets:
                matching = [t for t in targets if t['name'] == self.color_targets[cube['label']]]
            else:
                matching = list(targets)
            if not matching:  # this colour's target tag has never been seen (not the same as "already on its target")
                no_target.add(cube['label'])
                continue
            options = [t for t in matching if np.linalg.norm(np.array(cube['xyz'][:2]) - t['xyz'][:2]) > self.p('target_radius')]
            if not options:
                at_target += 1  # already on its target
            elif self.until_clear and self.failures(cube) >= self.max_attempts:
                given_up += 1
            else:
                items.append((cube, options))
        for label in sorted(no_target - self.warned_no_target):
            self.warned_no_target.add(label)
            print(f'  ⚠️  No target tag seen for the {label} object(s) ({self.color_targets.get(label, "any")}): they are skipped '
                  f'until the camera has seen that tag once.')
        if self.until_clear and self.initial_count is None:
            self.initial_count = len(items)
            print(f'  🧮  {len(items)} object(s) to place; failed picks are retried (up to {self.max_attempts} attempts per object).')
        if not items and no_target and not at_target:
            raise Aborted('no target tag seen for: ' + ', '.join(f'{label} ({self.color_targets.get(label, "any")})' for label in sorted(no_target))
                          + '. Put the tag in the camera view.')
        if not items:
            raise Finished(f'{at_target} object(s) on their target'
                           + (f', {given_up} given up after {self.max_attempts} failed attempts' if given_up else '')
                           + (f'; skipped, target tag never seen: {", ".join(sorted(no_target))}' if no_target else ''))
        reasons = []
        for i in self.rng.permutation(len(items)):
            cube, options = items[i]
            target = options[int(self.rng.integers(len(options)))]
            var = D.sample_variation(self.rng, bool(self.p('variation')), float(self.p('correction_prob')))
            target_xy = (np.array(target['xyz'][:2]) + self.target_offset
                         + D.place_offset(self.p('place_offsets'), self.placed_at.get(target['id'], 0)))
            try:
                plan = D.plan_pick_place(cube['xyz'][:2], target_xy, self.cfg, var)
            except D.PlanningError as e:
                reasons.append(f'{cube["label"]}: {e}')
                continue
            return cube, target, target_xy, plan, var, D.make_prompt(self.template, cube['label'], target['name'])
        message = f'no reachable cube ({"; ".join(reasons)})'
        if self.until_clear:
            raise Finished(f'{len(items)} object(s) left but none reachable: {"; ".join(reasons)}')
        raise Aborted(message)

    def print_routing(self):
        """Startup check of where each colour goes and which target tags the detector actually reports: a mismatch (a stale config,
        or object_detector started with its default tags) is shown here instead of silently sending objects to the wrong tag."""
        det = self.fresh_detections(timeout=3.0)
        seen = {t['name']: t['id'] for t in (det['targets'] if det else [])}
        with self.lock:
            for i, t in self.known_targets.items():
                seen.setdefault(t['name'], i)
        print('  🗺️  Routing (colour → target tag):')
        routed = set()
        for colour in self.colors:
            name = self.color_targets.get(colour)
            if name is None:
                names = ', '.join(f'{i} "{n}"' for n, i in seen.items()) or 'none seen yet'
                print(f'        {colour:6s} → ANY target tag (picked at random among: {names})')
                routed.update(seen)
            elif name in seen:
                print(f'        {colour:6s} → "{name}"  ✅ tag {seen[name]}')
                routed.add(name)
            else:
                print(f'        {colour:6s} → "{name}"  ⚠️ not reported by the detector (it reports: '
                      f'{", ".join(f"{i} {n!r}" for n, i in seen.items()) or "no target tags"})')
        ignored = [c for c in ('red', 'green', 'blue') if c not in self.colors]
        if ignored:
            print(f'        {", ".join(ignored)} → ignored (not in `colors`)')
        extra = [f'{i} "{n}"' for n, i in seen.items() if n not in routed]
        if extra:
            print(f'  ⚠️  The detector also reports {", ".join(extra)}: nothing is routed there. If a colour should go to a tag listed '
                  f'here, the config (`color_targets`) or the detector launch (`targets:=`) is out of date.')
        if det is None:
            print('  ⚠️  No detections yet (is object_detector running?).')
        elif not det.get('calibrated', True):
            print('  ⚠️  The camera is not calibrated: positions are unusable. Run calibrate_camera first.')

    def failures(self, cube):
        """Failed picks so far of this object (same colour, within 5 cm of where it lies now)."""
        return sum(1 for label, x, y in self.fail_log
                   if label == cube['label'] and np.hypot(x - cube['xyz'][0], y - cube['xyz'][1]) < 0.05)

    def note_outcome(self, cube, success, target_id):
        """Bookkeeping after an executed episode: a success moves that target's next drop point on, a failure counts against the object."""
        if success:
            self.placed += 1
            self.placed_at[target_id] = self.placed_at.get(target_id, 0) + 1
        else:
            self.fail_log.append((cube['label'], float(cube['xyz'][0]), float(cube['xyz'][1])))
        if self.until_clear and self.initial_count:
            print(f'  📈  {self.placed} of {self.initial_count} placed, {self.attempts} attempt(s), {len(self.fail_log)} failed'
                  + ('' if success else ' (will retry)'))

    # ------------------------------------------------------------------ executing a plan
    def run_plan(self, plan, watch=False):
        """Executes the plan. Returns (gripper position after closing in m, backed_out). If the grasp check finds nothing in
        the gripper after closing, the arm lifts, opens and returns to the start pose instead of carrying nothing to the
        target (backed_out True)."""
        grasp_m = None
        for group in D.plan_groups(plan):
            if group['kind'] == 'move':
                self.execute(group['times'], group['joints'])
            else:
                if group['name'] == 'close':
                    self.report_reach(plan)
                measured = self.gripper(group['gripper'], group['duration'])
                if group['name'] == 'close':
                    grasp_m = measured
                    if self.p('check_grasp') and not self.holding(measured):
                        print(f'  ✋  nothing in the gripper after closing ({measured} m): backing out')
                        self.back_out(plan)
                        return grasp_m, True
            if watch:
                for topic in self.monitor.stale(self.p('stale_after')):
                    print(f'  ⚠️  {topic} stopped during the episode.')
        return grasp_m, False

    def report_reach(self, plan):
        """Diagnostic, printed just before the gripper closes: where the fingertip really is (forward kinematics of the measured
        joints) against the planned grasp point. z far above plan = the arm has not arrived (lag, load); z on plan but the fingers
        hover above the table = table_z or the fingertip model is off."""
        q = self.current_joints()
        if q is None:
            return
        x, y, z, _ = D.forward_kinematics(q)
        w = plan.waypoints['grasp']
        print(f'  📏  before closing: fingertip z {z * 1000:.1f} mm (planned {w[2] * 1000:.1f}, table_z {self.cfg.table_z * 1000:.1f}), '
              f'xy error {np.hypot(x - w[0], y - w[1]) * 1000:.1f} mm, z error {(z - w[2]) * 1000:+.1f} mm')

    def holding(self, measured_m):
        """Something is between the fingers: the gripper stopped before its fully closed end (either joint polarity)."""
        return D.gripper_holding(measured_m, self.p('grasp_closed_threshold'), GRIPPER_OPEN_M, GRIPPER_CLOSE_M)

    def back_out(self, plan):
        """After a failed grasp: straight up, open the gripper, back to the start pose."""
        q = self.current_joints()
        lift = D.solve(plan.waypoints['lift'], near=q) if q is not None else None
        if lift is not None:
            times, joints = D.joint_segment(q, lift, self.cfg.max_joint_vel, self.cfg)
            self.execute(times, joints)
        self.gripper(0.0, 0.4)
        self.move_to(plan.start)

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
              f'{target["name"]} [tag {target["id"]}] ({target_xy[0]:.3f}, {target_xy[1]:.3f}), pitch {np.degrees(plan.pick_pitch):.0f}°, '
              f'{plan.duration:.1f} s' + ('  [target tag covered: using its last seen position]' if target.get('cached') else ''))
        print('        ' + height_report(cube, target, plan, self.cfg))
        if self.p('confirm') and not self.confirmed(plan):
            return 'skipped'

        self.gripper(0.0, 0.3)
        self.move_to(plan.start)
        self.ensure_gripper_open()
        self.attempts += 1  # counted only once the episode can really start (a closed gripper is not a failed pick)
        problems = self.preflight()
        if problems and self.record:
            raise Aborted('not ready: ' + '; '.join(problems))
        if not self.record:
            return self.trial(plan, cube, target_xy, target['id'])

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
            grasp_m, _ = self.run_plan(plan, watch=True)
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
            held = self.holding(grasp_m)
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
        self.note_outcome(cube, success, target['id'])

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

    def trial(self, plan, cube, target_xy, target_id):
        """The same motion and checks without recording (first runs on the real arm, tuning grasp_closed_threshold)."""
        grasp_m, _ = self.run_plan(plan)
        held = self.holding(grasp_m)
        after = self.fresh_detections(newer_than=time.monotonic() + 0.3, timeout=4.0)
        place = D.verify_place(cube['label'], cube['xyz'][:2], target_xy, [o for o in after['objects'] if o.get('xyz')],
                               self.p('target_radius')) if after else (False, 'no detections')
        print(f'  🧪  trial (not recorded): gripper after closing {grasp_m if grasp_m is None else round(grasp_m, 4)} m '
              f'-> {"holding" if held else "nothing held"} (threshold {self.p("grasp_closed_threshold")}); '
              f'place: {"ok" if place[0] else "failed"} ({place[1]})')
        self.session['trials'] += 1
        self.note_outcome(cube, (held or not self.p('check_grasp')) and (place[0] or not self.p('check_place')), target_id)
        return 'trial'

    def print_summary(self):
        s = self.session
        if not self.record:
            print(f'  📊  {s["trials"]} trial episode(s) run (nothing recorded), {s["skipped"]} skipped')
            if self.until_clear and self.initial_count:
                print(f'  📊  {self.placed} of {self.initial_count} object(s) placed in {self.attempts} attempt(s), '
                      f'{len(self.fail_log)} failed pick(s)')
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


def main(args=None, node_class=None, title='Scripted Pick-and-Place Demonstrations'):
    """Run loop shared with guided_demo (node_class = a ScriptedDemo subclass)."""
    rclpy.init(args=args)
    node = (node_class or ScriptedDemo)()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    print()
    print('=' * 66)
    print(f'  🤖  OpenManipulator-X  |  {title}')
    print('=' * 66)
    print(f'  Saving to: {node.data_dir}' if node.record else '  🧪  TRIAL MODE: episodes are executed but NOT recorded')
    print(f'  Prompt: "{node.template}"  colours {node.colors}, speed caps j1..j4 {list(node.cfg.max_joint_vel)} rad/s')
    print(f'  Scene: table_z {node.cfg.table_z * 1000:.1f} mm, cube_size {node.cfg.cube_size * 1000:.1f} mm (grasp height = table_z + cube_size/2)')
    print(f'  Checks: grasp {node.p("check_grasp")}, place {node.p("check_place")}. Topics: {", ".join(node.topics)}')
    print('  p = pause after this episode, ENTER = continue, q = quit after this episode, Ctrl+C = abort now')
    print('=' * 66)
    state = {'quit': False, 'pause': False, 'event': threading.Event()}
    threading.Thread(target=keyboard, args=(state, node), daemon=True).start()
    target = int(node.p('episodes'))
    try:
        time.sleep(1.5)  # subscriptions fill
        node.monitor.print_status(node.p('stale_after'))
        node.print_routing()
        saved = 0
        while not state['quit'] and (target <= 0 or saved < target):
            if state['pause']:
                node.print_summary()
                print('  ⏸️  Paused. ENTER = continue, q = quit.')
                state['event'].clear()
                state['event'].wait()
                continue
            if node.until_clear and node.initial_count and node.attempts >= node.initial_count * node.max_attempts:
                print(f'  🏁  Stopping: {node.attempts} attempts used (objects × {node.max_attempts}).')
                break
            try:
                if node.run_episode() in ('saved', 'trial'):
                    saved += 1
                if node.quit_requested:
                    state['quit'] = True
            except Finished as e:
                print(f'  🏁  {e}')
                if node.until_clear:
                    break
                node.session['skipped'] += 1
                node.hold()
                print('  ⏸️  Rearrange the objects, then ENTER = continue, q = quit.')
                state['pause'] = True
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
