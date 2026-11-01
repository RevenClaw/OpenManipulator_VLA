#!/usr/bin/env python3
#
# vla_bridge.py
#
# Streams action chunks from a pi0.5 policy server (vla_layer, openpi websocket protocol) to the OpenManipulator-X,
# behind a safety layer. The only path by which a learned policy can move the arm.
#
#   1. Builds observations: /joint_states (joint1..4 rad + gripper normalized 0 open .. 1 closed),
#      main + wrist camera images (RGB, resize_with_pad to 224x224), prompt. A view whose topic is '' (no camera yet)
#      gets black frames; fake_images:=true makes both views black. The exact 224x224 images sent are republished on
#      ~/model_input/<front|wrist> for checking (rqt_image_view).
#   2. A worker thread sends them to the server and receives (10, 5) chunks: 10 steps of [j1..j4, gripper].
#      The next chunk is requested before the current one runs out, so the arm never waits.
#   3. Each chunk passes the safety layer (custom_package/vla_safety.py): NaN rejection, stale-step dropping,
#      joint limits - margin, speed cap from the measured position, start-jump check, fingertip height >= min_z.
#   4. The safe steps go to arm_controller as ONE multi-point JointTrajectory; the gripper goes through
#      gripper_controller's GripperCommand action with hysteresis, only when its state changes.
#
# Starts DISABLED. Any fault (stale joint states or images, no chunk, bad chunk, lost server, e-stop) holds the arm and
# disables; only a new enable resumes.
# shadow:=true runs everything but publishes only to ~/preview (+ RViz path marker): the arm does not move.
#
# Requires: the hardware launch with the arm homed, and a policy server
#   (vla_layer: scripts/fake_policy_server.py for testing, scripts/serve.py for the real model).
#   openpi-client + typing_extensions (installed by docker/Dockerfile).
#
# Usage:
#   ros2 launch custom_package vla_bridge.launch.py server_host:=<server-ip> shadow:=true
#   terminal: e = enable, d = disable (hold), s = status, q = quit
#   ros2 service call /vla_bridge/enable std_srvs/srv/SetBool "{data: true}"
#   ros2 topic pub --once /vla_bridge/estop std_msgs/msg/Bool "{data: true}"
#

import json
import sys
import threading
import time

from builtin_interfaces.msg import Duration
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Point
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import Marker

from custom_package import vla_safety as S
from custom_package.ee_commander import forward_kinematics
from custom_package.joint_commander import JOINT_LIMITS

try:
    from openpi_client import image_tools
    from openpi_client import websocket_client_policy
except ImportError:  # reported in main() with the install command
    image_tools = websocket_client_policy = None

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4']
GRIPPER_JOINT = 'gripper_left_joint'
LOWER = [JOINT_LIMITS[j][0] for j in ARM_JOINTS]
UPPER = [JOINT_LIMITS[j][1] for j in ARM_JOINTS]


def _duration(seconds):
    seconds = max(0.0, float(seconds))
    return Duration(sec=int(seconds), nanosec=int((seconds % 1.0) * 1e9))


class VLABridgeNode(Node):

    def __init__(self):
        super().__init__('vla_bridge')

        self.declare_parameter('server_host', 'localhost')
        self.declare_parameter('server_port', 4000)
        self.declare_parameter('prompt', '')
        self.declare_parameter('fps', 15.0)                    # must match the training data
        self.declare_parameter('max_joint_vel', 0.3)           # rad/s, speed cap per joint (joint2 overload, 0x20)
        self.declare_parameter('limit_margin', 0.05)           # rad kept clear of JOINT_LIMITS
        self.declare_parameter('max_start_jump', 0.15)         # rad, chunk rejected if it starts farther away
        self.declare_parameter('min_z', 0.02)                  # m, fingertip height floor (forward kinematics)
        self.declare_parameter('gripper_close_above', 0.6)
        self.declare_parameter('gripper_open_below', 0.4)
        self.declare_parameter('gripper_max_effort', 1.0)
        self.declare_parameter('joint_state_timeout', 0.1)     # s
        self.declare_parameter('image_timeout', 0.2)           # s
        self.declare_parameter('chunk_timeout', 1.0)           # s without a new chunk while enabled
        self.declare_parameter('front_image_topic', '/camera/front/color/image_raw')  # RealSense, camera_front.launch.py
        self.declare_parameter('wrist_image_topic', '')        # '' = no wrist camera yet: black frames for that view
        self.declare_parameter('fake_images', False)           # True: black frames for both views, no cameras needed
        self.declare_parameter('image_reliable_qos', True)     # False only for a camera that publishes best effort
        self.declare_parameter('shadow', False)                # publish to ~/preview only; the arm does not move

        p = self.get_parameter
        self.host = p('server_host').value
        self.port = int(p('server_port').value)
        self.prompt = p('prompt').value
        self.fps = float(p('fps').value)
        self.max_vel = float(p('max_joint_vel').value)
        self.margin = float(p('limit_margin').value)
        self.max_start_jump = float(p('max_start_jump').value)
        self.min_z = float(p('min_z').value)
        self.gripper_max_effort = float(p('gripper_max_effort').value)
        self.joint_state_timeout = float(p('joint_state_timeout').value)
        self.image_timeout = float(p('image_timeout').value)
        self.chunk_timeout = float(p('chunk_timeout').value)
        self.min_lead = 0.5 / self.fps  # skip chunk steps due sooner than half a period (see vla_safety.first_future_step)
        self.fake_images = bool(p('fake_images').value)
        topics = {'front': p('front_image_topic').value, 'wrist': p('wrist_image_topic').value}
        self.cameras = {} if self.fake_images else {n: t for n, t in topics.items() if t}  # views with a real camera
        self.shadow = bool(p('shadow').value)
        self.hysteresis = S.GripperHysteresis(
            close_above=float(p('gripper_close_above').value), open_below=float(p('gripper_open_below').value)
        )

        self.lock = threading.Lock()            # sensor data shared with the worker thread
        # Serializes everything that can command the arm: the control tick and the service/topic callbacks run on the
        # executor thread, the e/d terminal commands on the main thread. Without it a disable could land between a tick's
        # checks and its publish, and the arm would keep moving after the hold.
        self.control_lock = threading.RLock()
        self.joints = None              # np (4,) rad
        self.gripper_norm = None        # 0 open .. 1 closed
        self.joints_time = 0.0          # monotonic time of the last /joint_states
        self.max_effort = 0.0           # largest |effort| seen on joint1..4 (logged, not yet acted on)
        self.images = {'front': None, 'wrist': None}        # latest sensor_msgs/Image
        self.image_times = {'front': 0.0, 'wrist': 0.0}

        self.enabled = False
        self.estop_latched = False
        self.warmed_up = False
        self.connected = False
        self.worker_fault = ''          # set by the worker (lost server, bad observation); the control loop faults on it
        self.client = None              # the worker's WebsocketClientPolicy (closed to abort a stalled request)
        self.plan_times = None          # monotonic times of the steps sent to arm_controller
        self.plan_grip = None
        self.new_chunk = None           # (t_obs, chunk) from the worker
        self.inflight = False
        self.latency = 0.2              # s, running estimate of the round trip
        self.last_chunk_time = 0.0
        self.counters = {'chunks': 0, 'holds': 0, 'stale_chunks': 0}
        self.last_fault = ''
        self.request_event = threading.Event()
        self.running = True

        cmd_topic = '~/preview' if self.shadow else '/arm_controller/joint_trajectory'
        self.traj_pub = self.create_publisher(JointTrajectory, cmd_topic, 10)
        self.marker_pub = self.create_publisher(Marker, '~/preview_path', 10)
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.gripper_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')

        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.create_subscription(Bool, '~/estop', self.estop_callback, 10)
        # Reliable, latest frame only: a 640x480 frame is ~900 KB, split into many packets; best effort loses the whole
        # frame when one packet drops (measured: 9 Hz with 0.46 s gaps from a 15 Hz D435, vs 14.9 Hz reliable).
        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE) if p('image_reliable_qos').value \
            else qos_profile_sensor_data
        for name, topic in self.cameras.items():
            self.create_subscription(Image, topic, lambda msg, n=name: self.image_callback(n, msg), image_qos)
        self.model_input_pubs = {n: self.create_publisher(Image, f'~/model_input/{n}', 1) for n in self.cameras}
        self.create_service(SetBool, '~/enable', self.enable_callback)
        self.create_timer(1.0 / self.fps, self.control_tick)
        self.create_timer(0.5, self.publish_status)

        if not self.prompt:
            self.get_logger().warn('prompt is empty: fine for the fake policy server, wrong for a trained model.')
        mode = 'SHADOW (arm will not move)' if self.shadow else 'LIVE'
        self.get_logger().info(
            f'vla_bridge {mode}: server ws://{self.host}:{self.port}, {self.fps:.0f} Hz, '
            f'max {self.max_vel:.2f} rad/s, cameras: {self.camera_summary()}. Starts DISABLED.'
        )

    def camera_summary(self):
        return ', '.join(f'{n}={self.cameras.get(n, "black frames")}' for n in ('front', 'wrist'))

    def stale_cameras(self, now, image_times):
        return [n for n in self.cameras if now - image_times[n] > self.image_timeout]

    # ------------------------------------------------------------------ inputs
    def joint_state_callback(self, msg: JointState):
        names = list(msg.name)
        if not set(ARM_JOINTS + [GRIPPER_JOINT]).issubset(names):
            return
        joints = np.array([msg.position[names.index(j)] for j in ARM_JOINTS])
        gripper = S.gripper_metres_to_norm(msg.position[names.index(GRIPPER_JOINT)])
        effort = 0.0
        if len(msg.effort) == len(names):  # Present Current, logged for future overload protection
            effort = max(abs(msg.effort[names.index(j)]) for j in ARM_JOINTS)
        with self.lock:
            self.joints, self.gripper_norm = joints, gripper
            self.joints_time = time.monotonic()
            self.max_effort = max(self.max_effort, effort)

    def image_callback(self, name, msg: Image):
        with self.lock:
            self.images[name] = msg
            self.image_times[name] = time.monotonic()

    def estop_callback(self, msg: Bool):
        with self.control_lock:
            self._estop(msg.data)

    def _estop(self, engaged):
        if engaged:
            self.estop_latched = True
            if self.enabled:
                self.fault('e-stop')
            print('  🛑  E-STOP latched. Publish {data: false} on /vla_bridge/estop, then enable again.')
        elif self.estop_latched:
            self.estop_latched = False
            print('  ✅  E-stop released (still disabled).')

    def enable_callback(self, request, response):
        response.success, response.message = self.set_enabled(request.data)
        return response

    def set_enabled(self, enable):
        with self.control_lock:
            return self._set_enabled(enable)

    def _set_enabled(self, enable):
        now = time.monotonic()
        if not enable:
            if self.enabled:
                self.hold('disabled by operator')
            self.enabled = False
            return True, 'disabled'
        reason = None
        if self.estop_latched:
            reason = 'e-stop is latched'
        elif not self.connected or not self.warmed_up:
            reason = 'policy server not connected / warmed up yet'
        elif now - self.joints_time > self.joint_state_timeout:
            reason = 'no fresh /joint_states'
        elif self.stale_cameras(now, self.image_times):
            reason = f'no fresh camera images ({", ".join(self.stale_cameras(now, self.image_times))})'
        if reason:
            print(f'  ❌  Not enabled: {reason}.')
            return False, reason
        with self.lock:
            self.hysteresis.closed = self.gripper_norm > 0.5
            self.plan_times = self.plan_grip = None
            self.new_chunk = None
            self.inflight = True  # the first request below; the control loop must not send a second one
        self.worker_fault = ''
        self.last_chunk_time = now
        self.enabled = True
        self.request_event.set()
        print(f'  ✅  ENABLED ({"shadow" if self.shadow else "live"}).')
        return True, 'enabled'

    # ------------------------------------------------------------------ inference worker
    def observation(self):
        """Snapshot of the current observation in the policy's format, and its monotonic time."""
        with self.lock:
            joints, gripper = self.joints.copy(), self.gripper_norm
            images = dict(self.images)
            t_obs = time.monotonic()
        obs = {S.OBS_STATE: np.append(joints, gripper).astype(np.float32), S.PROMPT: self.prompt}
        for name, key in (('front', S.OBS_FRONT), ('wrist', S.OBS_WRIST)):
            if name not in self.cameras:
                obs[key] = np.zeros((S.IMAGE_SIZE, S.IMAGE_SIZE, 3), dtype=np.uint8)
                continue
            msg = images[name]
            rgb = S.ros_image_to_rgb(msg.encoding, msg.height, msg.width, msg.step, msg.data)
            # Same preprocessing as the training dataset (vla_layer plan D8).
            obs[key] = image_tools.convert_to_uint8(image_tools.resize_with_pad(rgb, S.IMAGE_SIZE, S.IMAGE_SIZE))
            self.publish_model_input(name, obs[key], msg.header)
        return obs, t_obs

    def publish_model_input(self, name, image, header):
        """Republishes the exact image sent to the policy (224x224 RGB), stamped like the camera frame it came from."""
        h, w = image.shape[:2]
        self.model_input_pubs[name].publish(
            Image(header=header, height=h, width=w, encoding='rgb8', is_bigendian=0, step=w * 3, data=image.tobytes())
        )

    def worker(self):
        client = None
        waiting_printed = False
        while self.running:
            if client is None:
                self.connected = self.warmed_up = False
                print(f'  🔌  Connecting to ws://{self.host}:{self.port} ...')
                try:
                    client = websocket_client_policy.WebsocketClientPolicy(host=self.host, port=self.port)
                except Exception as e:  # noqa: BLE001 - keep retrying, the bridge stays disabled meanwhile
                    print(f'  ❌  Connection failed: {e}. Retrying in 2 s.')
                    time.sleep(2.0)
                    continue
                self.client = client
                self.connected = True
                print(f'  ✅  Connected. Server metadata: {client.get_server_metadata()}')
            if not self.warmed_up:
                missing = [n for n in self.cameras if self.images[n] is None]
                if self.joints is None or missing:
                    if missing and not waiting_printed:
                        print(f'  ⏳  Waiting for camera images: {", ".join(self.cameras[n] for n in missing)}')
                        waiting_printed = True
                    time.sleep(0.2)  # wait for sensors before the warmup call
                    continue
                print('  ⏳  Warmup inference (first call compiles the model, ~20 s)...')
            elif not self.request_event.wait(timeout=0.1):
                continue
            self.request_event.clear()
            try:
                obs, t_obs = self.observation()
            except Exception as e:  # noqa: BLE001 - e.g. an unsupported image encoding; the server is fine
                print(f'  ❌  Observation failed: {e}')
                self.worker_fault = f'observation failed: {e}'
                with self.lock:
                    self.inflight = False
                time.sleep(1.0)
                continue
            try:
                t0 = time.monotonic()
                chunk = client.infer(obs)['actions']
                latency = time.monotonic() - t0
            except Exception as e:  # noqa: BLE001 - connection lost, aborted after a stall, or server error
                print(f'  ❌  Inference failed: {e}')
                client = self.client = None
                self.worker_fault = 'policy server connection lost'  # never resume silently on a (possibly new) server
                with self.lock:
                    self.inflight = False
                continue
            with self.lock:
                self.inflight = False
                if not self.warmed_up:
                    self.warmed_up = True
                    print(f'  ✅  Warmup done ({latency:.1f} s), chunk shape {np.shape(chunk)}. Ready to enable (e).')
                    continue
                self.latency = 0.8 * self.latency + 0.2 * latency
                self.new_chunk = (t_obs, chunk)

    # ------------------------------------------------------------------ control loop (fps)
    def control_tick(self):
        with self.control_lock:
            self._control_tick()

    def _control_tick(self):
        if not self.enabled:
            return
        now = time.monotonic()
        with self.lock:
            joints, joints_time = self.joints, self.joints_time
            image_times = dict(self.image_times)
            new_chunk, self.new_chunk = self.new_chunk, None

        if self.worker_fault:
            return self.fault(self.worker_fault)
        if now - joints_time > self.joint_state_timeout:
            return self.fault(f'/joint_states stale ({now - joints_time:.2f} s)')
        stale = self.stale_cameras(now, image_times)
        if stale:
            return self.fault(f'camera images stale ({", ".join(stale)})')
        if new_chunk is not None and not self.apply_chunk(*new_chunk, joints, now):
            return
        if now - self.last_chunk_time > self.chunk_timeout:
            self.abort_inference()  # a stalled server would otherwise block the worker forever
            return self.fault(f'no chunk for {now - self.last_chunk_time:.2f} s')

        remaining = 0 if self.plan_times is None else int(np.sum(self.plan_times > now))
        with self.lock:
            if not self.inflight and remaining <= S.request_threshold_steps(self.latency, self.fps):
                self.inflight = True
                self.request_event.set()

        if self.plan_times is not None and not self.shadow:
            k = min(int(np.searchsorted(self.plan_times, now)), len(self.plan_grip) - 1)
            close = self.hysteresis.update(self.plan_grip[k])
            if close is not None:
                self.send_gripper(close)

    def apply_chunk(self, t_obs, chunk, joints, now):
        """Runs a chunk through the safety layer and sends it. Returns False if it caused a fault."""
        try:
            chunk = S.validate_chunk(chunk)
        except ValueError as e:
            self.fault(f'bad chunk: {e}')
            return False
        k0 = S.first_future_step(t_obs, self.fps, len(chunk), now, self.min_lead)
        if k0 >= len(chunk):
            self.counters['stale_chunks'] += 1  # arrived too late to use; the next one is already requested
            return True
        times = S.step_times(t_obs, self.fps, len(chunk))[k0:]
        targets, grip = chunk[k0:, :4], chunk[k0:, 4]
        jump = S.start_jump(joints, targets[0])
        if jump > self.max_start_jump:
            self.fault(f'chunk starts {jump:.2f} rad from the arm (max {self.max_start_jump})')
            return False
        targets = S.clamp_joints(targets, LOWER, UPPER, self.margin)
        targets = S.rate_limit(joints, targets, self.max_vel, np.diff(np.concatenate([[now], times])))
        targets = S.truncate_below_z(targets, forward_kinematics, self.min_z)
        if len(targets) == 0:
            self.fault(f'next step goes below min_z={self.min_z} m')
            return False
        times, grip = times[: len(targets)], grip[: len(targets)]
        self.publish_trajectory(targets, times - now)
        self.plan_times, self.plan_grip = times, grip
        self.last_chunk_time = now
        self.counters['chunks'] += 1
        return True

    # ------------------------------------------------------------------ outputs
    def publish_trajectory(self, targets, offsets):
        msg = JointTrajectory()
        msg.joint_names = ARM_JOINTS
        for q, dt in zip(targets, offsets):
            msg.points.append(JointTrajectoryPoint(positions=[float(v) for v in q], time_from_start=_duration(dt)))
        self.traj_pub.publish(msg)
        marker = Marker(type=Marker.LINE_STRIP, action=Marker.ADD, ns='vla_bridge')
        marker.header.frame_id = 'world'
        marker.scale.x = 0.004
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.1, 0.4, 1.0, 1.0
        marker.points = [Point(x=x, y=y, z=z) for x, y, z, _ in (forward_kinematics(q) for q in targets)]
        self.marker_pub.publish(marker)

    def send_gripper(self, close):
        if not self.gripper_client.server_is_ready():
            self.get_logger().warn('/gripper_controller/gripper_cmd not available.')
            return
        goal = GripperCommand.Goal()
        goal.command.position = S.GRIPPER_CLOSE_M if close else S.GRIPPER_OPEN_M
        goal.command.max_effort = self.gripper_max_effort
        self.gripper_client.send_goal_async(goal)
        print('  ✊  Gripper close.' if close else '  🖐️  Gripper open.')

    def hold(self, reason):
        """Stops the arm where it is: a one-point trajectory at the measured position."""
        with self.lock:
            joints = self.joints
        if joints is not None:
            self.publish_trajectory(joints[None], [0.2])
        self.plan_times = self.plan_grip = None
        self.counters['holds'] += 1
        self.last_fault = reason

    def abort_inference(self):
        """Closes the websocket so a request stuck waiting on the server fails; the worker then reconnects."""
        client = self.client
        if client is None:
            return

        def close():
            try:
                client._ws.close()  # noqa: SLF001 - openpi-client exposes no timeout or abort
            except Exception:  # noqa: BLE001 - already closed
                pass

        # The close handshake can wait up to 10 s on a hung server: never block the control loop (or e-stop) on it.
        threading.Thread(target=close, daemon=True).start()

    def fault(self, reason):
        self.hold(reason)
        self.enabled = False
        print(f'  ❌  HOLD + DISABLED: {reason}')

    def publish_status(self):
        now = time.monotonic()
        remaining = 0 if self.plan_times is None else int(np.sum(self.plan_times > now))
        status = {
            'enabled': self.enabled, 'shadow': self.shadow, 'estop': self.estop_latched,
            'connected': self.connected, 'warmed_up': self.warmed_up,
            'latency_ms': round(self.latency * 1000), 'steps_remaining': remaining,
            'max_effort': round(self.max_effort, 3), 'last_fault': self.last_fault, **self.counters,
            'image_age_ms': {n: round((now - self.image_times[n]) * 1000) for n in self.cameras},
        }
        self.status_pub.publish(String(data=json.dumps(status)))
        return status


def print_header(node):
    print()
    print('=' * 66)
    print('  🦾  OpenManipulator-X  |  VLA Bridge')
    print('=' * 66)
    print(f'  Server: ws://{node.host}:{node.port}   Mode: {"SHADOW (arm will not move)" if node.shadow else "LIVE"}')
    print(f'  Speed cap {node.max_vel:.2f} rad/s, {node.fps:.0f} Hz, prompt: "{node.prompt}"')
    print(f'  Cameras: {node.camera_summary()}')
    print('  e = enable,  d = disable (hold),  s = status,  q = quit')
    print('  Also: /vla_bridge/enable (SetBool), /vla_bridge/estop (Bool), /vla_bridge/status')
    print('  Starts DISABLED. Keep a hand on the hardware e-stop when live.')
    print('=' * 66)


def main(args=None):
    if websocket_client_policy is None:
        print('  ❌  openpi-client is not installed. In the container:')
        print('      pip install --break-system-packages typing_extensions "openpi-client @ git+https://github.com/'
              'Physical-Intelligence/openpi.git@215abfb#subdirectory=packages/openpi-client"')
        return
    rclpy.init(args=args)
    node = VLABridgeNode()

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    worker_thread = threading.Thread(target=node.worker, daemon=True)
    worker_thread.start()

    print_header(node)
    try:
        if not sys.stdin.isatty():
            # Started by ros2 launch: no keyboard. Control it with /vla_bridge/enable and /vla_bridge/estop.
            print('  ℹ️  No terminal input (launched). Use the /vla_bridge/enable service. Ctrl+C to stop.')
            try:
                while spin_thread.is_alive():
                    spin_thread.join(timeout=0.5)
            except KeyboardInterrupt:
                pass
        while sys.stdin.isatty():
            try:
                line = input('\n  >> ').strip().lower()
            except (EOFError, KeyboardInterrupt):
                print('\n  👋  Interrupted. Exiting...')
                break
            if line in ('q', 'quit', 'exit'):
                print('  👋  Exiting VLA bridge.')
                break
            if line == 'e':
                node.set_enabled(True)
            elif line == 'd':
                node.set_enabled(False)
                print('  ⏸️  Disabled (arm holds).')
            elif line == 's':
                print('  📊  ' + json.dumps(node.publish_status()))
            elif line:
                print('  ⚠️  Unknown command. e / d / s / q')
    finally:
        if node.enabled:
            try:
                node.set_enabled(False)  # final hold; the arm also stops on its own when the trajectory ends
            except Exception:  # noqa: BLE001 - Ctrl+C may already have shut the ROS context down
                pass
        node.running = False
        node.abort_inference()
        worker_thread.join(timeout=1.0)
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
