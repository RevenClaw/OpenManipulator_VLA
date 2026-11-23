#!/usr/bin/env python3
#
# move_test.py
#
# Tests the MOVING half of the YOLO pick-and-place pipeline without the camera: you type the coordinates the camera would
# have supplied, and they go through exactly the same code as scripted_demo / guided_demo (demo_planning IK + straight-line
# segments + speed caps -> FollowJointTrajectory on arm_controller, gripper -> GripperCommand). No MoveIt, no camera, no YOLO.
#
# All coordinates: robot base frame (world = link1), metres, x forward, y left, z up; z is the FINGERTIP point
# (end_effector_link, the point the demos put at the cube centre). Pitch in degrees, 90 = pointing down.
#
# Commands:
#   x y z [pitch] [o|c]    move the fingertip there (pitch optional: steepest reachable, as the planner does), then
#                          optionally open/close. Far moves: joint move to `approach_height` above the point, then a
#                          straight line down (like approach + descend); moves at the same x, y: straight line only.
#   o / c                  open / close the gripper (prints the measured position)
#   pp cx cy bx by         a complete planned pick-and-place: cube at (cx, cy), bin/marker at (bx, by), exactly
#                          plan_pick_place as in the demos (heights from table_z, cube_size, ...), shown first, ENTER = run
#   w                      where am I: fingertip x y z pitch (forward kinematics of the measured joints) + gripper
#   r                      back to READY
#   h                      help;   q = quit;   Ctrl+C during a move = stop the arm where it is
#
# Safety: targets with the fingertip lower than table_z + min_z_above_table are refused, as are moves whose path dips
# below that (every sample is checked with forward kinematics).
#
# Usage (arm launch running; nothing else needed):
#   ros2 run custom_package move_test --ros-args --params-file \
#       $(ros2 pkg prefix custom_package)/share/custom_package/config/move_test.yaml
#

import math
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor

from custom_package import demo_planning as D
from custom_package import scripted_demo as S

HELP = """  Commands (metres, base frame, z = fingertip; pitch in degrees, 90 = down):
    x y z [pitch] [o|c]   move there, then optionally open (o) / close (c)      e.g.  0.20 0.05 -0.03 90 c
    o | c                 open | close the gripper
    pp cx cy bx by        full planned pick-and-place: cube (cx, cy) -> bin (bx, by), as the demos do it
    w                     where am I (fingertip pose from the joint encoders)
    r                     back to READY            h = help, q = quit, Ctrl+C = stop the arm"""


# ------------------------------------------------------------------------------------------- pure helpers (unit-tested)
def parse_move(line):
    """'x y z [pitch] [o|c]' -> (x, y, z, pitch_deg or None, 'o'|'c'|None), or None if malformed."""
    tokens = line.split()
    grip = None
    if tokens and tokens[-1] in ('o', 'c'):
        grip = tokens.pop()
    if len(tokens) not in (3, 4):
        return None
    try:
        values = [float(t) for t in tokens]
    except ValueError:
        return None
    if not all(math.isfinite(v) for v in values):
        return None
    return values[0], values[1], values[2], (values[3] if len(values) == 4 else None), grip


def path_min_z(joints):
    """Lowest fingertip z (forward kinematics) over a joint trajectory."""
    return min(D.forward_kinematics(q)[2] for q in joints)


class MoveTest(S.ScriptedDemo):

    def __init__(self):
        super().__init__('move_test')
        self.declare_parameter('approach_height', 0.05)     # m: far moves arrive this much above the point, then go straight down
        self.declare_parameter('min_z_above_table', 0.0)    # m: the fingertip may not go below table_z + this
        self.declare_parameter('same_xy_radius', 0.01)      # m: closer than this to the current x, y = straight line only
        self.declare_parameter('settle', 0.5)               # s to wait after a move before measuring where the arm is
        self.record = False

    def floor(self):
        return self.cfg.table_z + float(self.p('min_z_above_table'))

    def pose(self):
        q = self.current_joints()
        if q is None:
            raise S.Aborted('no fresh /joint_states (is the arm launch running?)')
        return q, D.forward_kinematics(q)

    def where(self, label='  📍  now'):
        q, (x, y, z, pitch) = self.pose()
        with self.lock:
            g = self.gripper_m
        print(f'{label}: x {x:.3f}  y {y:+.3f}  z {z:+.3f}  pitch {math.degrees(pitch):.0f}°   gripper '
              f'{"?" if g is None else f"{g:.4f} m"}   (table_z {self.cfg.table_z:+.3f})')

    # ------------------------------------------------------------------ one fingertip goal
    def plan_move(self, x, y, z, pitch_deg):
        """(times, joints, description) from the current joints to the fingertip at (x, y, z), or raises Aborted."""
        if z < self.floor():
            raise S.Aborted(f'z {z:+.3f} is below the floor {self.floor():+.3f} (table_z + min_z_above_table)')
        q0, (cx, cy, cz, cpitch) = self.pose()
        same_xy = math.hypot(x - cx, y - cy) < float(self.p('same_xy_radius'))
        above = (x, y, max(z + float(self.p('approach_height')), z))
        points = [(x, y, z)] if same_xy else [above, (x, y, z)]
        if pitch_deg is None:
            pitch = D.choose_pitch(points, self.cfg)
            if pitch is None:
                raise S.Aborted(f'({x:.3f}, {y:.3f}, {z:+.3f}) not reachable at pitch '
                                f'{self.cfg.min_pitch_deg:g}..{self.cfg.max_pitch_deg:g}°')
        else:
            pitch = math.radians(pitch_deg)
            if any(D.solve((*p, pitch)) is None for p in points):
                raise S.Aborted(f'({x:.3f}, {y:.3f}, {z:+.3f}) not reachable at pitch {pitch_deg:g}°')
        vel = self.cfg.max_joint_vel
        if same_xy:
            seg = D.cartesian_segment((cx, cy, cz), (x, y, z), pitch, q0, vel, self.cfg)
            if seg is None:  # e.g. the pitch changes too: no straight line, a joint move instead
                q1 = D.solve((x, y, z, pitch), near=q0)
                seg = D.joint_segment(q0, q1, vel, self.cfg)
                how = 'joint move'
            else:
                how = 'straight line'
            times, joints = seg
        else:
            q_above = D.solve((*above, pitch), near=q0)
            t1, j1 = D.joint_segment(q0, q_above, vel, self.cfg)
            seg = D.cartesian_segment(above, (x, y, z), pitch, j1[-1], vel, self.cfg)
            if seg is None:
                raise S.Aborted('straight line down from the approach point not reachable')
            t2, j2 = seg
            times = np.concatenate([t1, t1[-1] + t2[1:]])
            joints = np.vstack([j1, j2[1:]])
            how = f'joint move to {float(self.p("approach_height")) * 100:.0f} cm above, then straight down'
        lowest = path_min_z(joints)
        if lowest < self.floor() - 0.002:
            raise S.Aborted(f'the path dips to z {lowest:+.3f}, below the floor {self.floor():+.3f}: go up first')
        return times, joints, f'{how}, pitch {math.degrees(pitch):.0f}°, {times[-1]:.1f} s'

    def go(self, x, y, z, pitch_deg, grip):
        times, joints, how = self.plan_move(x, y, z, pitch_deg)
        print(f'  ➡️   target x {x:.3f}  y {y:+.3f}  z {z:+.3f}: {how}')
        self.execute(times, joints)
        time.sleep(float(self.p('settle')))
        _, (ax, ay, az, apitch) = self.pose()
        print(f'  📏  reached x {ax:.3f}  y {ay:+.3f}  z {az:+.3f}  pitch {math.degrees(apitch):.0f}°   error '
              f'xy {math.hypot(ax - x, ay - y) * 1000:.1f} mm, z {(az - z) * 1000:+.1f} mm  '
              f'(fingertip {(az - self.cfg.table_z) * 1000:.0f} mm above table_z)')
        if grip:
            self.grip(grip)

    def grip(self, which):
        measured = self.gripper(1.0 if which == 'c' else 0.0, 1.2)
        state = ''
        if which == 'c' and measured is not None:
            state = '  (holding something)' if self.holding(measured) else '  (closed on nothing)'
        print(f'  ✋  gripper {"closed" if which == "c" else "opened"}: {measured if measured is None else f"{measured:.4f} m"}{state}')

    # ------------------------------------------------------------------ full planned pick-and-place
    def pick_place(self, cube_xy, bin_xy):
        try:
            plan = D.plan_pick_place(cube_xy, bin_xy, self.cfg)
        except D.PlanningError as e:
            raise S.Aborted(f'cannot plan: {e}')
        for name, w in plan.waypoints.items():
            print(f'        {name:13s} x {w[0]:.3f}  y {w[1]:+.3f}  z {w[2]:+.3f}  pitch {np.degrees(w[3]):.0f}°')
        print(f'        {plan.duration:.1f} s; heights: table_z {self.cfg.table_z:+.3f} + cube_size/2 '
              f'{self.cfg.cube_size / 2:.4f} (grasp), + bin_height {self.cfg.bin_height:.3f} + release_clearance '
              f'{self.cfg.release_clearance:.3f} + far-cube extra {D.release_extra(cube_xy, self.cfg):.3f} (release)')
        if input('  ❓  ENTER = run it (READY first), anything else = cancel: ').strip():
            print('  ↩️   cancelled')
            return
        self.gripper(0.0, 0.3)
        self.move_to(plan.start)
        grasp_m, backed_out = self.run_plan(plan)
        print('  🏁  ' + ('backed out: nothing in the gripper after closing' if backed_out else
                         f'done (gripper after closing {grasp_m if grasp_m is None else f"{grasp_m:.4f} m"})'))

    # ------------------------------------------------------------------ terminal loop
    def run_command(self, line):
        words = line.split()
        if line in ('o', 'c'):
            self.grip(line)
        elif line == 'w':
            self.where()
        elif line == 'r':
            print('  ➡️   READY')
            self.move_to(D.READY)
            self.where()
        elif line in ('h', '?', 'help'):
            print(HELP)
        elif words and words[0] == 'pp':
            try:
                cx, cy, bx, by = (float(v) for v in words[1:])
            except ValueError:
                print('  ❓  pp cx cy bx by   e.g.  pp 0.20 0.05 0.14 -0.15')
                return
            self.pick_place((cx, cy), (bx, by))
        else:
            move = parse_move(line)
            if move is None:
                print('  ❓  not understood (h = help)')
                return
            self.go(*move)


def main(args=None):
    rclpy.init(args=args)
    node = MoveTest()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    print()
    print('=' * 66)
    print('  🤖  OpenManipulator-X  |  Move test (the demos\' motion pipeline, no camera)')
    print('=' * 66)
    print(f'  table_z {node.cfg.table_z:+.3f} m (floor {node.floor():+.3f}), cube_size {node.cfg.cube_size:.3f}, '
          f'speed caps j1..j4 {list(node.cfg.max_joint_vel)} rad/s')
    print(HELP)
    print('=' * 66)
    try:
        time.sleep(1.0)  # /joint_states fills
        try:
            node.where()
        except S.Aborted as e:
            print(f'  ⚠️  {e}')
        while True:
            try:
                line = input('move> ').strip().lower()
            except EOFError:
                break
            if line == 'q':
                break
            if not line:
                continue
            try:
                node.run_command(line)
            except S.Aborted as e:
                print(f'  ⚠️  {e}')
                node.hold()
            except KeyboardInterrupt:
                print('\n  🛑  stopped: the arm holds where it is.')
                node.abort.set()
                node.hold()
                time.sleep(0.2)
                node.abort.clear()
    except KeyboardInterrupt:
        node.abort.set()
        node.hold()
    finally:
        print('  👋  bye (the arm stays where it is)')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
