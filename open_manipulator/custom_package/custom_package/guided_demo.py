#!/usr/bin/env python3
#
# guided_demo.py
#
# Operator-guided pick-and-place demonstrations for fine-tuning: the same planning, execution, recording, checks and
# metadata as scripted_demo, but YOU choose every task instead of a random one:
#   1. lists every cube object_detector (YOLO) sees that the arm can reach: colour, position, confidence, and which bin it is
#      already in
#   2. asks which cube to pick, then which bin to place it in (bins = AprilTags, `bins` parameter; default
#      tag 25 = red bin, 20 = green bin, 18 = blue bin)
#   3. builds the prompt from your choice, e.g. "pick up the blue cube and place it in the red bin", shows the plan
#   4. ENTER: re-checks that the cube is still where it was, then runs (and records) the episode like scripted_demo
#
# Bin positions come from every AprilTag the detector sees (its `tags` output), so the detector's own `target_tags` don't
# matter here. A bin whose tag is covered keeps its last seen position.
#
# Keys at the prompts: number = choose, r = look again (refresh), b = back, q = quit. Ctrl+C = abort now (the arm holds,
# the current episode is deleted).
#
# Usage (arm launch + camera with depth + object_detector running, camera calibrated):
#   ros2 run custom_package guided_demo --ros-args --params-file \
#       $(ros2 pkg prefix custom_package)/share/custom_package/config/guided_demo.yaml
#   ... -p record:=false          # trial mode: run the motions, record nothing
#

import math
import queue
import time

import numpy as np

from custom_package import demo_planning as D
from custom_package import scripted_demo as S


class Quit(Exception):
    """The operator chose q at a prompt."""


# ------------------------------------------------------------------------------------------- pure helpers (unit-tested)
def parse_bins(entries):
    """["25:red bin", ...] -> [(25, "red bin"), ...] in the given order; malformed entries are skipped."""
    bins = []
    for entry in entries:
        tag, sep, name = str(entry).partition(':')
        if sep and tag.strip().lstrip('-').isdigit() and name.strip():
            bins.append((int(tag), name.strip()))
    return bins


def cube_in_bin(cube_xy, bins_xy, radius):
    """Name of the bin (name -> (x, y)) the cube lies within `radius` of, the closest one; None if none."""
    near = [(math.dist(cube_xy, xy), name) for name, xy in bins_xy.items() if math.dist(cube_xy, xy) <= radius]
    return min(near)[1] if near else None


def match_cube(cube, objects, max_dist=0.03):
    """The detection of the same cube in a newer frame: same colour, closest within max_dist (m) of its old position."""
    same = [o for o in objects if o.get('label') == cube['label'] and o.get('xyz')
            and math.dist(o['xyz'][:2], cube['xyz'][:2]) <= max_dist]
    return min(same, key=lambda o: math.dist(o['xyz'][:2], cube['xyz'][:2])) if same else None


def parse_choice(answer, count):
    """'2' -> index 1 if 1 <= 2 <= count, else None."""
    answer = answer.strip()
    if answer.isdigit() and 1 <= int(answer) <= count:
        return int(answer) - 1
    return None


# ------------------------------------------------------------------------------------------------------------- node
class GuidedDemo(S.ScriptedDemo):

    def __init__(self):
        super().__init__('guided_demo')
        self.declare_parameter('bins', ['25:red bin', '20:green bin', '18:blue bin'])  # "<AprilTag id>:<name in the prompt>"
        self.declare_parameter('recheck_max_move', 0.03)   # m: the chosen cube may have moved this much before the run
        self.bins = parse_bins(self.p('bins'))
        self.bin_ids = {tag for tag, _ in self.bins}
        self.bin_seen = {}  # tag id -> last seen world xyz (a placed cube can cover the tag)

    def detections_callback(self, msg):
        super().detections_callback(msg)
        with self.lock:
            tags = (self.detections or {}).get('tags', {})
            for tag_id, tag in tags.items():
                if int(tag_id) in self.bin_ids and tag.get('xyz'):
                    self.bin_seen[int(tag_id)] = tag['xyz']

    # ------------------------------------------------------------------ prompts
    def ask(self, text):
        """Prints text and waits for one keyboard line (routed here by scripted_demo's keyboard thread)."""
        print(text, flush=True)
        while not self.answers.empty():
            self.answers.get_nowait()
        self.awaiting = True
        try:
            while True:
                try:
                    return self.answers.get(timeout=0.2)
                except queue.Empty:
                    if self.abort.is_set():
                        raise S.Aborted('aborted')
        finally:
            self.awaiting = False

    def bin_positions(self, det):
        """[(tag id, name, xyz or None, status)] for the configured bins: visible now, last seen, or never seen."""
        tags = det.get('tags', {}) if det else {}
        out = []
        with self.lock:
            for tag, name in self.bins:
                now = tags.get(str(tag), {}).get('xyz')
                if now:
                    out.append((tag, name, now, 'visible'))
                elif tag in self.bin_seen:
                    out.append((tag, name, self.bin_seen[tag], 'last seen (tag covered?)'))
                else:
                    out.append((tag, name, None, 'NOT SEEN: put the tag in the camera view'))
        return out

    def print_scene(self, cubes, bins, ignored=()):
        bins_xy = {name: xyz[:2] for _, name, xyz, _ in bins if xyz}
        grasp_z = self.cfg.table_z + self.cfg.cube_size / 2               # what the plan aims for (± grasp_dz variation)
        release_z = self.cfg.table_z + self.cfg.bin_height + self.cfg.release_clearance
        print(f'\n  🧊  Cubes on the table (aim z = table_z {self.cfg.table_z:+.3f} + cube_size/2; the camera z is not used):')
        if not cubes:
            print('        (no reachable cube detected)')
        for i, c in enumerate(cubes, 1):
            inside = cube_in_bin(c['xyz'][:2], bins_xy, self.p('target_radius'))
            print(f'     {i:2d}  {c["label"]:6s} x {c["xyz"][0]:.3f}  y {c["xyz"][1]:+.3f}  z {c["xyz"][2]:+.3f} (camera, centre)'
                  f'  aim z {grasp_z:+.3f}  conf {c["confidence"]:.2f}'
                  + (f'   [already in the {inside}]' if inside else ''))
        shown = [(o, why) for o, why in ignored if why != 'colour not used']
        if shown:
            print('  🚫  Ignored detections (not cubes, or not pickable):')
            for o, why in shown:
                pos = (f'x {o["xyz"][0]:+.3f}  y {o["xyz"][1]:+.3f}  z {o["xyz"][2]:+.3f}' if o.get('xyz') else ' ' * 30)
                print(f'          {o["label"]:6s} {pos}  conf {o["confidence"]:.2f}   {why}')
        print(f'  🗑️   Bins (tag z = table surface as the camera sees it; drop aim z {release_z:+.3f}):')
        for i, (tag, name, xyz, status) in enumerate(bins, 1):
            pos = f'x {xyz[0]:.3f}  y {xyz[1]:+.3f}  z {xyz[2]:+.3f} (camera)' if xyz else ' ' * 37
            print(f'     {i:2d}  {name:12s} tag {tag:<3d} {pos}  {"✅" if status == "visible" else "⚠️ "} {status}')

    def print_routing(self):
        """Startup: which bins the camera sees (replaces scripted_demo's colour routing)."""
        det = self.fresh_detections(timeout=3.0)
        if det is None:
            print('  ⚠️  No detections yet (is object_detector running?).')
            return
        if not det.get('calibrated', True):
            print('  ⚠️  The camera is not calibrated: positions are unusable. Run calibrate_camera first.')
        print('  🗺️  Bins (AprilTag → name used in the prompt):')
        for tag, name, xyz, status in self.bin_positions(det):
            print(f'        tag {tag:<3d} → "{name}"  {"✅" if status == "visible" else "⚠️ "} {status}')

    # ------------------------------------------------------------------ the task, chosen by the operator
    def choose_task(self, det):
        while True:
            det = self.fresh_detections() or det
            cubes, ignored = self.screen(det)
            bins = self.bin_positions(det)
            self.print_scene(cubes, bins, ignored)
            if not cubes:
                if self.ask('  ❓  ENTER = look again, q = quit').strip() == 'q':
                    raise Quit()
                continue
            answer = self.ask(f'  ❓  Pick which cube? 1-{len(cubes)}, r = look again, q = quit').strip()
            if answer == 'q':
                raise Quit()
            i = parse_choice(answer, len(cubes))
            if i is None:
                continue
            cube = cubes[i]

            answer = self.ask(f'  ❓  Place the {cube["label"]} cube in which bin? 1-{len(bins)}, b = back, q = quit').strip()
            if answer == 'q':
                raise Quit()
            j = parse_choice(answer, len(bins))
            if j is None:
                continue
            tag, name, xyz, status = bins[j]
            if xyz is None:
                print(f'  ⚠️  The {name} (tag {tag}) has never been seen: put the tag in the camera view.')
                continue

            task = self.plan_task(cube, tag, name, xyz, status)
            if task is None:
                continue
            plan, prompt = task[3], task[5]
            print(f'  🎯  "{prompt}"  pitch {np.degrees(plan.pick_pitch):.0f}°, {plan.duration:.1f} s'
                  + ('' if self.record else '  (trial: not recorded)'))
            print('        ' + S.height_report(cube, task[1], plan, self.cfg))
            answer = self.ask(f'  ❓  ENTER = {"run and record" if self.record else "run"}, b = choose again, q = quit').strip()
            if answer == 'q':
                raise Quit()
            if answer not in ('', 'y'):
                continue
            task = self.recheck(task)
            if task is not None:
                return task

    def plan_task(self, cube, tag, name, xyz, status, var=None):
        """(cube, target, target_xy, plan, var, prompt) or None (reason printed)."""
        if var is None:
            var = D.sample_variation(self.rng, bool(self.p('variation')), float(self.p('correction_prob')))
        target = {'id': tag, 'name': name, 'xyz': xyz, 'cached': status != 'visible'}
        target_xy = (np.array(xyz[:2]) + self.target_offset
                     + D.place_offset(self.p('place_offsets'), self.placed_at.get(tag, 0)))
        try:
            plan = D.plan_pick_place(cube['xyz'][:2], target_xy, self.cfg, var)
        except D.PlanningError as e:
            print(f'  ❌  Cannot do that: {e}. Choose another cube or bin, or move the cube.')
            return None
        return cube, target, target_xy, plan, var, D.make_prompt(self.template, cube['label'], name)

    def recheck(self, task):
        """Just before running: the cube must still be there (a fresh frame); if it moved a little, the plan follows it."""
        cube, target, target_xy, plan, var, prompt = task
        det = self.fresh_detections(newer_than=time.monotonic())
        if det is None:
            print('  ⚠️  No fresh detections: not running. Choose again.')
            return None
        now = match_cube(cube, det['objects'], float(self.p('recheck_max_move')))
        if now is None:
            print(f'  ⚠️  The {cube["label"]} cube is no longer where it was. Choose again.')
            return None
        if math.dist(now['xyz'][:2], cube['xyz'][:2]) > 0.005:
            return self.plan_task(now, target['id'], target['name'], target['xyz'],
                                  'last seen' if target['cached'] else 'visible', var)
        return task

    def confirmed(self, plan):
        return True  # the operator already confirmed in choose_task

    def run_episode(self):
        try:
            return super().run_episode()
        except Quit:
            self.quit_requested = True
            return 'skipped'


def main(args=None):
    S.main(args, node_class=GuidedDemo, title='Guided Pick-and-Place Demonstrations (you choose cube + bin)')


if __name__ == '__main__':
    main()
