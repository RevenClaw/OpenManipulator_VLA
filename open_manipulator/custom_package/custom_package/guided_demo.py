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
# Arrangements (`arrangement` parameter): a fixed cube -> bin assignment per session, each in its own folder. Cubes are
# numbered in `colors` order (red 1, green 2, blue 3), bins in `bins` order (tag 25 red 1, 20 green 2, 18 blue 3).
# "231" = cube 1 -> bin 2, cube 2 -> bin 3, cube 3 -> bin 1, saved in <dataset>/arr_1to2_2to3_3to1/. With an arrangement you
# only choose the cube; its bin follows. arrangement "" = a menu at startup with all 6 and their progress (successful
# episodes / `episodes_per_arrangement`); "free" = the old free choice of cube and bin, saved in <dataset>/ itself.
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

import itertools
import json
import math
import pathlib
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


def parse_arrangement(text, count=3):
    """'231' (bin number for cube 1, 2, 3, ...) -> (1, 2, 0) as 0-based bin indices; None unless a permutation of 1..count."""
    text = str(text).strip()
    if len(text) != count or not text.isdigit():
        return None
    perm = tuple(int(c) - 1 for c in text)
    return perm if sorted(perm) == list(range(count)) else None


def arrangement_folder(perm):
    """(1, 2, 0) -> 'arr_1to2_2to3_3to1'."""
    return 'arr_' + '_'.join(f'{i + 1}to{j + 1}' for i, j in enumerate(perm))


def all_arrangements(count=3):
    """Every cube -> bin assignment (each bin used once), in a fixed order: (0, 1, 2), (0, 2, 1), ..."""
    return list(itertools.permutations(range(count)))


def count_episodes(folder):
    """(saved, successful) episodes in a dataset folder, from their metadata.json."""
    saved = success = 0
    for meta in pathlib.Path(folder).glob('episode_*/metadata.json'):
        try:
            ok = json.loads(meta.read_text()).get('success', False)
        except (OSError, ValueError):
            continue
        saved += 1
        success += bool(ok)
    return saved, success


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
        self.declare_parameter('arrangement', '')          # '' = choose at startup, 'free' = any cube to any bin, '231' = fixed
        self.declare_parameter('episodes_per_arrangement', 10)  # successful episodes per arrangement folder (then it stops)
        self.base_dir = self.data_dir
        self.arrangement = None  # tuple: cube index -> bin index, or None = free choice
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
        far = (f' + up to {self.cfg.far_release_extra * 100:.1f} cm for cubes {self.cfg.far_release_start * 100:.0f}-'
               f'{self.cfg.far_release_full * 100:.0f} cm from the base' if self.cfg.far_release_extra > 0 else '')
        print(f'\n  🧊  Cubes on the table (aim z = table_z {self.cfg.table_z:+.3f} + cube_size/2; the camera z is not used):')
        if not cubes:
            print('        (no reachable cube detected)')
        for i, c in enumerate(cubes, 1):
            inside = cube_in_bin(c['xyz'][:2], bins_xy, self.p('target_radius'))
            goes = self.arranged_bin(c, bins)
            arrow = f' → {goes[1]:10s}' if goes else ''
            print(f'     {i:2d}  {c["label"]:6s}{arrow} x {c["xyz"][0]:.3f}  y {c["xyz"][1]:+.3f}  '
                  f'z {c["xyz"][2]:+.3f} (camera, centre)'
                  f'  aim z {grasp_z:+.3f}  conf {c["confidence"]:.2f}'
                  + (f'   [already in the {inside}]' if inside else ''))
        shown = [(o, why) for o, why in ignored if why != 'colour not used']
        if shown:
            print('  🚫  Ignored detections (not cubes, or not pickable):')
            for o, why in shown:
                pos = (f'x {o["xyz"][0]:+.3f}  y {o["xyz"][1]:+.3f}  z {o["xyz"][2]:+.3f}' if o.get('xyz') else ' ' * 30)
                print(f'          {o["label"]:6s} {pos}  conf {o["confidence"]:.2f}   {why}')
        print(f'  🗑️   Bins (tag z = table surface as the camera sees it; drop aim z {release_z:+.3f}{far}):')
        for i, (tag, name, xyz, status) in enumerate(bins, 1):
            pos = f'x {xyz[0]:.3f}  y {xyz[1]:+.3f}  z {xyz[2]:+.3f} (camera)' if xyz else ' ' * 37
            print(f'     {i:2d}  {name:12s} tag {tag:<3d} {pos}  {"✅" if status == "visible" else "⚠️ "} {status}')

    def print_routing(self):
        """Startup: which bins the camera sees (replaces scripted_demo's colour routing)."""
        det = self.fresh_detections(timeout=3.0)
        if det is None:
            print('  ⚠️  No detections yet (is object_detector running?).')
            self.choose_arrangement()
            return
        if not det.get('calibrated', True):
            print('  ⚠️  The camera is not calibrated: positions are unusable. Run calibrate_camera first.')
        print('  🗺️  Bins (AprilTag → name used in the prompt):')
        for tag, name, xyz, status in self.bin_positions(det):
            print(f'        tag {tag:<3d} → "{name}"  {"✅" if status == "visible" else "⚠️ "} {status}')
        self.choose_arrangement()

    # ------------------------------------------------------------------ arrangements (fixed cube -> bin per folder)
    def describe(self, perm):
        return ', '.join(f'{self.colors[i]} → {self.bins[j][1]}' for i, j in enumerate(perm))

    def choose_arrangement(self):
        setting = str(self.p('arrangement')).strip().lower()
        if setting == 'free':
            print(f'  🔀  Free choice of cube and bin, saving to {self.data_dir}')
            return
        count = min(len(self.colors), len(self.bins))
        if count < 2:
            print('  ⚠️  Arrangements need >= 2 colours and bins: free choice.')
            return
        perm = parse_arrangement(setting, count) if setting else None
        if setting and perm is None:
            print(f'  ⚠️  arrangement "{setting}" is not a permutation of 1..{count} (e.g. "231").')
        target = int(self.p('episodes_per_arrangement'))
        while perm is None:
            options = all_arrangements(count)
            print(f'  📂  Arrangements (cube → bin; cubes {", ".join(f"{i + 1}={c}" for i, c in enumerate(self.colors[:count]))}; '
                  f'bins {", ".join(f"{i + 1}={n}" for i, (_, n) in enumerate(self.bins[:count]))}):')
            for k, option in enumerate(options, 1):
                saved, ok = count_episodes(self.base_dir / arrangement_folder(option))
                done = '✅' if target > 0 and ok >= target else '  '
                print(f'     {k}  {done} {arrangement_folder(option):20s} {self.describe(option):55s} '
                      f'{ok:2d}/{target} ok ({saved} saved)')
            answer = self.ask(f'  ❓  Record which arrangement? 1-{len(options)}, q = quit').strip()
            if answer == 'q':
                self.quit_requested = True
                return
            k = parse_choice(answer, len(options))
            perm = options[k] if k is not None else None
        self.arrangement = perm
        self.data_dir = self.base_dir / arrangement_folder(perm)
        saved, ok = count_episodes(self.data_dir)
        print(f'  📂  Arrangement {arrangement_folder(perm)}: {self.describe(perm)}')
        print(f'      saving to {self.data_dir}  ({ok}/{target} successful so far, {saved} saved)')

    def arranged_bin(self, cube, bins):
        """The bin entry the arrangement assigns to this cube's colour, or None."""
        if self.arrangement is None or cube['label'] not in self.colors[:len(self.arrangement)]:
            return None
        return bins[self.arrangement[self.colors.index(cube['label'])]]

    def metadata_extra(self):
        if self.arrangement is None:
            return {}
        return {'arrangement': arrangement_folder(self.arrangement), 'arrangement_map': self.describe(self.arrangement)}

    def episode_saved(self, success):
        if self.arrangement is None:
            return
        saved, ok = count_episodes(self.data_dir)
        target = int(self.p('episodes_per_arrangement'))
        print(f'  📂  {arrangement_folder(self.arrangement)}: {ok}/{target} successful ({saved} saved)')
        if target > 0 and ok >= target:
            print(f'  🏁  {target} successful episodes in this arrangement: done. Restart guided_demo for the next one.')
            self.quit_requested = True

    # ------------------------------------------------------------------ the task, chosen by the operator
    def choose_task(self, det):
        while True:
            det = self.fresh_detections() or det
            cubes, ignored = self.screen(det)
            bins = self.bin_positions(det)
            if self.arrangement is not None:
                used = set(self.colors[:len(self.arrangement)])
                ignored = list(ignored) + [(c, 'colour not in this arrangement') for c in cubes if c['label'] not in used]
                cubes = [c for c in cubes if c['label'] in used]
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

            if self.arrangement is not None:
                tag, name, xyz, status = self.arranged_bin(cube, bins)
                print(f'  ➡️   {cube["label"]} cube → {name} (arrangement {arrangement_folder(self.arrangement)})')
            else:
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
        if self.quit_requested:  # q in the arrangement menu
            return 'skipped'
        try:
            return super().run_episode()
        except Quit:
            self.quit_requested = True
            return 'skipped'


def main(args=None):
    S.main(args, node_class=GuidedDemo, title='Guided Pick-and-Place Demonstrations (you choose cube + bin)')


if __name__ == '__main__':
    main()
