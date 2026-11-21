# Fine-tuning Procedure (v1): scripted YOLO pick-and-place demonstrations → π0.5

The exact steps, in order, from an empty table to a fine-tuned π0.5 running on the OM-X. Why this approach and what v1 can and
can't do: `vla_layer/docs/FINETUNING_STAGES.md`, `vla_layer/docs/DATASET_PLAN.md`. Known issue #14 tracks its test status.

**Status (2026-11-05):** all software below is tested in simulation (mock arm, fake camera streams, the real YOLO model on a
stock photo, the real converter), **nothing yet on the real arm or with the real camera**. Expect to tune the values marked
🔧 during sections 4–6.

```
 ┌── robot PC (Docker container) ───────────────────────────────────────────────────────────────┐
 │ camera_front (color + aligned depth) ─► object_detector (YOLO cubes + AprilTag bins, metres)  │
 │                                              │                                                │
 │ scripted_demo: task → plan (IK, smooth, random variation) → arm + gripper → checks → label     │
 │   publishes the commanded targets on /teleop/action, records with ros2 bag record             │
 │   → /workspace/recordings/scripted_pick_place/episode_NNNN/ (bag + metadata.json)             │
 └───────────────────────────────────────────────────────────────────────────────────────────────┘
          │ rsync
 ┌── A30 server (vla_layer) ──── convert_rosbag_to_lerobot → norm stats → LoRA training → serve ─┐
          │ websocket
 robot PC: vla_bridge (the trained policy drives the arm, starting at READY)
```

---

## 0. What you need
- The OM-X on the robot PC, the RealSense D435 on a **USB 3** port, the container built from the current Dockerfile.
- Cubes the YOLO model knows: **red, green, blue**, 3.5 cm (measure yours). **No other coloured objects** on the table:
  the model only knows these three classes and forces anything else into one of them (a yellow cube was detected as "green"
  in testing).
- One target: a **flat tray or shallow bin** that the arm can reach, with an **AprilTag 36h11** (e.g. ID 20) flat at its centre.
- 4–6 more AprilTags for calibration, flat on the table, spread over the workspace, with ids that are not used as targets. For the
  pipeline test (targets 19 and 25) use 16, 17, 18 and 20; for recorded demos with the bin on 20 use 16–19. A target tag can also serve
  as a calibration point if the fingertip can reach it. Printable tags: `yolo_pipeline/data/tags/` (16–20, 24, 25, 27). Print them about 4–5 cm wide.

---

## 1. One-time setup

### 1.1 Software (robot PC)
```bash
cd low_level_controller && git pull                       # branch side
cd open_manipulator/docker && ./container.sh start && ./container.sh enter
bash /root/ros2_ws/src/open_manipulator/docker/install_vla_deps.sh   # openpi-client + CPU PyTorch + YOLO; ends with ✅ numpy 1.26.x, cv2 4.10.0
```
`./container.sh start` pulls the prebuilt ROBOTIS image (it does **not** build our Dockerfile) and `./container.sh stop`
removes the container, so **run `install_vla_deps.sh` after every start** (it skips what is already there). The PyTorch
download (~175 MB) can time out on a slow link: the script's header explains how to pass pre-downloaded wheel files.
Build, and put the YOLO model where `object_detector` looks for it (`/workspace` = host `open_manipulator/docker/workspace`):
```bash
cd /root/ros2_ws && colcon build --symlink-install --packages-select custom_package && source install/setup.bash
mkdir -p /workspace/models /workspace/calibration /workspace/recordings
cp <path to>/yolo_pipeline/YOLO_MODEL/YOLO.pt /workspace/models/yolo_cubes.pt   # copy it on the host into docker/workspace/models/
```

### 1.2 The scene (fix it once, never move it afterwards)
1. **Camera:** mounted rigidly, looking at the whole workspace and the gripper from the front/above. Mark its position.
   The policy learns the scene from exactly this viewpoint, for recording **and** for running the model later.
2. **Lighting:** fixed (curtains closed or always on). Later: lock exposure/white balance (`PHASE4_GUIDE.md` T4.2).
3. **Target:** tray/bin with its tag at the centre, inside the reach (≈ 0.10–0.28 m from the base, see §3.3).
4. **Calibration tags:** 4–6 tags flat on the table, spread around the workspace (corners plus middle), all visible.
   They can stay (and the bin tag can be one of them). The policy sees them too, so keep them for every session.

---

## 2. Start everything (every session)

Six container terminals (`./container.sh enter`, then `source /root/ros2_ws/install/setup.bash` in each):
```bash
# T0 Zenoh router (not auto-started)
zenohd
# T1 arm: homing menu option 2 (HOME)
ls -l /dev/serial/by-id/                                     # which ttyUSBx is the U2D2
ros2 launch custom_package open_manipulator_x_interactive.launch.py port_name:=/dev/ttyUSB0
# T2 camera with depth aligned to color
ros2 launch custom_package camera_front.launch.py depth:=true
# T3 detector
ros2 run custom_package object_detector --ros-args --params-file \
    $(ros2 pkg prefix custom_package)/share/custom_package/config/object_detector.yaml
# T4 RViz: Add → By topic → /object_detector/annotated → Image; /object_detector/markers → MarkerArray (fixed frame: world)
rviz2
# T5 the tools of the following sections (calibrate_camera, scripted_demo, check_episode)
```
Check the rates before going on: `ros2 topic hz /camera/front/color/metadata` (~15 Hz) and
`ros2 topic hz /object_detector/detections` (~3 Hz).

---

## 3. Check YOLO and the targets on your scene (5 min)
1. Put one cube of each colour on the table. In RViz's `/object_detector/annotated` image every cube must have a box with the
   right colour name and confidence ≥ 0.5, and the bin tag must be marked with its name (`bin` by default).
2. Move each cube to several places (near, far, left, right of the workspace): it must stay detected.
3. If cubes are missed or mislabelled: better light, fewer reflections, or lower `confidence` (0.4). If it stays poor, the
   model has to be fine-tuned on images of your table (not covered by v1: capture ~200 frames, label them, fine-tune YOLO.pt
   with ultralytics).
4. Names: set `target_tags` in `config/object_detector.yaml` to the words used in the prompt, e.g. `["20:black bin"]`.

---

## 4. Calibrate camera → robot (once, and again whenever the camera moves)
```bash
ros2 run custom_package calibrate_camera
```
1. Arm at HOME (out of the view), all tags visible → **ENTER**: the camera measures every tag (2 s).
2. For each tag: put the **fingertip centre** (gripper closed) **on the tag centre**, then **ENTER**. Easiest: homing menu
   option 1 in T1 (torque off, move the arm by hand); press ENTER in T1 afterwards to lock the torque again.
3. Read the fit: **aim for mean < 5 mm, max < 10 mm**. Above 15 mm the tool warns; re-touch the worst tags (or skip one with
   `s`) and run again. Answer `y` to save → `/workspace/calibration/camera_calibration.yaml`; `object_detector` reloads it.
4. 🔧 **table_z:** the recorded tag heights (`world (x, y, z)` lines) are the table height in the robot frame. Put their
   average into `table_z` in `config/scripted_demo.yaml` (0.0 if the base stands on the table).
5. **Verify:** place a cube; its RViz marker must sit on the cube, and the `(x, y)` in the annotated image must match a ruler
   measurement from the base within ~1 cm.

(In simulation with known tag positions the tool recovered the camera pose to 0.00° and 0.7 mm.)

---

## 5. Measure the scene values 🔧 (`config/scripted_demo.yaml`, `config/object_detector.yaml`)
| Value | How |
|---|---|
| `cube_size` (and `object_size` in the detector) | ruler, metres (0.035 = 3.5 cm) |
| `bin_height` | rim height of the tray/bin above the table |
| `target_xy_offset` | `[0, 0]` if the tag is at the bin centre; otherwise the offset from the tag to the drop point |
| (no workspace box) | every cube the arm can reach is used; `min_radius` (0.08 m) ignores detections at the base, `cube_max_above` (0.03 m) ignores things above the table (the arm itself) |
| `colors`, `prompt_template`, `color_targets` | which cubes to pick and how prompts read; `color_targets: ["red:red bin"]` fixes a bin per colour |

---

## 6. Pipeline test: blue object → tag 19, green object → tag 25 (no recording)
Do this **before** recording anything: it proves detection, calibration, grasping and the motion work together. The test
config `config/pick_place_test.yaml` places every **blue** object **on AprilTag 19** ("blue marker") and every **green** object **on
AprilTag 25** ("green marker"), in random order, lowering the fingertips to just above the table and letting go (not into a bin).
**Red objects are ignored.** It does exactly as many episodes as there are blue and green objects (plus retries after failures) and then
stops. Nothing is recorded. To change the colours or tags: `colors` and `color_targets` in the yaml, and `targets:="19:blue marker,25:green marker"`
on the launch (the names must match).

**What you need on the table:** tags 19 and 25 flat on the table, 12–25 cm from the base, away from where objects lie (at least 8 cm
from every object), and 1–3 small blue/green objects about the size of the cubes (`cube_size`, default 3.5 cm). Red ones may stay on
the table. The calibration tags must use other ids (§0). No other coloured things in view (YOLO only knows red, green and blue and
forces anything else into one of them).

**Terminals** (each: `source /root/ros2_ws/install/setup.bash`; the container needs `install_vla_deps.sh` and the YOLO model
from §1 first):
```bash
# T0
zenohd
# T1: arm, homing menu option 2 (HOME). scripted_demo itself moves the arm from HOME to READY before each pick.
ros2 launch custom_package open_manipulator_x_interactive.launch.py port_name:=/dev/ttyUSB0
# T2: camera with depth + object_detector in one launch (tag 19 = "blue marker", tag 25 = "green marker")
ros2 launch custom_package pick_place_test.launch.py
# T3 (optional): rviz2, Add → By topic → /object_detector/annotated → Image, and /object_detector/markers → MarkerArray
rviz2
```

**Step A: look, nothing moves (§3).** Every object must have a box with the right colour in `/object_detector/annotated`,
and the marker must be labelled `marker`.

**Step B: calibrate (§4, once).** `ros2 run custom_package calibrate_camera`, then put the average tag height it prints into
`table_z` in `config/pick_place_test.yaml`. Check: an object's marker in RViz sits on the object (±1 cm of a ruler).

**Step C: read the plans, nothing moves.** The test config has `confirm: true`: every plan is printed first and waits.
```bash
ros2 run custom_package scripted_demo --ros-args --params-file \
    $(ros2 pkg prefix custom_package)/share/custom_package/config/pick_place_test.yaml
```
```
🎯  episode_0000: "pick up the red object and place it on the marker"  cube (0.200, 0.100) -> marker (0.140, -0.150), pitch 85°, 35.1 s
        pregrasp      x 0.200  y 0.100  z 0.072  pitch 85°
        grasp         x 0.200  y 0.100  z 0.018  pitch 85°
        ...
        release       x 0.140  y -0.150  z 0.015  pitch 85°
❓  ENTER = execute this plan, s = skip it (nothing moves), q = quit
```
Type **`s`** a few times and compare the numbers with a ruler: `x, y` of the pick must match the object's position, `x, y` of
the release must match the marker (measured from the robot base, x forward, y left), and the grasp height should be about
half the object's height. If not, fix calibration, `table_z` or `cube_size` before anything moves.

**Step D: first real motion.** One object on the table, **hand on the power switch**, speed already reduced in the config
(`max_joint_vel [0.3, 0.2, 0.4, 0.5]`). Press **ENTER** on a plan and watch: the arm goes READY → above the object → down →
close → up → over the marker → down → open → up → READY. Press **Ctrl+C** at any moment: the arm holds.
Before each pick the script checks that the gripper really is open (`✅ gripper open (0.0189 m)`); if it isn't after one retry,
it pauses with `⚠️ gripper did not open …`. Disable with `-p check_gripper_open:=false`; widen with `gripper_open_tolerance`.

**Step D2: is the arm reaching the table? 🔧** Before every close the script prints where the fingertip really is:
```
📏  before closing: fingertip z 12.4 mm (planned 12.5, table_z 0.0), xy error 3.1 mm, z error -0.1 mm
```
The robot's z = 0 is the origin of its first link, **not necessarily the table surface** (the model's base plate extends to about
−8 mm and ROBOTIS's shoulder height suggests up to −17 mm). Measure it: put the fingertip centre flat on the table (homing
option 1 = torque off, by hand) and read its z, for example from `calibrate_camera` (it prints the fingertip z of every tag
touch; the tags lie flat on the table) — that value is `table_z`, usually a **negative** number of 1–2 cm. If the printed
`z error` is within a few mm but the fingers still hover above the table, `table_z` (or the fingertip model) is the cause; if the
z error is large, the arm did not arrive (lag, joint 2 load).

**Step E: grasp threshold 🔧.** Each run prints the gripper position after closing and both checks:
```
🧪  trial (not recorded): gripper after closing 0.0042 m -> holding (threshold -0.007); place: ok (cube no longer visible ...)
```
Close the gripper once on nothing and once on an object (`joint_commander`), note the two `gripper_left_joint` values, and put
`grasp_closed_threshold` between them.

**Step F: several objects, clearing the table.** Put 2–3 objects down and keep ENTER-ing through the plans. It counts the objects
at the first look (`🧮 3 object(s) to place`), runs exactly until all are on the marker, and stops by itself (`🏁 3 object(s) on the
marker`). **Failures are retried:** a grasp with nothing in the gripper is caught right after closing (`✋ … backing out`: lift,
open, back to READY), and an object that did not end up on the marker is picked again on the next round. It gives up on an object
after `max_attempts_per_object` (3) failed tries and stops the whole run after objects × 3 attempts. Each object is placed at the next spot
around the marker (centre, then ±5 cm in x and y, `place_offsets`) so they do not land on each other; the marker is covered
by the first object, so the script keeps using its last seen position (it prints `[target tag covered…]`). Objects already
within 8 cm of the marker are not picked again. When no reachable object is left it pauses: put the objects back, ENTER.

**Pass when it clears the table with at most about 1 failed attempt per 4 picks** (e.g. 3 objects placed in ≤ 4 attempts, repeated
with different layouts; the final `📊` line gives placed / attempts / failed picks). Then set `confirm: false` and raise
`max_joint_vel` towards the default `[0.5, 0.3, 0.6, 0.8]` (shoulder last), watching for joint2 errors (`0x20`).

| Symptom | Likely cause |
|---|---|
| Gripper closes beside the object, constant offset | calibration (§4) or `table_z` |
| Closes in front of/behind the object only | `object_size` in `object_detector.yaml` (the visible face is moved back by half of it) |
| Squeezes air ("nothing held") | `cube_size` too small/large, or `grasp_closed_threshold` wrong |
| Releases high above the table | `release_clearance` / `bin_height` (0.0 for the marker) in the test config |
| Objects knock each other at the marker | increase the `place_offsets` spacing (default 5 cm) |
| `no target (AprilTag) visible` | the target tags were never seen (or the camera is not calibrated: the message then says so): put them in view once before starting |
| Every object goes to tag 20 (or one tag) instead of blue → 19, green → 25 | A stale or wrong setup. Read the **`🗺️ Routing`** block `scripted_demo` prints at startup: `blue → "blue marker" ✅ tag 19` must appear for both colours. `⚠️ not reported by the detector (it reports: 20 'bin')` = `object_detector` was started with its default config instead of `pick_place_test.launch.py`. `ANY target tag` for every colour = the recording config `scripted_demo.yaml` was used instead of `pick_place_test.yaml`. An old yaml/launch = `git pull` the robot PC and run `colcon build --symlink-install --packages-select custom_package` again, then restart the detector launch AND `scripted_demo`. Each plan line also shows its tag: `-> blue marker [tag 19]`. |
| Prompt text says "object … marker" | by design: the template is `pick up the {color} object and place it on the {target}` |

---

## 7. Pilot: 5–10 recorded episodes
```bash
ros2 run custom_package scripted_demo --ros-args --params-file \
    $(ros2 pkg prefix custom_package)/share/custom_package/config/scripted_demo.yaml -p episodes:=10
```
- Each episode: task line (🎯), quick check per topic, then `💾 saved: ✅ SUCCESS` or `❌ FAILURE (kept, not trained on)`.
  Episodes with recording problems (gaps, stopped streams) are **deleted automatically** and the loop moves on.
- When no reachable cube is left (all in the bin), it pauses: **put the cubes back at new places, ENTER** to continue.
- Keys: `p` pause after this episode, `ENTER` continue, `q` quit after this episode, **Ctrl+C** abort now (the arm holds,
  the current episode is deleted).
- If dropped camera frames show up often, set `storage_preset: "none"` (no compression).

Check and copy to the A30, then convert (full commands: `vla_layer/docs/PHASE5_GUIDE.md`):
```bash
ros2 run custom_package check_episode /workspace/recordings/scripted_pick_place
# on the host, from low_level_controller/
rsync -avP open_manipulator/docker/workspace/recordings/scripted_pick_place/ \
    shradul@iras-hub:~/vla/openpi/omx_pie0.5/data/rosbags/scripted_pick_place/
# on the A30 (source scripts/env.sh)
python scripts/convert_rosbag_to_lerobot.py data/rosbags/scripted_pick_place --dry-run --no-wrist
```
Every episode should be converted, with ~15 Hz front, ~30 Hz action, ~100 Hz state. Two warnings are expected in v1:
several task strings (several colours), and black wrist frames (no wrist camera yet).

---

### 7b. Alternative: guided recording (you choose cube + bin)
`ros2 run custom_package guided_demo --ros-args --params-file $(ros2 pkg prefix custom_package)/share/custom_package/config/guided_demo.yaml`
lists the detected cubes and the bins (tags 25 red, 20 green, 18 blue; `bins` parameter), asks which cube and which bin, builds
the prompt from that ("pick up the {color} cube and place it in the {target}") and records the episode exactly like `scripted_demo`
(same planning, checks, metadata; dataset `guided_pick_place`). Use it to control the mix of colour/bin combinations in the data.
Quick reference: `docs/custom_commands_guide.md` §12.

## 8. Bulk recording: ~200–300 episodes
- Same command as §7 with `-p episodes:=0` (until `q`); several sessions are fine (numbering continues).
- Spread the cubes over the whole workspace each time you reset; vary which colours are left.
- Same camera, light, tags and bin in every session. Episodes take ~20–35 s; plan ~3 h of robot time for 300.
- Disk: ~0.4 GB per episode → ~120 GB for 300. Convert and archive in batches if the robot PC's disk is small
  (`min_free_gb` stops recording at 5 GB free).
- Every episode's `metadata.json` records the prompt, the cube and target positions, the waypoints, the random variation
  and the checks (`source: scripted`).

---

## 9. Convert, train, serve (A30)
```bash
python scripts/convert_rosbag_to_lerobot.py data/rosbags/scripted_pick_place --repo-id local/omx_scripted_v1 --no-wrist
#   note the dataset hash it prints in docs/experiments.md
# omx_pi05/train_configs.py, DATASETS:  "scripted_v1": "local/omx_scripted_v1",
python scripts/compute_norm_stats.py pi05_omx_lora_scripted_v1
bash scripts/gpu_check.sh train && python scripts/train.py pi05_omx_lora_scripted_v1 --exp-name=scripted_v1   # in tmux, ~32 h
```
There is no wrist camera: the wrist view is masked out by default (`wrist_camera=False` in `omx_pi05`), so the model ignores it
in training and serving. First run `python -m pytest -q tests/test_policy.py` on the A30 (the masking was written on the laptop,
where openpi isn't installed, so these tests have not run yet).
Evaluate the 5k/10k/15k checkpoints early (`IMPLEMENTATION_PLAN.md` T6.3).

## 10. Run the policy on the arm
1. Serve the checkpoint on the A30 (`scripts/serve.py`, port 4000).
2. Move the arm to **READY**, the pose every demo started from: `ros2 run custom_package joint_commander` → `0 -14.6 -8.7 93.3 1`.
3. Start the bridge with the **demos' per-joint speed caps**, so the learned motion isn't clipped, and the exact prompt:
   ```bash
   ros2 run custom_package vla_bridge --ros-args --params-file \
       $(ros2 pkg prefix custom_package)/share/custom_package/config/vla_bridge.yaml -p server_host:=<server-ip> \
       -p "max_joint_vel_per_joint:=[0.5, 0.3, 0.6, 0.8]" -p prompt:="pick up the red cube and place it in the bin"
   ```
4. Shadow mode first (`-p shadow:=true`), then live. 20 trials per prompt, results in `vla_layer/docs/experiments.md`.

---

## Troubleshooting
| Symptom | Likely cause / fix |
|---|---|
| `no fresh detections (is object_detector running?)` | T3 not running, or the camera was started without `depth:=true` |
| detector: `depth and color sizes differ` | camera without `depth:=true` (no aligned depth) |
| `no target (AprilTag) visible` | tag covered, too small, or not in `target_tags` |
| `no reachable cube` | the cubes are out of the arm's reach (or filtered: on a tag, at the base); it pauses: move them closer, ENTER |
| grasps miss by a constant offset | calibration (§4), `table_z`, `cube_size`/`object_size` |
| every trial says "nothing held" | `grasp_closed_threshold` too high: it must lie between the closed-on-nothing value (−0.010) and the value with an object held |
| `gripper did not open (… m, expected 0.019 ± 0.004 m)` → paused | something still held, gripper servo (ID 15) in error, or polarity wrong (issue #15); `None m` = no `gripper_left_joint` in `/joint_states` |
| place check fails although the cube is in the bin | `target_radius` too small, or `target_xy_offset` wrong |
| `/joint_states: 0.5 s gap` → episode deleted | load spike on the PC; if frequent: `storage_preset: "none"`, close other programs |
| joint2 overload (`0x20`) | lower `max_joint_vel` (shoulder value first); report it |
