# OpenManipulator-X VLA Controller — Project Context

ROS 2 **Jazzy** workspace (runs in Docker, `open_manipulator/docker/`, workspace at `/root/ros2_ws` inside the container) for a
ROBOTIS **OpenMANIPULATOR-X** (4-DOF arm + 1 gripper, Dynamixel XM430, IDs 11–15, U2D2 on `/dev/ttyUSB0` or `/dev/ttyUSB1`, 1 Mbps).
The U2D2 port changes after a replug/reboot: check `ls -l /dev/serial/by-id/` (the FTDI entry) before launching; never assume it.
Goal: a controller stack that a separate **VLA (vision-language-action) node** can drive.
Work happens on branch `side`; `main` is the PR target.

Read `docs/` before changing behaviour. It is the record of what was changed and why:
- `docs/openmanipulator_changes_summary.md` — itemized list of every change (start here)
- `docs/commands_guide.md` — build / launch / teleop / MoveIt / `ee_commander` usage
- `docs/known_issues.md` — open & fixed issues (numbered; refer to them by number)
- `docs/joint_offset_investigation.md` — servo Homing Offset calibration (why the URDF is stock again)
- `docs/SYSTEM_WORKFLOW_ARCHITECTURE_MANUAL.md` — architecture, node graph, topics/services/actions
- `docs/input_needed.md` — how a VLA node sends end-effector goals to `ee_commander` via `/ee_goal`, and the limits of that path
- `docs/custom_commands_guide.md` — quick list of the day-to-day commands (robot launch, bridge, pipeline test §10, recorded demos §11)
- `docs/finetuning_procedure.md` — exact step-by-step procedure: scripted YOLO demos → dataset → π0.5 fine-tune (section 6 = pipeline test)
- `docs/policy_runs.md` — log of every run of the fine-tuned policy on the arm (settings, result, our insight, Claude's insight) + how to record a run (bag + video); `trajectory_compare` (record/plot, pure part `trajectory_tools.py`) = divergence of each checkpoint from the scripted YOLO run

Keep these docs updated when behaviour changes (the user expects it; issue #6 is about docs drifting).

## BTP report material
Everything for the BTP report (LaTeX sources in `btp_material/btp_report/`, the Overleaf zip, reference papers, photos, the
IIIT-Delhi template and sample report in `btp_material/refrence report/`) lives **only** in `btp_material/`, which is
git-ignored. Write all new report files (drafts, figures, zips, notes) inside that folder, never elsewhere in the repo, and
never commit it.

## OPEN ISSUE for the next agent: gripper flips close/open on the first live policy run (known issue #16)
First live run of the two-camera π0.5 policy (2026-11-25, `vla_bridge`, prompt "pick up the blue cube and place it in the red
bin", `-p min_z:=-0.05`, per-joint caps `[0.5, 0.3, 0.6, 0.8]`): right after enabling, the bridge printed `✊ Gripper close.` then
`🖐️ Gripper open.`, and the user stopped it (`d`) and reports the gripper oscillating between open and close.
- **Already ruled out on the robot side:** the bridge sends a gripper command only when the model's value crosses the hysteresis
  (`GripperHysteresis` in `vla_safety.py`: close > 0.6, open < 0.4), so the model itself asked for closed, then open. Robot-side
  conventions agree: `/joint_states` `gripper_left_joint` is +0.019 m open / −0.010 m closed (issue #15 mirror in the driver xacro);
  the bridge's observation is `(0.019 − q) / 0.029` → 0 open .. 1 closed (`gripper_metres_to_norm`); the demos' action on
  `/teleop/action` (`scripted_demo.py`, `JointState` name `gripper`) is 0 open .. 1 closed. `pick_place_2cam` was recorded on
  2026-11-23, after the #15 fix (2026-11-19), so its `/joint_states` gripper is in the stock convention.
- **Hypotheses, most likely first:**
  1. The converter on the A30 (`vla_layer`, `shradul@iras-hub:~/vla/openpi/omx_pie0.5/scripts/convert_rosbag_to_lerobot.py`) encodes
     the gripper **state** or **action** differently from the bridge (metres instead of 0..1, inverted, or another formula), so after
     closing the model sees a state it never saw in training and reopens. Check the converter and `omx_pi05` against
     `vla_safety.gripper_metres_to_norm`; `vla_layer/tests/test_bridge_contract.py` is meant to catch this. `vla_layer` is NOT on the
     robot PC: ask the user to paste the relevant code, or work on the A30.
  2. The policy closes early (in the air) and corrects itself: ask the user where the gripper was when it closed.
  3. The model's gripper value hovers near 0.5 and crosses both thresholds; diagnostic only: `-p gripper_close_above:=0.7
     -p gripper_open_below:=0.2` (hides cause 1 if that is the real one).
- **Checked 2026-11-25 against the server repo (`../omx/omx_pie0.5`) and the real bags:** the converter (`omx_pi05/dataset.py`) uses the
  same formula (`gripper.metres_to_norm`, action taken as-is 0..1), the delta mask keeps the gripper absolute, and the recorded demos
  are clean: action 0 → 1 once at the grasp, back to 0 at the release; while a cube is held the measured state is only **0.58–0.79**.
  Failed episodes (incl. the 2 closed-on-nothing ones) are excluded from training, so a fully closed gripper (state ≈ 1.0) **never
  appears in the training data**. Hypothesis 1 is therefore unlikely; leading explanation: the policy closed where there was no cube
  (the arm had not been put back to READY after the `min_z` stop, it was left low near the cube), the state became ≈ 1.0 (out of
  distribution) and the model reopened.
- **Diagnostic added (not yet tested on hardware):** `-p log_gripper:=true` (also in shadow mode) prints per chunk the measured gripper
  state, the fingertip z and the model's 10 raw gripper values (`🔎` lines). Next: start from READY, shadow + log, then live + log.
- **Cause found (2026-11-25, `log_gripper` run from READY):** the model is fine. It plans the close a few steps AHEAD in every
  chunk (`0.00 … 0.99 1.00`) and each new chunk starts again from "open now, close soon"; the bridge acted on the value at the current
  step, so it closed at the end of an old chunk and reopened when the next chunk arrived, once per chunk, until the model said 1.0
  everywhere (then state 0.70 = cube held, as in the demos). **Fix (not yet tested on hardware):** `GripperHysteresis.update(g,
  upcoming)`: while closed, open only if no remaining step of the current chunk plans a close (> close_above). Test
  `test_gripper_stays_closed_while_the_chunk_still_plans_a_close` replays the logged values. Next: a live run with `log_gripper`.
- **Second live run (after the fix):** one close, no flip, but state 0.79 → 0.99 = closed on nothing (cube slipped / missed); the model
  then froze around z ≈ 0 (fully closed is out of distribution). The policy also goes to z −0.045..−0.048 (demos grasp −0.0425..−0.0347,
  median −0.038) and the finger edges hit the table. **Change (not yet tested on hardware):** `min_z` default −0.043 in
  `config/vla_bridge.yaml`, and new `min_z_hold` (default true): at the floor the arm stops there, the bridge stays enabled and the gripper
  keeps the whole chunk's plan (before: hold + disable, and the gripper plan was cut with the truncated steps). Also seen: the policy
  sometimes heads to the wrong place (likely overfitting, 60 episodes, checkpoint 25000; compare 15000/20000 on the server).
- **Related finding, same session:** `min_z` default 0.02 m in `config/vla_bridge.yaml` is wrong for this table (table at
  z = −0.056 m in the `link1` frame; demos grasp at z ≈ −0.0385), so the safety layer stopped the first descent
  (`next step goes below min_z`). The run used `-p min_z:=-0.05`; changing the yaml default was proposed to the user, not yet approved.
- Rules: the user runs the arm and the policy server; ask before any motion. Mark fixes "not yet tested on hardware" in
  `docs/known_issues.md` (#16) until the user confirms. Remove this section from CLAUDE.md once #16 is resolved.

## Repository layout
- `DynamixelSDK/`, `dynamixel_hardware_interface/`, `dynamixel_interfaces/` — upstream drivers. **Never modify (isolation policy).**
- `open_manipulator/` — ROBOTIS packages (description, bringup, moveit_config, teleop, gui, …) with local config edits.
- `open_manipulator/custom_package/` — **all our own code** (ament_python):
  - `interactive_homing_node.py` — startup menu: [1] hand-pose (deactivates `arm_controller`, torque off, ENTER → torque on + reactivate),
    [2] default home via 2-stage trajectory (elbow first to avoid joint2 overload `0x20`), [3] zero pose
    (use [3] when MoveIt flags home as `START_STATE_IN_COLLISION`, issue #12).
    `z` key → min-stress exit pose, then [1] back to session home or [2] torque off + quit (ends the launch).
  - `joint_commander.py` — terminal: `<j1°> <j2°> <j3°> <j4°> <gripper 0|1>` → `FollowJointTrajectory` on `arm_controller`.
    Defines `JOINT_LIMITS`, `GRIPPER_OPEN_M` (0.019), `GRIPPER_CLOSE_M` (−0.010): ROBOTIS's stock values; this arm's gripper is physically inverted and mirrored in the driver config (issue #15). Gripper → `GripperCommand` on `/gripper_controller/gripper_cmd`
    (issue #1 fixed 2026-11-02, not yet tested on hardware).
  - `ee_commander.py` — Cartesian goals. Closed-form 4-DOF IK (`inverse_kinematics`, `forward_kinematics`) → MoveIt `/move_action`
    (plan + execute) → gripper via `GripperCommand`. Inputs: terminal `x y z pitch° [grip]` or topic `/ee_goal`
    (`std_msgs/Float64MultiArray` `[x, y, z, pitch_rad, gripper(1 open/0 close/-1 keep)]`). Latest-goal-wins queue, no preemption.
  - `vla_bridge.py` + `vla_safety.py` — π0.5 policy server → arm, behind a safety layer (see "VLA command interface").
  - `launch/cameras.launch.py` — both RealSense cameras by serial: **D455 above the table = front** (`_207322250030`) →
    `/camera/front/color/image_raw` 1280×800 @ 15 Hz (+ aligned depth, 848×480 sensor, with `depth:=true`), **D435 on the gripper = wrist** (`_927522071177`, color only)
    → `/camera/wrist/color/image_raw` 640×480 @ 15 Hz; `wrist:=false` = front only. `launch/camera_front.launch.py` = the front one alone (included).
  - `record_episodes.py` + `check_episode.py` + `episode_tools.py` — demo recorder: one rosbag2 episode (`ros2 bag record` subprocess,
    mcap/zstd_fast) + `metadata.json` per episode in `/workspace/recordings/<task>/`, pre-flight and quick checks, s/f/d labels,
    2-stage return to the start pose (READY by default). Output = input contract of `vla_layer/scripts/convert_rosbag_to_lerobot.py` (issue #13).
    `recording.py` holds the shared parts (StreamMonitor, BagRecorder), also used by `scripted_demo`.
  - Scripted demonstrations for fine-tuning (issue #14; procedure: `docs/finetuning_procedure.md`):
    `object_detector.py` (YOLO cubes + AprilTag targets from the ROS color + aligned-depth topics → world-frame positions in m,
    `~/detections` JSON, `~/annotated` image), `calibrate_camera.py` (touch tags → Kabsch camera→world, `/workspace/calibration/`),
    `scripted_demo.py` (task → plan → execute → `/teleop/action` → record → checks → metadata; `record:=false` = trials; `config/pick_place_test.yaml` = the pipeline test: blue → tag 19, green → tag 25, red ignored; `confirm` shows each plan first),
    pure modules `perception_tools.py` and `demo_planning.py` (unit-tested). YOLO model: `/workspace/models/yolo_cubes.pt`
    (copied from the collaborator's `yolo_pipeline/YOLO_MODEL/YOLO.pt`, classes red/green/blue; that folder is reference only).
    `launch/pick_place_test.launch.py` starts camera (depth) + detector (`targets:="19:blue marker,25:green marker"`). Test mode (`until_clear`, `confirm`,
    `remember_targets`, `place_offsets` in `scripted_demo`): counts the objects, retries failed picks (≤ 3 per object, ≤ objects × 3 in
    total), backs out after a failed grasp, keeps a covered tag's last position, drops successive objects 5 cm apart around the tag.
    `guided_demo.py` + `config/guided_demo.yaml`: same pipeline, but the operator chooses cube + bin from a list (bins = tags 25 red,
    20 green, 18 blue), prompt built from the choice, then recorded (dataset `pick_place_2cam`, front + wrist camera;
    `arrangement`: one fixed cube→bin permutation per session in `arr_1to2_2to3_3to1`-style subfolders, startup menu, 10 each). Subclass of `ScriptedDemo`.
    `move_test.py` + `config/move_test.yaml`: typed fingertip goals (`x y z [pitch] [o|c]`, `pp cx cy bx by`) through the same motion
    code, no camera (tests the moving half alone).
    Before every pick (all modes) it checks the gripper measures open (`check_gripper_open`, ±`gripper_open_tolerance` 0.004 m), else pauses.
    Grasp height = `table_z + cube_size/2` from the config only (the camera supplies x, y). `table_z` is NOT necessarily 0: z = 0 is
    link1's origin, the table is probably 8–17 mm lower; measure it (the fingertip z that `calibrate_camera` prints for tags on the table).
  - **READY** = `episode_tools.READY` `[0.0, -0.2547, -0.1520, 1.6284]` (gripper 70° down at (0.16, 0, 0.12) m): start pose of every
    demo and policy run. HOME (folded, gripper horizontal) is only the session/storage pose.
  - `log_joint_states.py`, `log_ee_pose.py` (live EE x y z + pitch from `/joint_states` via `ee_commander.forward_kinematics`), `record_home_position.py`
  - `launch/open_manipulator_x_interactive.launch.py` — hardware bringup + homing node (main launch).

## Robot facts
- Controllers (100 Hz, `open_manipulator_bringup/config/open_manipulator_x/hardware_controller_manager.yaml`):
  - `arm_controller` — `JointTrajectoryController`, joints `joint1..joint4`, position command,
    action `/arm_controller/follow_joint_trajectory`, topic `/arm_controller/joint_trajectory`. `allow_partial_joints_goal: true`.
  - `gripper_controller` — `GripperActionController` on `gripper_left_joint` (metres), **action only**: `/gripper_controller/gripper_cmd`.
  - `joint_state_broadcaster` → `/joint_states`.
- Joint limits (rad): j1 [−π, π], j2 [−1.82, 1.5], j3 [−1.5, 1.55], j4 [−1.7, 1.97]. Gripper [−0.010, 0.019] m, **+0.019 = open, −0.010 = closed** (ROBOTIS's stock convention; this unit's gripper is physically inverted, so the real-hardware servo conversion is mirrored in `open_manipulator_x_position.ros2_control.xacro`, known issue #15; mock hardware is unaffected) All copies of the two values must agree.
- Frame for Cartesian goals: `world` = `link1` (base), x forward, y left, z up. EE = `end_effector_link`. Pitch 0 = horizontal, +90° = down.
  Yaw is forced to `atan2(y, x)`, roll = 0 (4-DOF cannot do arbitrary orientation).
- Home `[0.6090, -1.3622, 1.3898, -0.0154]`; min-stress exit `[0.4633, -1.8190, 1.4972, 0.4019]`.
  The home pose is duplicated in ~8 files (issue #5) — update all of them together.
- Joint zero is calibrated in servo EEPROM via `Homing Offset` + position limits pinned in
  `open_manipulator_description/ros2_control/open_manipulator_x_position.ros2_control.xacro`. The URDF is stock (`rpy="0 0 0"`).
  There are 3 URDF copies (xacro, evaluated description URDF, `open_manipulator_moveit_config/config/open_manipulator_x.urdf`) — keep in sync.
- Don't call `/dynamixel_hardware_interface/get_dxl_data` on a running arm: it times out and stalls the control loop (issue #8).
- joint2 (shoulder) overloads when moving fast in extended poses — keep velocities low and lift the elbow first.

## Build & run (inside the container)
```bash
zenohd                                                                                         # T0 Zenoh router: NOT auto-started; without it nodes can't discover each other and launches hang
cd /root/ros2_ws
colcon build --symlink-install --packages-select custom_package open_manipulator_moveit_config open_manipulator_description open_manipulator_bringup
source install/setup.bash
ls -l /dev/serial/by-id/                                                                       # which ttyUSBx is the U2D2
ros2 launch custom_package open_manipulator_x_interactive.launch.py port_name:=/dev/ttyUSB0   # T1 hardware + homing
#   no arm connected: ... open_manipulator_x_interactive.launch.py use_mock_hardware:=true
ros2 launch open_manipulator_moveit_config open_manipulator_x_moveit.launch.py                # T2 MoveIt + RViz
ros2 run custom_package ee_commander                                                           # T3 Cartesian goals
```
Hardware changes are tested by the user on the real arm; mark fixes "not yet tested on hardware" in `known_issues.md` until confirmed.

## VLA command interface
- Discrete, low-rate goals (≤ ~1 Hz, collision-checked): `/ee_goal` → `ee_commander` (exists).
- Streaming π0.5 action chunks: **`vla_bridge`** (exists, ✅ tested 2026-11-01, Phase 3 done, known issue #10). The bridge itself is the
  websocket **client** of the policy server in the sibling repo `vla_layer` (openpi protocol, port 4000; `openpi-client` via
  `docker/install_vla_deps.sh`, run after every `./container.sh start`, which pulls the prebuilt image instead of building the Dockerfile). It sends observations (joints by name, gripper 0 open..1 closed, two 224×224 RGB images, prompt) and gets `(10, 5)` joint-space
  chunks. Front image = RealSense (`camera_front.launch.py`, subscribed with **reliable** QoS: best effort dropped ~40 % of the ~900 KB
  frames); wrist = `/camera/wrist/color/image_raw` (D435, since 2026-11-22; `wrist_image_topic: ""` = black frames for policies
  trained without it). Images sent are republished on
  `/vla_bridge/model_input/<view>`. Safety lives in `vla_safety.py` (pure numpy, `test/test_vla_safety.py`): starts disabled, NaN rejection, late-step dropping,
  limits − 0.05 rad, time-based speed cap (0.3 rad/s, or per joint via `max_joint_vel_per_joint`, e.g. the scripted demos'
  `[0.5, 0.3, 0.6, 0.8]`), start-jump check, fingertip ≥ 2 cm, watchdog, lost connection, e-stop topic.
  Output: one multi-point `JointTrajectory` per chunk → `/arm_controller/joint_trajectory`; gripper `GripperCommand` with 0.6/0.4 hysteresis.
  No MoveIt in the streaming path. Observation keys and gripper formula must match `vla_layer/omx_pi05`
  (`vla_layer/tests/test_bridge_contract.py`). Design and test ladder: `vla_layer/docs/PHASE3_GUIDE.md`.

## Conventions
- Python nodes: ROS 2 rclpy, explicit `SingleThreadedExecutor` + spin thread, terminal UI with emoji status prints (match existing style).
- Radians on all ROS interfaces; degrees only in terminal UIs.
- Commits end with the `Co-Authored-By` line; don't commit unless asked.
