# OpenManipulator-X Comprehensive Changes & Architectural Summary

This document provides a complete, itemized technical record of all modifications, custom packages, configurations, node logic, and documentation added to the OpenManipulator-X workspace.

---

## 🏗️ 1. Architectural Design & Constraints

* **Strict Isolation Policy**: The 3 external driver repositories (`DynamixelSDK`, `dynamixel_hardware_interface`, `dynamixel_interfaces`) were kept **100% pristine and unmodified**.
* **Standalone Package (`custom_package`)**: All new nodes, homing menus, logging scripts, and custom launch files were isolated inside a dedicated ROS 2 package at `open_manipulator/custom_package/`.
* **Zero-Jerk Homing & Minimum-Stress Exit**: Enforced a 2-stage smooth trajectory system to prevent current overload shutdowns on Joint 2, coupled with an automatic minimum-stress exit routine on `z` keypress.

---

## 📁 2. Detailed File-by-File Changes

### A. New Custom Package (`open_manipulator/custom_package/`)
1. **[`package.xml`](file:///home/vla/VLA/openManipulator/open_manipulator/custom_package/package.xml)** & **[`setup.py`](file:///home/vla/VLA/openManipulator/open_manipulator/custom_package/setup.py)**:
   * Created standard Python ROS 2 package configuration.
   * Registered 6 executable entry points: `record_home_position`, `interactive_homing_node`, `log_joint_states`, `joint_commander`, `ee_commander`, and `vla_bridge` (see item 6) (Cartesian end-effector goals: closed-form IK + MoveIt plan/execute, see `commands_guide.md` Step 3b).
2. **[`custom_package/interactive_homing_node.py`](file:///home/vla/VLA/openManipulator/open_manipulator/custom_package/custom_package/interactive_homing_node.py)**:
   * **Instant Torque Unlock**: Automatically invokes `/dynamixel_hardware_interface/set_dxl_torque(enable=false)` on node startup so the user can position the physical arm freely by hand.
   * **Option 1 (Manual Hand Pose)**: Prompts user to position arm and press `ENTER`. `arm_controller` is deactivated (`/controller_manager/switch_controller`) while the arm is moved by hand. After `ENTER`, torque is enabled and `arm_controller` is re-activated, so it holds the new pose instead of pulling back to its old hold target.
   * **Option 2 (Default Home `[0.6090, -1.3622, 1.3898, -0.0154]`)**: Enables torque and sends a 2-stage 5.0-second smooth trajectory via `send_safe_trajectory()`.
   * **Minimum-Stress Exit Routine (`z` key)**: Listens directly to `/dev/tty`. Pressing `z` drives the arm smoothly over 5 seconds to the **minimum-stress exit position** (`[0.4633, -1.8190, 1.4972, 0.4019]`, the measured torque-off resting pose), then offers **[1]** return to the session home and keep listening for `z`, or **[2]** disable torque on all motors and quit, which also ends the launch.
3. **[`custom_package/log_joint_states.py`](file:///home/vla/VLA/openManipulator/open_manipulator/custom_package/custom_package/log_joint_states.py)**:
   * Continuous live telemetry node listening to `/joint_states`. Prints joint angles formatted in Radians and Degrees at configurable Hz.
   * **`custom_package/log_ee_pose.py`** (2026-11-19): same idea for the end effector: x, y, z (m, `world` = base frame) and pitch (°) of `end_effector_link`, from `/joint_states` through `ee_commander.forward_kinematics`. Params `log_rate_hz` (5.0), `once` (print one line and exit). Tested read-only against the running arm.
4. **[`custom_package/launch/open_manipulator_x_interactive.launch.py`](file:///home/vla/VLA/openManipulator/open_manipulator/custom_package/launch/open_manipulator_x_interactive.launch.py)**:
   * Launch file spawning `ros2_control_node`, `robot_state_publisher`, `joint_state_broadcaster`, `arm_controller`, `gripper_controller`, and `interactive_homing_node`.
5. **[`custom_package/config/configured_home_position.yaml`](file:///home/vla/VLA/openManipulator/open_manipulator/custom_package/config/configured_home_position.yaml)**:
   * Configured home pose values: `[0.6090, -1.3622, 1.3898, -0.0154]`.
6. **`custom_package/vla_bridge.py`** + **`custom_package/vla_safety.py`** (2026-10-30; ✅ tested 2026-11-01: mock hardware, fault drills, shadow/live switch):
   * `vla_bridge` (6th entry point) streams action chunks from the π0.5 policy server (`vla_layer`, openpi websocket, port 4000) to
     `arm_controller` (one multi-point `JointTrajectory` per chunk on `/arm_controller/joint_trajectory`) and `gripper_controller`
     (`GripperCommand`, 0.6/0.4 hysteresis). Observations: `/joint_states` (joints by name, gripper normalized 0 open..1 closed),
     main + wrist camera (RGB, 224×224 `resize_with_pad`) or black frames (`fake_images`), prompt.
   * Safety layer (`vla_safety.py`, pure numpy, unit-tested in `test/test_vla_safety.py`): starts disabled; NaN rejection;
     late-step dropping; joint limits − 0.05 rad; **time-based speed cap 0.3 rad/s** from the measured position; start-jump
     check 0.15 rad; fingertip height ≥ 2 cm (FK); watchdog (`/joint_states` 0.1 s, images 0.2 s, chunk 1 s); lost server
     connection; e-stop topic. Any fault holds the arm and disables. `shadow:=true` publishes to `/vla_bridge/preview` only.
   * `config/vla_bridge.yaml`, `launch/vla_bridge.launch.py`. Design: `vla_layer/docs/PHASE3_GUIDE.md`.
   * `docker/Dockerfile` installs `openpi-client` (pinned to openpi `215abfb`, the commit the server uses).
7. **Front camera for `vla_bridge`** (2026-11-01, tested with a D435 on mock hardware, *not yet with the real arm*):
   * `launch/camera_front.launch.py`: wraps `realsense2_camera`'s `rs_launch.py` (namespace `camera`, name `front`), color only
     (depth/infrared/IMU/pointcloud off), `640,480,15` → `/camera/front/color/image_raw` (rgb8). Args `serial_no`, `color_profile`.
     `package.xml` exec-depends on `realsense2_camera` (already built in the Docker image).
   * `vla_bridge`: `fake_images` now defaults to `false`; a view whose topic is `""` gets black frames and no staleness check
     (wrist, for now); camera subscriptions use reliable QoS, depth 1 (`image_reliable_qos`), because best effort delivered only
     9 Hz with 0.46 s gaps from the 15 Hz D435 and tripped the 0.2 s image watchdog; the exact 224×224 images sent are republished on
     `~/model_input/<front|wrist>`; status shows `image_age_ms`; the warmup waits (with a message) until every camera has a frame.
8. **`custom_package/joint_commander.py` gripper fix** (2026-11-02, tested on mock hardware, *not yet tested on hardware*):
   * The gripper uses `GripperCommand` on `/gripper_controller/gripper_cmd` instead of a `FollowJointTrajectory` goal to an action that
     doesn't exist (known issue #1). Reports the final position; a stall (e.g. on an object) counts as success.
   * Clean exit: explicit `SingleThreadedExecutor`, shut down and joined before `rclpy.try_shutdown()` (it aborted with `terminate called`).
9. **Episode recorder: `record_episodes.py`, `check_episode.py`, `episode_tools.py`** (2026-11-02, tested on mock hardware with stand-in
   camera/teleop publishers, *not yet tested on hardware*; known issue #13):
   * `record_episodes` (7th entry point): terminal tool. Pre-flight (stream freshness/rates via raw subscriptions, task, disk, arm at
     HOME), `ros2 bag record` as a subprocess (`--disable-keyboard-controls`, stdin closed, own process group, SIGINT to stop), 1 s
     lead-in, live stream monitor, quick check, s/f/d label, `metadata.json`, 2-stage return to HOME with the gripper open
     (same path as `interactive_homing_node`), optional teleop pause/resume via `/teleop/enable`. `config/record_episodes.yaml`.
   * `check_episode` (8th entry point): per-topic count/rate/largest gap of recorded episodes from receive times (`rosbag2_py`).
   * `episode_tools.py`: the pure parts (naming, HOME path, analysis, metadata), unit-tested in `test/test_episode_tools.py`.
   * `package.xml`: exec-depends on `ros2bag`, `rosbag2_py`, `rosbag2_storage_mcap` (all in the Docker image).
10. **Scripted demonstrations for fine-tuning** (2026-11-05, tested in simulation, *not yet tested on hardware*; known issue #14;
    procedure `docs/finetuning_procedure.md`):
   * New nodes `object_detector`, `calibrate_camera`, `scripted_demo` (entry points 9–11) and pure modules `perception_tools.py`
     (Kabsch, projection, robust depth, AprilTags for OpenCV 4.6 and ≥ 4.7) and `demo_planning.py` (waypoints, pitch choice, IK,
     minimum-jerk segments with per-joint caps, Cartesian descents, random variation, success check), unit-tested in
     `test/test_perception_tools.py` and `test/test_demo_planning.py`. Configs `object_detector.yaml`, `scripted_demo.yaml`.
     Ideas from the `yolo_pipeline` project (Kabsch calibration, YOLO wrapper, AprilTag bins), reimplemented for ROS topics, metres
     and the 4-DOF arm.
   * `recording.py`: `StreamMonitor` and `BagRecorder` moved out of `record_episodes.py` (shared with `scripted_demo`).
   * **READY start pose** (`episode_tools.READY`) for every demo and policy run; `record_episodes` uses it by default
     (`start_pose`). Starting episodes from the folded HOME doubled their length (~65 s → ~27 s median).
   * `vla_bridge`: optional `max_joint_vel_per_joint` (per-joint speed caps; default unchanged: 0.3 for all joints).
   * `camera_front.launch.py`: `depth:=true` adds depth aligned to color (`depth_profile`).
   * Pipeline test (2026-11-19): `config/pick_place_test.yaml` + `launch/pick_place_test.launch.py`; `scripted_demo` got `confirm`,
     `remember_targets` and `place_offsets` (+ `demo_planning.place_offset`), so objects can be placed ON a tag.
   * `scripted_demo until_clear` (on in the test config): counts the objects, retries failed picks (cap `max_attempts_per_object` per
     object and objects × that in total), stops by itself; `run_plan`/`back_out` abort a failed grasp (lift, open, READY).
   * Gripper-open check (2026-11-21): `scripted_demo.ensure_gripper_open` (+ `demo_planning.gripper_is_open`) runs before every pick
     and pauses the loop if the gripper does not measure open after one retry. Params `check_gripper_open` (true),
     `gripper_open_tolerance` (0.004 m).
   * `guided_demo` (2026-11-21, entry point + `config/guided_demo.yaml` + `test/test_guided_demo.py`): subclass of `ScriptedDemo` whose
     `choose_task` lists the cubes and bins and asks the operator (cube, then bin, then ENTER); prompt from the template, cube re-checked
     just before running. `scripted_demo`: `ScriptedDemo(node_name=…)` and `main(args, node_class, title)` so the run loop is shared.
   * Real cube size 3.5 cm (2026-11-21): defaults `cube_size` 0.035 / `release_clearance` 0.015 (`demo_planning.PlanConfig`,
     `scripted_demo`, `guided_demo.yaml`) and the detector's `object_size` 0.035 (`object_detector.py` + `.yaml`).
   * Height diagnostics (2026-11-21): `scripted_demo.height_report` prints under every 🎯 plan line the camera z of the cube (centre)
     and of the tag next to the planned grasp / release z and `table_z`; `guided_demo`'s menu lists z (camera) and aim z per cube and bin.
     Print only: the planned heights still come from the config.
   * `move_test` (2026-11-21, entry point + `config/move_test.yaml` + `test/test_move_test.py`): terminal `x y z [pitch] [o|c]`,
     `o`/`c`, `pp cx cy bx by`, `w`, `r` through the demos' motion code (subclass of `ScriptedDemo`: `execute`, `gripper`,
     `run_plan`), no camera. Reports the reached fingertip pose and error after each move; floor = `table_z + min_z_above_table`.
     Not yet tested on hardware.
   * Cube screening (2026-11-21, `demo_planning.screen_cubes`, used by `scripted_demo` and `guided_demo`): YOLO boxes are ignored if they
     contain a visible AprilTag's centre (tags detected as blue cubes), if their centre is > `cube_max_above` (0.03 m) above a cube on the
     table (robot parts / gripper detected as blue), closer than `min_radius` (0.08 m) to the base, outside the workspace, or a second box
     on the same cube (same colour within 2 cm). `guided_demo` lists the ignored detections with the reason.
   * No workspace box any more (2026-11-21): `workspace_x` / `workspace_y` removed (still present in older yamls, ignored). A cube is
     pickable if the arm can reach it: `demo_planning.cube_reachable` (IK for the grasp point and 7 cm above it at some pitch in
     `min_pitch_deg..max_pitch_deg`); e.g. a cube beside the base at x -0.04, y 0.20 is now offered.
   * Two cameras (2026-11-22, not yet tested on hardware): D455 above the table = front, D435 on the gripper = wrist. New
     `launch/cameras.launch.py` starts both by serial number (`front_serial` _207322250030, `wrist_serial` _927522071177, each in a
     scoped group so `rs_launch.py` arguments don't leak; `depth:=true` for the front, `wrist:=false` = front only);
     `camera_front.launch.py` defaults to the D455's serial (no more "first device"); `pick_place_test.launch.py` uses
     `cameras.launch.py` (`wrist` argument). Wrist topic = `/camera/wrist/color/image_raw` (was the placeholder
     `/camera/wrist/image_raw`): set in `record_episodes.yaml`, `guided_demo.yaml`, `vla_bridge.yaml`, the `record_episodes`
     default, `episode_tools` rates. `scripted_demo` keeps `wrist_topic: ""` by default. The new front camera needs a new
     `calibrate_camera` run and `table_z`.
   * Front resolution (2026-11-22, not yet tested on hardware): D455 color 1280×800 (was 640×480) and depth 848×480, still 15 fps. Depth
     1280×720 was tried first: its ~52 cm minimum distance gave the cube tops (0.49 m from the camera) 4 % valid pixels → "no depth";
     848×480 has ~35 cm (`color_profile` / `depth_profile` in `camera_front.launch.py` and `cameras.launch.py`). The wrist D435
     stays 640×480 (`wrist_profile`; it is on USB 2). Intrinsics come live from `camera_info`, so detection and calibration are
     unaffected; the policy still gets 224×224. Episodes are ~3× larger on disk.
   * Front colour settings (2026-11-22, not yet tested on hardware): `config/camera_front.yaml` (flat `rgb_camera.*` parameters:
     auto exposure / white balance, brightness, contrast, saturation, gamma, sharpness, power line frequency), passed to
     `rs_launch.py`'s `config_file` via the `color_config` argument of `camera_front.launch.py` / `cameras.launch.py`.
   * Gripper polarity (2026-11-19, known issue #15): this arm's gripper is inverted w.r.t. ROBOTIS's convention. Fixed in the real-hardware driver
     config: servo 15's `[unit info]` in `open_manipulator_x_position.ros2_control.xacro` is mirrored (factor negated, offset 0.045099321), so
     everything above the driver (and mock hardware) keeps the stock +0.019 open / −0.010 close.
   * `docker/install_vla_deps.sh` (openpi-client + CPU PyTorch + ultralytics 8.3.40 + OpenCV 4.10, numpy < 2) and the same pins in
     the Dockerfile. `package.xml`: `python3-yaml`, `python3-opencv`.

---

### B. MoveIt 2 Configuration Updates (`open_manipulator/open_manipulator_moveit_config/`)
1. **[`config/open_manipulator_x/open_manipulator_x.srdf`](file:///home/vla/VLA/openManipulator/open_manipulator/open_manipulator_moveit_config/config/open_manipulator_x/open_manipulator_x.srdf)** & **[`config/open_manipulator_x.srdf`](file:///home/vla/VLA/openManipulator/open_manipulator/open_manipulator_moveit_config/config/open_manipulator_x.srdf)**:
   * Updated `<group_state name="home">` and `<group_state name="init">` to `[0.6090, -1.3622, 1.3898, -0.0154]`.
2. **[`config/open_manipulator_x.urdf`](file:///home/vla/VLA/openManipulator/open_manipulator/open_manipulator_moveit_config/config/open_manipulator_x.urdf)**:
   * Created top-level evaluated URDF inside `config/` to eliminate MoveIt 2 launch-time fallback warnings (`Cannot infer URDF ...`). Kinematics are stock ROBOTIS; it must stay identical to the description URDF.
3. **[`launch/open_manipulator_x_moveit.launch.py`](file:///home/vla/VLA/openManipulator/open_manipulator/open_manipulator_moveit_config/launch/open_manipulator_x_moveit.launch.py)**:
   * Updated `MoveItConfigsBuilder` parameters to explicitly load `config/open_manipulator_x.urdf` and `config/open_manipulator_x.srdf`.
4. **[`config/moveit.rviz`](file:///home/vla/VLA/openManipulator/open_manipulator/open_manipulator_moveit_config/config/moveit.rviz)**:
   * Configured default `Query Start State` and `Query Goal State` parameters to `<current>`.

---

### C. Bringup Configuration Updates (`open_manipulator/open_manipulator_bringup/`)
1. **[`config/open_manipulator_x/initial_positions.yaml`](file:///home/vla/VLA/openManipulator/open_manipulator/open_manipulator_bringup/config/open_manipulator_x/initial_positions.yaml)**:
   * Updated `home` pose parameter to `[0.6090, -1.3622, 1.3898, -0.0154]`.
2. **[`config/open_manipulator_x/configured_home_position.yaml`](file:///home/vla/VLA/openManipulator/open_manipulator/open_manipulator_bringup/config/open_manipulator_x/configured_home_position.yaml)**:
   * Updated stored home pose to `[0.6090, -1.3622, 1.3898, -0.0154]`.

---

### D. Robot Description Updates (`open_manipulator/open_manipulator_description/`)
1. **Joint-Zero Calibration (Servo `Homing Offset`, replaces the earlier +45° URDF workaround)**:
   * The joint3/joint4 URDF origins are back to stock `rpy="0 0 0"` in all three URDF copies.
   * The real cause was non-zero `Homing Offset` values stored in the servos' EEPROM (ID13 −545 ticks ≈ −47.9°). The calibration is now
     pinned in the ros2_control xacro (item 2). See [`docs/joint_offset_investigation.md`](file:///home/vla/VLA/openManipulator/docs/joint_offset_investigation.md).
2. **[`ros2_control/open_manipulator_x_position.ros2_control.xacro`](file:///home/vla/VLA/openManipulator/open_manipulator/open_manipulator_description/ros2_control/open_manipulator_x_position.ros2_control.xacro)**:
   * Added `<param name="initial_value">` to position state interfaces for joints 1-4 and gripper:
     * `joint1`: `0.6090` rad (+34.9°)
     * `joint2`: `-1.3622` rad (-78.0°)
     * `joint3`: `1.3898` rad (+79.6°)
     * `joint4`: `-0.0154` rad (-0.9°)
     * `gripper_left_joint`: `-0.01` m
   * Ensures mock simulation hardware (`mock_components/GenericSystem`) spawns and initializes the arm directly at the working home position.
   * Added `Homing Offset`, `Min Position Limit` and `Max Position Limit` to gpio blocks `dxl1`–`dxl4` (IDs 11–14:
     −17 / 0–4095, 166 / 862–3506, 38 / 712–3094, −124 / 715–3340). The driver writes them to EEPROM at init with torque off.

---

### E. Documentation Files Created
1. **[`docs/commands_guide.md`](file:///home/vla/VLA/openManipulator/docs/commands_guide.md)**: Detailed step-by-step commands guide for building, launching hardware homing, keyboard teleop, MoveIt 2 RViz, and real-time joint logging.
2. **[`docs/known_issues.md`](file:///home/vla/VLA/openManipulator/docs/known_issues.md)** & **[`docs/joint_offset_investigation.md`](file:///home/vla/VLA/openManipulator/docs/joint_offset_investigation.md)**: Open issues list and the joint-zero calibration investigation.
3. **[`docs/openmanipulator_changes_summary.md`](file:///home/vla/VLA/openManipulator/docs/openmanipulator_changes_summary.md)**: Exhaustive architectural summary of all project changes.

---

## 🎯 3. Positional Reference Guide

| Position Name | Radians $[j_1, j_2, j_3, j_4]$ | Degrees $(j_1, j_2, j_3, j_4)$ | Purpose & Usage |
| :--- | :--- | :--- | :--- |
| **Working Default Home** | `[0.6090, -1.3622, 1.3898, -0.0154]` | $(+34.9^\circ, -78.0^\circ, +79.6^\circ, -0.9^\circ)$ | Primary operational home position for startup, MoveIt, RViz, and teleop. |
| **Minimum-Stress Exit** | `[0.4633, -1.8190, 1.4972, 0.4019]` | $(+26.5^\circ, -104.2^\circ, +85.8^\circ, +23.0^\circ)$ | Reached on **`z`** keypress; then [1] back to session home or [2] torque off and quit. |

---

## 🛠️ 4. Quick Build & Execution Commands

```bash
# Build updated workspace
cd /root/ros2_ws
colcon build --symlink-install --packages-select custom_package open_manipulator_moveit_config open_manipulator_description open_manipulator_bringup
source /root/ros2_ws/install/setup.bash

# Terminal 1: Launch Interactive Hardware Homing
ros2 launch custom_package open_manipulator_x_interactive.launch.py

# Terminal 2: Keyboard Teleop
ros2 run open_manipulator_teleop open_manipulator_x_teleop

# Terminal 3: MoveIt 2 & RViz
ros2 launch open_manipulator_moveit_config open_manipulator_x_moveit.launch.py

# Terminal 4: Real-Time Joint State Logging
ros2 run custom_package log_joint_states
```
