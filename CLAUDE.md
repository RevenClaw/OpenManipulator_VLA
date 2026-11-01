# OpenManipulator-X VLA Controller — Project Context

ROS 2 **Jazzy** workspace (runs in Docker, `open_manipulator/docker/`, workspace at `/root/ros2_ws` inside the container) for a
ROBOTIS **OpenMANIPULATOR-X** (4-DOF arm + 1 gripper, Dynamixel XM430, IDs 11–15, U2D2 on `/dev/ttyUSB1`, 1 Mbps).
Goal: a controller stack that a separate **VLA (vision-language-action) node** can drive.
Work happens on branch `side`; `main` is the PR target.

Read `docs/` before changing behaviour. It is the record of what was changed and why:
- `docs/openmanipulator_changes_summary.md` — itemized list of every change (start here)
- `docs/commands_guide.md` — build / launch / teleop / MoveIt / `ee_commander` usage
- `docs/known_issues.md` — open & fixed issues (numbered; refer to them by number)
- `docs/joint_offset_investigation.md` — servo Homing Offset calibration (why the URDF is stock again)
- `docs/SYSTEM_WORKFLOW_ARCHITECTURE_MANUAL.md` — architecture, node graph, topics/services/actions
- `docs/input_needed.md` — how a VLA node sends end-effector goals to `ee_commander` via `/ee_goal`, and the limits of that path

Keep these docs updated when behaviour changes (the user expects it; issue #6 is about docs drifting).

## Repository layout
- `DynamixelSDK/`, `dynamixel_hardware_interface/`, `dynamixel_interfaces/` — upstream drivers. **Never modify (isolation policy).**
- `open_manipulator/` — ROBOTIS packages (description, bringup, moveit_config, teleop, gui, …) with local config edits.
- `open_manipulator/custom_package/` — **all our own code** (ament_python):
  - `interactive_homing_node.py` — startup menu: [1] hand-pose (deactivates `arm_controller`, torque off, ENTER → torque on + reactivate),
    [2] default home via 2-stage trajectory (elbow first to avoid joint2 overload `0x20`), [3] zero pose.
    `z` key → min-stress exit pose, then [1] back to session home or [2] torque off + quit (ends the launch).
  - `joint_commander.py` — terminal: `<j1°> <j2°> <j3°> <j4°> <gripper 0|1>` → `FollowJointTrajectory` on `arm_controller`.
    Defines `JOINT_LIMITS`, `GRIPPER_OPEN_M` (0.019), `GRIPPER_CLOSE_M` (−0.010). Gripper path is broken (issue #1).
  - `ee_commander.py` — Cartesian goals. Closed-form 4-DOF IK (`inverse_kinematics`, `forward_kinematics`) → MoveIt `/move_action`
    (plan + execute) → gripper via `GripperCommand`. Inputs: terminal `x y z pitch° [grip]` or topic `/ee_goal`
    (`std_msgs/Float64MultiArray` `[x, y, z, pitch_rad, gripper(1 open/0 close/-1 keep)]`). Latest-goal-wins queue, no preemption.
  - `vla_bridge.py` + `vla_safety.py` — π0.5 policy server → arm, behind a safety layer (see "VLA command interface").
  - `log_joint_states.py`, `record_home_position.py`
  - `launch/open_manipulator_x_interactive.launch.py` — hardware bringup + homing node (main launch).

## Robot facts
- Controllers (100 Hz, `open_manipulator_bringup/config/open_manipulator_x/hardware_controller_manager.yaml`):
  - `arm_controller` — `JointTrajectoryController`, joints `joint1..joint4`, position command,
    action `/arm_controller/follow_joint_trajectory`, topic `/arm_controller/joint_trajectory`. `allow_partial_joints_goal: true`.
  - `gripper_controller` — `GripperActionController` on `gripper_left_joint` (metres), **action only**: `/gripper_controller/gripper_cmd`.
  - `joint_state_broadcaster` → `/joint_states`.
- Joint limits (rad): j1 [−π, π], j2 [−1.82, 1.5], j3 [−1.5, 1.55], j4 [−1.7, 1.97]. Gripper [−0.010, 0.019] m.
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
cd /root/ros2_ws
colcon build --symlink-install --packages-select custom_package open_manipulator_moveit_config open_manipulator_description open_manipulator_bringup
source install/setup.bash
ros2 launch custom_package open_manipulator_x_interactive.launch.py port_name:=/dev/ttyUSB1   # T1 hardware + homing
ros2 launch open_manipulator_moveit_config open_manipulator_x_moveit.launch.py                # T2 MoveIt + RViz
ros2 run custom_package ee_commander                                                           # T3 Cartesian goals
```
Hardware changes are tested by the user on the real arm; mark fixes "not yet tested on hardware" in `known_issues.md` until confirmed.

## VLA command interface
- Discrete, low-rate goals (≤ ~1 Hz, collision-checked): `/ee_goal` → `ee_commander` (exists).
- Streaming π0.5 action chunks: **`vla_bridge`** (exists, ✅ tested 2026-11-01, Phase 3 done, known issue #10). The bridge itself is the
  websocket **client** of the policy server in the sibling repo `vla_layer` (openpi protocol, port 4000; `openpi-client` installed by the
  Dockerfile). It sends observations (joints by name, gripper 0 open..1 closed, two 224×224 RGB images, prompt) and gets `(10, 5)` joint-space
  chunks. Safety lives in `vla_safety.py` (pure numpy, `test/test_vla_safety.py`): starts disabled, NaN rejection, late-step dropping,
  limits − 0.05 rad, time-based 0.3 rad/s speed cap, start-jump check, fingertip ≥ 2 cm, watchdog, lost connection, e-stop topic.
  Output: one multi-point `JointTrajectory` per chunk → `/arm_controller/joint_trajectory`; gripper `GripperCommand` with 0.6/0.4 hysteresis.
  No MoveIt in the streaming path. Observation keys and gripper formula must match `vla_layer/omx_pi05`
  (`vla_layer/tests/test_bridge_contract.py`). Design and test ladder: `vla_layer/docs/PHASE3_GUIDE.md`.

## Conventions
- Python nodes: ROS 2 rclpy, explicit `SingleThreadedExecutor` + spin thread, terminal UI with emoji status prints (match existing style).
- Radians on all ROS interfaces; degrees only in terminal UIs.
- Commits end with the `Co-Authored-By` line; don't commit unless asked.
