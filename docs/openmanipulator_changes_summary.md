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
   * Registered 5 executable entry points: `record_home_position`, `interactive_homing_node`, `log_joint_states`, `joint_commander`, and `ee_commander` (Cartesian end-effector goals: closed-form IK + MoveIt plan/execute, see `commands_guide.md` Step 3b).
2. **[`custom_package/interactive_homing_node.py`](file:///home/vla/VLA/openManipulator/open_manipulator/custom_package/custom_package/interactive_homing_node.py)**:
   * **Instant Torque Unlock**: Automatically invokes `/dynamixel_hardware_interface/set_dxl_torque(enable=false)` on node startup so the user can position the physical arm freely by hand.
   * **Option 1 (Manual Hand Pose)**: Prompts user to position arm and press `ENTER`. `arm_controller` is deactivated (`/controller_manager/switch_controller`) while the arm is moved by hand. After `ENTER`, torque is enabled and `arm_controller` is re-activated, so it holds the new pose instead of pulling back to its old hold target.
   * **Option 2 (Default Home `[0.6090, -1.3622, 1.3898, -0.0154]`)**: Enables torque and sends a 2-stage 5.0-second smooth trajectory via `send_safe_trajectory()`.
   * **Minimum-Stress Exit Routine (`z` key)**: Listens directly to `/dev/tty`. Pressing `z` drives the arm smoothly over 5 seconds to the **minimum-stress exit position** (`[0.4633, -1.8190, 1.4972, 0.4019]`, the measured torque-off resting pose), then offers **[1]** return to the session home and keep listening for `z`, or **[2]** disable torque on all motors and quit, which also ends the launch.
3. **[`custom_package/log_joint_states.py`](file:///home/vla/VLA/openManipulator/open_manipulator/custom_package/custom_package/log_joint_states.py)**:
   * Continuous live telemetry node listening to `/joint_states`. Prints joint angles formatted in Radians and Degrees at configurable Hz.
4. **[`custom_package/launch/open_manipulator_x_interactive.launch.py`](file:///home/vla/VLA/openManipulator/open_manipulator/custom_package/launch/open_manipulator_x_interactive.launch.py)**:
   * Launch file spawning `ros2_control_node`, `robot_state_publisher`, `joint_state_broadcaster`, `arm_controller`, `gripper_controller`, and `interactive_homing_node`.
5. **[`custom_package/config/configured_home_position.yaml`](file:///home/vla/VLA/openManipulator/open_manipulator/custom_package/config/configured_home_position.yaml)**:
   * Configured home pose values: `[0.6090, -1.3622, 1.3898, -0.0154]`.

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
