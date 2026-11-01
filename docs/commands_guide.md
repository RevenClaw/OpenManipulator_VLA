# OpenManipulator-X Launch & Control Commands Guide

This guide contains the exact step-by-step commands to build, launch, teleoperate, and control the **OpenManipulator-X** robot arm using your custom package and MoveIt 2.

---

## 🛠️ Step 0: Build Packages Inside Container

Whenever you enter the Docker container or make code changes, build the workspace:

```bash
cd /root/ros2_ws
colcon build --symlink-install --packages-select custom_package open_manipulator_moveit_config open_manipulator_description open_manipulator_bringup
source /root/ros2_ws/install/setup.bash
```

---

## 🚀 Step 1: Launch the Hardware & Interactive Homing (Terminal 1)

In your primary container terminal, run:

```bash
ros2 launch custom_package open_manipulator_x_interactive.launch.py
```

### Homing Options on Startup:
* **Option 1 (Custom Manual Home)**: Motor torque is disabled immediately so you can move the arm freely by hand to your desired initial pose. Press **ENTER** in the terminal to lock torque cleanly without any movement or jerk.
* **Option 2 (MoveIt Default Home `[0.6090, -1.3622, 1.3898, -0.0154]`)**: Torque locks and smoothly moves the arm over 5 seconds to the recommended home position.

> [!IMPORTANT]
> **Best Practice & Safe Exit Routine (`z` key)**:
> - **Operational Guidance**: Unless necessary, always return the robot to the default home position before ending your session.
> - **Minimum Stress Exit Position**: When you press **`z`** in the launch terminal to exit, the robot arm will automatically move to the minimum-stress exit position (`j1: +0.4633 rad (+26.5°), j2: -1.8190 rad (-104.2°), j3: +1.4972 rad (+85.8°), j4: +0.4019 rad (+23.0°)`), the arm's measured torque-off resting pose. It then asks: **`1`** returns to the session home position and listens for `z` again; **`2`** disables torque on all motors, quits the node and shuts down the whole launch.
> - **Why**: Transitioning to this minimum-stress position ensures that when motor torque is disabled upon disconnection, gravity and arm cantilever forces do not cause a sudden physical jerk or drop.

---

## 🎮 Step 2: Keyboard Teleoperation (Terminal 2)

Open a **second** container terminal (`./container.sh enter`):

```bash
source /root/ros2_ws/install/setup.bash
ros2 run open_manipulator_teleop open_manipulator_x_teleop
```

### Keyboard Controls:
| Joint / Function | Increase (+ Step) | Decrease (- Step) |
| :--- | :---: | :---: |
| **Joint 1** (Base Waist) | `1` | `q` |
| **Joint 2** (Shoulder) | `2` | `w` |
| **Joint 3** (Elbow) | `3` | `e` |
| **Joint 4** (Wrist) | `4` | `r` |
| **Gripper** | `o` (Open) | `p` (Close) |
| **Exit Teleop** | `ESC` | `ESC` |

---

## 🦾 Step 3: Launch MoveIt 2 & RViz (Terminal 2 or 3)

Open another container terminal (`./container.sh enter`):

```bash
source /root/ros2_ws/install/setup.bash
ros2 launch open_manipulator_moveit_config open_manipulator_x_moveit.launch.py
```

### How to Use MoveIt 2 in RViz:
1. RViz automatically detects your physical robot's live joint position (**Start State** and **Goal State** default to `<current>`).
2. Drag the **3D interactive ring gizmo** attached to the end-effector to your desired 3D spatial position $(x, y, z)$.
3. In the left **MotionPlanning** panel:
   * Click **Plan** to compute an obstacle-free trajectory.
   * Click **Execute** to send trajectory goals to the physical robot arm!

---

## 🎯 Step 3b: Command the End Effector by Coordinates (Terminal 3 or 4)

Needs Step 1 (hardware) **and** Step 3 (MoveIt) running. Then:

```bash
source /root/ros2_ws/install/setup.bash
ros2 run custom_package ee_commander
```

The node solves the 4-DOF inverse kinematics itself (closed form; MoveIt's IK is position-only for this arm and ignores pitch),
then sends the joint goal to MoveIt `move_group`, which plans a collision-free path and executes it on `arm_controller`.
The gripper is driven via `/gripper_controller/gripper_cmd` (GripperCommand) after the arm arrives.

**Terminal input:** `<x_m> <y_m> <z_m> <pitch°> [<gripper 0|1>]`, e.g. `0.20 0.00 0.15 0 1`. `p` prints the current pose, `q` quits.

* **Frame:** `world` (= `link1`, the robot base). x forward, y left, z up, in metres. The pose is that of `end_effector_link`
  (between the fingertips).
* **Pitch:** 0 = gripper horizontal, +90° = pointing straight down. Yaw always follows the base direction (`atan2(y, x)`)
  and roll is always 0; the 4-joint arm cannot set them independently.
* **Reach:** for reference, the zero pose puts the end effector at `x=0.286, z=0.1875`. Targets the arm can only reach by
  leaning back over its base are not supported.

**From other nodes:** publish `std_msgs/Float64MultiArray` on `/ee_goal` with `[x_m, y_m, z_m, pitch_rad, gripper]`
(gripper: 1 open, 0 close, -1 unchanged). A new goal replaces one that has not started yet; the running motion is not
interrupted.
```bash
ros2 topic pub --once /ee_goal std_msgs/msg/Float64MultiArray "{data: [0.20, 0.0, 0.15, 0.0, -1]}"
```

**Parameters:** `velocity_scaling` (0.3), `acceleration_scaling` (0.3), `planning_time` (5.0 s), `min_z` (0.0 m, targets
below are rejected), e.g. `ros2 run custom_package ee_commander --ros-args -p velocity_scaling:=0.1`.

---

## 📊 Step 4: Continuous Real-Time Joint State Logging (Terminal 2 or 3)

To continuously print and stream live joint angles formatted in Radians and Degrees:

```bash
source /root/ros2_ws/install/setup.bash
ros2 run custom_package log_joint_states
```

### Optional Logging Frequency Parameter:
```bash
ros2 run custom_package log_joint_states --ros-args -p log_rate_hz:=10.0
```

---

## 🤖 Step 5: VLA Bridge — let a policy server drive the arm (✅ tested 2026-11-01; front camera tested 2026-11-01 on mock hardware)

Streams action chunks from the π0.5 policy server (`vla_layer`) to the arm behind a safety layer. Design and test ladder:
`vla_layer/docs/PHASE3_GUIDE.md`. **Starts disabled.** Keep a hand on the hardware e-stop when live.

**Once per container:** `openpi-client` is in the Dockerfile; in an older container install it by hand:
```bash
pip install --break-system-packages typing_extensions "openpi-client @ git+https://github.com/Physical-Intelligence/openpi.git@215abfb#subdirectory=packages/openpi-client"
```

**Front camera (Intel RealSense D435):** the bridge now uses it by default (`fake_images: false`). Start it before the bridge:
```bash
ros2 launch custom_package camera_front.launch.py        # /camera/front/color/image_raw, rgb8 640x480 @ 15 Hz, color only
ros2 topic hz /camera/front/color/metadata               # expect ~15 Hz (hz on image_raw itself under-reads: the Python tool is slow on big images)
```
There is no wrist camera yet: `wrist_image_topic: ""` sends black frames for that view. To run without any camera, add
`-p fake_images:=true` to the bridge. Check what the model receives: `ros2 run rqt_image_view rqt_image_view /vla_bridge/model_input/front`
(the exact 224×224 image sent), `s` in the bridge shows `image_age_ms`, and `fake_policy_server.py --save-images <dir>` on the server
saves the received images as PNG and logs their shape every 5 s.

**Rung 1, mock hardware (no arm):**
```bash
# T1: simulated arm. It starts at HOME; the homing node's "torque service not available" errors are expected (no servos).
ros2 launch custom_package open_manipulator_x_interactive.launch.py use_mock_hardware:=true
# T2, on the A30 server (its env already has omx_pi05 + openpi-client; stop serve.py first, it uses the same port):
#     cd ~/vla/openpi/omx_pie0.5 && source scripts/env.sh && python scripts/fake_policy_server.py
# T3: bridge with terminal menu (e = enable, d = disable, s = status, q = quit)
ros2 run custom_package vla_bridge --ros-args --params-file \
    $(ros2 pkg prefix custom_package)/share/custom_package/config/vla_bridge.yaml -p server_host:=<server-ip>
```
Fault drills (mock hardware only): restart the fake server with `--fault nan|jump|out_of_range|below_table|stall|disconnect`;
each must end in a hold with the reason printed (`out_of_range` instead keeps running with the joint clamped).

**Rung 3, shadow mode on the real arm** (the arm does not move; watch `/vla_bridge/preview_path` in RViz):
```bash
ros2 launch custom_package vla_bridge.launch.py server_host:=<server-ip> shadow:=true
ros2 service call /vla_bridge/enable std_srvs/srv/SetBool "{data: true}"
```

**Rung 4, live on the real arm:** same without `shadow:=true`. Stop it with any of:
```bash
ros2 service call /vla_bridge/enable std_srvs/srv/SetBool "{data: false}"   # hold + disable
ros2 topic pub --once /vla_bridge/estop std_msgs/msg/Bool "{data: true}"      # latched e-stop
ros2 topic echo /vla_bridge/status                                             # why it stopped
```
Server address: the A30's IP, or `localhost` through `ssh -N -L 4000:localhost:4000 shradul@iras-hub`.
