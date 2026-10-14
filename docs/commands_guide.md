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
* **Option 2 (MoveIt Default Home `[0.6090, -1.3806, 0.4955, -0.8437]`)**: Torque locks and smoothly moves the arm over 5 seconds to the recommended home position.

> [!IMPORTANT]
> **Best Practice & Safe Exit Routine (`z` key)**:
> - **Operational Guidance**: Unless necessary, always return the robot to the default home position before ending your session.
> - **Minimum Stress Exit Position**: When you press **`z`** in the launch terminal to exit, the robot arm will automatically move to the minimum-stress exit position (`j1: +0.6090 rad (+34.9°), j2: -1.8484 rad (-105.9°), j3: +0.6949 rad (+39.8°), j4: -0.3436 rad (-19.7°)`).
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
