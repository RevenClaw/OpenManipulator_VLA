# OpenManipulator-X Controller Stack & Custom Package Summary

## Executive Summary
This document provides a comprehensive summary of the architecture analysis, modifications, and new ROS 2 custom control features developed for the **OpenManipulator-X** robotic arm.

All custom control logic has been engineered into a dedicated, isolated package named **`custom_package`** inside the main `open_manipulator` repository. No modifications were made to the external controller stack repositories (`DynamixelSDK`, `dynamixel_hardware_interface`, `dynamixel_interfaces`).

---

## 🏗️ Repository & Architecture Overview

The software stack operates within a ROS 2 Jazzy Docker container (`ros:jazzy-ros-base`) managed via s6-overlay and Zenoh RMW (`rmw_zenoh_cpp`):

```
 +-----------------------------------------------------------------------------------------+
 |                                  Docker Container                                       |
 |                       (open_manipulator/docker/Dockerfile)                               |
 |                                                                                         |
 |  +-----------------------------------------------------------------------------------+  |
 |  |                            open_manipulator                                       |  |
 |  |  - open_manipulator_bringup (Launch scripts & configs)                            |  |
 |  |  - open_manipulator_description (URDF, Xacro, ros2_control specs)                 |  |
 |  |  - open_manipulator_moveit_config (MoveIt 2 motion planning & kinematics)       |  |
 |  |  - open_manipulator_teleop / gui / playground / collision                           |  |
 |  |  - custom_package [NEW] (Interactive homing, manual teaching, session manager)    |  |
 |  +----------------------------------------+------------------------------------------+  |
 |                                           |                                             |
 |                                      ros2_control                                       |
 |                                           v                                             |
 |  +-----------------------------------------------------------------------------------+  |
 |  |                       dynamixel_hardware_interface                                |  |
 |  |  - DynamixelHardware C++ Plugin (hardware_interface::SystemInterface)             |  |
 |  |  - Services: set_dxl_torque, reboot_dxl, get_dxl_data, set_dxl_data                  |  |
 |  +-------------------+---------------------------------------+-----------------------+  |
 |                      |                                       |                          |
 |                      v                                       v                          |
 |  +--------------------------------------+   +----------------------------------------+  |
 |  |         dynamixel_interfaces         |   |              DynamixelSDK              |  |
 |  |  - DynamixelState.msg                |   |  - Low-level serial communication      |  |
 |  |  - GetDataFromDxl / SetDataToDxl srv |   |  - Protocol 1.0 / 2.0 packet handler   |  |
 |  +--------------------------------------+   +-------------------+--------------------+  |
 +-----------------------------------------------------------------|-----------------------+
                                                                   |
                                                            /dev/ttyUSB0 (U2D2)
                                                                   v
                                                       [ OpenManipulator-X ]
```

---

## 📦 Custom Package (`custom_package`)

Located at: `open_manipulator/custom_package/`

### Directory Structure
```
open_manipulator/
└── custom_package/
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    ├── resource/
    │   └── custom_package
    ├── custom_package/
    │   ├── __init__.py
    │   ├── record_home_position.py
    │   └── interactive_homing_node.py
    ├── config/
    │   └── configured_home_position.yaml
    └── launch/
        ├── open_manipulator_x_custom.launch.py
        └── open_manipulator_x_interactive.launch.py
```

### Container Mounting Rationale
The Docker compose setup ([`docker-compose.yml`](file:///home/vla/VLA/openManipulator/open_manipulator/docker/docker-compose.yml)) mounts host path `../` to `/root/ros2_ws/src/open_manipulator/`. Therefore, any package placed inside `open_manipulator/` on the host machine automatically appears inside `/root/ros2_ws/src/open_manipulator/` inside the running container.

---

## 🎮 Key Features Developed

### 1. Interactive 2-Option Homing Startup Menu
When launching `open_manipulator_x_interactive.launch.py`, the terminal displays an interactive homing menu:

* **Option 1: Custom Home Position (Manual Hand Positioning)**:
  * Motor torque is disabled (`SetBool(data=False)`).
  * The user positions the arm by hand and presses **ENTER** in the terminal.
  * Torque is enabled (`SetBool(data=True)`).
  * `sync_controller_goal_to_current()` immediately updates `arm_controller`'s target goal to match the manual pose, eliminating motor jerk.
  * This manual pose is saved as `session_home_position`.

* **Option 2: MoveIt Default Home Position (`[0.0, -1.0, 0.7, 0.3]`)**:
  * Motor torque is enabled.
  * `send_safe_trajectory()` executes a smooth 2-stage trajectory over 5 seconds (lifting joint 3 first before tilting joint 2).
  * This prevents peak current spikes and eliminates Dynamixel overload protection shutdowns (`0x20` error).
  * Saved as `session_home_position`.

### 2. Return-Home and Quit (`z` Keypress Listener)
* While the robot operates, `interactive_homing_node` listens on `/dev/tty` for keypress **`z`**.
* Pressing **`z`** smoothly drives the arm back to the `session_home_position` established at startup.
* Upon arrival, it logs confirmation and cleanly terminates the launch process.
* Backup ROS 2 service: `ros2 service call /go_home_and_quit std_srvs/srv/Trigger`.

### 3. MoveIt 2 Automatic Live Pose Detection
* Updated [`open_manipulator_moveit_config/config/moveit.rviz`](file:///home/vla/VLA/openManipulator/open_manipulator/open_manipulator_moveit_config/config/moveit.rviz) with:
  ```yaml
  Query Start State:
    State: <current>
  Query Goal State:
    State: <current>
  ```
* RViz automatically initializes both Start State and Goal State to `<current>`, syncing the MoveIt 3D visualizer to the physical robot's live joint position on launch.

---

## 🛠️ Build & Run Commands

### 1. Rebuilding Packages inside Container
```bash
cd /root/ros2_ws
colcon build --symlink-install --packages-select custom_package open_manipulator_moveit_config
source /root/ros2_ws/install/setup.bash
```

### 2. Launching Interactive Control Stack
```bash
ros2 launch custom_package open_manipulator_x_interactive.launch.py
```

### 3. Launching Keyboard Teleoperation (Optional Terminal 2)
```bash
ros2 run open_manipulator_teleop open_manipulator_x_teleop
```

### 4. Launching MoveIt 2 Planning (Optional Terminal 2)
```bash
ros2 launch open_manipulator_moveit_config open_manipulator_x_moveit.launch.py
```
