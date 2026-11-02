# OpenManipulator-X Launch & Control Commands Guide

This guide contains the exact step-by-step commands to build, launch, teleoperate, and control the **OpenManipulator-X** robot arm using your custom package and MoveIt 2.

---

## 🔌 Step −1: Start the Zenoh Router (once per container start, own terminal)

The container uses `rmw_zenoh_cpp`. Nodes only find each other through a Zenoh router, and **nothing starts it automatically**.
Without it everything hangs (the controller spawner never finishes, the homing menu never appears, MoveIt/ee_commander wait
for robot state, and every node warns `Unable to connect to a Zenoh router`). Start it first and leave it running:
```bash
zenohd                          # alias for: ros2 run rmw_zenoh_cpp rmw_zenohd
pgrep -af rmw_zenohd            # check from any terminal that it runs
```
Nodes started before the router connect on their own once it is up.

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

In your primary container terminal, run **one** of:

```bash
# Real robot. The U2D2 port can change after a replug/reboot (ttyUSB0 or ttyUSB1); check first:
ls -l /dev/serial/by-id/          # the FTDI_USB__-__Serial_Converter entry points to the U2D2's ttyUSBx
ros2 launch custom_package open_manipulator_x_interactive.launch.py port_name:=/dev/ttyUSB0

# Fake (mock) robot, no arm connected: ros2_control mirrors every command straight back as the joint state.
ros2 launch custom_package open_manipulator_x_interactive.launch.py use_mock_hardware:=true
```
With the mock robot the arm appears only in RViz. It starts at HOME, and the homing node's "torque service not available"
warnings are expected (there are no servos); pick option **2** and everything downstream (MoveIt, `ee_commander`, `vla_bridge`)
works as with the real arm.

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

After `git pull`, rebuild so new launch files (e.g. `camera_front.launch.py`) are installed:
```bash
cd /root/ros2_ws && colcon build --symlink-install --packages-select custom_package && source install/setup.bash
```
<<<<<<< Updated upstream
=======
There is no wrist camera yet: `wrist_image_topic: ""` sends black frames for that view. To run without any camera, add
`-p fake_images:=true` to the bridge. View the camera in RViz (`rviz2`, the image has no `rqt_image_view`): **Add → By topic →**
`/camera/front/color/image_raw` **→ Image** for the live feed, `/vla_bridge/model_input/front` **→ Image** for the exact 224×224 image sent, `s` in the bridge shows `image_age_ms`, and `fake_policy_server.py` on the server logs the received joint state and
images every 5 s (`🤖 state [...]  📷 front_image: (224, 224, 3) uint8 ...`); `--save-images <dir>` also saves the images (PNG) and
the matching states and prompt (`observations.csv`).
>>>>>>> Stashed changes

### 5a. Check the front camera on its own (no robot, no bridge)

The bridge uses the Intel RealSense D435 by default (`fake_images: false`). Check it first:
```bash
# 1. Is the camera seen? (only one program can open it at a time: run this while nothing else uses the camera)
rs-enumerate-devices -s          # lists name, serial number, firmware and USB type (3.x is best; 2.x works at 640x480 @ 15 Hz)

# 2. Start the ROS camera node (T1)
ros2 launch custom_package camera_front.launch.py        # /camera/front/color/image_raw, rgb8 640x480 @ 15 Hz, color only

# 3. Check it publishes (T2)
ros2 topic list | grep camera/front
ros2 topic hz /camera/front/color/metadata               # expect ~15 Hz (hz on image_raw itself under-reads: the Python tool is slow on big images)
ros2 topic echo --once /camera/front/color/camera_info --field width   # 640
```
**See the picture:** `rviz2` → **Add** → **By topic** → `/camera/front/color/image_raw` → **Image**.
(`rqt_image_view` is not installed in the container; `apt install ros-jazzy-rqt-image-view` adds it, then
`ros2 run rqt_image_view rqt_image_view /camera/front/color/image_raw`.)
Without ROS at all: stop the camera launch and run `realsense-viewer`.
With two RealSense cameras attached, pick one by serial: `camera_front.launch.py serial_no:=_<serial>` (note the leading `_`).

There is no wrist camera yet: `wrist_image_topic: ""` sends black frames for that view. To run the bridge without any camera,
add `-p fake_images:=true`. To check what the model receives while the bridge runs: in RViz **Add → By topic →**
`/vla_bridge/model_input/front` **→ Image** (the exact 224×224 image sent), `s` in the bridge shows `image_age_ms`, and `fake_policy_server.py --save-images <dir>`
on the server saves the received images as PNG and logs their shape every 5 s.

### 5b. Rung 1: fake (mock) robot, no arm connected

```bash
# T1: simulated arm (pick option 2 in the menu; the "torque service not available" warnings are expected)
ros2 launch custom_package open_manipulator_x_interactive.launch.py use_mock_hardware:=true

# T2: front camera (or skip it and add -p fake_images:=true to the bridge in T4)
ros2 launch custom_package camera_front.launch.py

# T3, on the A30 server (its env already has omx_pi05 + openpi-client; stop serve.py first, it uses the same port):
#     cd ~/vla/openpi/omx_pie0.5 && source scripts/env.sh && python scripts/fake_policy_server.py
#     (or the real policy: serve.py)

# T4: bridge with terminal menu (e = enable, d = disable, s = status, q = quit). Works from any directory:
ros2 run custom_package vla_bridge --ros-args \
    --params-file $(ros2 pkg prefix custom_package)/share/custom_package/config/vla_bridge.yaml \
    -p server_host:=192.168.3.187            # A30 server IP (as of 2026-11-01); add -p fake_images:=true if no camera
```
`--params-file vla_bridge.yaml` (a bare file name) only works when run from `custom_package/config/`; the path above works anywhere.
Fault drills (mock hardware only): restart the fake server with `--fault nan|jump|out_of_range|below_table|stall|disconnect`;
each must end in a hold with the reason printed (`out_of_range` instead keeps running with the joint clamped).

### 5c. Real robot

Same terminals as 5b, but T1 is the real arm (`port_name:=/dev/ttyUSB0`, check the port as in Step 1) and the camera must be running.

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

---

## 🎬 Step 5b: Record demonstrations (*not yet tested on hardware*, known issue #13)

Records teleoperated episodes for training: one rosbag2 folder per episode plus `metadata.json`, in exactly the format the converter on
the server reads (`vla_layer/docs/PHASE5_GUIDE.md`). Needs: the arm (Step 1), the front camera (`camera_front.launch.py`), and the teleop
node publishing `/teleop/action` (Phase 4, not built yet).
```bash
ros2 run custom_package record_episodes --ros-args --params-file \
    $(ros2 pkg prefix custom_package)/share/custom_package/config/record_episodes.yaml \
    -p task:="pick up the red cube and place it in the bowl" -p operator:=<name>
```
The task must be **exactly** the prompt used later at run time. Keys:
* **ENTER**: pre-flight check (every stream alive and fast enough, task set, ≥ 5 GB free, arm at HOME) → it suggests an object
  position (1–9 in turn) → **ENTER** starts recording → **🟢 GO** after a 1 s lead-in → do the task → **ENTER** stops →
  quick check per topic → label: **s** success, **f** failure (kept, not trained on), **d** discard (deleted) → back to HOME, gripper open.
* **h** go HOME (2-stage, elbow first), **c** show stream rates, **l** session summary, **q** quit.

Saved to `/workspace/recordings/<task>/episode_NNNN/` = host `open_manipulator/docker/workspace/recordings/` (git-ignored, survives
container rebuilds). About 0.5 GB per 20 s episode with two cameras. Check a session afterwards and copy it to the A30:
```bash
ros2 run custom_package check_episode /workspace/recordings/pick_up_the_red_cube_and_place_it_in_the_bowl
rsync -avP open_manipulator/docker/workspace/recordings/pick_up_the_red_cube_and_place_it_in_the_bowl/ \
    shradul@iras-hub:~/vla/openpi/omx_pie0.5/data/rosbags/pick_cube/          # on the host, from low_level_controller/
```
Options in `config/record_episodes.yaml`: `wrist_topic` (set once the wrist camera exists), `extra_topics`, `storage_preset`
(`none` if the quick check shows dropped camera frames), `max_duration`, `grid_positions`, `auto_home`. If a teleop node offers
`/teleop/enable` (SetBool), the recorder pauses it before going HOME and resumes it before the next episode; without it, pause teleop
yourself before pressing `h`.

---

## ✅ Step 6: Full Test Checklist (everything built so far)

Run Part A (no robot) first; start Part B (real arm) only when Part A passes. Report results so `docs/known_issues.md` can be
updated (#2, #3, #9 and #11 are still "not yet tested on hardware").

**Every terminal (host):**
```bash
cd ~/VLA/openManipulator/open_manipulator/docker && ./container.sh enter
source /root/ros2_ws/install/setup.bash
```
**First, in its own terminal:** `zenohd` (the Zenoh router, Step −1). Nothing works without it.
**Once after a `git pull`:**
```bash
cd /root/ros2_ws && colcon build --symlink-install --packages-select custom_package open_manipulator_moveit_config open_manipulator_description open_manipulator_bringup
source install/setup.bash
```
**Once per container:** `openpi-client` (see Step 5).

### Part A — no robot connected (mock robot)

**A0. Safety-layer unit tests**
```bash
cd /root/ros2_ws/src/open_manipulator/custom_package && python3 -m pytest test/test_vla_safety.py -q    # expect: 11 passed
```

**A1. Mock robot, homing menu and RViz** (T1, keep running)
```bash
ros2 launch custom_package open_manipulator_x_interactive.launch.py use_mock_hardware:=true
```
1. Pick **2**: the RViz arm goes to home. The "torque service not available" warnings are expected (no servos).
2. Press `z`: the arm goes to the folded exit pose.
3. Press `1`: it goes back to home.
4. Press `z` again, then `2`: the node quits and the whole launch ends, with no `terminate called` crash (#9).
5. Relaunch it for the next steps.

**A2. Joint commander** (T2)
```bash
ros2 run custom_package joint_commander
>> 0 0 0 0 1          # RViz arm goes to the L-shaped zero pose
```
The gripper opens (`1`) or closes (`0`) after the arm move and prints its final position; `stalled` means it stopped on an object (#1, fixed 2026-11-02).

**A3. MoveIt + coordinate commander** (T2: MoveIt, T3: commander)
```bash
ros2 launch open_manipulator_moveit_config open_manipulator_x_moveit.launch.py     # wait for "You can start planning now!"
ros2 run custom_package ee_commander --ros-args -p velocity_scaling:=0.1 -p min_z:=0.03
>> p                  # prints the current pose
>> 0.20 0 0.15 0 1    # 20 cm forward, 15 cm up, gripper level, then open
```
From home, MoveIt refuses with `START_STATE_IN_COLLISION` (#12). Move off home first with `joint_commander`, e.g.
`34.9 -78 77.6 -1 1` (home with j3 2° less folded) or `0 0 0 0 1`.

**A4. Front camera on its own**
```bash
rs-enumerate-devices -s                              # D435 listed? (run while nothing else uses the camera)
ros2 launch custom_package camera_front.launch.py    # T4, keep running
ros2 topic hz /camera/front/color/metadata           # expect ~15 Hz
rviz2                                                # Add → By topic → /camera/front/color/image_raw → Image
```
Details and alternatives (`rqt_image_view`, `realsense-viewer`, choosing a camera by serial): Step 5a.

**A5. VLA bridge with the fake policy server** (keep A1 and A4 running)
```bash
# On the A30 server (stop serve.py first, it uses the same port):
cd ~/vla/openpi/omx_pie0.5 && source scripts/env.sh && python scripts/fake_policy_server.py
# T5:
ros2 run custom_package vla_bridge --ros-args \
    --params-file $(ros2 pkg prefix custom_package)/share/custom_package/config/vla_bridge.yaml \
    -p server_host:=192.168.3.187            # add -p fake_images:=true if the camera is not running
```
1. Press `s`: it shows connected, with a small `image_age_ms`.
2. Press `e`: the RViz arm starts moving.
3. Press `d`: it holds.
4. Check the model input: view `/vla_bridge/model_input/front` in RViz (the exact 224×224 image sent).
5. Fault drills: restart the fake server with `--fault nan`, then `jump`, `out_of_range`, `below_table`, `stall` and `disconnect`.
   Each must end in a hold with the reason printed (`out_of_range` instead keeps running with the joint clamped).

### Part B — real robot

Keep the arm's movement area clear and a hand on the power.

**B1. Homing on the real arm**
```bash
ls -l /dev/serial/by-id/          # find the ttyUSB number (FTDI entry = U2D2)
ros2 launch custom_package open_manipulator_x_interactive.launch.py port_name:=/dev/ttyUSB0
```
1. **Option 2:** the arm moves smoothly to home, and RViz matches the real arm.
2. **Option 1 (#3):** move the arm by hand, press ENTER. It locks **without jerking**.
3. **`z` then `2` (#2, #9):** the arm folds to its rest pose and drops at most ~1° when torque cuts. The launch ends cleanly.
4. **Torque check:** afterwards the arm moves freely by hand. If it doesn't, torque stayed on (#9).
5. **Option 3:** the arm goes to the L-shaped zero pose without hitting the base.

**B2. `joint_commander`, then `ee_commander`:** same as A2 and A3, with speeds at 0.1. The first `ee_commander` target should
be close to the arm's current pose (`p` shows it).

**B3. VLA bridge in shadow mode** (the arm does not move)
```bash
ros2 launch custom_package camera_front.launch.py
ros2 launch custom_package vla_bridge.launch.py server_host:=192.168.3.187 shadow:=true
ros2 service call /vla_bridge/enable std_srvs/srv/SetBool "{data: true}"
```
Watch `/vla_bridge/preview_path` in RViz: the planned paths should look sensible (#11).

**B4. VLA bridge live:** same as B3 without `shadow:=true`. Stop it with any of:
```bash
ros2 service call /vla_bridge/enable std_srvs/srv/SetBool "{data: false}"   # hold + disable
ros2 topic pub --once /vla_bridge/estop std_msgs/msg/Bool "{data: true}"      # latched e-stop
ros2 topic echo /vla_bridge/status                                             # why it stopped
```
