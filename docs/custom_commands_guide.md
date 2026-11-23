# OpenManipulator-X Quick Commands

Short list of the commands used day to day. Run every command **inside the container**
(`cd ~/VLA/openManipulator/open_manipulator/docker && ./container.sh enter`), one terminal per block unless noted.
Full explanations: `docs/commands_guide.md`.

---

## 1. Zenoh router (always first, own terminal)

```bash
zenohd                          # alias for: ros2 run rmw_zenoh_cpp rmw_zenohd
```
Nothing can talk to anything without it (launches hang, `Unable to connect to a Zenoh router`). Leave it running.

## 2. Build and source

```bash
cd /root/ros2_ws
colcon build --symlink-install --packages-select custom_package open_manipulator_moveit_config open_manipulator_description open_manipulator_bringup
source /root/ros2_ws/install/setup.bash
```
Rebuild after a `git pull` that adds files; with `--symlink-install`, edits to existing Python/YAML/URDF files need no rebuild.
Run the `source` line in **every** new terminal.

## 3. Robot launch (interactive homing)

```bash
# Real robot, default port (/dev/ttyUSB0)
ros2 launch custom_package open_manipulator_x_interactive.launch.py

# Real robot, explicit port: check which ttyUSB the U2D2 is first (FTDI entry)
ls -l /dev/serial/by-id/
ros2 launch custom_package open_manipulator_x_interactive.launch.py port_name:=/dev/ttyUSB0

# No robot connected (mock hardware): the arm exists only in RViz
ros2 launch custom_package open_manipulator_x_interactive.launch.py use_mock_hardware:=true
```
Menu: **2** = default home, **3** = zero pose, 1 = move by hand. `z` = exit pose, then `1` back to the chosen home or `2` torque off + quit.
If MoveIt flags the home pose as colliding (`START_STATE_IN_COLLISION`, `link2 - link4`, known issue #12), pick **3** (zero pose) instead.
With mock hardware, "torque service not available" warnings are expected.

## 4. RViz

```bash
# View the robot: model driven by /joint_states through robot_state_publisher (started by the launch above)
rviz2 -d $(ros2 pkg prefix open_manipulator_description)/share/open_manipulator_description/rviz/open_manipulator.rviz

# Or start it together with the robot launch
ros2 launch custom_package open_manipulator_x_interactive.launch.py use_mock_hardware:=true start_rviz:=true

# Plain RViz (e.g. camera images: Add → By topic → /camera/front/color/image_raw → Image)
rviz2
```

## 5. Joint commander (type joint angles)

```bash
ros2 run custom_package joint_commander
>> 0 0 0 0 1          # <j1°> <j2°> <j3°> <j4°> <gripper 0|1>    (q = quit)
```
After the arm move the gripper opens (`1`) or closes (`0`) and prints its final position; "stalled" means it stopped on an object
(known issue #1, fixed 2026-11-02).

## 6. Keyboard teleop

```bash
ros2 run open_manipulator_teleop open_manipulator_x_teleop
```
`1`/`q` joint1, `2`/`w` joint2, `3`/`e` joint3, `4`/`r` joint4 (+/−), `o` open / `p` close gripper, `ESC` exit.

## 7. Joint logs

```bash
ros2 run custom_package log_joint_states                               # live joint angles, rad and degrees
ros2 run custom_package log_joint_states --ros-args -p log_rate_hz:=10.0
ros2 run custom_package log_ee_pose                                    # live end-effector x y z (m, base frame) + pitch°
ros2 run custom_package log_ee_pose --ros-args -p once:=true            # one line, then exit
```
`log_ee_pose` uses the same forward kinematics as `ee_commander`; its last column can be typed straight into `ee_commander`.

## 8. VLA bridge

```bash
# Cameras first: D455 above the table (/camera/front) + D435 on the wrist (/camera/wrist), picked by serial number.
# No camera: black frames with -p fake_images:=true (ros2 run) or fake_images:=true (launch)
ros2 launch custom_package cameras.launch.py
#   a policy trained WITHOUT wrist images: add -p wrist_image_topic:="" to the bridge (black wrist frames, as in training)

# Bridge with terminal menu: e = enable, d = disable (hold), s = status, q = quit
ros2 run custom_package vla_bridge --ros-args \
    --params-file $(ros2 pkg prefix custom_package)/share/custom_package/config/vla_bridge.yaml \
    -p server_host:=192.168.3.187

# Bridge as a launch (no terminal menu; enable with the service below). shadow:=true = the arm does not move
ros2 launch custom_package vla_bridge.launch.py server_host:=192.168.3.187 shadow:=true
ros2 service call /vla_bridge/enable std_srvs/srv/SetBool "{data: true}"
```

## 9. Bridge state (after launching the bridge)

```bash
ros2 topic echo /vla_bridge/status                                           # enabled, connected, last_fault, image_age_ms, ...
ros2 topic hz /camera/front/color/metadata                                   # camera rate, expect ~15 Hz
ros2 service call /vla_bridge/enable std_srvs/srv/SetBool "{data: false}"   # stop: hold + disable
ros2 topic pub --once /vla_bridge/estop std_msgs/msg/Bool "{data: true}"      # stop: latched e-stop
```
In the bridge's own terminal, `s` prints the same status. `last_fault` says why it stopped.

## 10. Pick-and-place pipeline test (YOLO + scripted demo, nothing recorded)

Place every **blue** object **on AprilTag 19** and every **green** object **on AprilTag 25** (red ones are ignored). Needs the Zenoh router (§1) and the robot launch (§3,
homing option 2) first. Once per container start: `bash /root/ros2_ws/src/open_manipulator/docker/install_vla_deps.sh`, with the
model in `workspace/models/yolo_cubes.pt`. Full procedure with checks and troubleshooting: `docs/finetuning_procedure.md`, section 6.

```bash
# Cameras (front D455 with depth + wrist D435) + object_detector, one launch. Leave it running. Tag 19 = "blue marker", tag 25 = "green marker".
ros2 launch custom_package pick_place_test.launch.py
ros2 launch custom_package pick_place_test.launch.py wrist:=false     # front camera only
ros2 launch custom_package pick_place_test.launch.py model_path:=/workspace/models/yolo_cubes.pt targets:="19:blue marker,25:green marker"   # the defaults

# Look at what it sees (RViz: Add → By topic → /object_detector/annotated → Image)
rviz2

# Camera → robot calibration. Once, and again whenever the camera or the robot base moves. Needs the launch above running.
# Then put the average tag height it prints into table_z in config/pick_place_test.yaml.
ros2 run custom_package calibrate_camera

# The test. Prints every plan and waits: ENTER = execute, s = skip (nothing moves), q = quit. Ctrl+C = stop the arm.
ros2 run custom_package scripted_demo --ros-args --params-file \
    $(ros2 pkg prefix custom_package)/share/custom_package/config/pick_place_test.yaml
```
**How it ends:** it counts the objects at the first look (`🧮 N object(s) to place`) and keeps going until every one is on the
its tag, then stops by itself (`🏁 N object(s) on their target`). **Failed picks are retried:** if nothing is in the gripper after
closing, the arm lifts, opens and returns to start (`✋ … backing out`); if the object is not on the marker afterwards, it is simply
picked again on the next round. Per object it gives up after 3 failed attempts, and the whole run stops after objects × 3 attempts
(`max_attempts_per_object`), so a bad object cannot loop forever. Progress line after each attempt: `📈 1 of 2 placed, 2 attempt(s), 1 failed`.

Useful variations (add to the `scripted_demo` command):
```bash
-p confirm:=false                          # run without asking for each plan (after the first successful runs)
-p max_attempts_per_object:=5              # more (or fewer) retries per object before giving up on it
-p "max_joint_vel:=[0.5, 0.3, 0.6, 0.8]"   # faster (j1..j4 rad/s; default in the test config: [0.3, 0.2, 0.4, 0.5])
-p table_z:=-0.056 -p cube_size:=0.035    # scene values: table height in the robot frame, object edge (m)
-p grasp_closed_threshold:=-0.005          # gripper position (m) at or below which "nothing is held" (closed end = -0.010)
-p "colors:=['blue']"                      # pick only blue objects (green ones are then left alone)
-p seed:=1                                 # same choices every run
```
Keys while it runs: `p` pause after this episode, ENTER continue, `q` quit after this episode.

## 11. Scripted demonstrations for fine-tuning (recorded)

Same setup as §10 (calibration done), but every episode is recorded with the commanded action, checked, labelled and saved in
`/workspace/recordings/scripted_pick_place/episode_NNNN/` (= host `open_manipulator/docker/workspace/recordings/`).
Only after the pipeline test (§10) passes. Procedure: `docs/finetuning_procedure.md`, sections 7–8.

```bash
# Pilot: 10 recorded episodes with the template prompt "pick up the {color} cube and place it in the {target}" (bin tag name from object_detector.yaml)
ros2 run custom_package scripted_demo --ros-args --params-file \
    $(ros2 pkg prefix custom_package)/share/custom_package/config/scripted_demo.yaml -p episodes:=10

# Exactly one successful episode per object, failed picks retried (also records the failed attempts, labelled failure):
# add  -p until_clear:=true  to the command above (it stops by itself when all objects are on the target)

# Bulk: until you press q (the current episode finishes first)
ros2 run custom_package scripted_demo --ros-args --params-file \
    $(ros2 pkg prefix custom_package)/share/custom_package/config/scripted_demo.yaml -p episodes:=0

# Check what was recorded
ros2 run custom_package check_episode /workspace/recordings/scripted_pick_place
```

## 12. Guided demonstrations: you choose the cube and the bin (recorded)

Same setup as §10/§11 (arm launch, `pick_place_test.launch.py` = both cameras + `object_detector`, camera calibrated).
The episodes record the front D455 **and** the wrist D435 (`wrist_topic: /camera/wrist/color/image_raw` in `guided_demo.yaml`;
`""` = front only). Instead of a random task, the script
lists every cube it sees and asks which one to pick and which bin to put it in; the prompt is built from your choice
("pick up the blue cube and place it in the red bin") and the episode is executed and recorded like §11.
Bins = AprilTags: **25 = red bin, 20 = green bin, 18 = blue bin** (`bins` in `config/guided_demo.yaml`). Episodes go to
`/workspace/recordings/pick_place_2cam/` (since 2026-11-23; the earlier `guided_pick_place/` trial episodes and the 80 `scripted_pick_place` episodes without wrist view were deleted).

```bash
ros2 run custom_package guided_demo --ros-args --params-file \
    $(ros2 pkg prefix custom_package)/share/custom_package/config/guided_demo.yaml
#   -p record:=false    run the motions without recording (trials)
```
**Arrangements (since 2026-11-23):** each session records one fixed cube → bin assignment into its own folder,
`pick_place_2cam/arr_<cube>to<bin>_...` (cubes 1 red, 2 green, 3 blue; bins 1 = tag 25 red, 2 = tag 20 green, 3 = tag 18 blue).
At startup a menu lists all 6 with their progress; you pick one, then only choose the cube (its bin follows). After
`episodes_per_arrangement` (10) successful episodes the session stops; restart for the next arrangement.

| # | Folder | red → | green → | blue → |
|---|---|---|---|---|
| 1 | `arr_1to1_2to2_3to3` | red bin | green bin | blue bin |
| 2 | `arr_1to1_2to3_3to2` | red bin | blue bin | green bin |
| 3 | `arr_1to2_2to1_3to3` | green bin | red bin | blue bin |
| 4 | `arr_1to2_2to3_3to1` | green bin | blue bin | red bin |
| 5 | `arr_1to3_2to1_3to2` | blue bin | red bin | green bin |
| 6 | `arr_1to3_2to2_3to1` | blue bin | green bin | red bin |

Far cubes (near the front camera) are grasped near their top, so they are released higher: up to `far_release_extra` (2 cm)
more than `release_clearance`, ramping between 18 and 26 cm cube distance from the base (`guided_demo.yaml`; 0 = off).
The 🎯 line's `release aim` shows the height actually used.

Skip the menu: `-p arrangement:=231` (bin number for cube 1, 2, 3 = row 4). Old free choice: `-p arrangement:=free`.
`metadata.json` gets `arrangement` and `arrangement_map`.

At the prompts: number = choose, `r` = look again, `b` = back, `q` = quit; ENTER on the 🎯 line runs it. Just before running it
re-checks that the cube is still there (follows it if it moved ≤ 3 cm). Cubes already in a bin are marked `[already in the … bin]`.
A bin whose tag the camera has never seen can't be chosen; a covered tag keeps its last seen position.
Detections that are not cubes are listed under 🚫 with the reason (a box on an AprilTag, something high above the table such as the
gripper, at the base, out of the arm's reach, a second box on the same cube); tune with `cube_max_above`, `min_radius`. There is no
workspace box: every cube the arm can reach is offered.

## 13. Move test: type the coordinates instead of the camera (no camera, no YOLO)
Tests only the moving half of the pick-and-place pipeline: typed fingertip coordinates go through the same code as
`scripted_demo`/`guided_demo` (demo_planning IK, straight-line segments, speed caps → `arm_controller`, gripper action). Needs only
the arm launch. Coordinates: robot base frame (m), z = fingertip; pitch in degrees (90 = down), optional.
```bash
ros2 run custom_package move_test --ros-args --params-file \
    $(ros2 pkg prefix custom_package)/share/custom_package/config/move_test.yaml
move> 0.20 0.05 -0.0385          # fingertip to (x, y, z), steepest reachable pitch; far moves: 5 cm above, then straight down
move> 0.20 0.05 -0.0385 80 c     # with pitch 80°, then close the gripper
move> o | c | w | r              # open | close | where am I (FK) | back to READY
move> pp 0.20 0.05 0.14 -0.15    # full planned pick-and-place, cube (0.20, 0.05) -> bin (0.14, -0.15), heights from the yaml
```
After every move it prints where the fingertip really got to (forward kinematics) and the error in mm. Goals below
`table_z + min_z_above_table` are refused (set `table_z` in `config/move_test.yaml` to the same value as the demo you test).

## 14. Two cameras: front D455 + wrist D435

| Camera | Serial | Where | Topics |
|---|---|---|---|
| D455 | `207322250030` | above the table (front) | `/camera/front/color/image_raw` **1280×800** (+ `/camera/front/aligned_depth_to_color/image_raw`, depth 848×480 aligned to it, with `depth:=true`; not 1280×720: the D455 then sees nothing closer than ~52 cm, i.e. no cube tops) |
| D435 | `927522071177` | on the gripper (wrist) | `/camera/wrist/color/image_raw` 640×480 (color only) |

All at **15 fps** (the policy's control rate; don't change it). Smaller front frames: `color_profile:=640,480,15 depth_profile:=640,480,15`.
A bigger wrist picture (`wrist_profile:=1280,720,15`) needs the D435 on USB 3. Disk: a 1280×800 front episode is ~3× the old
~180 MB (≈ 0.6 GB per 20–30 s episode with the wrist view): check `df -h` before long sessions.

```bash
rs-enumerate-devices -s          # both listed? (close realsense-viewer first: only one program can open a camera)
lsusb -t                         # D455 with depth must be on USB 3 (5000M); the color-only wrist D435 also works at 480M
ros2 launch custom_package cameras.launch.py               # both, color only (vla_bridge, record_episodes)
ros2 launch custom_package cameras.launch.py depth:=true   # + front depth (object_detector; pick_place_test.launch.py does this)
ros2 launch custom_package cameras.launch.py front_serial:=_<serial> wrist_serial:=_<serial>   # other cameras (leading _)
ros2 topic hz /camera/wrist/color/metadata                  # ~15 Hz
```
**Colour settings** of the front D455 (exposure, white balance, saturation, gamma, ...): `config/camera_front.yaml`, loaded by both
launches (`color_config:=` another file, `''` = camera defaults). Try values live while watching RViz, then copy them into the yaml:
```bash
ros2 param dump /camera/front | grep -A20 rgb_camera        # current values (incl. what auto exposure / white balance chose)
ros2 param set /camera/front rgb_camera.saturation 80
```
For the dataset, lock the colours once they look right (`enable_auto_exposure` / `enable_auto_white_balance: false` + the values
auto had chosen) and keep them for every session.

Power: each camera directly in a PC port (or a powered hub), not both on one unpowered hub: the D455 browns the hub out when it
starts streaming and every device on it (cameras, U2D2) disconnects.
YOLO and the calibration use only the front camera. A new front camera = **recalibrate** (`calibrate_camera`, §10) and re-measure
`table_z`. Recorded: `wrist_topic` in `record_episodes.yaml` / `guided_demo.yaml` (set; `scripted_demo.yaml` / `pick_place_test.yaml`:
add `wrist_topic: "/camera/wrist/color/image_raw"` to record it there too).
