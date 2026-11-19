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
# Front camera first. No camera: black frames with -p fake_images:=true (ros2 run) or fake_images:=true (launch)
ros2 launch custom_package camera_front.launch.py

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
# Camera (with depth) + object_detector, one launch. Leave it running. Tag 19 = "blue marker", tag 25 = "green marker".
ros2 launch custom_package pick_place_test.launch.py
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
-p table_z:=0.012 -p cube_size:=0.03       # scene values: table height in the robot frame, object edge (m)
-p grasp_closed_threshold:=0.014           # gripper position (m) at or above which "nothing is held" (closed end = 0.019)
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
