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
```

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
