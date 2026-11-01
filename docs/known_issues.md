# OpenManipulator-X Known Issues

Issues found during the code review of the `side` branch (2026-10-14). Issue #4 is being worked on now; the rest are
deferred. All paths are relative to `open_manipulator/`.

---

## 1. `joint_commander` gripper command always fails

* **File:** `custom_package/custom_package/joint_commander.py` (`JointCommanderNode.__init__`, `move_gripper`)
* **Problem:** The gripper client sends a `FollowJointTrajectory` goal to `/gripper_controller/follow_joint_trajectory`.
  `gripper_controller` is a `position_controllers/GripperActionController`
  (`open_manipulator_bringup/config/open_manipulator_x/hardware_controller_manager.yaml`), which only serves a
  `control_msgs/action/GripperCommand` action on `/gripper_controller/gripper_cmd`. The same mapping appears in `moveit_controllers.yaml`.
* **Symptom:** Every command waits 5 s for a server that never appears, then prints "Action server not available for gripper".
  The arm still moves.
* **Fix direction:** Use a `GripperCommand` action client on `/gripper_controller/gripper_cmd` (`command.position`, `command.max_effort`).

## 2. Minimum-stress exit pose is outside the URDF joint2 limit  *(🔧 fixed 2026-10-14, not yet tested on hardware)*

* **Files:** `custom_package/custom_package/interactive_homing_node.py` (`min_stress_exit_position`); joint limits in
  `open_manipulator_description/urdf/open_manipulator_x/*`
* **Problem:** The exit pose (after the #4 recalibration: `[0.6090, -1.8300, 1.5892, 0.4847]`) has j2 = −1.830 rad and
  j3 = 1.589 rad, outside the URDF limits j2 `[-1.5, 1.5]` and j3 `[-1.5, 1.4]`.
* **Symptom:** `arm_controller` still executes the move. While the arm is in this pose, MoveIt reports the start state as
  out of bounds and refuses to plan, and `joint_commander` rejects the pose. Option 1 (moving the arm by hand) can also leave joints
  outside the limits.
* **Fix direction:** After issue #4 is resolved, pick an exit pose that is within the limits, or widen the limit if the
  hardware range has been verified.
* **Fix:** Moving the exit pose onto the old limits made the arm drop when torque was cut. The exit pose is now the measured torque-off
  resting pose `[0.4633, -1.8190, 1.4972, 0.4019]` (j2 rests at −1.8392, so it is held just inside the servo Min Position Limit −1.8193).
  The URDF limits (all 3 copies) and `joint_commander` `JOINT_LIMITS` are widened to j2 `[-1.82, 1.5]` and j3 `[-1.5, 1.55]`, within
  the servo limits. After torque-off the arm rests 0.02 rad past j2's lower limit, which is within MoveIt's 0.05 rad start-state
  tolerance. Option 1 can still leave joints outside the limits.

## 3. Option 1 can jerk briefly when torque is re-enabled  *(🔧 fixed 2026-10-14, not yet tested on hardware)*

* **File:** `custom_package/custom_package/interactive_homing_node.py` (`main`, Option 1 branch)
* **Problem:** The node calls `set_torque(True)` *before* `sync_controller_goal_to_current()`. On torque enable,
  `DynamixelHardware::ChangeDxlTorqueState()` copies the current state into its command buffer. However, `arm_controller` writes its
  own hold position (captured when the controller activated, before the arm was moved by hand) on every 10 ms cycle, which replaces that copy.
* **Symptom:** The motors can move briefly toward the old activation pose until the sync goal arrives.
* **Fix direction:** Send the sync goal first and wait for it to be accepted, then enable torque.
* **Fix:** Option 1 now deactivates `arm_controller` (`/controller_manager/switch_controller`) before the hand-move, then after
  ENTER enables torque (the driver copies state → command) and re-activates it. The JTC (`set_last_command_interface_value_as_state_on_activation`,
  default true) then holds the measured pose. If deactivation fails, it falls back to sync-goal-then-torque.

## 4. +45° URDF offset on joint3/joint4 hides a servo zero-offset problem  *(✅ resolved 2026-10-14)*

* **Files:** `rpy="0 ${pi/4} 0"` in `open_manipulator_description/urdf/open_manipulator_x/open_manipulator_x_arm.urdf.xacro`,
  `open_manipulator_description/urdf/open_manipulator_x/open_manipulator_x.urdf`, and
  `open_manipulator_moveit_config/config/open_manipulator_x.urdf`
* **Problem:** The horns are mounted on their alignment marks, yet RViz and the real arm disagree by 45° on joint3 and joint4.
  Rotating the joint origins in the URDF makes RViz match, but it changes MoveIt's FK/IK and collision model, and it shifts the meaning of
  the URDF joint limits by 45°.
* **Likely cause:** A non-zero `Homing Offset` (EEPROM addr 20) on servos ID 13/14. 45° is exactly 512 ticks
  (`rad = ticks × 0.0015340 − π`). The OMX ros2_control xacro never writes `Homing Offset`, so a leftover value persists. The OMY xacros
  set it to 0 explicitly (e.g. `omy_f3m_position.ros2_control.xacro`). Less likely: Drive Mode direction bit, or a different reference
  zero pose.
* **Diagnosis:**
  ```bash
  for id in 11 12 13 14 15; do
    ros2 service call /dynamixel_hardware_interface/get_dxl_data dynamixel_interfaces/srv/GetDataFromDxl \
      "{id: $id, item_name: 'Homing Offset', timeout_sec: 1.0}"
  done   # values are uint32: -512 reads as 4294966784
  ```
  Then move the arm by hand (torque off) into the ROBOTIS zero pose and check `ros2 run custom_package log_joint_states` reads ≈ 0.
  Repeat at a second pose to confirm the error is a constant offset.
* **Fix direction:** Add `<param name="Homing Offset">0</param>` to the `dxl1`–`dxl4` gpio blocks in
  `open_manipulator_description/ros2_control/open_manipulator_x_position.ros2_control.xacro`. Revert the three URDF origins to
  `rpy="0 0 0"`. Re-record the home and exit poses, because their j3/j4 readings shift by ∓0.785 rad.
  **Update 2026-10-14:** register dump confirmed non-zero Homing Offsets (ID12 +154, ID13 −545, ID14 −664). Full analysis,
  deciding measurement and both fix variants: `docs/joint_offset_investigation.md`.

## 5. Configuration duplicated across many files

* **Problem:** The home pose `[0.6090, -1.3622, 1.3898, -0.0154]` is written in about 8 places:
  `interactive_homing_node.py`, `custom_package/config/configured_home_position.yaml`, the bringup
  `initial_positions.yaml` and `configured_home_position.yaml`, both SRDFs, the ros2_control `initial_value`s, and the docs.
  The URDF and SRDF also exist as copies (`open_manipulator_moveit_config/config/open_manipulator_x.{urdf,srdf}` vs. the
  description URDF and `config/open_manipulator_x/open_manipulator_x.srdf`). The static MoveIt URDF does not pick up later xacro edits.
* **Symptom:** It's easy to update one copy and miss another (issue #4's fix will require touching all of them).
* **Fix direction:** Have the nodes read poses from one YAML file. Generate the MoveIt URDF from the xacro at launch instead of keeping
  a static copy.

## 6. Documentation out of date

* `docs/openmanipulator_changes_summary.md` says 3 executables are registered. There are 4 (`joint_commander` is missing).
* `joint_commander` and homing **Option 3 (Zero Pose)** are not documented in `docs/commands_guide.md`.
* The `InteractiveHomingNode` docstring and the `listen_for_z_key` messages say `z` returns to the *session home*. It actually goes to
  the minimum-stress exit pose.

## 7. Option 3 (Zero Pose) drove the gripper into the base while the URDF workaround was active  *(✅ resolved with #4)*

* **File:** `custom_package/custom_package/interactive_homing_node.py` (Option 3)
* **Problem:** With the +45° joint3/joint4 workaround, `[0,0,0,0]` puts the gripper pointing straight down. The model places the
  tip about 1 cm below the base plane.
* **Fix direction:** Resolve issue #4 first, or remove/guard Option 3 until then.

## 8. `get_dxl_data` service times out and stalls the control loop

* **Component:** `dynamixel_hardware_interface` (upstream; do not modify, see isolation policy)
* **Problem:** Every `/dynamixel_hardware_interface/get_dxl_data` call timed out (`result=False`). Each call caused a ~1 s
  `controller_manager` overrun.
* **Workaround:** Read registers with the launch stopped (see Appendix A of `docs/joint_offset_investigation.md`).

## 9. `interactive_homing_node` crashes on exit and never disables torque itself  *(🔧 fixed 2026-10-14, not yet tested on hardware)*

* **File:** `custom_package/custom_package/interactive_homing_node.py` (`main`)
* **Problem:** After the `z` exit move, the node calls `rclpy.shutdown()` while its `rclpy.spin` thread is still running. The process aborts
  with `terminate called without an active exception` (exit code −6). The node never calls `set_torque(False)`. It relies on the driver's
  `stop()` running at shutdown, but on 2026-10-14 `stop()` did not run when the launch was interrupted, and all servos stayed torque-enabled.
* **Fix direction:** Disable torque explicitly after the exit move, and stop the executor and join the spin thread before `rclpy.shutdown()`.
* **Fix:** After the `z` move a menu asks [1] return to session home (loop, `z` armed again) or [2] quit. Quit calls
  `set_torque(False)`, shuts down an explicit `SingleThreadedExecutor`, joins the spin thread, then calls `rclpy.shutdown()`. An
  `OnProcessExit` → `Shutdown` handler in `open_manipulator_x_interactive.launch.py` ends the launch when the node exits.

## 10. `vla_bridge` had not run on ROS or hardware yet  *(✅ resolved 2026-11-01)*

* **Files:** `custom_package/custom_package/vla_bridge.py`, `vla_safety.py`, `config/vla_bridge.yaml`, `launch/vla_bridge.launch.py`
* **State:** `test/test_vla_safety.py` passes (11 tests). The node's logic was exercised off-ROS (ROS stubbed, a simulated arm following the
  published trajectories, against `vla_layer/scripts/fake_policy_server.py`): a normal 15 s run (34 chunks, 0 holds, ≤ 0.25 rad/s) and all
  six fault drills (nan, jump, below_table, stall, disconnect → hold + disable; out_of_range → clamped) behaved as designed. That run found
  and fixed two bugs: the speed cap was per step instead of per second (0.58 rad/s seen), and a lost server connection resumed silently.
* **Review 2026-10-30, fixed:** a terminal `d` could race the control loop and let one more trajectory out after the hold (now one lock
  serializes all arm commands); a stalled server blocked the worker forever (now the connection is closed on chunk timeout and the worker
  reconnects); the next-chunk request ignored the 15 Hz tick delay (threshold now `ceil(latency·fps) + 2`); Ctrl+C could raise during the
  final hold; camera-conversion errors were reported as a lost server; `typing_extensions` was missing from the Dockerfile install.
  Re-tested off-ROS with a servo-lagging simulated arm: normal runs at 150/200/250 ms latency (steady speed within ~10 % of the planned
  path, plan never ran dry), all six drills, and re-enable after `stall`/`disconnect`.
* **Resolved 2026-11-01 (Phase 3 done):** built with colcon and run in ROS against the fake policy server. Rung 1 on mock hardware
  works (smooth motion in RViz), the fault drills were tested, and the shadow switch behaves as designed (shadow on: the arm stays
  still; shadow off: the arm moves). Latency from the robot PC is not logged yet. Test ladder: `vla_layer/docs/PHASE3_GUIDE.md` §7.

## 11. Front camera input to `vla_bridge`  *(🔧 new 2026-11-01, not yet tested with the real arm)*

* **Files:** `custom_package/launch/camera_front.launch.py`, `custom_package/custom_package/vla_bridge.py`, `config/vla_bridge.yaml`
* **Tested 2026-11-01** on the development laptop (D435 serial 927522071177, firmware 5.17.3.10) with mock hardware and the fake policy
  server: 30 s enabled, 84 chunks, 0 holds, newest frame ≤ 68 ms old, round trip ~153 ms; the server received 224×224×3 uint8 front
  images showing the scene (correct colours, letterbox bars from 640×480) and black wrist images.
* **Found and fixed:** with best-effort QoS the bridge received only ~9 Hz with gaps up to 0.46 s (each ~900 KB frame is many packets;
  losing one loses the frame) and faulted with `camera images stale (front)`. Reliable QoS, depth 1: 14.9 Hz, max gap 0.079 s.
* **Open:** the D435 was on a USB 2.1 link (the driver warns "Reduced performance"); 640×480 color at 15 Hz fits, but use a USB 3 port and
  cable before enabling depth or higher resolutions. Not yet run with the real arm or on the robot PC, and the test used Fast DDS;
  check the image rate again under `rmw_zenoh_cpp` (the container's interactive default). No wrist camera yet (black frames).
  Auto exposure / white balance are still on (Phase 4 plan T4.2 wants them fixed before recording demos).
