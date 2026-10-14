# Joint 3 / Joint 4 RViz Misalignment — Investigation & Fix

**Status:** ✅ Resolved and verified on hardware 2026-10-14. Calibration in §9; the stock URDF matches the physical arm.
**Date:** 2026-10-14 · **Branch:** `side` · **Related:** `docs/known_issues.md` issue #4

---

## 1. Symptom

With the stock URDF, the arm in RViz did not overlay the physical arm, even though every servo horn is mounted on its
alignment mark. Commit `8f8ddc5` ("offset configuration to match with rviz") worked around this by rotating the joint
origins of joint3 and joint4 by +45° in all three URDF copies:

```xml
<origin xyz="0.024 0 0.128" rpy="0 ${pi/4} 0"/>   <!-- joint3 -->
<origin xyz="0.124 0.0 0.0" rpy="0 ${pi/4} 0"/>   <!-- joint4 -->
```

Files: `open_manipulator_description/urdf/open_manipulator_x/open_manipulator_x_arm.urdf.xacro`,
`open_manipulator_description/urdf/open_manipulator_x/open_manipulator_x.urdf`,
`open_manipulator_moveit_config/config/open_manipulator_x.urdf`.

On 2026-10-14, at the default home pose, the user confirmed that RViz visually matches the physical arm with this workaround in place.

### Why the URDF workaround is a problem
A rotation in a joint origin changes the robot *model*, but the encoder reading has not changed. This has two effects:
* MoveIt's FK, IK and collision checks run against the rotated model.
* URDF joint limits (`joint3 [-1.5, 1.4]`, `joint4 [-1.7, 1.97]`) are expressed relative to a zero that now sits 45° away
  from the ROBOTIS zero, so they no longer describe the same physical range.

The workaround also has to be kept in sync across three files, and any ROBOTIS example or pose that assumes the stock
zero (e.g. Option 3 "Zero Pose", §6) behaves differently than its author expected.

---

## 2. How a joint angle is produced (the measurement chain)

```
 magnetic encoder ──► raw position (0‥4095)
                         │  + Homing Offset   (EEPROM addr 20, stored inside the servo)
                         ▼
                    Present Position (addr 132)  ── read by dynamixel_hardware_interface
                         │  × 0.0015339808 rad/tick  − π      (xm430_w350.model, [unit info])
                         ▼
                    /joint_states  q_reported
                         │  + URDF joint-origin rotation δ   (robot_state_publisher / MoveIt)
                         ▼
                    pose drawn in RViz
```

* Tick 2048 maps to 0 rad. A Dynamixel horn sitting on its alignment mark corresponds to **raw** 2048.
* `Homing Offset` is applied **inside the servo** before the driver sees anything. Neither the ROS driver nor the
  URDF can see or log that it exists.
* The OMX ros2_control xacro (`open_manipulator_x_position.ros2_control.xacro`) writes Drive Mode, gains and profiles
  to every servo at startup, but it **never writes `Homing Offset`**. Whatever value is stored in EEPROM therefore persists.
  By contrast, the OMY xacros pin it explicitly (`<param name="Homing Offset">0</param>` in
  `omy_f3m_position.ros2_control.xacro`).

So the angle RViz draws is:

$$q_{RViz} = \underbrace{(raw-2048)\,k}_{\text{true angle if horn is on mark}} + h + \delta$$

where *h* = Homing Offset converted to an angle, and δ = the URDF origin rotation.

---

## 3. What was measured

### 3.1 Servo EEPROM registers (read-only dump, 2026-10-14)

Read directly over `/dev/ttyUSB1` with the launch stopped and torque off. The dump used Protocol 2.0 READ packets only;
nothing was written (method in Appendix A).

| ID | Joint | Operating / Drive Mode | **Homing Offset** | as angle | Min–Max Position Limit | Present Position (rest pose) |
|---|---|---|---|---|---|---|
| 11 | joint1 | 3 / 4 | **−17** | −1.49° | 0 – 4095 | 1938 → −0.1687 rad |
| 12 | joint2 | 3 / 4 | **+154** | +13.54° | 850 – 3494 | 837 → −1.8577 rad |
| 13 | joint3 | 3 / 4 | **−545** | −47.90° | 129 – 2511 | 2487 → +0.6734 rad |
| 14 | joint4 | 3 / 4 | **−664** | −58.36° | 175 – 2800 | 1860 → −0.2884 rad |
| 15 | gripper | 5 / 5 | **+1112** | — | 454 – 2208 | 1487 |

**Cross-check:** for ID13, Present Position 2487 gives (2487 − 2048) × 0.0015339808 = +0.6734 rad. This matches what
`/joint_states` reported for joint3 at the same moment. The registers are therefore what the driver is using.

Drive Mode 4 on IDs 11–14 means the direction bit is normal and the profile is time-based, so a reversed motor is ruled out.

### 3.2 Visual check with the workaround in place
After relaunching (interactive launch, Option 2 → default home `[0.6090, -1.3806, 0.4955, -0.8437]`, MoveIt + RViz), the
user reported that **all joints match**.

---

## 4. Analysis

**Finding: the servos carry non-zero Homing Offsets that the software stack does not know about.** joint3 (−47.9°) and
joint4 (−58.4°) are the two big ones, and they are exactly the two joints the URDF workaround patches by about 45°. This is the
mechanism behind the misalignment.

The open question is whether these offsets are **wrong** or **deliberate calibration**. Under the user's premise that the
horns sit exactly on their marks (raw 2048 = true zero), the workaround should leave these residual RViz errors:

| Joint | h | δ (workaround) | h + δ = predicted RViz error |
|---|---|---|---|
| joint1 | −1.5° | 0 | −1.5° (invisible) |
| joint2 | +13.5° | 0 | **+13.5°** |
| joint3 | −47.9° | +45° | −2.9° (barely visible) |
| joint4 | −58.4° | +45° | **−13.4°** |

The visual check found no mismatch on joint2 or joint4. That leaves two explanations:

* **Hypothesis A: the offsets are partly deliberate.** The horns are not exactly on the 2048 mark (about 13° on joint2 and joint4),
  and the stored offsets on those joints compensate for that. joint3's offset (about 48°) is the only real error, and the +45° URDF
  rotation cancels it.
* **Hypothesis B: the offsets are all wrong, and the ~13° residuals went unnoticed.** A 13.5° shoulder error moves the
  elbow by about 30 mm, and joint2 and joint4 residuals of opposite sign largely cancel in gripper orientation. That makes them easy to
  miss by eye.

Supporting evidence for A: under B, the default home pose would really be j2 = −1.617 rad, which is outside the URDF limit of
±1.5 rad that ROBOTIS designed for this arm. Under A it is −1.381 rad, inside the limit.

---

## 5. Deciding measurement

At the default home pose, place a phone inclinometer along the **upper arm**, on the line from the shoulder axis (joint2) to the
elbow axis (joint3). Read its elevation above the horizontal, measured toward the back of the robot.

| Reading | Conclusion | Apply |
|---|---|---|
| **≈ 21–22°** | Hypothesis A: current RViz model is right | Fix A (§7.1) |
| **≈ 8°** | Hypothesis B: all offsets are wrong | Fix B (§7.2) |

How these values are derived: at j2 = 0 the upper-arm vector (0.024, 0.128) m points 79.4° above the forward horizontal.
Positive j2 pitches it forward. The true j2 under A is −79.1°, giving 158.5° from forward = 21.5° above the backward horizontal.
Under B it is −92.6°, giving 172° = 8° above the backward horizontal.

---

## 6. Related hazard found during testing

**Do not use Option 3 (Zero Pose) while the URDF workaround is active.** At `[0, 0, 0, 0]` the workaround model puts the
forearm 45° down and the gripper pointing straight down. The gripper tip ends up about 1 cm *below* the base plane, so it would
strike the mounting surface. Option 3 was skipped during testing for this reason.

---

## 7. Fix

Both fixes follow the same principle: **keep the URDF stock and put the joint-zero calibration where it belongs, in the
servo's `Homing Offset`, pinned from source control** so it can't drift again. In both cases:

* **Update the Min/Max Position Limits.** They are compared against Present Position, which already includes the offset.
  Changing the offset by Δ means the limits must change by Δ as well, or the servo will clip the wrong physical range.
* **Re-express every stored pose.** Changing the offset by Δ ticks changes that joint's reading by Δ·k rad for the same
  physical posture. The poses live in `interactive_homing_node.py`, both bringup YAMLs, `custom_package/config/configured_home_position.yaml`,
  both SRDFs, the ros2_control `initial_value`s, and the docs.
* **Leave the gripper (ID15) alone.** Its `+1112` offset works together with the gripper's `[unit info]` conversion
  (`-0.036099321` m) and is out of scope.
* **Write EEPROM only with torque off.** The driver already starts with `disable_torque_at_init=true` and writes gpio params
  during init, which satisfies this. The arm must rest in a supported pose when torque is off.

### 7.1 Fix A: the current RViz model is right (inclinometer ≈ 21°)
Move the +45° from the URDF into the servos. joint3 and joint4 then read in the ROBOTIS convention, and the stock limits apply again.

| ID | Homing Offset | Min Position Limit | Max Position Limit |
|---|---|---|---|
| 13 | −545 → **−33** | 129 → **641** | 2511 → **3023** |
| 14 | −664 → **−152** | 175 → **687** | 2800 → **3312** |

(IDs 11 and 12 unchanged.)

1. In `open_manipulator_x_position.ros2_control.xacro`, add `<param name="Homing Offset">…</param>`,
   `<param name="Min Position Limit">…</param>` and `<param name="Max Position Limit">…</param>` to `dxl3` and `dxl4`.
   Also add the current values (unchanged) to `dxl1` and `dxl2` so every arm servo is pinned.
2. Revert joint3 and joint4 to `rpy="0 0 0"` in the three URDF files listed in §1.
3. Shift the stored poses by +π/4 on j3 and j4. For example, home `[0.6090, -1.3806, 0.4955, -0.8437]` becomes
   `[0.6090, -1.3806, 1.2809, -0.0583]`.
   ⚠ The minimum-stress exit pose's j3 becomes 1.4803 rad, above the joint3 limit of 1.4. Re-teach that pose (see known issue #2).

### 7.2 Fix B: all offsets are wrong (inclinometer ≈ 8°)
Zero the arm offsets so that raw encoder = joint angle, as the horn marks imply.

| ID | Homing Offset | Min Position Limit | Max Position Limit |
|---|---|---|---|
| 11 | −17 → **0** | 0 → **17** | 4095 (keep) |
| 12 | +154 → **0** | 850 → **696** | 3494 → **3340** |
| 13 | −545 → **0** | 129 → **674** | 2511 → **3056** |
| 14 | −664 → **0** | 175 → **839** | 2800 → **3464** |

1. Pin these values in the xacro gpio blocks as in Fix A.
2. Revert the three URDF files to `rpy="0 0 0"`.
3. Shift the stored poses by −h per joint (j1 +0.0261, j2 −0.2362, j3 +0.8360, j4 +1.0186 rad).
   Home becomes `[0.6351, -1.6168, 1.3315, 0.1749]`.
   ⚠ This puts home j2 outside the URDF limit of ±1.5. Either re-teach a home inside the limits or, after checking that
   the arm clears its base, widen the joint2 limit.

---

## 8. Verification (after either fix)

1. Re-run the register dump (Appendix A) and confirm the new Homing Offset and limit values on IDs 11–14.
2. Launch the interactive bringup, choose Option 2, and start MoveIt. RViz must overlay the arm using the **stock URDF**.
3. Repeat the inclinometer check from §5. The upper arm must agree with RViz within about 2°.
4. In MoveIt, Plan & Execute to the `home` named state and to two arbitrary goals. The start state must not be reported as out of bounds.
5. Only after steps 1–4 pass, test Option 3 (Zero Pose): with a stock URDF it is the ROBOTIS L-shaped pose and
   clears the base.

---

## Appendix A: reading servo registers

* The driver's `get_dxl_data` service (`/dynamixel_hardware_interface/get_dxl_data`) timed out on every request. Its
  callback busy-waits for a read buffer that `read()` never completes, and each attempt stalled the 100 Hz control loop for
  about 1 s (controller_manager overrun warnings). **Do not use it on a moving arm.**
* Working method: with the launch stopped (port free) and torque off, send Protocol 2.0 READ packets (instruction `0x02`)
  directly at 1 Mbps. The container has no `pyserial`, so the script uses only `termios`/`os`. Packet:
  `FF FF FD 00 | ID | 07 00 | 02 | addr_L addr_H | len_L len_H | CRC16` (CRC-16, poly 0x8005, init 0).
  Registers read: Model Number (0), Drive Mode (10), Operating Mode (11), Max/Min Position Limit (48/52), Homing Offset (20,
  signed 32-bit), Torque Enable (64), Present Position (132). The script used for §3.1 is `dxl_read.py`. It is not yet part of
  `custom_package`.

---

## 9. Calibration applied (2026-10-14)

§5's single-reading test was inconclusive: the forearm read about 1.5° *below* horizontal, matching neither 6° nor 16° up.
That showed neither hypothesis A nor hypothesis B was exactly right. The joints were therefore calibrated directly from link angles, with the arm holding the
default home pose under torque. Reported angles at that moment: j2 −79.189°, j3 +28.389°, j4 −48.428°.

| Measurement (centre-to-centre of motor axes) | Value | Model relation | True angle |
|---|---|---|---|
| Elbow fold, upper arm vs forearm | 21° | 100.62° − j3 (100.62° = 180° − atan2(0.128, 0.024)) | j3 = **+79.62°** |
| Forearm vs horizontal | 1.5° wrist-end down | j2 + j3 | j2 = **−78.12°** |
| Gripper vs forearm line | ≈ −1° | j4 | j4 = **−1.0°** |

Correction per joint: Δ = true − reported. New Homing Offset = old + Δ / 0.087890625° (ticks). The limits shift by the same tick count.

| ID | Joint | Δ | Homing Offset | Min–Max Position Limit |
|---|---|---|---|---|
| 11 | joint1 | — (not measurable by gravity, left as is) | −17 (pinned) | 0–4095 (pinned) |
| 12 | joint2 | +1.07° / +12 | 154 → **166** | 850–3494 → **862–3506** |
| 13 | joint3 | +51.23° / +583 | −545 → **38** | 129–2511 → **712–3094** |
| 14 | joint4 | +47.43° / +540 | −664 → **−124** | 175–2800 → **715–3340** |

**Conclusion:** servo 13's −545 offset was the dominant error, and its corrected value is about 3°. Servos 12 and 14 keep sizeable offsets,
so those were at least partly deliberate horn calibration. The +45°/+45° URDF workaround happened to approximate this to within about 6°, which is why
it looked right by eye.

**Changes made:**
* `open_manipulator_x_position.ros2_control.xacro`: `Homing Offset`, `Min Position Limit` and `Max Position Limit` were added to
  `dxl1`–`dxl4`. The driver writes them at init with torque off. The mock `initial_value`s were converted.
* The URDF joint3/joint4 origins were reverted to stock `rpy="0 0 0"` (all three copies).
* The stored poses were converted by Δ, so they describe the same physical posture:
  * home `[0.6090, -1.3806, 0.4955, -0.8437]` → `[0.6090, -1.3622, 1.3898, -0.0154]`
  * exit `[0.6090, -1.8484, 0.6949, -0.3436]` → `[0.6090, -1.8300, 1.5892, 0.4847]`
  
  Files changed: `interactive_homing_node.py`, `custom_package/config/configured_home_position.yaml`, the bringup
  `initial_positions.yaml` and `configured_home_position.yaml`, and both SRDFs.
* The exit pose is still outside the URDF limits (j2 −1.830 < −1.5, j3 1.589 > 1.4). This is known issue #2. The servo's Min Position Limit
  clamps j2 at 862 ticks (−1.819 rad), the same physical clamp as before.

**Assumptions:** all three readings were taken centre-to-centre. Measurement accuracy is about ±1–2° per joint. Verify with §8, and use
Option 3, which should now give the ROBOTIS stock L-pose.

**Hardware verification (2026-10-14):** after relaunch with the §9 values, RViz (stock URDF) overlays the physical arm, and
`[0, 0, 0, 0]` (Option 3) produces the ROBOTIS stock L-pose.
