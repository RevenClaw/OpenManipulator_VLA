# Commanding End-Effector Coordinates from the VLA via `ee_commander`

Yes, you can. `ee_commander` already subscribes to `/ee_goal`, so a VLA node can send it end-effector coordinates right now
with no code changes.

## What the VLA publishes

- Topic: `/ee_goal`
- Type: `std_msgs/msg/Float64MultiArray`
- `data = [x, y, z, pitch, gripper]`
  - x, y, z in metres, in the robot base frame (`world` = `link1`): x forward, y left, z up. This is the point between the fingertips.
  - **pitch in radians** (not degrees as in the terminal): 0 = gripper horizontal, π/2 = pointing straight down.
  - gripper: `1` open, `0` close, `-1` leave as is.

```python
from std_msgs.msg import Float64MultiArray
self.ee_pub = self.create_publisher(Float64MultiArray, '/ee_goal', 10)

msg = Float64MultiArray()
msg.data = [0.20, 0.0, 0.15, 0.0, 1.0]   # x, y, z, pitch_rad, gripper
self.ee_pub.publish(msg)
```

Test it from a terminal:

```bash
ros2 topic pub --once /ee_goal std_msgs/msg/Float64MultiArray "{data: [0.20, 0.0, 0.15, 0.0, -1]}"
```

## When this works well

This works well when the VLA produces **one target pose per decision**, for example "reach above the cup", then "go down", then
"close gripper". Each goal gets a collision-checked MoveIt move.

## Limits to design the VLA loop around

1. **It's slow per goal.** Each goal takes about 0.1–1 s of planning, then a move at 0.3× speed. Don't publish at 10 Hz. Send a goal,
   let the arm get there, then run inference again.
2. **The VLA gets no "done" signal.** `ee_commander` only prints results to its terminal. Until a status topic exists, the VLA can
   decide the move is done when the `/joint_states` velocities are about zero. Failures (no solution, target out of reach, `z` below
   `min_z`, MoveIt failure) are silent to the VLA; the arm just doesn't move.
3. **Only the latest waiting goal is kept.** If you publish several goals during a move, only the last one runs afterwards. The move
   in progress is never interrupted.
4. **Yaw is fixed and roll is always 0.** The gripper always faces along `atan2(y, x)`. Only x, y, z and pitch are controllable.
5. Both the hardware launch and the MoveIt launch must be running.

## Suggested additions (not yet implemented)

Both are small changes to `ee_commander`:

- A `/ee_status` topic so the VLA knows when each goal succeeded or failed and can trigger its next inference immediately.
- Optionally, a `velocity_scaling` above 0.3, or a fast mode that skips MoveIt and sends the solved joint angles straight to the
  controller.

If the VLA outputs continuous action chunks at a high rate rather than one pose per decision, use the streaming bridge design
instead (VLA → `vla_bridge` → `/arm_controller/joint_trajectory`; see the "VLA command interface" section of `CLAUDE.md`).
