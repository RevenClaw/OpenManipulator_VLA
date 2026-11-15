"""Unit tests for demo_planning (needs the ROS env for ee_commander's imports): python3 -m pytest test/test_demo_planning.py"""

import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
pytest.importorskip('rclpy')
from custom_package import demo_planning as D  # noqa: E402
from custom_package.ee_commander import forward_kinematics  # noqa: E402

CFG = D.PlanConfig()


def _all_joints(plan):
    return np.vstack([s.joints for s in plan.segments if s.kind == 'move'])


def test_plan_reaches_the_cube_and_the_target():
    plan = D.plan_pick_place((0.20, 0.05), (0.15, -0.15), CFG)
    names = [s.name for s in plan.segments]
    assert names == ['approach', 'descend', 'close', 'lift', 'transfer', 'lower', 'open', 'retreat', 'ready']
    by_name = {s.name: s for s in plan.segments}
    x, y, z, pitch = forward_kinematics(by_name['descend'].joints[-1])
    assert math.dist((x, y), (0.20, 0.05)) < 0.002 and abs(z - CFG.cube_size / 2) < 0.002
    x, y, z, _ = forward_kinematics(by_name['lower'].joints[-1])
    assert math.dist((x, y), (0.15, -0.15)) < 0.002 and abs(z - (CFG.bin_height + CFG.release_clearance)) < 0.002
    np.testing.assert_allclose(by_name['ready'].joints[-1], D.READY, atol=1e-9)
    assert plan.pick_pitch == pytest.approx(math.radians(90))  # close enough to point straight down


def test_far_cube_uses_a_tilted_gripper():
    plan = D.plan_pick_place((0.29, 0.0), (0.15, -0.15), CFG)
    assert math.radians(30) <= plan.pick_pitch < math.radians(90)


def test_unreachable_cube_is_refused():
    with pytest.raises(D.PlanningError, match='not reachable'):
        D.plan_pick_place((0.45, 0.0), (0.15, -0.15), CFG)


def test_speed_cap_and_continuity():
    rng = np.random.default_rng(3)
    for _ in range(10):
        var = D.sample_variation(rng)
        plan = D.plan_pick_place((rng.uniform(0.14, 0.24), rng.uniform(-0.08, 0.12)), (0.12, -0.17), CFG, var)
        prev = plan.start
        for seg in plan.segments:
            if seg.kind != 'move':
                continue
            assert np.max(np.abs(seg.joints[0] - prev)) < 1e-6, f'{seg.name} does not start where the last move ended'
            vel = np.abs(np.diff(seg.joints, axis=0)) / np.diff(seg.times)[:, None]
            assert np.all(vel.max(axis=0) <= np.asarray(CFG.max_joint_vel) * 1.01), f'{seg.name} too fast: {vel.max(axis=0)}'
            prev = seg.joints[-1]
        heights = D.fingertip_heights(plan)
        assert min(heights.values()) >= CFG.table_z + CFG.cube_size / 2 - 0.005


def test_cartesian_moves_are_straight():
    plan = D.plan_pick_place((0.20, 0.05), (0.15, -0.15), CFG)
    descend = next(s for s in plan.segments if s.name == 'descend')
    xy = np.array([forward_kinematics(q)[:2] for q in descend.joints])
    assert np.max(np.linalg.norm(xy - xy[0], axis=1)) < 0.001  # vertical line: x, y stay put


def test_variation_changes_the_plan_but_stays_valid():
    rng = np.random.default_rng(7)
    plans = [D.plan_pick_place((0.2, 0.0), (0.12, -0.17), CFG, D.sample_variation(rng)) for _ in range(5)]
    durations = {round(p.duration, 2) for p in plans}
    assert len(durations) > 1
    corrected = D.plan_pick_place((0.2, 0.0), (0.12, -0.17), CFG, D.Variation(correction=(0.02, 0.0)))
    assert [s.name for s in corrected.segments][:3] == ['approach_off', 'correct', 'descend']
    via = D.plan_pick_place((0.2, 0.0), (0.12, -0.17), CFG, D.Variation(via_offset=(0.0, 0.02, 0.02)))
    assert 'via' in [s.name for s in via.segments]


def test_sample_trajectory_follows_the_plan():
    plan = D.plan_pick_place((0.20, 0.05), (0.15, -0.15), CFG)
    q0, g0 = D.sample_trajectory(plan, 0.0)
    np.testing.assert_allclose(q0, plan.start)
    assert g0 == 0.0
    t_close = sum(s.length for s in plan.segments[:2]) + 0.1  # just after "close"
    assert D.sample_trajectory(plan, t_close)[1] == 1.0
    q_end, g_end = D.sample_trajectory(plan, plan.duration + 5.0)
    np.testing.assert_allclose(q_end, D.READY, atol=1e-9)
    assert g_end == 0.0
    samples = np.array([D.sample_trajectory(plan, t)[0] for t in np.arange(0, plan.duration, 1 / 30)])
    assert np.all(np.max(np.abs(np.diff(samples, axis=0)), axis=0) * 30 <= np.asarray(CFG.max_joint_vel) * 1.05)


def test_verify_place():
    target = (0.12, -0.17)
    assert D.verify_place('red', (0.2, 0.0), target, [{'label': 'red', 'xyz': (0.13, -0.16, 0.05)}])[0]
    assert D.verify_place('red', (0.2, 0.0), target, [{'label': 'blue', 'xyz': (0.2, 0.0, 0.01)}])[0]  # hidden in the bin
    ok, reason = D.verify_place('red', (0.2, 0.0), target, [{'label': 'red', 'xyz': (0.201, 0.001, 0.01)}])
    assert not ok and 'start position' in reason
    ok, reason = D.verify_place('red', (0.2, 0.0), target, [{'label': 'red', 'xyz': (0.25, 0.1, 0.01)}])
    assert not ok and 'from the target' in reason


def test_prompt():
    assert D.make_prompt('pick up the {color} cube and place it in the {target}', 'red', 'black bin') == \
        'pick up the red cube and place it in the black bin'


def test_ready_pose_is_above_the_workspace():
    x, y, z, pitch = forward_kinematics(D.READY)
    assert abs(x - 0.16) < 0.002 and abs(y) < 0.002 and abs(z - 0.12) < 0.002 and abs(math.degrees(pitch) - 70) < 0.5


def test_episode_length_is_reasonable():
    rng = np.random.default_rng(5)
    durations = [D.plan_pick_place((rng.uniform(0.12, 0.24), rng.uniform(-0.05, 0.15)), (0.12, -0.18), CFG,
                                   D.sample_variation(rng)).duration for _ in range(20)]
    assert np.median(durations) < 32.0  # ~27 s: per-joint caps + READY start (was ~65 s from HOME at 0.3 rad/s everywhere)
