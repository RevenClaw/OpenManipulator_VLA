"""Unit tests for episode_tools (no ROS needed): python3 -m pytest test/test_episode_tools.py"""

import importlib.util
import pathlib

import numpy as np

# Load the module by path so these tests run without building the ROS package.
_spec = importlib.util.spec_from_file_location(
    'episode_tools', pathlib.Path(__file__).resolve().parent.parent / 'custom_package' / 'episode_tools.py'
)
E = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(E)

HOME = [0.6090, -1.3622, 1.3898, -0.0154]


def test_home_matches_configured_home():
    config = pathlib.Path(__file__).resolve().parent.parent / 'config' / 'configured_home_position.yaml'
    if config.exists():  # known issue #5: HOME is duplicated, keep the copies equal
        values = [float(v) for v in __import__('re').findall(r'-?\d+\.\d+', config.read_text())[:4]]
        np.testing.assert_allclose(values, E.HOME, atol=1e-4)
    assert E.HOME == HOME


def test_slugify():
    assert E.slugify('Pick up the red cube and place it in the bowl!') == 'pick_up_the_red_cube_and_place_it_in_the_bowl'
    assert E.slugify('  ') == 'task'
    assert len(E.slugify('x' * 200)) == 60


def test_next_episode_index(tmp_path):
    assert E.next_episode_index(tmp_path / 'missing') == 0
    for name in ('episode_0000', 'episode_0003', 'episode_tmp', 'notes'):
        (tmp_path / name).mkdir()
    assert E.next_episode_index(tmp_path) == 4  # never reuses a number, even after deletions
    assert E.episode_name(4) == 'episode_0004'


def test_grid_position():
    assert [E.grid_position(i, 9) for i in (0, 8, 9, 10)] == [1, 9, 1, 2]
    assert E.grid_position(3, 0) is None


def test_two_stage_waypoints_fold_the_elbow():
    start = [0.0, 0.5, 0.2, 1.0]  # extended forward
    mid, target = E.two_stage_waypoints(start, HOME)
    assert target == HOME
    assert mid == [0.0, (0.5 + HOME[1]) / 2, max(0.2, HOME[2]), HOME[3]]  # interactive_homing_node's path


def test_home_duration():
    assert E.home_duration(HOME, HOME, 0.25) == 4.0           # minimum
    assert E.home_duration([0, 0, 0, 0], [1.5, 0, 0, 0], 0.25) == 6.0
    assert E.home_duration([0, 0, 0, 0], [3.0, 0, 0, 0], 0.25) == 10.0  # maximum
    assert E.at_pose([0.61, -1.36, 1.39, -0.01], HOME, 0.05) and not E.at_pose([0.7, -1.36, 1.39, -0.01], HOME, 0.05)


def _times(rate, duration, start=100.0, drop=None):
    t = start + np.arange(int(duration * rate)) / rate
    if drop:
        t = t[(t < start + drop[0]) | (t >= start + drop[1])]
    return t


RATES = {'/joint_states': 50.0, '/camera/front/color/image_raw': 12.0}


def test_analyze_good_episode():
    rows, problems = E.analyze_stamps({'/joint_states': _times(100, 10), '/camera/front/color/image_raw': _times(15, 10)}, RATES)
    assert problems == []
    by_topic = {r['topic']: r for r in rows}
    assert abs(by_topic['/joint_states']['rate_hz'] - 100) < 0.5 and by_topic['/camera/front/color/image_raw']['max_gap_s'] < 0.07


def test_analyze_finds_gap_slow_and_missing():
    stamps = {'/joint_states': _times(30, 10), '/camera/front/color/image_raw': _times(15, 10, drop=(4.0, 4.6))}
    _, problems = E.analyze_stamps(stamps, {**RATES, '/teleop/action': 20.0})
    text = '; '.join(problems)
    assert '/joint_states: 30 Hz (min 50)' in text
    assert '/camera/front/color/image_raw: 0.6' in text and 'gap' in text
    assert '/teleop/action: no messages' in text


def test_analyze_ignores_startup_gap_but_not_early_stop():
    cam = np.concatenate([[100.0], _times(15, 9.6, start=100.4)])  # 0.4 s hole right after the bag starts: ignored
    _, problems = E.analyze_stamps({'/joint_states': _times(100, 10), '/camera/front/color/image_raw': cam}, RATES)
    assert problems == []
    _, problems = E.analyze_stamps({'/joint_states': _times(100, 10), '/camera/front/color/image_raw': _times(15, 6)}, RATES)
    assert any('camera' in p and 'gap' in p for p in problems)  # camera died at 6 s of 10


def test_metadata_has_the_converter_fields():
    meta = E.build_metadata(task='pick', success=False, index=3, start_time=1_800_000_000.0, duration_s=12.345,
                            topics=['/joint_states'], grid_pos=4, storage='mcap/zstd_fast')
    assert meta['task'] == 'pick' and meta['success'] is False  # what vla_layer's converter reads
    assert meta['episode_id'] == 3 and meta['duration_s'] == 12.35 and meta['grid_position'] == 4
    assert 'start_time' in meta and meta['recorder_version'] == E.RECORDER_VERSION
