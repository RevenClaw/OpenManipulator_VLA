"""Unit tests for guided_demo's pure helpers (needs the ROS env for the imports): python3 -m pytest test/test_guided_demo.py"""

import json

import pytest

pytest.importorskip('rclpy')
from custom_package import guided_demo as G  # noqa: E402


def cube(label, x, y, conf=0.9):
    return {'label': label, 'xyz': [x, y, -0.04], 'confidence': conf}


def test_parse_bins_keeps_order_and_skips_bad_entries():
    assert G.parse_bins(['25:red bin', '20: green bin ', 'x:bad', '18:', '18:blue bin', '']) == \
        [(25, 'red bin'), (20, 'green bin'), (18, 'blue bin')]


def test_cube_in_bin_closest_within_radius():
    bins = {'red bin': (0.20, 0.10), 'green bin': (0.20, -0.10)}
    assert G.cube_in_bin((0.21, 0.09), bins, 0.06) == 'red bin'
    assert G.cube_in_bin((0.20, 0.0), bins, 0.06) is None


def test_match_cube_same_colour_nearby_only():
    old = cube('red', 0.20, 0.00)
    assert G.match_cube(old, [cube('red', 0.21, 0.0), cube('red', 0.30, 0.0)])['xyz'][0] == 0.21
    assert G.match_cube(old, [cube('blue', 0.20, 0.0)]) is None
    assert G.match_cube(old, [cube('red', 0.26, 0.0)]) is None


def test_parse_choice():
    assert G.parse_choice('2', 3) == 1
    assert G.parse_choice(' 3 ', 3) == 2
    assert G.parse_choice('0', 3) is None and G.parse_choice('4', 3) is None and G.parse_choice('r', 3) is None


def test_parse_arrangement_only_permutations():
    assert G.parse_arrangement('231') == (1, 2, 0)
    assert G.parse_arrangement(' 123 ') == (0, 1, 2)
    assert G.parse_arrangement('221') is None and G.parse_arrangement('12') is None and G.parse_arrangement('abc') is None
    assert G.parse_arrangement('124') is None


def test_arrangement_folders_six_distinct():
    perms = G.all_arrangements(3)
    assert len(perms) == 6
    assert G.arrangement_folder((1, 2, 0)) == 'arr_1to2_2to3_3to1'
    assert len({G.arrangement_folder(p) for p in perms}) == 6
    assert all(G.parse_arrangement(''.join(str(j + 1) for j in p)) == p for p in perms)


def test_count_episodes(tmp_path):
    for i, ok in enumerate([True, False, True]):
        (tmp_path / f'episode_{i:04d}').mkdir()
        (tmp_path / f'episode_{i:04d}' / 'metadata.json').write_text(json.dumps({'success': ok}))
    (tmp_path / 'episode_0003').mkdir()  # no metadata yet: not counted
    assert G.count_episodes(tmp_path) == (3, 2)
    assert G.count_episodes(tmp_path / 'missing') == (0, 0)
