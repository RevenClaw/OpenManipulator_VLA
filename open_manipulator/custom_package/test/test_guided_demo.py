"""Unit tests for guided_demo's pure helpers (needs the ROS env for the imports): python3 -m pytest test/test_guided_demo.py"""

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
