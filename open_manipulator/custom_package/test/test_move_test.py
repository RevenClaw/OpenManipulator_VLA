"""Unit tests for move_test's parser (needs the ROS env for the imports): python3 -m pytest test/test_move_test.py"""

import pytest

pytest.importorskip('rclpy')
from custom_package import move_test as M  # noqa: E402


def test_parse_move():
    assert M.parse_move('0.2 0.05 -0.03') == (0.2, 0.05, -0.03, None, None)
    assert M.parse_move('0.2 0.05 -0.03 80') == (0.2, 0.05, -0.03, 80.0, None)
    assert M.parse_move('0.2 0.05 -0.03 c') == (0.2, 0.05, -0.03, None, 'c')
    assert M.parse_move('0.2 0.05 -0.03 90 o') == (0.2, 0.05, -0.03, 90.0, 'o')
    for bad in ('0.2 0.05', '0.2 0.05 x', '1 2 3 4 5', 'o', '0.2 0.05 nan', ''):
        assert M.parse_move(bad) is None
