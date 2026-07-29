"""Unit tests for examples/common/geo.py — pure math, no ROS needed."""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import pytest

from examples.common.geo import EARTH_R, haversine_m, bearing_deg, yaw_from_quat


def test_haversine_zero_distance():
    assert haversine_m((50.1, -97.3), (50.1, -97.3)) == 0.0


def test_haversine_one_degree_latitude():
    # One degree of latitude is R * (pi/180) metres along a meridian.
    expected = EARTH_R * math.radians(1.0)
    assert haversine_m((0.0, 0.0), (1.0, 0.0)) == pytest.approx(expected, rel=1e-9)


def test_haversine_symmetric():
    a, b = (49.0, -97.0), (50.0, -96.0)
    assert haversine_m(a, b) == pytest.approx(haversine_m(b, a))


def test_bearing_cardinals():
    origin = (0.0, 0.0)
    assert bearing_deg(origin, (1.0, 0.0)) == pytest.approx(0.0, abs=1e-6)    # north
    assert bearing_deg(origin, (0.0, 1.0)) == pytest.approx(90.0, abs=1e-6)   # east
    assert bearing_deg(origin, (-1.0, 0.0)) == pytest.approx(180.0, abs=1e-6)  # south
    assert bearing_deg(origin, (0.0, -1.0)) == pytest.approx(270.0, abs=1e-6)  # west


def test_bearing_in_range():
    assert 0.0 <= bearing_deg((10.0, 20.0), (11.0, 19.0)) < 360.0


def test_yaw_identity_quaternion():
    assert yaw_from_quat(0.0, 0.0, 0.0, 1.0) == pytest.approx(0.0)


def test_yaw_ninety_about_z():
    s = math.sqrt(0.5)  # 90 deg about Z: (0, 0, sin(45), cos(45))
    assert yaw_from_quat(0.0, 0.0, s, s) == pytest.approx(math.pi / 2)
