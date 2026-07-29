"""Shared geospatial primitives for the map/mission examples.

Pure stdlib (no rclpy, no clearpath_* imports) so it stays trivially testable
and importable outside a ROS environment. This is the home for geometry/route
math that more than one example needs — put the next shared helper here instead
of copy-pasting a seventh `haversine_m`. Example-specific algorithms
(path simplification, route ordering) stay inline in their own scripts.

Coordinates are (latitude, longitude) tuples in degrees unless noted.
"""

from __future__ import annotations
import math

EARTH_R = 6_371_000.0  # mean Earth radius, metres


def haversine_m(a: tuple, b: tuple) -> float:
    """Great-circle distance between two (lat, lon) points, in metres."""
    lat1, lon1 = math.radians(a[0]), math.radians(a[1])
    lat2, lon2 = math.radians(b[0]), math.radians(b[1])
    dlat, dlon = lat2 - lat1, lon2 - lon1
    h = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * EARTH_R * math.asin(math.sqrt(h))


def bearing_deg(a: tuple, b: tuple) -> float:
    """Initial compass bearing from a to b. 0=N, 90=E."""
    lat1, lat2 = math.radians(a[0]), math.radians(b[0])
    dlon = math.radians(b[1] - a[1])
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    """Yaw (rotation about Z) in radians from a quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
