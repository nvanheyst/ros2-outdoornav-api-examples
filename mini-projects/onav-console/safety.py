"""Operator confirmation gate for motion commands on a real robot."""
from __future__ import annotations

MOTION_TOOLS = {"go_to_poi", "run_mission", "dock", "dock_local", "undock"}


def needs_confirmation(name: str, target: str) -> bool:
    return target == "real" and name in MOTION_TOOLS
