"""OutdoorNav-specific fetch and interactive-selection helpers.

Shared across examples that need to pick a map, mission, or POI from the robot's
database. All fetch_* functions take a node + pre-created service client and call
the service synchronously. All select_* functions wrap fetch_* with an interactive
numbered-list menu, skipping the menu if an explicit UUID is supplied.
"""

from __future__ import annotations
from clearpath_mission_manager_msgs.srv import (
    GetAllMaps, GetAllNetworkMissions, GetAllPointsOfInterest,
)
from clearpath_dock_msgs.srv import GetDockDatabase
from examples.common.ros_helpers import call_service


# ---------------------------------------------------------------------------
# Interactive picker
# ---------------------------------------------------------------------------

def pick_from_list(items: list, label_fn, noun: str):
    """Print a numbered list and return the item the user picks."""
    for i, item in enumerate(items, 1):
        print(f"  {i}. {label_fn(item)}")
    while True:
        raw = input(f"Select {noun} [1-{len(items)}]: ").strip()
        if raw.isdigit() and 1 <= int(raw) <= len(items):
            return items[int(raw) - 1]
        print(f"  enter a number between 1 and {len(items)}")


# ---------------------------------------------------------------------------
# Fetch helpers
# ---------------------------------------------------------------------------

def fetch_maps(node, client) -> list:
    resp = call_service(node, client, GetAllMaps.Request())
    return sorted(getattr(resp, "maps", None) or [], key=lambda m: m.name)


def fetch_pois(node, client) -> list:
    resp = call_service(node, client, GetAllPointsOfInterest.Request())
    return sorted(getattr(resp, "points_of_interest", None) or [], key=lambda p: p.name)


def fetch_missions(node, client) -> list:
    resp = call_service(node, client, GetAllNetworkMissions.Request())
    return sorted(getattr(resp, "missions", None) or [], key=lambda m: m.name)


def fetch_docks(node, client) -> list:
    resp = call_service(node, client, GetDockDatabase.Request())
    db = getattr(resp, "database", None)
    docks = getattr(db, "docks", None) or [] if db else []
    return sorted(docks, key=lambda d: d.name)


# ---------------------------------------------------------------------------
# Interactive select helpers  (skip menu when explicit_uuid is supplied)
# ---------------------------------------------------------------------------

def select_map(node, client, explicit_uuid: str = "") -> tuple[str, str]:
    """Return (uuid, name). Shows a menu unless explicit_uuid is given."""
    if explicit_uuid:
        return explicit_uuid, explicit_uuid
    maps = fetch_maps(node, client)
    if not maps:
        raise RuntimeError("no maps found — pass --map-uuid <uuid> to skip this menu")
    print("\nMaps:")
    m = pick_from_list(maps, lambda m: m.name, "map")
    return m.uuid, m.name


def select_mission(node, client, explicit_uuid: str = "") -> tuple[str, str]:
    """Return (uuid, name). Shows a menu unless explicit_uuid is given."""
    if explicit_uuid:
        return explicit_uuid, explicit_uuid
    missions = fetch_missions(node, client)
    if not missions:
        raise RuntimeError("no missions found — pass --mission-uuid <uuid> to skip this menu")
    print("\nMissions:")
    m = pick_from_list(missions, lambda m: m.name, "mission")
    return m.uuid, m.name


def select_dock(node, client, explicit_name: str = "") -> str:
    """Return dock name. Shows a menu unless explicit_name is given."""
    if explicit_name:
        return explicit_name
    docks = fetch_docks(node, client)
    if not docks:
        raise RuntimeError("no docks found — pass --dock-name <name> to skip this menu")
    print("\nDocks:")
    d = pick_from_list(docks, lambda d: d.name, "dock")
    return d.name


def select_poi(node, client, explicit_uuid: str = "") -> tuple[str, str]:
    """Return (uuid, name). Shows a menu unless explicit_uuid is given."""
    if explicit_uuid:
        return explicit_uuid, explicit_uuid
    pois = fetch_pois(node, client)
    if not pois:
        raise RuntimeError("no POIs found — pass --poi-uuid <uuid> to skip this menu")
    print("\nPOIs:")
    p = pick_from_list(
        pois,
        lambda p: f"{p.name}  ({p.latitude:.6f}, {p.longitude:.6f})",
        "POI",
    )
    return p.uuid, p.name
