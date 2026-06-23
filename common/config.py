"""Env-var defaults for OutdoorNav example scripts.

Override at runtime:
  ONAV_NAMESPACE=/a300_00003   # robot namespace (with leading slash)
  ONAV_MAP_ID=<uuid>           # default map UUID
  ONAV_MISSION_ID=<uuid>       # default mission UUID
  ONAV_POI_ID=<uuid>           # default POI UUID
  ONAV_WAYPOINT_ID=<uuid>      # default waypoint UUID
"""

from __future__ import annotations
import os


def namespace() -> str:
    """Return robot namespace with leading slash, no trailing slash."""
    ns = os.environ.get("ONAV_NAMESPACE", "/a300_00003").strip()
    if ns and not ns.startswith("/"):
        ns = "/" + ns
    return ns.rstrip("/")


def map_id() -> str:
    return os.environ.get("ONAV_MAP_ID", "")


def mission_id() -> str:
    return os.environ.get("ONAV_MISSION_ID", "")


def poi_id() -> str:
    return os.environ.get("ONAV_POI_ID", "")


def waypoint_id() -> str:
    return os.environ.get("ONAV_WAYPOINT_ID", "")


def topic(name: str) -> str:
    """Build a namespaced topic/service/action path. Pass without leading slash."""
    return f"{namespace()}/{name.lstrip('/')}"
