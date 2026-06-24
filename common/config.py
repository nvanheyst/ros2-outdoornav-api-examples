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


def normalize_namespace(ns: str) -> str:
    """Normalize a namespace: leading slash, no trailing slash.

    Accepts whatever a user passes via $ONAV_NAMESPACE or --namespace
    ('a300_00003', '/a300_00003/', ' a300_00003 ') and returns a
    canonical '/a300_00003'. Empty input stays empty.
    """
    ns = (ns or "").strip()
    if not ns:
        return ""
    if not ns.startswith("/"):
        ns = "/" + ns
    return ns.rstrip("/")


def namespace() -> str:
    """Return robot namespace with leading slash, no trailing slash."""
    return normalize_namespace(os.environ.get("ONAV_NAMESPACE", "/a300_00003"))



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
