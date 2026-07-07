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
    """Robot namespace from $ONAV_NAMESPACE (leading slash, no trailing), or ''.

    No hardcoded serial — scripts are namespace-agnostic. When unset, callers either
    pass --namespace, load a transport profile that sets ONAV_NAMESPACE (docker/*.env),
    or use resolve_namespace() to auto-detect from the live graph.
    """
    return normalize_namespace(os.environ.get("ONAV_NAMESPACE", ""))



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


def _detect_robot_namespaces(node) -> list[str]:
    """Top-level namespaces on the graph that look like a Clearpath robot.

    Prefers namespaces exposing a robot anchor (`localization/fix` or
    `platform/bms/state`); falls back to every non-root namespace. The node must
    have spun long enough to have discovered the graph.
    """
    topics = {t for t, _ in node.get_topic_names_and_types()}
    roots: set[str] = set()
    for _name, ns in node.get_node_names_and_namespaces():
        ns = normalize_namespace(ns)
        if ns:
            roots.add("/" + ns.lstrip("/").split("/")[0])
    anchored = sorted(
        r for r in roots
        if f"{r}/localization/fix" in topics or f"{r}/platform/bms/state" in topics
    )
    return anchored or sorted(roots)


def resolve_namespace(node, explicit: str = "") -> str:
    """Namespace-agnostic resolution: explicit/env first, else auto-detect from the graph.

    Order: `explicit` (e.g. --namespace) → $ONAV_NAMESPACE → the single robot namespace
    on the live graph → raise. Lets one script target a300_00003, a300_00070, or any unit
    with no baked-in serial. Requires `node` to have spun enough to discover the graph.
    """
    ns = normalize_namespace(explicit) or namespace()
    if ns:
        return ns
    detected = _detect_robot_namespaces(node)
    if len(detected) == 1:
        return detected[0]
    if not detected:
        raise RuntimeError(
            "no robot namespace on the graph — pass --namespace, set $ONAV_NAMESPACE, "
            "or load a transport profile (docker/*.env)."
        )
    raise RuntimeError(f"multiple robot namespaces {detected} — pass --namespace to pick one.")
