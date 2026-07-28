#!/usr/bin/env python3
"""Troubleshooting snapshot: env, autonomy state, battery, GPS, topic liveness, Nav2 lifecycle.

Run this when something feels off — not as a pre-mission check (use preflight.py for that).
One command replaces the where_am_i → service_inventory → topic echo → lifecycle list
scavenger hunt.

  ./doctor.py                                  # 3s snapshot, default namespace
  ./doctor.py --collect 5                      # subscribe longer before reporting
  ./doctor.py --include-lifecycle              # also query every Nav2 get_state service
  ONAV_NAMESPACE=/a300_00003 ./doctor.py

What you get:
  - Environment   : ROS_DOMAIN_ID, RMW, ROS distro
  - Autonomy      : state, paused flag, current goal id (from /autonomy/status)
  - Last mission  : status + age (from /autonomy/mission/_action/status)
  - Localization  : latest GPS fix + age
  - Power         : battery percent (from /platform/bms/state)
  - Topic liveness: publisher count for each key OutdoorNav topic
  - Nav2 lifecycle: every lifecycle node's current state (active/inactive/etc.)

Touches:
  topic   <ns>/autonomy/status                          (AutonomyStatus)
  topic   <ns>/localization/fix                         (NavSatFix)
  topic   <ns>/platform/bms/state                       (BatteryState)
  topic   <ns>/autonomy/mission/_action/status          (GoalStatusArray)
  service <lifecycle_node>/get_state                    (lifecycle_msgs/GetState, --include-lifecycle)
"""

from __future__ import annotations
import os
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy,
)
from action_msgs.msg import GoalStatusArray, GoalStatus
from sensor_msgs.msg import NavSatFix, BatteryState
from clearpath_navigation_msgs.msg import AutonomyStatus
from lifecycle_msgs.srv import GetState

from examples.common.argparse_base import make_parser


ACTIVITY_NAMES = {
    AutonomyStatus.IDLE: "IDLE",
    AutonomyStatus.MISSION: "MISSION",
    AutonomyStatus.MISSION_FROM_GOAL: "MISSION_FROM_GOAL",
    AutonomyStatus.GOTO: "GOTO",
    AutonomyStatus.GOTO_POI: "GOTO_POI",
    AutonomyStatus.DOCKING_LOCAL: "DOCKING_LOCAL",
    AutonomyStatus.UNDOCKING_LOCAL: "UNDOCKING_LOCAL",
    AutonomyStatus.DOCKING_MAP: "DOCKING_MAP",
}

GOAL_STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
    GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "EXECUTING",
    GoalStatus.STATUS_CANCELING: "CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
}

LIFECYCLE_STATE_NAMES = {
    0: "unknown", 1: "unconfigured", 2: "inactive", 3: "active", 4: "finalized",
    10: "configuring", 11: "cleaningup", 12: "shuttingdown", 13: "activating",
    14: "deactivating", 15: "errorprocessing",
}

KEY_TOPICS = [
    "autonomy/status",
    "localization/fix",
    "platform/bms/state",
    "navigation/current_goal_id",
    "mission_manager/state",
    "autonomy/mission/_action/status",
]


@dataclass
class Snapshot:
    autonomy_state: str = "(no message)"
    autonomy_paused: bool | None = None
    autonomy_current_goal: str = ""

    fix_lat: float | None = None
    fix_lon: float | None = None
    fix_alt: float | None = None
    fix_status: int | None = None
    fix_age_sec: float | None = None

    battery_pct: float | None = None
    battery_charging: bool | None = None
    battery_age_sec: float | None = None

    last_mission_status: str = "(no goals seen)"
    last_mission_goal_uuid: str = ""
    last_mission_age_sec: float | None = None

    topic_pub_count: dict[str, int] = field(default_factory=dict)
    lifecycle_states: list[tuple[str, str]] = field(default_factory=list)


def action_status_qos() -> QoSProfile:
    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


class Doctor(Node):
    def __init__(self, namespace: str):
        super().__init__("onav_doctor")
        self.namespace = namespace
        self.snap = Snapshot()
        self._fix_stamp = None
        self._battery_stamp = None
        self._last_mission_stamp = None

        self.create_subscription(AutonomyStatus, f"{namespace}/autonomy/status",
                                 self._autonomy_cb, 10)
        self.create_subscription(NavSatFix, f"{namespace}/localization/fix",
                                 self._fix_cb, 10)
        self.create_subscription(BatteryState, f"{namespace}/platform/bms/state",
                                 self._battery_cb, 10)
        self.create_subscription(GoalStatusArray,
                                 f"{namespace}/autonomy/mission/_action/status",
                                 self._mission_status_cb, action_status_qos())

    def _autonomy_cb(self, msg: AutonomyStatus) -> None:
        self.snap.autonomy_state = ACTIVITY_NAMES.get(msg.state, f"raw={msg.state}")
        self.snap.autonomy_paused = bool(msg.paused)
        self.snap.autonomy_current_goal = msg.current_goal or ""

    def _fix_cb(self, msg: NavSatFix) -> None:
        self.snap.fix_lat = msg.latitude
        self.snap.fix_lon = msg.longitude
        self.snap.fix_alt = msg.altitude
        self.snap.fix_status = msg.status.status
        self._fix_stamp = time.time()

    def _battery_cb(self, msg: BatteryState) -> None:
        self.snap.battery_pct = float(msg.percentage) * 100.0
        self.snap.battery_charging = bool(msg.power_supply_status == 1)  # CHARGING
        self._battery_stamp = time.time()

    def _mission_status_cb(self, msg: GoalStatusArray) -> None:
        if not msg.status_list:
            return
        entry = msg.status_list[-1]
        self.snap.last_mission_status = GOAL_STATUS_NAMES.get(
            entry.status, f"raw={entry.status}")
        self.snap.last_mission_goal_uuid = bytes(entry.goal_info.goal_id.uuid).hex()
        self._last_mission_stamp = time.time()

    def collect(self, seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        now = time.time()
        self.snap.fix_age_sec = (now - self._fix_stamp) if self._fix_stamp else None
        self.snap.battery_age_sec = (now - self._battery_stamp) if self._battery_stamp else None
        self.snap.last_mission_age_sec = (
            (now - self._last_mission_stamp) if self._last_mission_stamp else None
        )

    def check_topics(self) -> None:
        for suffix in KEY_TOPICS:
            topic = f"{self.namespace}/{suffix}"
            self.snap.topic_pub_count[topic] = self.count_publishers(topic)

    def check_lifecycle(self, probe_timeout: float = 0.5) -> None:
        nodes = self.get_node_names_and_namespaces()
        for name, ns in nodes:
            full = ns.rstrip("/") + "/" + name if ns and ns != "/" else "/" + name
            if self.namespace and not full.startswith(self.namespace):
                continue
            try:
                services = self.get_service_names_and_types_by_node(name, ns)
            except Exception:
                continue
            for srv_name, srv_types in services:
                if srv_name.endswith("/get_state") and \
                   "lifecycle_msgs/srv/GetState" in srv_types:
                    state = self._query_state(srv_name, probe_timeout)
                    self.snap.lifecycle_states.append((full, state))
                    break

    def _query_state(self, srv_name: str, timeout_sec: float) -> str:
        client = self.create_client(GetState, srv_name)
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return "(service unavailable)"
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            return "(timeout)"
        resp = future.result()
        if resp is None:
            return "(no response)"
        return LIFECYCLE_STATE_NAMES.get(resp.current_state.id, f"raw={resp.current_state.id}")


def render(snap: Snapshot, namespace: str, lifecycle_checked: bool) -> str:
    lines = []
    lines.append(f"=== OutdoorNav doctor: ns={namespace} ===\n")

    lines.append("Environment")
    lines.append(f"  ROS_DOMAIN_ID    : {os.environ.get('ROS_DOMAIN_ID', '(unset)')}")
    lines.append(f"  RMW              : {os.environ.get('RMW_IMPLEMENTATION', '(default)')}")
    lines.append(f"  ROS distro       : {os.environ.get('ROS_DISTRO', '(unset)')}")
    lines.append("")

    lines.append("Autonomy")
    lines.append(f"  state            : {snap.autonomy_state}")
    if snap.autonomy_paused is not None:
        lines.append(f"  paused           : {snap.autonomy_paused}")
    lines.append(f"  current goal     : {snap.autonomy_current_goal or '(none)'}")
    if snap.last_mission_age_sec is not None:
        lines.append(f"  last mission     : {snap.last_mission_status} "
                     f"{snap.last_mission_age_sec:.1f}s ago "
                     f"(goal {snap.last_mission_goal_uuid[:8]})")
    else:
        lines.append("  last mission     : (no goals seen)")
    lines.append("")

    lines.append("Localization")
    if snap.fix_lat is not None:
        lines.append(f"  GPS fix          : {snap.fix_lat:.6f}, {snap.fix_lon:.6f} "
                     f"(alt {snap.fix_alt:.1f} m, status={snap.fix_status})")
        lines.append(f"  fix age          : {snap.fix_age_sec:.1f}s")
    else:
        lines.append("  GPS fix          : (no message on localization/fix)")
    lines.append("")

    lines.append("Power")
    if snap.battery_pct is not None:
        charging = "yes" if snap.battery_charging else "no"
        lines.append(f"  battery          : {snap.battery_pct:.1f}%  charging: {charging}")
        lines.append(f"  reading age      : {snap.battery_age_sec:.1f}s")
    else:
        lines.append("  battery          : (no message on platform/bms/state)")
    lines.append("")

    lines.append("Topic liveness (key topics)")
    width = max((len(t) for t in snap.topic_pub_count), default=20)
    for topic, n in snap.topic_pub_count.items():
        mark = "ok" if n > 0 else "NONE"
        lines.append(f"  {topic.ljust(width)}  publishers={n}  {mark}")
    lines.append("")

    if lifecycle_checked:
        lines.append("Nav2 / managed nodes (lifecycle)")
        if not snap.lifecycle_states:
            lines.append("  (no lifecycle services found in this namespace)")
        else:
            width = max(len(n) for n, _ in snap.lifecycle_states)
            for node_name, state in snap.lifecycle_states:
                lines.append(f"  {node_name.ljust(width)}  {state}")
        lines.append("")

    return "\n".join(lines)


def main(argv=None):
    parser = make_parser(doc=__doc__, add_dry_run=False)
    parser.add_argument("--collect", type=float, default=3.0,
                        help="Seconds to subscribe before reporting (default 3).")
    parser.add_argument("--include-lifecycle", action="store_true",
                        help="Also discover and query Nav2 lifecycle node states.")
    args = parser.parse_args(argv)

    rclpy.init()
    node = Doctor(args.namespace)
    try:
        node.collect(args.collect)
        node.check_topics()
        if args.include_lifecycle:
            node.check_lifecycle()
        print(render(node.snap, args.namespace, args.include_lifecycle))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
