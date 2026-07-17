#!/usr/bin/env python3
"""Go / no-go readiness gate - will autonomy actually run right now?

Checks the things that STOP normal operation (missions, docking, collision
avoidance) and prints one verdict: READY or NOT READY + the blockers. Read-only:
subscribes, introspects, and probes action servers with wait_for_server - never
sends a goal or command, so it's safe to run anytime (typically stationary at the
dock before a mission).

Philosophy: fail only on functional blockers, not warning noise. The system's own
top-level diagnostic being ERROR is the same gate OutdoorNav uses to reject
missions - that's a BLOCK. Diagnostics at WARN (PTP jitter, memory usage, etc.)
and other non-fatal conditions are shown as notes, never a failure.

  ./preflight.py
  ./preflight.py --collect 4 --battery-warn 50
  ./preflight.py --battery-critical 15

Touches (all read-only):
  topics  diagnostics_toplevel_state, diagnostics_agg, localization/fix, tf,
          platform/emergency_stop, platform/bms/state,
          sensors/lidar3d_0/nonground_filtered, docking/dock_manager/database
  params  collision_monitor/get_parameters (PolygonStop.enabled)
  actions autonomy/mission, autonomy/dock_map (wait_for_server only)
"""

from __future__ import annotations
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.action import ActionClient
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy,
)
from std_msgs.msg import Bool
from sensor_msgs.msg import NavSatFix, BatteryState, PointCloud2
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer
from rcl_interfaces.srv import GetParameters
from clearpath_navigation_msgs.action import ExecuteMission
from clearpath_dock_msgs.action import MapDock
from clearpath_dock_msgs.msg import DockDatabase

from examples.common.argparse_base import make_parser

# Check severities.
OK, BLOCK, NOTE = "OK", "BLOCK", "NOTE"


def level_int(level) -> int:
    """DiagnosticStatus.level as a plain int (it's a ROS `byte`)."""
    if isinstance(level, (bytes, bytearray)):
        return level[0] if level else 0
    if isinstance(level, str):
        return ord(level) if level else 0
    return int(level)


def latched_qos() -> QoSProfile:
    return QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                      reliability=QoSReliabilityPolicy.RELIABLE,
                      durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def best_effort_qos() -> QoSProfile:
    return QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=5,
                      reliability=QoSReliabilityPolicy.BEST_EFFORT,
                      durability=QoSDurabilityPolicy.VOLATILE)


class Preflight(Node):
    def __init__(self, ns: str, map_frame: str, base_frame: str):
        super().__init__("preflight")
        self.ns = ns
        self.map_frame = map_frame
        self.base_frame = base_frame

        self.toplevel_level: int | None = None
        self.agg_error: list[str] = []
        self.agg_warn: list[str] = []
        self.fix: NavSatFix | None = None
        self.battery_pct: float | None = None
        self.estop: bool | None = None
        self.collision_source_count = 0
        self.dock_count: int | None = None

        self.tf_buffer = Buffer()

        self.create_subscription(DiagnosticStatus, f"{ns}/diagnostics_toplevel_state",
                                 self._toplevel_cb, 10)
        self.create_subscription(DiagnosticArray, f"{ns}/diagnostics_agg",
                                 self._agg_cb, 10)
        self.create_subscription(NavSatFix, f"{ns}/localization/fix", self._fix_cb, 10)
        self.create_subscription(BatteryState, f"{ns}/platform/bms/state",
                                 self._battery_cb, 10)
        self.create_subscription(Bool, f"{ns}/platform/emergency_stop",
                                 self._estop_cb, best_effort_qos())
        self.create_subscription(PointCloud2, f"{ns}/sensors/lidar3d_0/nonground_filtered",
                                 self._collision_src_cb, best_effort_qos())
        self.create_subscription(TFMessage, f"{ns}/tf", self._tf_cb, 10)
        self.create_subscription(TFMessage, f"{ns}/tf_static", self._tf_static_cb, latched_qos())
        self.create_subscription(DockDatabase, f"{ns}/docking/dock_manager/database",
                                 self._dock_cb, latched_qos())

    # --- subscriptions ---
    def _toplevel_cb(self, m: DiagnosticStatus) -> None:
        self.toplevel_level = level_int(m.level)

    def _agg_cb(self, m: DiagnosticArray) -> None:
        err, warn = [], []
        for s in m.status:
            lvl = level_int(s.level)
            # only leaf entries (skip the category roll-ups) keep the list readable
            if lvl == 2:
                err.append(s.name)
            elif lvl == 1:
                warn.append(s.name)
        self.agg_error, self.agg_warn = err, warn

    def _fix_cb(self, m: NavSatFix) -> None:
        if not math.isnan(m.latitude) and not math.isnan(m.longitude):
            self.fix = m

    def _battery_cb(self, m: BatteryState) -> None:
        self.battery_pct = float(m.percentage) * 100.0

    def _estop_cb(self, m: Bool) -> None:
        self.estop = bool(m.data)

    def _collision_src_cb(self, _m: PointCloud2) -> None:
        self.collision_source_count += 1

    def _tf_cb(self, m: TFMessage) -> None:
        for t in m.transforms:
            self.tf_buffer.set_transform(t, "preflight")

    def _tf_static_cb(self, m: TFMessage) -> None:
        for t in m.transforms:
            self.tf_buffer.set_transform_static(t, "preflight")

    def _dock_cb(self, m) -> None:
        # DockDatabase field name varies; count whatever list of docks it carries.
        for attr in ("docks", "database", "dock_list"):
            v = getattr(m, attr, None)
            if isinstance(v, (list, tuple)):
                self.dock_count = len(v)
                return
        self.dock_count = 0

    # --- active probes (spin) ---
    def collect(self, seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def have_tf(self) -> bool:
        return self.tf_buffer.can_transform(self.map_frame, self.base_frame, Time())

    def node_base_names(self) -> set[str]:
        return {n for n, _ in self.get_node_names_and_namespaces()}

    def polygon_stop_enabled(self):
        """True/False, or None if the param service couldn't be reached."""
        cli = self.create_client(GetParameters, f"{self.ns}/collision_monitor/get_parameters")
        if not cli.wait_for_service(timeout_sec=2.0):
            return None
        req = GetParameters.Request()
        req.names = ["PolygonStop.enabled"]
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        if not fut.done() or fut.result() is None or not fut.result().values:
            return None
        return bool(fut.result().values[0].bool_value)

    def action_server_up(self, action_type, name: str) -> bool:
        client = ActionClient(self, action_type, f"{self.ns}/{name}")
        ok = client.wait_for_server(timeout_sec=3.0)
        client.destroy()
        return ok


def run_checks(node: Preflight, battery_critical: float, battery_warn: float):
    """Return a list of (severity, label, detail)."""
    checks = []
    nodes = node.node_base_names()

    # 1. System gate - the same signal OutdoorNav uses to accept/reject missions.
    if node.toplevel_level is None:
        checks.append((BLOCK, "system health", "no diagnostics_toplevel_state - is the stack up?"))
    elif node.toplevel_level >= 2:
        who = ", ".join(node.agg_error[:6]) or "unknown component"
        checks.append((BLOCK, "system health", f"top-level ERROR - blockers: {who}"))
    else:
        note = " (WARN present, non-blocking)" if node.toplevel_level == 1 else ""
        checks.append((OK, "system health", f"top-level {'WARN' if node.toplevel_level==1 else 'OK'}{note}"))
    if node.agg_warn:
        checks.append((NOTE, "diagnostics warnings", f"{len(node.agg_warn)}: " + ", ".join(node.agg_warn[:5])))

    # 2. Localization - fix + map->base_link tf.
    if node.fix is None:
        checks.append((BLOCK, "localization", "no GPS fix on localization/fix"))
    elif not node.have_tf():
        checks.append((BLOCK, "localization", f"no {node.map_frame}->{node.base_frame} tf"))
    else:
        # NavSatFix.status is driver-specific here (fixposition uses non-standard
        # values), so don't judge fix quality from it - a valid lat/lon is the
        # functional signal for readiness.
        checks.append((OK, "localization",
                       f"fix {node.fix.latitude:.6f},{node.fix.longitude:.6f}, tf present"))

    # 3. Collision detection - enabled AND actually fed.
    if "collision_monitor" not in nodes:
        checks.append((BLOCK, "collision detection", "collision_monitor node not running"))
    else:
        enabled = node.polygon_stop_enabled()
        if enabled is False:
            checks.append((BLOCK, "collision detection", "PolygonStop disabled"))
        elif node.collision_source_count == 0:
            checks.append((BLOCK, "collision detection",
                           "enabled but source cloud (nonground_filtered) not publishing - monitor is blind"))
        else:
            en = "enabled" if enabled else "enabled(?)"
            checks.append((OK, "collision detection",
                           f"{en}, source live ({node.collision_source_count} clouds)"))

    # 4. Docking - the functional signal is the dock action server. The docking
    # nodes are composed under one 'docking' process, so don't rely on individual
    # node names.
    if not node.action_server_up(MapDock, "autonomy/dock_map"):
        checks.append((BLOCK, "docking", "autonomy/dock_map action server unavailable"))
    else:
        checks.append((OK, "docking", "dock_map action server available"))
        if not node.dock_count:
            checks.append((NOTE, "docks", "no docks in the database (map-docking has no target)"))

    # 5. Motion / hardware blockers.
    if node.estop is True:
        checks.append((BLOCK, "e-stop", "emergency_stop engaged"))
    elif node.estop is False:
        checks.append((OK, "e-stop", "clear"))
    else:
        checks.append((NOTE, "e-stop", "no message on platform/emergency_stop"))

    if node.battery_pct is None:
        checks.append((NOTE, "battery", "no reading on platform/bms/state"))
    elif node.battery_pct < battery_critical:
        checks.append((BLOCK, "battery", f"{node.battery_pct:.0f}% - critically low"))
    elif node.battery_pct < battery_warn:
        checks.append((NOTE, "battery", f"{node.battery_pct:.0f}% - consider charging before a long mission"))
    else:
        checks.append((OK, "battery", f"{node.battery_pct:.0f}%"))

    # 6. Mission action server.
    if node.action_server_up(ExecuteMission, "autonomy/mission"):
        checks.append((OK, "mission server", "autonomy/mission available"))
    else:
        checks.append((BLOCK, "mission server", "autonomy/mission action server unavailable"))

    return checks


def render(checks, ns: str) -> tuple[str, bool]:
    blocks = [c for c in checks if c[0] == BLOCK]
    ready = not blocks
    mark = {OK: "  ok  ", BLOCK: " FAIL ", NOTE: " note "}
    width = max(len(label) for _, label, _ in checks)
    lines = [f"=== preflight: {ns} ===", ""]
    for sev, label, detail in checks:
        lines.append(f"[{mark[sev]}] {label.ljust(width)}  {detail}")
    lines.append("")
    if ready:
        lines.append("VERDICT: READY")
    else:
        lines.append("VERDICT: NOT READY - " + "; ".join(f"{l}: {d}" for s, l, d in blocks))
    return "\n".join(lines), ready


def main(argv=None):
    parser = make_parser(doc=__doc__, add_dry_run=False)
    parser.add_argument("--collect", type=float, default=4.0,
                        help="Seconds to subscribe before checking (default 4).")
    parser.add_argument("--battery-critical", type=float, default=15.0,
                        help="Battery %% below which readiness fails (default 15).")
    parser.add_argument("--battery-warn", type=float, default=50.0,
                        help="Battery %% below which to advise charging (default 50).")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_link")
    args = parser.parse_args(argv)

    if not args.namespace:
        print("error: no namespace - set ONAV_NAMESPACE (docker/*.env) or pass --namespace",
              file=sys.stderr)
        return 2

    rclpy.init()
    node = Preflight(args.namespace, args.map_frame, args.base_frame)
    try:
        node.get_logger().info(f"collecting for {args.collect:.0f}s...")
        node.collect(args.collect)
        checks = run_checks(node, args.battery_critical, args.battery_warn)
        report, ready = render(checks, args.namespace)
        print(report)
        return 0 if ready else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main() or 0)
