#!/usr/bin/env python3
"""Loop a mission indefinitely, docking to charge when battery is low.

  ./loop_mission_dock_charge.py
  ./loop_mission_dock_charge.py --dock-threshold 20 --resume-threshold 80
  ./loop_mission_dock_charge.py --dock-name charging_dock --map-uuid <uuid> --mission-uuid <uuid>

When battery drops below --dock-threshold the robot docks and waits. Once the
battery reaches --resume-threshold it undocks and resumes looping. Runs forever
by default (--loops 0); set --loops N for a finite count.

Touches:
  action <namespace>/autonomy/mission       (ExecuteMission)
  action <namespace>/autonomy/dock_local    (Dock)
  action <namespace>/autonomy/undock        (Undock)
  topic  <namespace>/platform/bms/state     (BatteryState, subscribe)
"""

from __future__ import annotations
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import BatteryState
from clearpath_navigation_msgs.action import ExecuteMission
from clearpath_dock_msgs.action import Dock, Undock

from clearpath_mission_manager_msgs.srv import GetAllMaps, GetAllNetworkMissions
from clearpath_dock_msgs.srv import GetDockDatabase

from examples.common.argparse_base import make_parser
from examples.common.config import map_id as default_map_id, mission_id as default_mission_id
from examples.common.ros_helpers import wait_for_action, wait_for_service
from examples.common.onav import select_map, select_mission, select_dock


class MissionWithCharging(Node):
    def __init__(self, namespace: str, dock_threshold: float, resume_threshold: float,
                 max_loops: int):
        super().__init__("loop_mission_dock_charge")
        self.dock_name = ""
        self.dock_threshold = dock_threshold
        self.resume_threshold = resume_threshold
        self.max_loops = max_loops
        self.mission_uuid = ""
        self.map_uuid = ""
        self.latest_percent: float | None = None
        self._goal_handle = None

        self.mission_action = f"{namespace}/autonomy/mission"
        self.dock_action_path = f"{namespace}/autonomy/dock_local"
        self.undock_action_path = f"{namespace}/autonomy/undock"
        self.maps_srv = f"{namespace}/mission_manager/get_all_maps"
        self.missions_srv = f"{namespace}/mission_manager/get_all_missions"
        self.dock_db_srv = f"{namespace}/docking/get_dock_database"

        self.create_subscription(BatteryState, f"{namespace}/platform/bms/state", self._bms_cb, 10)
        self.mission_client = ActionClient(self, ExecuteMission, self.mission_action)
        self.dock_client = ActionClient(self, Dock, self.dock_action_path)
        self.undock_client = ActionClient(self, Undock, self.undock_action_path)
        self.maps_client = self.create_client(GetAllMaps, self.maps_srv)
        self.missions_client = self.create_client(GetAllNetworkMissions, self.missions_srv)
        self.dock_db_client = self.create_client(GetDockDatabase, self.dock_db_srv)

    def _bms_cb(self, msg: BatteryState) -> None:
        self.latest_percent = float(msg.percentage) * 100.0

    def wait_for_initial(self) -> None:
        wait_for_action(self, self.mission_client, self.mission_action)
        wait_for_action(self, self.dock_client, self.dock_action_path)
        wait_for_action(self, self.undock_client, self.undock_action_path)
        wait_for_service(self, self.maps_client, self.maps_srv)
        wait_for_service(self, self.missions_client, self.missions_srv)
        # dock_db is optional — select_dock probes it and falls back to a typed name.
        self.get_logger().info("waiting for first BMS reading …")
        while self.latest_percent is None:
            rclpy.spin_once(self, timeout_sec=1.0)
        self.get_logger().info(f"initial battery: {self.latest_percent:.1f}%")

    def _run_action(self, client, goal, label: str) -> bool:
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if not handle or not handle.accepted:
            self.get_logger().error(f"{label}: goal rejected")
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        success = bool(getattr(result.result, "success", False)) if result else False
        msg = getattr(result.result, "message", "") if result else ""
        self.get_logger().info(f"{label}: success={success} msg={msg!r}")
        return success

    def charge_cycle(self) -> bool:
        self.get_logger().info(
            f"battery {self.latest_percent:.1f}% below {self.dock_threshold:.1f}% "
            f"— docking at {self.dock_name!r}"
        )
        if not self._run_action(self.dock_client, Dock.Goal(dock_name=self.dock_name), "Dock"):
            self.get_logger().error("dock failed — stopping loop")
            return False
        self.get_logger().info(f"charging … waiting for {self.resume_threshold:.1f}%")
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=5.0)
            pct = self.latest_percent
            if pct is not None:
                self.get_logger().info(f"  battery: {pct:.1f}%")
                if pct >= self.resume_threshold:
                    break
        if not self._run_action(self.undock_client, Undock.Goal(dock_name=self.dock_name), "Undock"):
            self.get_logger().error("undock failed — stopping loop")
            return False
        self.get_logger().info("resuming mission loop")
        return True

    def run_mission_blocking(self) -> bool:
        goal = ExecuteMission.Goal(mission_uuid=self.mission_uuid, map_uuid=self.map_uuid)
        send_future = self.mission_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        self._goal_handle = send_future.result()
        if not self._goal_handle or not self._goal_handle.accepted:
            self.get_logger().error("mission goal rejected")
            return False
        result_future = self._goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.5)
        self._goal_handle = None
        status = result_future.result().status if result_future.result() else None
        return status == 4

    def cancel_in_flight(self) -> None:
        if self._goal_handle is None:
            return
        self.get_logger().warn("cancelling in-flight mission …")
        cancel_future = self._goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future)

    def loop(self) -> None:
        limit = "∞" if self.max_loops == 0 else str(self.max_loops)
        i = 0
        while self.max_loops == 0 or i < self.max_loops:
            i += 1
            rclpy.spin_once(self, timeout_sec=0.1)
            rclpy.spin_once(self, timeout_sec=0.1)
            pct = self.latest_percent
            self.get_logger().info(
                f"loop {i}/{limit}  battery={pct:.1f}%  dock_threshold={self.dock_threshold:.1f}%"
            )
            if pct < self.dock_threshold:
                if not self.charge_cycle():
                    return
            if not self.run_mission_blocking():
                self.get_logger().error(f"loop {i} mission failed — stopping")
                return
        self.get_logger().info(f"completed all {self.max_loops} loops")


def main(argv=None):
    parser = make_parser(doc=__doc__)
    parser.add_argument("--dock-name", default=None,
                        help="Dock to charge at. Omit for interactive menu.")
    parser.add_argument("--dock-threshold", type=float, default=20.0,
                        help="Dock when battery drops below this percent (default 20).")
    parser.add_argument("--resume-threshold", type=float, default=80.0,
                        help="Undock and resume when battery reaches this percent (default 80).")
    parser.add_argument("--loops", type=int, default=0,
                        help="Maximum loop count (default 0 = infinite).")
    parser.add_argument("--mission-uuid", default=default_mission_id() or None,
                        help="Mission UUID (or $ONAV_MISSION_ID).")
    parser.add_argument("--map-uuid", default=default_map_id() or None,
                        help="Map UUID (or $ONAV_MAP_ID).")
    args = parser.parse_args(argv)

    if args.dry_run:
        ns = args.namespace
        limit = "∞" if args.loops == 0 else str(args.loops)
        print(f"[dry-run] loop {limit}x ExecuteMission via {ns}/autonomy/mission")
        print(f"[dry-run] map/mission: {'provided' if args.map_uuid and args.mission_uuid else 'interactive menu'}")
        print(f"[dry-run] dock at {args.dock_name!r} when battery < {args.dock_threshold}%")
        print(f"[dry-run] resume when battery >= {args.resume_threshold}%")
        return

    rclpy.init()
    node = MissionWithCharging(
        args.namespace,
        args.dock_threshold, args.resume_threshold,
        args.loops,
    )
    try:
        node.wait_for_initial()
        node.map_uuid, _ = select_map(node, node.maps_client, args.map_uuid or "")
        node.mission_uuid, _ = select_mission(node, node.missions_client, args.mission_uuid or "")
        node.dock_name = select_dock(node, node.dock_db_client, args.dock_name or "")
        node.loop()
    except KeyboardInterrupt:
        node.cancel_in_flight()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
