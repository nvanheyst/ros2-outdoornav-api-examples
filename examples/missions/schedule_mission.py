#!/usr/bin/env python3
"""Wait until a target wall-clock time, then fire ExecuteMission.

  ./schedule_mission.py 2026-06-22T18:00:00Z
  ./schedule_mission.py +30s
  ./schedule_mission.py +5m --mission-uuid <uuid> --map-uuid <uuid>

Not a scheduler. This is the wait-then-fire that you would invoke from
a real scheduler (systemd timer, cron, etc.) - the python loop here is
just to demonstrate the action call.

Touches: action <namespace>/autonomy/mission (ExecuteMission).
"""

from __future__ import annotations
import re
import sys
import time
from datetime import datetime, timezone, timedelta
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from clearpath_navigation_msgs.action import ExecuteMission

from clearpath_mission_manager_msgs.srv import GetAllMaps, GetAllNetworkMissions

from examples.common.argparse_base import make_parser
from examples.common.config import map_id as default_map_id, mission_id as default_mission_id
from examples.common.ros_helpers import wait_for_action, wait_for_service
from examples.common.onav import select_map, select_mission


HEARTBEAT_S = 5.0


def parse_target(arg: str) -> datetime:
    """Accept '+30s' / '+5m' / '+2h' shortcut or ISO 8601 with timezone."""
    m = re.match(r"^\+(\d+)([smh])$", arg)
    if m:
        n, unit = int(m.group(1)), m.group(2)
        seconds = n * {"s": 1, "m": 60, "h": 3600}[unit]
        return datetime.now(timezone.utc) + timedelta(seconds=seconds)
    return datetime.fromisoformat(arg).astimezone(timezone.utc)


class ScheduledMission(Node):
    def __init__(self, namespace: str, target: datetime):
        super().__init__("scheduled_mission")
        self.target = target
        self.mission_uuid = ""
        self.map_uuid = ""
        self.mission_action = f"{namespace}/autonomy/mission"
        self.maps_srv = f"{namespace}/mission_manager/get_all_maps"
        self.missions_srv = f"{namespace}/mission_manager/get_all_missions"
        self.mission_client = ActionClient(self, ExecuteMission, self.mission_action)
        self.maps_client = self.create_client(GetAllMaps, self.maps_srv)
        self.missions_client = self.create_client(GetAllNetworkMissions, self.missions_srv)
        self._goal_handle = None

    def wait_for_server(self) -> None:
        wait_for_action(self, self.mission_client, self.mission_action)
        wait_for_service(self, self.maps_client, self.maps_srv)
        wait_for_service(self, self.missions_client, self.missions_srv)

    def countdown(self) -> None:
        while True:
            now = datetime.now(timezone.utc)
            remaining = (self.target - now).total_seconds()
            if remaining <= 0:
                self.get_logger().info("target reached - firing mission")
                return
            self.get_logger().info(f"  T-{int(remaining)}s  (target {self.target.isoformat()})")
            time.sleep(min(HEARTBEAT_S, remaining))

    def fire(self) -> bool:
        goal = ExecuteMission.Goal(mission_uuid=self.mission_uuid, map_uuid=self.map_uuid)
        send_future = self.mission_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        self._goal_handle = send_future.result()
        if not self._goal_handle or not self._goal_handle.accepted:
            self.get_logger().error("ExecuteMission goal rejected")
            return False
        self.get_logger().info("mission accepted - blocking on result")
        result_future = self._goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        status = result_future.result().status if result_future.result() else None
        if status == 4:
            self.get_logger().info("mission SUCCEEDED")
            return True
        self.get_logger().error(f"mission ended with status {status}")
        return False

    def cancel_in_flight(self) -> None:
        if self._goal_handle is None:
            return
        self.get_logger().warn("cancelling in-flight mission...")
        cancel_future = self._goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future)


def main(argv=None):
    parser = make_parser(doc=__doc__)
    parser.add_argument("when", help="Target time: ISO-8601 (2026-06-22T18:00:00Z) or +30s / +5m / +2h.")
    parser.add_argument("--mission-uuid", default=default_mission_id() or None,
                        help="Mission UUID (or $ONAV_MISSION_ID).")
    parser.add_argument("--map-uuid", default=default_map_id() or None,
                        help="Map UUID (or $ONAV_MAP_ID).")
    args = parser.parse_args(argv)

    target = parse_target(args.when)
    if target <= datetime.now(timezone.utc):
        parser.error("target time is in the past")
    if args.dry_run:
        print(f"[dry-run] would wait until {target.isoformat()} then fire ExecuteMission")
        print(f"[dry-run] action={args.namespace}/autonomy/mission")
        print(f"[dry-run] map/mission: {'provided' if args.map_uuid and args.mission_uuid else 'interactive menu'}")
        return

    rclpy.init()
    node = ScheduledMission(args.namespace, target)
    try:
        node.wait_for_server()
        node.map_uuid, _ = select_map(node, node.maps_client, args.map_uuid or "")
        node.mission_uuid, _ = select_mission(node, node.missions_client, args.mission_uuid or "")
        node.countdown()
        node.fire()
    except KeyboardInterrupt:
        node.cancel_in_flight()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
