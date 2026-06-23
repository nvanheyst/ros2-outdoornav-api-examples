#!/usr/bin/env python3
"""Start a mission, sleep, then cancel it via the action goal handle.

Demonstrates the correct cancel pattern: you cancel an action *goal*, not
a service. The handle you need comes back from `send_goal_async`, so the
process doing the cancelling must be the one that started the mission.

  ./cancel_mission.py --mission-uuid <u> --map-uuid <u> --after 10
  ONAV_MISSION_ID=<u> ONAV_MAP_ID=<u> ./cancel_mission.py --after 5

If you need to stop a mission started elsewhere (e.g. from the OnAV UI)
this pattern won't help — use `stop_autonomy.py` (Trigger) instead.

Touches: action <namespace>/autonomy/mission
         (ExecuteMission send_goal_async + goal_handle.cancel_goal_async).
"""

from __future__ import annotations
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from clearpath_navigation_msgs.action import ExecuteMission

from common.argparse_base import make_parser
from common.config import map_id as default_map_id, mission_id as default_mission_id
from common.ros_helpers import wait_for_action


class CancelMission(Node):
    def __init__(self, namespace: str, mission_uuid: str, map_uuid: str):
        super().__init__("cancel_mission")
        self.mission_uuid = mission_uuid
        self.map_uuid = map_uuid
        self.mission_action = f"{namespace}/autonomy/mission"
        self.client = ActionClient(self, ExecuteMission, self.mission_action)
        self._goal_handle = None

    def wait(self) -> None:
        wait_for_action(self, self.client, self.mission_action)

    def start(self) -> bool:
        goal = ExecuteMission.Goal(mission_uuid=self.mission_uuid, map_uuid=self.map_uuid)
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        self._goal_handle = send_future.result()
        if not self._goal_handle or not self._goal_handle.accepted:
            self.get_logger().error("ExecuteMission goal rejected")
            return False
        self.get_logger().info("mission accepted — robot should be moving")
        return True

    def sleep_with_spin(self, seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=min(0.5, end - time.time()))

    def cancel(self) -> bool:
        if self._goal_handle is None:
            return False
        self.get_logger().info("cancelling goal")
        cancel_future = self._goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future)
        resp = cancel_future.result()
        n = len(getattr(resp, "goals_canceling", [])) if resp else 0
        self.get_logger().info(f"  cancel returned, goals_canceling={n}")

        result_future = self._goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)
        status = result_future.result().status if result_future.result() else None
        self.get_logger().info(f"  mission ended with status {status} (5=CANCELED, 4=SUCCEEDED)")
        self._goal_handle = None
        return status == 5


def main(argv=None):
    parser = make_parser(doc=__doc__)
    parser.add_argument("--mission-uuid", default=default_mission_id() or None,
                        help="Mission UUID (or $ONAV_MISSION_ID).")
    parser.add_argument("--map-uuid", default=default_map_id() or None,
                        help="Map UUID (or $ONAV_MAP_ID).")
    parser.add_argument("--after", type=float, default=10.0,
                        help="Seconds to let the mission run before cancelling (default 10).")
    args = parser.parse_args(argv)

    if not args.mission_uuid or not args.map_uuid:
        parser.error("--mission-uuid and --map-uuid required (or set $ONAV_MISSION_ID and $ONAV_MAP_ID)")

    if args.dry_run:
        print(f"[dry-run] would start mission {args.mission_uuid} on map {args.map_uuid}")
        print(f"[dry-run] then sleep {args.after}s and cancel via the goal handle")
        return

    rclpy.init()
    node = CancelMission(args.namespace, args.mission_uuid, args.map_uuid)
    try:
        node.wait()
        if not node.start():
            return
        node.sleep_with_spin(args.after)
        node.cancel()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
