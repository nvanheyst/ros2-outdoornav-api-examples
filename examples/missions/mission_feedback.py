#!/usr/bin/env python3
"""Run a mission and print per-waypoint progress via the feedback callback.

Most mission scripts use spin_until_future_complete, which blocks the executor
and swallows feedback. This script uses send_goal_async with a feedback_callback
and a manual spin loop so every feedback message arrives.

  ./mission_feedback.py
  ./mission_feedback.py --map-uuid <uuid> --mission-uuid <uuid>

Touches:
  service <namespace>/mission_manager/get_all_maps            (GetAllMaps)
  service <namespace>/mission_manager/get_all_network_missions (GetAllNetworkMissions)
  action  <namespace>/autonomy/mission                        (ExecuteMission)
"""

from __future__ import annotations
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from clearpath_navigation_msgs.action import ExecuteMission
from clearpath_mission_manager_msgs.srv import GetAllMaps, GetAllNetworkMissions

from examples.common.argparse_base import make_parser
from examples.common.config import map_id as default_map_id, mission_id as default_mission_id
from examples.common.ros_helpers import wait_for_service, wait_for_action
from examples.common.onav import select_map, select_mission


class MissionWithFeedback(Node):
    def __init__(self, namespace: str):
        super().__init__("mission_feedback")
        self.maps_srv = f"{namespace}/mission_manager/get_all_maps"
        self.missions_srv = f"{namespace}/mission_manager/get_all_network_missions"
        self.mission_action = f"{namespace}/autonomy/mission"
        self.maps_client = self.create_client(GetAllMaps, self.maps_srv)
        self.missions_client = self.create_client(GetAllNetworkMissions, self.missions_srv)
        self.mission_client = ActionClient(self, ExecuteMission, self.mission_action)
        self._goal_handle = None
        self._done = False
        self._status = None

    def wait(self) -> None:
        wait_for_service(self, self.maps_client, self.maps_srv)
        wait_for_service(self, self.missions_client, self.missions_srv)
        wait_for_action(self, self.mission_client, self.mission_action)

    def _on_feedback(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        # ExecuteMission feedback carries current_waypoint and total_waypoints
        current = getattr(fb, "current_waypoint", None)
        total = getattr(fb, "total_waypoints", None)
        pct = getattr(fb, "percent_complete", None)
        if current is not None and total is not None:
            self.get_logger().info(f"waypoint {current}/{total}  ({pct:.0f}%)" if pct else f"waypoint {current}/{total}")
        else:
            self.get_logger().info(f"feedback: {fb}")

    def _on_result(self, future) -> None:
        result = future.result()
        self._status = result.status if result else None
        self._done = True

    def run(self, map_uuid: str, mission_uuid: str) -> int | None:
        goal = ExecuteMission.Goal(mission_uuid=mission_uuid, map_uuid=map_uuid)
        send_future = self.mission_client.send_goal_async(
            goal, feedback_callback=self._on_feedback
        )
        rclpy.spin_until_future_complete(self, send_future)
        self._goal_handle = send_future.result()
        if not self._goal_handle or not self._goal_handle.accepted:
            self.get_logger().error("mission goal rejected")
            return None
        self.get_logger().info("mission accepted — waiting for feedback")
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)
        while rclpy.ok() and not self._done:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._status

    def cancel(self) -> None:
        if self._goal_handle is not None:
            self.get_logger().warn("cancelling in-flight mission")
            rclpy.spin_until_future_complete(self, self._goal_handle.cancel_goal_async())


def main(argv=None):
    parser = make_parser(doc=__doc__)
    parser.add_argument("--map-uuid", default=default_map_id() or None,
                        help="Map UUID (or $ONAV_MAP_ID). Omit for interactive menu.")
    parser.add_argument("--mission-uuid", default=default_mission_id() or None,
                        help="Mission UUID (or $ONAV_MISSION_ID). Omit for interactive menu.")
    args = parser.parse_args(argv)

    if args.dry_run:
        ns = args.namespace
        print(f"[dry-run] {ns}/mission_manager/get_all_maps  (map selection)")
        print(f"[dry-run] {ns}/mission_manager/get_all_network_missions  (mission selection)")
        print(f"[dry-run] {ns}/autonomy/mission  (ExecuteMission + feedback_callback)")
        return

    rclpy.init()
    node = MissionWithFeedback(args.namespace)
    try:
        node.wait()
        map_uuid, map_name = select_map(node, node.maps_client, args.map_uuid or "")
        mission_uuid, mission_name = select_mission(
            node, node.missions_client, args.mission_uuid or ""
        )
        node.get_logger().info(
            f"running {mission_name!r} on map {map_name!r}"
        )
        status = node.run(map_uuid, mission_uuid)
        if status == 4:
            node.get_logger().info("mission succeeded")
        else:
            node.get_logger().error(f"mission ended with status {status} (4=SUCCEEDED)")
    except KeyboardInterrupt:
        node.cancel()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
