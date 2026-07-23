#!/usr/bin/env python3
"""Loop a mission until battery drops below threshold or max loops hit.

  ./loop_mission_battery_aware.py --threshold 30 --loops 5
  ./loop_mission_battery_aware.py --threshold 30 --loops 0   # run forever
  ONAV_MAP_ID=<uuid> ONAV_MISSION_ID=<uuid> ./loop_mission_battery_aware.py

Spins the executor manually while the mission action is in flight so the BMS
subscription keeps draining.

Touches:
  action <namespace>/autonomy/mission     (ExecuteMission)
  topic  <namespace>/platform/bms/state   (BatteryState, subscribe)
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

from clearpath_mission_manager_msgs.srv import GetAllMaps, GetAllNetworkMissions

from examples.common.argparse_base import make_parser
from examples.common.config import map_id as default_map_id, mission_id as default_mission_id
from examples.common.ros_helpers import wait_for_action, wait_for_service
from examples.common.onav import select_map, select_mission


class BatteryAwareLoop(Node):
    def __init__(self, namespace: str, threshold_percent: float, max_loops: int):
        super().__init__("battery_aware_loop")
        self.threshold = threshold_percent
        self.max_loops = max_loops
        self.mission_uuid = ""
        self.map_uuid = ""
        self.latest_percent: float | None = None
        self.battery_topic = f"{namespace}/platform/bms/state"
        self.mission_action = f"{namespace}/autonomy/mission"
        self.maps_srv = f"{namespace}/mission_manager/get_all_maps"
        self.missions_srv = f"{namespace}/mission_manager/get_all_network_missions"
        self.create_subscription(BatteryState, self.battery_topic, self._bms_cb, 10)
        self.mission_client = ActionClient(self, ExecuteMission, self.mission_action)
        self.maps_client = self.create_client(GetAllMaps, self.maps_srv)
        self.missions_client = self.create_client(GetAllNetworkMissions, self.missions_srv)
        self._goal_handle = None

    def _bms_cb(self, msg: BatteryState) -> None:
        # BatteryState.percentage is 0.0-1.0
        self.latest_percent = float(msg.percentage) * 100.0

    def wait_for_initial(self) -> None:
        wait_for_action(self, self.mission_client, self.mission_action)
        wait_for_service(self, self.maps_client, self.maps_srv)
        wait_for_service(self, self.missions_client, self.missions_srv)
        self.get_logger().info("waiting for first BMS reading...")
        while self.latest_percent is None:
            rclpy.spin_once(self, timeout_sec=1.0)
        self.get_logger().info(f"initial battery: {self.latest_percent:.1f}%")

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
        self.get_logger().warn("cancelling in-flight mission...")
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
                f"loop {i}/{limit}  battery={pct:.1f}%  threshold={self.threshold:.1f}%"
            )
            if pct < self.threshold:
                self.get_logger().warn(
                    f"battery {pct:.1f}% below threshold {self.threshold:.1f}% - stopping"
                )
                return
            ok = self.run_mission_blocking()
            if not ok:
                self.get_logger().error(f"loop {i} mission failed - stopping")
                return
        self.get_logger().info(f"completed all {self.max_loops} loops")


def main(argv=None):
    parser = make_parser(doc=__doc__)
    parser.add_argument("--threshold", type=float, default=30.0,
                        help="Stop when battery drops below this percent (default 30).")
    parser.add_argument("--loops", type=int, default=10,
                        help="Maximum loop count (default 10). Use 0 for infinite.")
    parser.add_argument("--mission-uuid", default=default_mission_id() or None,
                        help="Mission UUID (or $ONAV_MISSION_ID).")
    parser.add_argument("--map-uuid", default=default_map_id() or None,
                        help="Map UUID (or $ONAV_MAP_ID).")
    args = parser.parse_args(argv)

    if args.dry_run:
        print(f"[dry-run] would loop ExecuteMission via {args.namespace}/autonomy/mission")
        print(f"[dry-run] threshold={args.threshold}% max_loops={args.loops}")
        print(f"[dry-run] map/mission: {'provided' if args.map_uuid and args.mission_uuid else 'interactive menu'}")
        return

    rclpy.init()
    node = BatteryAwareLoop(args.namespace, args.threshold, args.loops)
    try:
        node.wait_for_initial()
        node.map_uuid, _ = select_map(node, node.maps_client, args.map_uuid or "")
        node.mission_uuid, _ = select_mission(node, node.missions_client, args.mission_uuid or "")
        node.loop()
    except KeyboardInterrupt:
        node.cancel_in_flight()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
