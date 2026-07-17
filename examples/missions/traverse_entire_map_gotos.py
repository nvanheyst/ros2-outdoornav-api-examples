#!/usr/bin/env python3
"""Visit every map node ad-hoc via ExecuteGoTo, in greedy NN order from the current fix.

  ./traverse_entire_map_gotos.py --map-uuid <uuid>
  ./traverse_entire_map_gotos.py --map-uuid <uuid> --naive   # for comparison
  ONAV_MAP_ID=<uuid> ./traverse_entire_map_gotos.py

Map nodes aren't POIs, so this uses ExecuteGoTo with a synthetic
Waypoint per node rather than ExecuteGoToPOI.

Compare with `generate_traversal_mission.py`: that one builds a
persistent NetworkMission + Waypoints (stored, UI-visible, re-runnable)
and the robot follows map edges. This one is the ephemeral version:
no records left behind, robot picks its own GPS path between nodes
(ignoring edges), and the order varies with where you start because
it's greedy NN from the live fix.

Touches:
  service <namespace>/mission_manager/get_map  (GetMap)
  action  <namespace>/autonomy/goto            (ExecuteGoTo)
  topic   <namespace>/localization/fix         (NavSatFix, subscribe)
"""

from __future__ import annotations
import math
import sys
import uuid
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import NavSatFix
from clearpath_mission_manager_msgs.srv import GetMap
from clearpath_navigation_msgs.msg import Waypoint
from clearpath_navigation_msgs.action import ExecuteGoTo

from examples.common.argparse_base import make_parser
from examples.common.config import map_id as default_map_id
from examples.common.ros_helpers import wait_for_service, wait_for_action, call_service


POSITION_TOLERANCE_M = 1.0
# Negative disables heading constraint - pass-through, no spin to match.
YAW_TOLERANCE_RAD = -1.0
EARTH_R = 6_371_000.0


def haversine_m(a, b):
    lat1, lon1 = math.radians(a[0]), math.radians(a[1])
    lat2, lon2 = math.radians(b[0]), math.radians(b[1])
    dlat, dlon = lat2 - lat1, lon2 - lon1
    h = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * EARTH_R * math.asin(math.sqrt(h))


def bearing_deg(a, b):
    lat1, lat2 = math.radians(a[0]), math.radians(b[0])
    dlon = math.radians(b[1] - a[1])
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0


def route_length(order):
    return sum(haversine_m(order[i], order[i + 1]) for i in range(len(order) - 1))


def greedy_nearest(start, pts):
    remaining = list(range(len(pts)))
    here = start
    order = []
    while remaining:
        nxt = min(remaining, key=lambda i: haversine_m(here, pts[i]))
        order.append(nxt)
        here = pts[nxt]
        remaining.remove(nxt)
    return order


class TraverseShortest(Node):
    def __init__(self, namespace: str, map_uuid: str, naive: bool):
        super().__init__("traverse_shortest")
        self.map_uuid = map_uuid
        self.naive = naive
        self.fix = None
        self.fix_topic = f"{namespace}/localization/fix"
        self.get_map_srv = f"{namespace}/mission_manager/get_map"
        self.goto_action = f"{namespace}/autonomy/goto"
        self.create_subscription(NavSatFix, self.fix_topic, self._fix_cb, 10)
        self.get_map_client = self.create_client(GetMap, self.get_map_srv)
        self.goto_client = ActionClient(self, ExecuteGoTo, self.goto_action)
        self._goal_handle = None

    def _fix_cb(self, msg: NavSatFix) -> None:
        if not math.isnan(msg.latitude) and not math.isnan(msg.longitude):
            self.fix = (msg.latitude, msg.longitude)

    def wait_for_inputs(self) -> None:
        wait_for_service(self, self.get_map_client, self.get_map_srv)
        wait_for_action(self, self.goto_client, self.goto_action)
        self.get_logger().info(f"waiting for first fix on {self.fix_topic}...")
        while self.fix is None:
            rclpy.spin_once(self, timeout_sec=1.0)
        self.get_logger().info(f"current fix: {self.fix[0]:.6f}, {self.fix[1]:.6f}")

    def fetch_points(self):
        resp = call_service(self, self.get_map_client, GetMap.Request(uuid=self.map_uuid))
        if resp is None or not resp.map.points:
            return []
        return [(p.uuid, p.latitude, p.longitude) for p in resp.map.points]

    def go_to(self, lat: float, lon: float, heading: float = 0.0) -> bool:
        wp = Waypoint()
        wp.uuid = str(uuid.uuid4())
        wp.name = f"shortest_{wp.uuid[:8]}"
        wp.latitude = lat
        wp.longitude = lon
        wp.heading = heading
        wp.position_tolerance = POSITION_TOLERANCE_M
        wp.yaw_tolerance = YAW_TOLERANCE_RAD

        goal = ExecuteGoTo.Goal()
        goal.map_uuid = self.map_uuid
        goal.waypoint = wp
        send_future = self.goto_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        self._goal_handle = send_future.result()
        if not self._goal_handle or not self._goal_handle.accepted:
            return False
        result_future = self._goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self._goal_handle = None
        status = result_future.result().status if result_future.result() else None
        return status == 4

    def cancel_in_flight(self) -> None:
        if self._goal_handle is None:
            return
        cancel_future = self._goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future)

    def run(self) -> None:
        points = self.fetch_points()
        if not points:
            self.get_logger().error("no points on map")
            return
        coords = [(p[1], p[2]) for p in points]
        naive_order = list(range(len(points)))
        greedy_order = greedy_nearest(self.fix, coords)
        naive_len = route_length([self.fix] + [coords[i] for i in naive_order])
        greedy_len = route_length([self.fix] + [coords[i] for i in greedy_order])
        savings = 100 * (1 - greedy_len / naive_len) if naive_len > 0 else 0.0
        self.get_logger().info(f"points: {len(points)}")
        self.get_logger().info(f"  naive  route: {naive_len:7.1f} m")
        self.get_logger().info(f"  greedy route: {greedy_len:7.1f} m  ({savings:.0f}% shorter)")

        order = naive_order if self.naive else greedy_order
        ordered_pts = [coords[i] for i in order]
        for i, idx in enumerate(order, 1):
            point_uuid, lat, lon = points[idx]
            if i < len(order):
                heading = bearing_deg(ordered_pts[i - 1], ordered_pts[i])
            elif i > 1:
                heading = bearing_deg(ordered_pts[i - 2], ordered_pts[i - 1])
            else:
                heading = 0.0
            self.get_logger().info(
                f"[{i}/{len(order)}] -> ({lat:.6f}, {lon:.6f}) hdg {heading:.0f} node={point_uuid[:8]}"
            )
            ok = self.go_to(lat, lon, heading)
            self.get_logger().info(f"    {'OK' if ok else 'FAILED'}")


def main(argv=None):
    parser = make_parser(doc=__doc__)
    parser.add_argument("--map-uuid", default=default_map_id() or None,
                        help="Map UUID (or $ONAV_MAP_ID).")
    parser.add_argument("--naive", action="store_true",
                        help="Use the order GetMap returned (no greedy reorder).")
    args = parser.parse_args(argv)

    if not args.map_uuid:
        parser.error("--map-uuid required (or set $ONAV_MAP_ID)")

    if args.dry_run:
        print(f"[dry-run] would visit every node on map {args.map_uuid}")
        print(f"[dry-run] action: {args.namespace}/autonomy/goto, mode={'naive' if args.naive else 'greedy'}")
        return

    rclpy.init()
    node = TraverseShortest(args.namespace, args.map_uuid, args.naive)
    try:
        node.wait_for_inputs()
        node.run()
    except KeyboardInterrupt:
        node.cancel_in_flight()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
