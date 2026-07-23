#!/usr/bin/env python3
"""Fire random GoTo destinations inside a map's bounding box.

  ./random_visit_mission.py --map-uuid <uuid> --count 10
  ./random_visit_mission.py --count 0 --seed 42        # infinite, deterministic
  ONAV_MAP_ID=<uuid> ./random_visit_mission.py

Uses ExecuteGoTo (free GPS pose), not GoToPOI - targets are random points in
the map bbox; autonomy rejects whatever it can't reach.

CAUTION: targets are random GPS points with no awareness of your map's edges,
known obstacles, or dock location. Run only in an open area clear of hazards.

Touches:
  service <namespace>/mission_manager/get_map  (GetMap)
  action  <namespace>/autonomy/goto            (ExecuteGoTo)
"""

from __future__ import annotations
import math
import random
import sys
import uuid
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from clearpath_mission_manager_msgs.srv import GetMap
from clearpath_navigation_msgs.msg import Waypoint
from clearpath_navigation_msgs.action import ExecuteGoTo

from examples.common.argparse_base import make_parser
from examples.common.config import map_id as default_map_id
from examples.common.ros_helpers import wait_for_service, wait_for_action, call_service


POSITION_TOLERANCE_M = 1.0
YAW_TOLERANCE_RAD = 0.4


def bbox(points):
    lats = [p.latitude for p in points]
    lons = [p.longitude for p in points]
    return min(lats), max(lats), min(lons), max(lons)


def haversine_m(a, b):
    lat1, lon1 = math.radians(a[0]), math.radians(a[1])
    lat2, lon2 = math.radians(b[0]), math.radians(b[1])
    dlat, dlon = lat2 - lat1, lon2 - lon1
    h = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * 6_371_000.0 * math.asin(math.sqrt(h))


class RandomVisit(Node):
    def __init__(self, namespace: str, map_uuid: str, count: int, seed: int | None):
        super().__init__("random_visit")
        self.map_uuid = map_uuid
        self.count = count
        self.rng = random.Random(seed)
        self.get_map_srv = f"{namespace}/mission_manager/get_map"
        self.goto_action = f"{namespace}/autonomy/goto"
        self.get_map_client = self.create_client(GetMap, self.get_map_srv)
        self.goto_client = ActionClient(self, ExecuteGoTo, self.goto_action)
        self._goal_handle = None
        self.bbox = None

    def wait(self) -> None:
        wait_for_service(self, self.get_map_client, self.get_map_srv)
        wait_for_action(self, self.goto_client, self.goto_action)

    def fetch_bbox(self) -> None:
        resp = call_service(self, self.get_map_client, GetMap.Request(uuid=self.map_uuid))
        if resp is None or not resp.map.points:
            raise RuntimeError("GetMap returned no points.")
        self.bbox = bbox(resp.map.points)
        lat_min, lat_max, lon_min, lon_max = self.bbox
        diag = haversine_m((lat_min, lon_min), (lat_max, lon_max))
        self.get_logger().info(
            f"map bbox: lat [{lat_min:.6f}, {lat_max:.6f}] "
            f"lon [{lon_min:.6f}, {lon_max:.6f}]  (diagonal {diag:.0f} m)"
        )

    def random_target(self):
        lat_min, lat_max, lon_min, lon_max = self.bbox
        lat = self.rng.uniform(lat_min, lat_max)
        lon = self.rng.uniform(lon_min, lon_max)
        heading = self.rng.uniform(-math.pi, math.pi)
        return lat, lon, heading

    def go_to(self, lat, lon, heading) -> bool:
        wp = Waypoint()
        wp.uuid = str(uuid.uuid4())
        wp.name = "random_visit"
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
        self.get_logger().warn("cancelling in-flight GoTo...")
        cancel_future = self._goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future)

    def run(self) -> None:
        self.fetch_bbox()
        i = 0
        while self.count == 0 or i < self.count:
            i += 1
            lat, lon, heading = self.random_target()
            self.get_logger().info(
                f"[{i}{'' if self.count == 0 else '/' + str(self.count)}] "
                f"GoTo ({lat:.6f}, {lon:.6f}) hdg {math.degrees(heading):+.0f}°"
            )
            ok = self.go_to(lat, lon, heading)
            self.get_logger().info(f"  {'OK' if ok else 'FAILED - continuing'}")


def main(argv=None):
    parser = make_parser(doc=__doc__)
    parser.add_argument("--map-uuid", default=default_map_id() or None,
                        help="Map UUID (or $ONAV_MAP_ID).")
    parser.add_argument("--count", type=int, default=10,
                        help="Number of hops (0 = infinite). Default 10.")
    parser.add_argument("--seed", type=int, default=None, help="RNG seed.")
    args = parser.parse_args(argv)

    if not args.map_uuid:
        parser.error("--map-uuid required (or set $ONAV_MAP_ID)")

    if args.dry_run:
        print(f"[dry-run] would fetch bbox via {args.namespace}/mission_manager/get_map")
        print(f"[dry-run] would fire {args.count or '∞'} GoTo goals to random bbox points")
        return

    rclpy.init()
    node = RandomVisit(args.namespace, args.map_uuid, args.count, args.seed)
    try:
        node.wait()
        node.run()
    except KeyboardInterrupt:
        node.cancel_in_flight()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
