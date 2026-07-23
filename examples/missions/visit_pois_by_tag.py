#!/usr/bin/env python3
"""Visit every POI with a given tag in a loop using GoToPOI.

  ./visit_pois_by_tag.py --tag inspect
  ./visit_pois_by_tag.py --tag goto --loops 3
  ./visit_pois_by_tag.py --tag inspect --map <uuid>   # skip map menu

Fetches all POIs from the robot, filters to those that carry --tag, then
drives to each in alphabetical order using ExecuteGoToPOI. Repeats for
--loops iterations (default 1; 0 = infinite).

Built-in OutdoorNav tags: "goto" (navigation goal), "inspect" (PTZ camera
target). Custom tags assigned in the UI also work.

Touches:
  service <namespace>/mission_manager/get_all_maps               (GetAllMaps)
  service <namespace>/mission_manager/get_all_points_of_interest (GetAllPointsOfInterest)
  action  <namespace>/autonomy/goto_poi                          (ExecuteGoToPOI)
"""

from __future__ import annotations
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from clearpath_mission_manager_msgs.srv import GetAllMaps, GetAllPointsOfInterest
from clearpath_navigation_msgs.action import ExecuteGoToPOI

from examples.common.argparse_base import make_parser
from examples.common.config import map_id as default_map_id
from examples.common.ros_helpers import wait_for_service, wait_for_action
from examples.common.onav import fetch_maps, fetch_pois, select_map, pick_from_list


class VisitPoisByTag(Node):
    def __init__(self, namespace: str):
        super().__init__("visit_pois_by_tag")

        self.maps_srv = f"{namespace}/mission_manager/get_all_maps"
        self.pois_srv = f"{namespace}/mission_manager/get_all_points_of_interest"
        self.goto_action = f"{namespace}/autonomy/goto_poi"

        self.maps_client = self.create_client(GetAllMaps, self.maps_srv)
        self.pois_client = self.create_client(GetAllPointsOfInterest, self.pois_srv)
        self.goto_client = ActionClient(self, ExecuteGoToPOI, self.goto_action)

    def wait(self) -> None:
        wait_for_service(self, self.maps_client, self.maps_srv)
        wait_for_service(self, self.pois_client, self.pois_srv)
        wait_for_action(self, self.goto_client, self.goto_action)

    def goto_poi(self, poi_uuid: str, map_uuid: str, poi_name: str) -> bool:
        goal = ExecuteGoToPOI.Goal(poi_uuid=poi_uuid, map_uuid=map_uuid)
        send_future = self.goto_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if not handle or not handle.accepted:
            self.get_logger().error(f"GoToPOI rejected: {poi_name!r}")
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        status = result_future.result().status if result_future.result() else None
        ok = status == 4
        self.get_logger().info(f"GoToPOI {poi_name!r}: {'succeeded' if ok else f'status={status}'}")
        return ok

    def run(self, map_uuid: str, pois: list, max_loops: int) -> None:
        limit = "∞" if max_loops == 0 else str(max_loops)
        loop = 0
        while max_loops == 0 or loop < max_loops:
            loop += 1
            self.get_logger().info(f"loop {loop}/{limit} — visiting {len(pois)} POIs")
            for poi in pois:
                if not rclpy.ok():
                    return
                if not self.goto_poi(poi.uuid, map_uuid, poi.name):
                    self.get_logger().error("stopping loop after failed GoToPOI")
                    return
        self.get_logger().info(f"completed {loop} loop(s)")


def main(argv=None):
    parser = make_parser(doc=__doc__)
    parser.add_argument("--tag", required=True,
                        help='POI tag to filter on (e.g. "goto", "inspect", or a custom tag).')
    parser.add_argument("--map", default=default_map_id() or None,
                        help="Map UUID (or $ONAV_MAP_ID). Skips the map menu.")
    parser.add_argument("--loops", type=int, default=1,
                        help="Loop count (default 1; 0 = infinite).")
    args = parser.parse_args(argv)

    rclpy.init()
    node = VisitPoisByTag(args.namespace)
    try:
        node.wait()

        map_uuid, map_name = select_map(node, node.maps_client, args.map)

        all_pois = fetch_pois(node, node.pois_client)
        pois = [p for p in all_pois if args.tag in (p.tags or [])]

        if not pois:
            node.get_logger().error(
                f"no POIs with tag {args.tag!r} found "
                f"(all POIs: {[p.name for p in all_pois]})"
            )
            return

        node.get_logger().info(
            f"map: {map_name!r}  tag: {args.tag!r}  "
            f"POIs: {[p.name for p in pois]}"
        )

        if args.dry_run:
            limit = "∞" if args.loops == 0 else str(args.loops)
            print(f"[dry-run] would visit {len(pois)} POIs x{limit} loops on map {map_uuid}")
            for p in pois:
                print(f"[dry-run]   {p.name}  ({p.latitude:.6f}, {p.longitude:.6f})")
            return

        node.run(map_uuid, pois, args.loops)
    except KeyboardInterrupt:
        node.get_logger().info("interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
