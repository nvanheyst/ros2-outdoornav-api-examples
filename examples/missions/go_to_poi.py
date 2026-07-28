#!/usr/bin/env python3
"""Drive to a single named POI via ExecuteGoToPOI.

  ./go_to_poi.py                            # interactive map + POI menu
  ./go_to_poi.py --poi-uuid <uuid>          # skip POI menu
  ONAV_MAP_ID=<uuid> ONAV_POI_ID=<uuid> ./go_to_poi.py

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
from examples.common.config import map_id as default_map_id, poi_id as default_poi_id
from examples.common.ros_helpers import wait_for_service, wait_for_action
from examples.common.onav import select_map, select_poi


def main(argv=None):
    parser = make_parser(doc=__doc__)
    parser.add_argument("--map-uuid", default=default_map_id() or None,
                        help="Map UUID (or $ONAV_MAP_ID). Omit for interactive menu.")
    parser.add_argument("--poi-uuid", default=default_poi_id() or None,
                        help="POI UUID (or $ONAV_POI_ID). Omit for interactive menu.")
    args = parser.parse_args(argv)

    ns = args.namespace
    maps_srv = f"{ns}/mission_manager/get_all_maps"
    pois_srv = f"{ns}/mission_manager/get_all_points_of_interest"
    goto_action = f"{ns}/autonomy/goto_poi"

    if args.dry_run:
        print(f"[dry-run] {maps_srv}  (GetAllMaps — map selection)")
        print(f"[dry-run] {pois_srv}  (GetAllPointsOfInterest — POI selection)")
        print(f"[dry-run] {goto_action}  (ExecuteGoToPOI)")
        return

    rclpy.init()
    node = Node("go_to_poi")
    maps_client = node.create_client(GetAllMaps, maps_srv)
    pois_client = node.create_client(GetAllPointsOfInterest, pois_srv)
    goto_client = ActionClient(node, ExecuteGoToPOI, goto_action)
    try:
        wait_for_service(node, maps_client, maps_srv)
        wait_for_service(node, pois_client, pois_srv)
        wait_for_action(node, goto_client, goto_action)

        map_uuid, map_name = select_map(node, maps_client, args.map_uuid or "")
        poi_uuid, poi_name = select_poi(node, pois_client, args.poi_uuid or "")

        node.get_logger().info(
            f"GoToPOI {poi_name!r} on map {map_name!r}"
        )
        goal = ExecuteGoToPOI.Goal(poi_uuid=poi_uuid, map_uuid=map_uuid)
        send_future = goto_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send_future)
        handle = send_future.result()
        if not handle or not handle.accepted:
            node.get_logger().error("GoToPOI goal rejected")
            return
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
        status = result_future.result().status if result_future.result() else None
        if status == 4:
            node.get_logger().info(f"arrived at {poi_name!r}")
        else:
            node.get_logger().error(f"GoToPOI ended with status {status} (4=SUCCEEDED)")
    except KeyboardInterrupt:
        node.get_logger().info("interrupted")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
