#!/usr/bin/env python3
"""Dock the robot via map coordinates (MapDock action).

The dock pose is stored in the map rather than relative to the robot's sensor.
Use this on robots where the dock was added through the OutdoorNav UI or API.
For local-sensor docking, use dock_now.py instead.

  ./dock_map.py                              # interactive map + dock menu
  ./dock_map.py --dock-name my_dock          # skip dock menu
  ONAV_MAP_ID=<uuid> ./dock_map.py --dock-name my_dock

Touches:
  service <namespace>/mission_manager/get_all_maps  (GetAllMaps)
  service <namespace>/docking/get_dock_database     (GetDockDatabase)
  action  <namespace>/autonomy/dock_map             (MapDock)
"""

from __future__ import annotations
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from clearpath_dock_msgs.action import MapDock
from clearpath_dock_msgs.srv import GetDockDatabase
from clearpath_mission_manager_msgs.srv import GetAllMaps

from examples.common.argparse_base import make_parser
from examples.common.config import map_id as default_map_id
from examples.common.ros_helpers import wait_for_action, wait_for_service
from examples.common.onav import select_map, select_dock


def main(argv=None):
    parser = make_parser(doc=__doc__, add_dry_run=False)
    parser.add_argument("--map-uuid", default=default_map_id() or None,
                        help="Map UUID (or $ONAV_MAP_ID). Omit for interactive menu.")
    parser.add_argument("--dock-name", default=None,
                        help="Dock name. Omit for interactive menu.")
    args = parser.parse_args(argv)

    ns = args.namespace
    maps_srv = f"{ns}/mission_manager/get_all_maps"
    dock_db_srv = f"{ns}/docking/get_dock_database"
    action_path = f"{ns}/autonomy/dock_map"

    rclpy.init()
    node = Node("dock_map")
    maps_client = node.create_client(GetAllMaps, maps_srv)
    dock_db_client = node.create_client(GetDockDatabase, dock_db_srv)
    client = ActionClient(node, MapDock, action_path)
    try:
        wait_for_service(node, maps_client, maps_srv)
        wait_for_service(node, dock_db_client, dock_db_srv)
        wait_for_action(node, client, action_path)

        map_uuid, map_name = select_map(node, maps_client, args.map_uuid or "")
        dock_name = select_dock(node, dock_db_client, args.dock_name or "")

        node.get_logger().info(
            f"MapDock: {dock_name!r} on map {map_name!r}"
        )
        goal = MapDock.Goal(dock_name=dock_name, map_uuid=map_uuid)
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send_future)
        handle = send_future.result()
        if not handle or not handle.accepted:
            node.get_logger().error("dock goal rejected")
            return
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
        result = result_future.result()
        success = bool(getattr(result.result, "success", False)) if result else False
        msg = getattr(result.result, "message", "") if result else ""
        if success:
            node.get_logger().info(f"docked at {dock_name!r}: {msg}")
        else:
            error_code = getattr(result.result, "error_code", None) if result else None
            node.get_logger().error(
                f"dock failed (code={error_code}): {msg}"
            )
    except KeyboardInterrupt:
        node.get_logger().info("interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
