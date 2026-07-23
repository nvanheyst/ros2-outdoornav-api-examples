#!/usr/bin/env python3
"""Dock the robot at a named dock already in the database.

  ./dock_now.py my_dock

The dock must exist in the robot's dock database before running. Run
service_inventory.py --grep dock to confirm the stack is up and to find
valid dock names.

Touches:
  action <namespace>/autonomy/dock_local  (Dock)
"""

from __future__ import annotations
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from clearpath_dock_msgs.action import Dock

from examples.common.argparse_base import make_parser
from examples.common.ros_helpers import wait_for_action


def main(argv=None):
    parser = make_parser(doc=__doc__, add_dry_run=False)
    parser.add_argument("dock_name", help="Name of the dock to target.")
    args = parser.parse_args(argv)

    rclpy.init()
    node = Node("dock_now")
    action_path = f"{args.namespace}/autonomy/dock_local"
    client = ActionClient(node, Dock, action_path)
    try:
        wait_for_action(node, client, action_path)
        send_future = client.send_goal_async(Dock.Goal(dock_name=args.dock_name))
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
            node.get_logger().info(f"docked at {args.dock_name!r}: {msg}")
        else:
            node.get_logger().error(f"dock failed: {msg}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
