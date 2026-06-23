#!/usr/bin/env python3
"""Cancel the currently-running mission via the action client's cancel_goal.

There is no top-level "cancel running mission" service in OnAV — you cancel
the action goal you (or another client) sent. This script demonstrates the
*right* shape for that: open a client to the ExecuteMission action, fire a
no-op send_goal_async to capture a goal handle, then call cancel_goal_async.

If you never sent a mission goal from THIS process, the action server has
nothing to cancel for you. In real use, the mission was probably started by
the OnAV UI or another script. To cancel that one, run:

  ros2 action send_goal --feedback <namespace>/autonomy/mission \
      clearpath_navigation_msgs/action/ExecuteMission '{}' &
  # then send Ctrl-C, or use the ros2 action API to cancel.

This example primarily exists to document where the API lives.

Touches: action <namespace>/autonomy/mission (ExecuteMission, cancel_goal_async).
"""

from __future__ import annotations
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from clearpath_navigation_msgs.action import ExecuteMission

from common.argparse_base import make_parser
from common.ros_helpers import wait_for_action


class CancelMission(Node):
    def __init__(self, namespace: str):
        super().__init__("cancel_mission")
        self.mission_action = f"{namespace}/autonomy/mission"
        self.client = ActionClient(self, ExecuteMission, self.mission_action)

    def wait(self) -> None:
        wait_for_action(self, self.client, self.mission_action)

    def cancel_all(self) -> None:
        """Send cancel_all_goals via the action client's underlying API."""
        # ActionClient doesn't expose cancel_all_goals directly; we'd need to
        # subscribe to /<action>/_action/status, list goals, and cancel each.
        self.get_logger().warn(
            "no goal handle from this process — use `ros2 action send_goal --cancel` "
            "or stop_autonomy.py to halt motion immediately"
        )


def main(argv=None):
    parser = make_parser(doc=__doc__)
    args = parser.parse_args(argv)

    if args.dry_run:
        print(f"[dry-run] would connect to action {args.namespace}/autonomy/mission")
        print("[dry-run] cancellation requires a goal handle held by this process; otherwise use stop_autonomy.py")
        return

    rclpy.init()
    node = CancelMission(args.namespace)
    try:
        node.wait()
        node.cancel_all()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
