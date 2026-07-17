#!/usr/bin/env python3
"""Cancel an in-flight action goal cleanly when the operator hits Ctrl-C.

Bare `rclpy.spin*` is interruptible, but if you exit without calling
`goal_handle.cancel_goal_async()` the goal keeps running on the server and
the robot keeps driving. This module wraps that pattern.

Two ways to plug it in:

  spin_until_done_or_cancel(node, goal_handle)
      Drop-in for `rclpy.spin_until_future_complete` on a goal's result
      future. On Ctrl-C, cancels and waits briefly for the cancel ack, then
      re-raises so your finally block still runs.

  with CancelOnShutdown(node, get_goal_handle):
      rclpy.spin(node)
      Context-manager form. Wrap any spin loop; on Ctrl-C it pulls the
      current goal handle from the callable and cancels it.

Both call `cancel_goal_blocking()` under the hood - exposed too if you
need it.

Demo at the bottom runs an ExecuteMission until Ctrl-C, then cancels.

Touches: any action's <handle>.cancel_goal_async (action_msgs/CancelGoal)
         and <handle>.get_result_async.

  ONAV_MAP_ID=<u> ONAV_MISSION_ID=<u> ./graceful_shutdown.py
"""

from __future__ import annotations
import sys
from pathlib import Path
from typing import Callable, Optional

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from clearpath_navigation_msgs.action import ExecuteMission

from examples.common.argparse_base import make_parser
from examples.common.config import map_id as default_map_id, mission_id as default_mission_id
from examples.common.ros_helpers import wait_for_action


def cancel_goal_blocking(node: Node, goal_handle, timeout_sec: float = 5.0) -> Optional[int]:
    """Send cancel + spin until the result future returns. Returns final status int."""
    cancel_future = goal_handle.cancel_goal_async()
    rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=timeout_sec)
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=timeout_sec)
    result = result_future.result()
    return result.status if result else None


def spin_until_done_or_cancel(node: Node, goal_handle,
                              timeout_sec: Optional[float] = None) -> Optional[int]:
    """Spin until the action result arrives. On Ctrl-C, cancel cleanly then re-raise."""
    result_future = goal_handle.get_result_async()
    try:
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=timeout_sec)
    except KeyboardInterrupt:
        node.get_logger().warn("Ctrl-C - cancelling in-flight goal")
        cancel_goal_blocking(node, goal_handle)
        raise
    result = result_future.result()
    return result.status if result else None


class CancelOnShutdown:
    """Cancel the active goal on KeyboardInterrupt. Goal handle pulled lazily."""

    def __init__(self, node: Node, get_goal_handle: Callable[[], object]):
        self.node = node
        self._get = get_goal_handle

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        if exc_type is KeyboardInterrupt:
            handle = self._get()
            if handle is not None:
                self.node.get_logger().warn("Ctrl-C - cancelling in-flight goal")
                cancel_goal_blocking(self.node, handle)
        return False


class DemoCancelOnInterrupt(Node):
    def __init__(self, namespace: str, mission_uuid: str, map_uuid: str):
        super().__init__("graceful_shutdown_demo")
        self.mission_uuid = mission_uuid
        self.map_uuid = map_uuid
        self.mission_action = f"{namespace}/autonomy/mission"
        self.client = ActionClient(self, ExecuteMission, self.mission_action)
        self._goal_handle = None

    def goal_handle(self):
        return self._goal_handle

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
        self.get_logger().info("mission accepted - Ctrl-C will cancel cleanly")
        return True


def main(argv=None):
    parser = make_parser(doc=__doc__)
    parser.add_argument("--mission-uuid", default=default_mission_id() or None,
                        help="Mission UUID (or $ONAV_MISSION_ID).")
    parser.add_argument("--map-uuid", default=default_map_id() or None,
                        help="Map UUID (or $ONAV_MAP_ID).")
    args = parser.parse_args(argv)

    if not args.mission_uuid or not args.map_uuid:
        parser.error("--mission-uuid and --map-uuid required (or set $ONAV_MISSION_ID and $ONAV_MAP_ID)")

    if args.dry_run:
        print(f"[dry-run] would run ExecuteMission via {args.namespace}/autonomy/mission")
        print(f"[dry-run] would cancel cleanly on Ctrl-C")
        return

    rclpy.init()
    node = DemoCancelOnInterrupt(args.namespace, args.mission_uuid, args.map_uuid)
    try:
        node.wait()
        if not node.start():
            return
        with CancelOnShutdown(node, node.goal_handle):
            spin_until_done_or_cancel(node, node.goal_handle())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
