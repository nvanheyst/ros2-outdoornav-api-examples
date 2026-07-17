#!/usr/bin/env python3
"""Hard-stop the autonomy stack via /autonomy/stop (Trigger).

  ./stop_autonomy.py                # call /autonomy/stop once
  ./stop_autonomy.py --dry-run      # print the call without executing

This calls the Trigger service that aborts whatever the autonomy stack is
doing - mission, goto, replan. Robot brakes; mission action returns aborted.

Touches: service <namespace>/autonomy/stop (std_srvs/Trigger).
"""

from __future__ import annotations
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from examples.common.argparse_base import make_parser
from examples.common.ros_helpers import wait_for_service, call_service


class StopAutonomy(Node):
    def __init__(self, namespace: str):
        super().__init__("stop_autonomy")
        self.srv = f"{namespace}/autonomy/stop"
        self.client = self.create_client(Trigger, self.srv)

    def wait(self) -> None:
        wait_for_service(self, self.client, self.srv)

    def stop(self) -> bool:
        resp = call_service(self, self.client, Trigger.Request())
        ok = bool(resp and resp.success)
        self.get_logger().info(f"{self.srv}: {'OK' if ok else 'FAILED'}  {getattr(resp, 'message', '')}")
        return ok


def main(argv=None):
    parser = make_parser(doc=__doc__)
    args = parser.parse_args(argv)

    if args.dry_run:
        print(f"[dry-run] would call {args.namespace}/autonomy/stop (Trigger)")
        return

    rclpy.init()
    node = StopAutonomy(args.namespace)
    try:
        node.wait()
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
