#!/usr/bin/env python3
"""Print the robot's latest GPS fix and exit.

  ./where_am_i.py
  ./where_am_i.py --timeout 15

Touches: topic <namespace>/localization/fix (NavSatFix, subscribe).
"""

from __future__ import annotations
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

from examples.common.argparse_base import make_parser


class WhereAmI(Node):
    def __init__(self, namespace: str):
        super().__init__("where_am_i")
        self.topic = f"{namespace}/localization/fix"
        self.fix: NavSatFix | None = None
        self.create_subscription(NavSatFix, self.topic, self._cb, 10)

    def _cb(self, msg: NavSatFix) -> None:
        if not math.isnan(msg.latitude) and not math.isnan(msg.longitude):
            self.fix = msg


def main(argv=None):
    parser = make_parser(doc=__doc__, add_dry_run=False)
    parser.add_argument("--timeout", type=float, default=10.0,
                        help="Seconds to wait for a fix (default 10).")
    args = parser.parse_args(argv)

    rclpy.init()
    node = WhereAmI(args.namespace)
    node.get_logger().info(f"waiting for fix on {node.topic} (timeout {args.timeout:.0f} s)...")
    deadline = node.get_clock().now().nanoseconds * 1e-9 + args.timeout
    try:
        while rclpy.ok() and node.fix is None:
            rclpy.spin_once(node, timeout_sec=0.5)
            if node.get_clock().now().nanoseconds * 1e-9 > deadline:
                node.get_logger().error("timed out waiting for fix")
                return 1
        f = node.fix
        node.get_logger().info(
            f"lat={f.latitude:.6f} lon={f.longitude:.6f} alt={f.altitude:.2f} "
            f"status={f.status.status} service={f.status.service}"
        )
        print(f"{f.latitude:.6f},{f.longitude:.6f},{f.altitude:.2f}")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main() or 0)
