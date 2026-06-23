#!/usr/bin/env python3
"""List the OnAV services live on the current ROS graph, grouped by namespace.

Useful first step in any debugging session: did the stack actually publish
the service path you're about to call? Run this before pause_resume.py or
the dock examples.

  ./service_inventory.py
  ./service_inventory.py --grep pause          # only services matching 'pause'

Touches: ROS 2 graph API (Node.get_service_names_and_types).
"""

from __future__ import annotations
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import rclpy
from rclpy.node import Node

from common.argparse_base import make_parser


def main(argv=None):
    parser = make_parser(doc=__doc__, add_dry_run=False)
    parser.add_argument("--grep", default=None, help="Substring filter on service path.")
    args = parser.parse_args(argv)

    rclpy.init()
    node = Node("service_inventory")
    try:
        # Let the discovery cache settle before snapshotting.
        for _ in range(4):
            rclpy.spin_once(node, timeout_sec=0.5)
            time.sleep(0.25)
        services = node.get_service_names_and_types()
        filt = args.grep
        for name, types in sorted(services):
            if filt and filt not in name:
                continue
            type_str = ",".join(types)
            print(f"{name}    [{type_str}]")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main() or 0)
