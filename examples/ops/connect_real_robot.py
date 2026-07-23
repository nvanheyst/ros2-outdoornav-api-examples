#!/usr/bin/env python3
"""Connect to a real OutdoorNav robot and confirm the link (read-only).

Prints the DDS/RMW transport in effect, waits for the robot's graph to appear,
auto-detects the robot namespace, and shows a couple of live signals so you know
you're actually talking to the robot.

The real robot uses Fast DDS + a discovery server (super-client); the sim uses
CycloneDDS multicast. Pick a transport profile instead of memorising env vars:

    docker compose --env-file real-amp.env run --rm dev \
        python3 examples/ops/connect_real_robot.py

or export the vars yourself (what real-amp.env sets):

    RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=0 \
    ROS_DISCOVERY_SERVER=127.0.0.1:11811 ROS_SUPER_CLIENT=TRUE \
    ./connect_real_robot.py

Run it ON the robot - the discovery server is on the robot's loopback. Off-robot
only works if the graph is bridged to you (e.g. a Zenoh router). See docker/real-amp.env.

Touches: graph introspection only (read-only).
"""

from __future__ import annotations
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import rclpy
from rclpy.node import Node

from examples.common.argparse_base import make_parser
from examples.common import config

_ENV_KEYS = [
    "RMW_IMPLEMENTATION", "ROS_DOMAIN_ID", "ROS_DISCOVERY_SERVER",
    "ROS_SUPER_CLIENT", "CYCLONEDDS_URI",
]


def main(argv=None):
    parser = make_parser(doc=__doc__, add_dry_run=False)
    parser.add_argument("--timeout", type=float, default=10.0,
                        help="Seconds to wait for the robot graph (default 10).")
    args = parser.parse_args(argv)

    print("DDS/RMW transport:")
    for k in _ENV_KEYS:
        print(f"  {k}={os.environ.get(k, '')!r}")

    rclpy.init()
    node = Node("connect_real_robot")
    node.get_logger().info("waiting for the robot graph to appear …")
    deadline = node.get_clock().now().nanoseconds * 1e-9 + args.timeout
    ns = ""
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
            try:
                ns = config.resolve_namespace(node, explicit=args.namespace)
            except RuntimeError:
                ns = ""
            if ns:
                break
            if node.get_clock().now().nanoseconds * 1e-9 > deadline:
                break

        if not ns:
            node.get_logger().error(
                "no robot namespace discovered - check the transport/profile, and that "
                "you're on the robot (or bridged). See docker/real-amp.env.")
            return 1

        # Namespace resolved (possibly from env var) before discovery fully populated.
        # Keep spinning until the anchor topics appear or the deadline expires.
        node.get_logger().info(f"namespace: {ns} — waiting for graph to populate …")
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
            topics_now = {t for t, _ in node.get_topic_names_and_types()}
            if f"{ns}/localization/fix" in topics_now or f"{ns}/platform/bms/state" in topics_now:
                break
            if node.get_clock().now().nanoseconds * 1e-9 > deadline:
                break

        topics = {t for t, _ in node.get_topic_names_and_types()}
        n_nodes = len(node.get_node_names_and_namespaces())
        have_fix = f"{ns}/localization/fix" in topics
        have_tf = f"{ns}/tf" in topics
        print(f"\n✓ connected. robot namespace: {ns}")
        print(f"  nodes discovered : {n_nodes}")
        print(f"  topics discovered: {len(topics)}")
        print(f"  localization/fix : {'present' if have_fix else 'MISSING'}")
        print(f"  tf               : {'present' if have_tf else 'MISSING'}")
        print(f"\nUse this namespace with other examples:  export ONAV_NAMESPACE={ns}")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main() or 0)
