#!/usr/bin/env python3
"""Open-loop forward drive via TwistStamped on ui_teleop/cmd_vel.

  ./drive_robot_forward.py                     # 2.0 m at 0.3 m/s
  ./drive_robot_forward.py --distance 5 --velocity 0.4
  ONAV_NAMESPACE=/a300_00003 ./drive_robot_forward.py

Bypasses autonomy. Distance is "drive for this long," not "drive exactly N
metres" - no odometry feedback. Don't run while a mission is active.

Touches: topic <namespace>/ui_teleop/cmd_vel (TwistStamped, publish only).
"""

from __future__ import annotations
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

from examples.common.argparse_base import make_parser


PUBLISH_HZ = 20.0


class DriveForward(Node):
    def __init__(self, namespace: str, distance_m: float, velocity_m_s: float):
        super().__init__("drive_robot_forward")
        self.distance = distance_m
        self.velocity = velocity_m_s
        self.topic = f"{namespace}/ui_teleop/cmd_vel"
        self.publisher = self.create_publisher(TwistStamped, self.topic, 10)

    def publish_twist(self, linear_x: float) -> None:
        ts = TwistStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.twist = Twist()
        ts.twist.linear.x = linear_x
        self.publisher.publish(ts)

    def drive(self) -> None:
        duration = self.distance / self.velocity
        period = 1.0 / PUBLISH_HZ
        self.get_logger().info(
            f"publishing TwistStamped on {self.topic}: "
            f"{self.distance:.2f} m at {self.velocity:.2f} m/s (~{duration:.1f} s)"
        )
        end = time.time() + duration
        while time.time() < end:
            self.publish_twist(self.velocity)
            time.sleep(period)
        for _ in range(5):
            self.publish_twist(0.0)
            time.sleep(period)
        self.get_logger().info("stopped")


def main(argv=None):
    parser = make_parser(doc=__doc__, add_dry_run=True)
    parser.add_argument("--distance", type=float, default=2.0, help="metres (default 2.0)")
    parser.add_argument("--velocity", type=float, default=0.3, help="m/s (default 0.3)")
    args = parser.parse_args(argv)

    if args.dry_run:
        print(f"[dry-run] would publish TwistStamped on {args.namespace}/ui_teleop/cmd_vel")
        print(f"[dry-run] distance={args.distance} m, velocity={args.velocity} m/s")
        return

    rclpy.init()
    node = DriveForward(args.namespace, args.distance, args.velocity)
    try:
        node.drive()
    except KeyboardInterrupt:
        for _ in range(5):
            node.publish_twist(0.0)
            time.sleep(0.05)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
