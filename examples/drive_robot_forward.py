"""
Open-loop forward drive via TwistStamped on ui_teleop/cmd_vel.

  python drive_robot_forward.py              # 2.0 m at 0.3 m/s
  python drive_robot_forward.py 5.0 0.4

Bypasses autonomy. Distance is "drive for this long," not "drive exactly
N metres" — no odometry feedback. Don't run while a mission is active.
"""

import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


ROBOT_NAMESPACE = '/a300_00003'
CMD_VEL_TOPIC = f'{ROBOT_NAMESPACE}/ui_teleop/cmd_vel'

PUBLISH_HZ = 20.0
DEFAULT_DISTANCE_M = 2.0
DEFAULT_VELOCITY_M_S = 0.3


class DriveForward(Node):
    def __init__(self, distance_m: float, velocity_m_s: float):
        super().__init__('drive_robot_forward')
        self.distance = distance_m
        self.velocity = velocity_m_s
        self.publisher = self.create_publisher(TwistStamped, CMD_VEL_TOPIC, 10)

    def publish_twist(self, linear_x: float):
        ts = TwistStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.twist = Twist()
        ts.twist.linear.x = linear_x
        self.publisher.publish(ts)

    def drive(self):
        duration = self.distance / self.velocity
        period = 1.0 / PUBLISH_HZ
        self.get_logger().info(
            f'Driving forward {self.distance:.2f} m at {self.velocity:.2f} m/s '
            f'(~{duration:.1f} s).'
        )

        end = time.time() + duration
        while time.time() < end:
            self.publish_twist(self.velocity)
            time.sleep(period)

        # Stop hard.
        for _ in range(5):
            self.publish_twist(0.0)
            time.sleep(period)
        self.get_logger().info('Stopped.')


def main(args=None):
    distance = float(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_DISTANCE_M
    velocity = float(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_VELOCITY_M_S

    rclpy.init(args=args)
    node = DriveForward(distance, velocity)
    try:
        node.drive()
    except KeyboardInterrupt:
        for _ in range(5):
            node.publish_twist(0.0)
            time.sleep(0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
