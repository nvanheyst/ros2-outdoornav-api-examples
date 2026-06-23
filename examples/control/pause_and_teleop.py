#!/usr/bin/env python3
"""Mid-mission pause + teleop + resume + completion.

Showcases the full pause/resume/stop interface in one flow:
  1. Start GoToPOI (non-blocking).
  2. Sleep 10 s to let the robot start moving.
  3. Pause via control_selection/pause.
  4. Drive 180° turn + 1 m forward via ui_teleop/cmd_vel.
  5. Resume via control_selection/resume.
  6. Block until GoToPOI completes.

  ./pause_and_teleop.py --poi <uuid> --map <uuid>
  ONAV_POI_ID=<uuid> ONAV_MAP_ID=<uuid> ./pause_and_teleop.py

CAUTION: pause/resume service path differs between OutdoorNav releases.
This example uses `<ns>/control_selection/pause` (SetBool). If your stack
exposes only `<ns>/autonomy/pause` (Trigger), see pause_resume.py for that
variant. Run `ros2 service list | grep pause` against your live stack to
confirm which you have.

Touches:
  action  <namespace>/autonomy/goto_poi            (ExecuteGoToPOI)
  topic   <namespace>/ui_teleop/cmd_vel            (TwistStamped, publish)
  service <namespace>/control_selection/pause      (SetBool)
  service <namespace>/control_selection/resume     (SetBool)
  service <namespace>/autonomy/stop                (Trigger, shutdown-time)
"""

from __future__ import annotations
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.task import Future
from std_srvs.srv import SetBool, Trigger
from geometry_msgs.msg import Twist, TwistStamped
from clearpath_navigation_msgs.action import ExecuteGoToPOI

from common.argparse_base import make_parser
from common.config import map_id as default_map_id, poi_id as default_poi_id
from common.ros_helpers import wait_for_service, wait_for_action


TURN_ANGLE_RAD = math.pi
TURN_VEL_RAD_S = 0.5
DRIVE_DISTANCE_M = 1.0
DRIVE_VEL_M_S = 0.3
PUBLISH_RATE_S = 0.05


class PauseAndTeleop(Node):
    def __init__(self, namespace: str, poi_uuid: str, map_uuid: str):
        super().__init__("pause_and_teleop")
        self.poi_uuid = poi_uuid
        self.map_uuid = map_uuid

        self.poi_action = f"{namespace}/autonomy/goto_poi"
        self.cmd_vel_topic = f"{namespace}/ui_teleop/cmd_vel"
        self.pause_srv = f"{namespace}/control_selection/pause"
        self.resume_srv = f"{namespace}/control_selection/resume"
        self.stop_srv = f"{namespace}/autonomy/stop"

        self.poi_client = ActionClient(self, ExecuteGoToPOI, self.poi_action)
        self.cmd_vel_pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)
        self.pause_client = self.create_client(SetBool, self.pause_srv)
        self.resume_client = self.create_client(SetBool, self.resume_srv)
        self.stop_client = self.create_client(Trigger, self.stop_srv)

    def wait(self) -> None:
        wait_for_action(self, self.poi_client, self.poi_action)
        wait_for_service(self, self.pause_client, self.pause_srv)
        wait_for_service(self, self.resume_client, self.resume_srv)
        wait_for_service(self, self.stop_client, self.stop_srv)

    def publish_twist(self, linear_x: float, angular_z: float) -> None:
        ts = TwistStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.twist = Twist()
        ts.twist.linear.x = linear_x
        ts.twist.angular.z = angular_z
        self.cmd_vel_pub.publish(ts)

    def teleop_for(self, linear: float, angular: float, duration: float, label: str) -> None:
        self.get_logger().info(f"{label} (linear={linear}, angular={angular}, duration={duration:.1f} s)")
        end = time.time() + duration
        while time.time() < end:
            self.publish_twist(linear, angular)
            time.sleep(PUBLISH_RATE_S)
        self.publish_twist(0.0, 0.0)

    def send_goto_poi(self) -> Future:
        goal = ExecuteGoToPOI.Goal(poi_uuid=self.poi_uuid, map_uuid=self.map_uuid)
        return self.poi_client.send_goal_async(goal)

    def wait_for_goto_completion(self, send_future: Future) -> bool:
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("GoToPOI goal rejected")
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        status = result_future.result().status if result_future.result() else None
        return status == 4

    def call_set_bool(self, client, data: bool, label: str) -> bool:
        future = client.call_async(SetBool.Request(data=data))
        rclpy.spin_until_future_complete(self, future)
        ok = bool(future.result() and future.result().success)
        self.get_logger().info(f"  {label}: {'OK' if ok else 'FAILED'}")
        return ok

    def call_trigger(self, client, label: str) -> bool:
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        ok = bool(future.result() and future.result().success)
        self.get_logger().info(f"  {label}: {'OK' if ok else 'FAILED'}")
        return ok

    def execute(self) -> None:
        self.get_logger().info("starting mission")
        poi_future = self.send_goto_poi()
        time.sleep(10)
        self.get_logger().info("pausing to interrupt GoToPOI")
        if not self.call_set_bool(self.pause_client, True, "PAUSE"):
            return
        self.teleop_for(0.0, TURN_VEL_RAD_S, TURN_ANGLE_RAD / TURN_VEL_RAD_S, "180° turn")
        self.teleop_for(DRIVE_VEL_M_S, 0.0, DRIVE_DISTANCE_M / DRIVE_VEL_M_S, "1 m drive")
        if not self.call_set_bool(self.resume_client, True, "RESUME"):
            return
        self.wait_for_goto_completion(poi_future)


def main(argv=None):
    parser = make_parser(doc=__doc__)
    parser.add_argument("--poi", default=default_poi_id() or None,
                        help="POI UUID to drive to (or $ONAV_POI_ID).")
    parser.add_argument("--map", default=default_map_id() or None,
                        help="Map UUID (or $ONAV_MAP_ID).")
    args = parser.parse_args(argv)

    if not args.poi or not args.map:
        parser.error("--poi and --map required (or set $ONAV_POI_ID and $ONAV_MAP_ID)")

    if args.dry_run:
        print(f"[dry-run] would drive to POI {args.poi} on map {args.map}, then pause+teleop+resume")
        print(f"[dry-run] services touched: control_selection/{{pause,resume}}, autonomy/stop")
        return

    rclpy.init()
    node = PauseAndTeleop(args.namespace, args.poi, args.map)
    try:
        node.wait()
        node.execute()
    finally:
        node.call_trigger(node.stop_client, "autonomy stop (shutdown)")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
