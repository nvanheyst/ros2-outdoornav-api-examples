#!/usr/bin/env python3
"""Pause a running mission to let the operator drive, then hand back to autonomy.

  ./pause_and_teleop.py --poi <uuid> --map <uuid>
  ./pause_and_teleop.py --poi <uuid> --map <uuid> --after 15

Sequence:
  1. Start GoToPOI (non-blocking).
  2. Wait --after seconds for the robot to start moving (default 10).
  3. Pause via autonomy/pause.
  4. Print a prompt — operator drives manually (joystick/gamepad is active while paused).
  5. Operator presses Enter to hand back to autonomy.
  6. Resume via autonomy/resume.
  7. Block until GoToPOI completes.

CAUTION: pause/resume paths differ across OutdoorNav releases. This example uses
`<ns>/autonomy/pause` (SetBool) - the OutdoorNav 2.3 default. Run pause_resume.py
with --variant to confirm your stack's interface before running this.

Touches:
  action  <namespace>/autonomy/goto_poi  (ExecuteGoToPOI)
  service <namespace>/autonomy/pause     (SetBool)
  service <namespace>/autonomy/resume    (SetBool)
  service <namespace>/autonomy/stop      (Trigger, shutdown-time)
"""

from __future__ import annotations
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.task import Future
from std_srvs.srv import SetBool, Trigger

from clearpath_navigation_msgs.action import ExecuteGoToPOI

from examples.common.argparse_base import make_parser
from examples.common.config import map_id as default_map_id, poi_id as default_poi_id
from examples.common.ros_helpers import wait_for_service, wait_for_action


class PauseAndTeleop(Node):
    def __init__(self, namespace: str, poi_uuid: str, map_uuid: str):
        super().__init__("pause_and_teleop")
        self.poi_uuid = poi_uuid
        self.map_uuid = map_uuid

        self.poi_action = f"{namespace}/autonomy/goto_poi"
        self.pause_srv = f"{namespace}/autonomy/pause"
        self.resume_srv = f"{namespace}/autonomy/resume"
        self.stop_srv = f"{namespace}/autonomy/stop"

        self.poi_client = ActionClient(self, ExecuteGoToPOI, self.poi_action)
        self.pause_client = self.create_client(SetBool, self.pause_srv)
        self.resume_client = self.create_client(SetBool, self.resume_srv)
        self.stop_client = self.create_client(Trigger, self.stop_srv)

    def wait(self) -> None:
        wait_for_action(self, self.poi_client, self.poi_action)
        wait_for_service(self, self.pause_client, self.pause_srv)
        wait_for_service(self, self.resume_client, self.resume_srv)
        wait_for_service(self, self.stop_client, self.stop_srv)

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
        self.get_logger().info(f"{label}: {'OK' if ok else 'FAILED'}")
        return ok

    def call_trigger(self, client, label: str) -> bool:
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        ok = bool(future.result() and future.result().success)
        self.get_logger().info(f"{label}: {'OK' if ok else 'FAILED'}")
        return ok

    def execute(self, after: float) -> None:
        self.get_logger().info("starting GoToPOI")
        poi_future = self.send_goto_poi()

        if after > 0:
            self.get_logger().info(f"letting the robot move for {after:.0f} s …")
            time.sleep(after)

        if not self.call_set_bool(self.pause_client, True, "PAUSE"):
            return

        print("\nAutonomy paused. Drive the robot manually with your joystick or gamepad.")
        input("Press Enter when you are done to hand back to autonomy … ")

        if not self.call_set_bool(self.resume_client, True, "RESUME"):
            return

        self.get_logger().info("resuming — waiting for GoToPOI to complete")
        ok = self.wait_for_goto_completion(poi_future)
        self.get_logger().info(f"GoToPOI {'succeeded' if ok else 'did not succeed'}")


def main(argv=None):
    parser = make_parser(doc=__doc__)
    parser.add_argument("--poi", default=default_poi_id() or None,
                        help="POI UUID to drive to (or $ONAV_POI_ID).")
    parser.add_argument("--map", default=default_map_id() or None,
                        help="Map UUID (or $ONAV_MAP_ID).")
    parser.add_argument("--after", type=float, default=10.0,
                        help="Seconds to let the mission run before pausing (default 10).")
    args = parser.parse_args(argv)

    if not args.poi or not args.map:
        parser.error("--poi and --map required (or set $ONAV_POI_ID and $ONAV_MAP_ID)")

    if args.dry_run:
        print(f"[dry-run] GoToPOI {args.poi} on map {args.map}")
        print(f"[dry-run] pause after {args.after:.0f} s, wait for Enter, resume")
        return

    rclpy.init()
    node = PauseAndTeleop(args.namespace, args.poi, args.map)
    try:
        node.wait()
        node.execute(args.after)
    finally:
        node.call_trigger(node.stop_client, "autonomy stop (shutdown)")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
