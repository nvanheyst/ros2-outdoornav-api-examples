#!/usr/bin/env python3
"""End-to-end dock demo: add a dock at the current pose, back up, dock, undock.

  ./dock_workflow.py --dock-name smoke_dock --template default_dock
  ./dock_workflow.py --dock-name smoke_dock --backup-distance 2.5 --hold 8

Sequence:
  1. AddDockCurrentPose stores a dock at the robot's current pose.
  2. Back the robot up `--backup-distance` metres at `--backup-velocity`.
  3. Dock action drives the robot onto the just-stored dock (local target
     tracker - needs the dock visible to the sensor).
  4. Hold `--hold` seconds (simulating charging).
  5. Undock action drives the robot off the dock.
  6. RemoveDock cleans up the entry (unless --keep-dock).

The dock template name is robot-specific (e.g. `default_dock`,
`a300_side_dock`). Get the list from your robot config or
`docking/dock_localizer/get_dock_database`.

Run `service_inventory.py --grep dock` first to confirm your stack
exposes these paths - they live under `<ns>/docking/` and
`<ns>/autonomy/`, not `<ns>/mission_manager/`.

Touches:
  service <namespace>/docking/dock_localizer/add_dock_current_pose (AddDockCurrentPose)
  service <namespace>/docking/dock_manager/delete_dock           (RemoveDock, only without --keep-dock)
  topic   <namespace>/ui_teleop/cmd_vel                            (TwistStamped, publish)
  action  <namespace>/autonomy/dock_local                          (Dock)
  action  <namespace>/autonomy/undock                              (Undock)
"""

from __future__ import annotations
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, TwistStamped
from clearpath_dock_msgs.srv import AddDockCurrentPose, RemoveDock
from clearpath_dock_msgs.action import Dock, Undock

from examples.common.argparse_base import make_parser
from examples.common.ros_helpers import wait_for_service, wait_for_action, call_service


PUBLISH_HZ = 20.0


class DockWorkflow(Node):
    def __init__(self, namespace: str, dock_name: str, dock_template: str):
        super().__init__("dock_workflow")
        self.dock_name = dock_name
        self.dock_template = dock_template

        self.add_srv = f"{namespace}/docking/dock_localizer/add_dock_current_pose"
        self.remove_srv = f"{namespace}/docking/dock_manager/delete_dock"
        self.cmd_vel_topic = f"{namespace}/ui_teleop/cmd_vel"
        self.dock_action = f"{namespace}/autonomy/dock_local"
        self.undock_action = f"{namespace}/autonomy/undock"

        self.add_client = self.create_client(AddDockCurrentPose, self.add_srv)
        self.remove_client = self.create_client(RemoveDock, self.remove_srv)
        self.cmd_vel_pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)
        self.dock_client = ActionClient(self, Dock, self.dock_action)
        self.undock_client = ActionClient(self, Undock, self.undock_action)

    def wait(self) -> None:
        wait_for_service(self, self.add_client, self.add_srv)
        wait_for_service(self, self.remove_client, self.remove_srv)
        wait_for_action(self, self.dock_client, self.dock_action)
        wait_for_action(self, self.undock_client, self.undock_action)

    def add_dock(self) -> bool:
        req = AddDockCurrentPose.Request()
        req.dock_name = self.dock_name
        req.dock_template = self.dock_template
        resp = call_service(self, self.add_client, req)
        ok = bool(resp and resp.success)
        msg = getattr(resp, "message", "") or "(no message)"
        self.get_logger().info(f"AddDockCurrentPose({self.dock_name!r}): {'OK' if ok else 'FAILED'} - {msg}")
        return ok

    def remove_dock(self) -> bool:
        req = RemoveDock.Request()
        req.name = self.dock_name
        resp = call_service(self, self.remove_client, req)
        ok = bool(resp and resp.success)
        self.get_logger().info(f"RemoveDock({self.dock_name!r}): {'OK' if ok else 'FAILED'}")
        return ok

    def back_up(self, distance: float, velocity: float) -> None:
        duration = distance / velocity
        period = 1.0 / PUBLISH_HZ
        self.get_logger().info(f"backing up {distance:.2f} m at {velocity:.2f} m/s")
        end = time.time() + duration
        while time.time() < end:
            ts = TwistStamped()
            ts.header.stamp = self.get_clock().now().to_msg()
            ts.twist = Twist()
            ts.twist.linear.x = -abs(velocity)
            self.cmd_vel_pub.publish(ts)
            time.sleep(period)
        # zero
        for _ in range(5):
            ts = TwistStamped()
            ts.header.stamp = self.get_clock().now().to_msg()
            ts.twist = Twist()
            self.cmd_vel_pub.publish(ts)
            time.sleep(period)

    def _run_action(self, client, goal, label: str) -> bool:
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if not handle or not handle.accepted:
            self.get_logger().error(f"{label}: goal rejected")
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        status = result.status if result else None
        success = bool(getattr(result.result, "success", False)) if result else False
        msg = getattr(result.result, "message", "") if result else ""
        self.get_logger().info(f"{label}: status={status} success={success} msg={msg!r}")
        return success

    def dock(self) -> bool:
        return self._run_action(self.dock_client, Dock.Goal(dock_name=self.dock_name), "Dock")

    def undock(self) -> bool:
        return self._run_action(self.undock_client, Undock.Goal(dock_name=self.dock_name), "Undock")


def main(argv=None):
    parser = make_parser(doc=__doc__)
    parser.add_argument("--dock-name", default="smoke_dock", help="Dock entry name.")
    parser.add_argument("--template", default="default_dock",
                        help="Dock template (robot-specific; e.g. 'default_dock', 'a300_side_dock').")
    parser.add_argument("--backup-distance", type=float, default=2.0,
                        help="Metres to back up after adding the dock (default 2.0).")
    parser.add_argument("--backup-velocity", type=float, default=0.2,
                        help="Backup velocity m/s (default 0.2).")
    parser.add_argument("--hold", type=float, default=5.0,
                        help="Seconds to hold docked before undocking (default 5).")
    parser.add_argument("--keep-dock", action="store_true",
                        help="Don't remove the dock entry at the end.")
    args = parser.parse_args(argv)

    if args.dry_run:
        ns = args.namespace
        print(f"[dry-run] AddDockCurrentPose @ {ns}/docking/dock_localizer/add_dock_current_pose")
        print(f"[dry-run]   dock_name={args.dock_name!r}  template={args.template!r}")
        print(f"[dry-run] back up {args.backup_distance} m at {args.backup_velocity} m/s")
        print(f"[dry-run] Dock action {ns}/autonomy/dock_local")
        print(f"[dry-run] hold {args.hold} s")
        print(f"[dry-run] Undock action {ns}/autonomy/undock")
        if not args.keep_dock:
            print(f"[dry-run] RemoveDock {ns}/docking/dock_manager/delete_dock")
        return

    rclpy.init()
    node = DockWorkflow(args.namespace, args.dock_name, args.template)
    try:
        node.wait()
        if not node.add_dock():
            return
        node.back_up(args.backup_distance, args.backup_velocity)
        if node.dock():
            node.get_logger().info(f"holding docked for {args.hold:.1f} s")
            time.sleep(args.hold)
            node.undock()
        if not args.keep_dock:
            node.remove_dock()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
