#!/usr/bin/env python3
"""Bracket an ExecuteMission call with start_recording → stop_recording.

  ./mission_with_recording.py --mission-uuid <u> --map-uuid <u>
  ONAV_MISSION_ID=<u> ONAV_MAP_ID=<u> ./mission_with_recording.py
  ./mission_with_recording.py --skip-mission     # just start/stop, useful for plumbing checks

Sequence:
  1. /<ns>/log_manager/start_recording (Trigger) — opens an EventLog
  2. /<ns>/autonomy/mission (ExecuteMission) — runs the mission
  3. /<ns>/log_manager/stop_recording (Trigger) — closes the log

The recorded log shows up in the OnAV UI under the operator's log list.
Mission telemetry, fix, video, and any other auto-recorded channels are
captured for the duration of step 1 → step 3.

Some OutdoorNav releases ship `StartRecording` / `StopRecording`
(`clearpath_logger_msgs`) which take a name / custom_fields_json instead
of Trigger. Run `service_inventory.py --grep log_manager` first to check
what type your stack actually exposes — this example assumes Trigger.

If the mission goal is rejected, the script still calls stop_recording
so you don't leave a dangling open log.

Touches:
  service <namespace>/log_manager/start_recording  (std_srvs/Trigger)
  service <namespace>/log_manager/stop_recording   (std_srvs/Trigger)
  action  <namespace>/autonomy/mission             (ExecuteMission, only without --skip-mission)
"""

from __future__ import annotations
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
from clearpath_navigation_msgs.action import ExecuteMission

from common.argparse_base import make_parser
from common.config import map_id as default_map_id, mission_id as default_mission_id
from common.ros_helpers import wait_for_service, wait_for_action, call_service


class MissionWithRecording(Node):
    def __init__(self, namespace: str, mission_uuid: str, map_uuid: str, skip_mission: bool):
        super().__init__("mission_with_recording")
        self.mission_uuid = mission_uuid
        self.map_uuid = map_uuid
        self.skip_mission = skip_mission
        self.start_srv = f"{namespace}/log_manager/start_recording"
        self.stop_srv = f"{namespace}/log_manager/stop_recording"
        self.mission_action = f"{namespace}/autonomy/mission"
        self.start_client = self.create_client(Trigger, self.start_srv)
        self.stop_client = self.create_client(Trigger, self.stop_srv)
        self.mission_client = ActionClient(self, ExecuteMission, self.mission_action) if not skip_mission else None
        self._goal_handle = None

    def wait(self) -> None:
        wait_for_service(self, self.start_client, self.start_srv)
        wait_for_service(self, self.stop_client, self.stop_srv)
        if self.mission_client:
            wait_for_action(self, self.mission_client, self.mission_action)

    def _trigger(self, client, srv_name: str) -> bool:
        resp = call_service(self, client, Trigger.Request())
        ok = bool(resp and resp.success)
        msg = getattr(resp, "message", "") or "(no message)"
        self.get_logger().info(f"{srv_name}: {'OK' if ok else 'FAILED'} — {msg}")
        return ok

    def start_recording(self) -> bool:
        return self._trigger(self.start_client, self.start_srv)

    def stop_recording(self) -> bool:
        return self._trigger(self.stop_client, self.stop_srv)

    def run_mission(self) -> bool:
        if not self.mission_client:
            self.get_logger().info("--skip-mission set; sleeping 5 s to give the recorder data to capture")
            time.sleep(5.0)
            return True
        goal = ExecuteMission.Goal(mission_uuid=self.mission_uuid, map_uuid=self.map_uuid)
        send_future = self.mission_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        self._goal_handle = send_future.result()
        if not self._goal_handle or not self._goal_handle.accepted:
            self.get_logger().error("ExecuteMission goal rejected")
            return False
        self.get_logger().info("mission accepted — blocking on result")
        result_future = self._goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self._goal_handle = None
        status = result_future.result().status if result_future.result() else None
        self.get_logger().info(f"mission ended with status {status} (4=SUCCEEDED)")
        return status == 4

    def cancel_in_flight(self) -> None:
        if self._goal_handle is None:
            return
        self.get_logger().warn("cancelling in-flight mission")
        cancel_future = self._goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future)


def main(argv=None):
    parser = make_parser(doc=__doc__)
    parser.add_argument("--mission-uuid", default=default_mission_id() or None,
                        help="Mission UUID (or $ONAV_MISSION_ID).")
    parser.add_argument("--map-uuid", default=default_map_id() or None,
                        help="Map UUID (or $ONAV_MAP_ID).")
    parser.add_argument("--skip-mission", action="store_true",
                        help="Start, sleep 5 s, stop. Useful to confirm log_manager is wired.")
    args = parser.parse_args(argv)

    if not args.skip_mission and (not args.mission_uuid or not args.map_uuid):
        parser.error("--mission-uuid and --map-uuid required (or set env vars), or pass --skip-mission")

    if args.dry_run:
        ns = args.namespace
        print(f"[dry-run] start_recording  Trigger {ns}/log_manager/start_recording")
        if args.skip_mission:
            print("[dry-run] sleep 5 s")
        else:
            print(f"[dry-run] ExecuteMission     {ns}/autonomy/mission "
                  f"mission={args.mission_uuid} map={args.map_uuid}")
        print(f"[dry-run] stop_recording   Trigger {ns}/log_manager/stop_recording")
        return

    rclpy.init()
    node = MissionWithRecording(args.namespace, args.mission_uuid, args.map_uuid, args.skip_mission)
    started = False
    try:
        node.wait()
        started = node.start_recording()
        if not started:
            return
        try:
            node.run_mission()
        except KeyboardInterrupt:
            node.cancel_in_flight()
    finally:
        if started:
            node.stop_recording()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
