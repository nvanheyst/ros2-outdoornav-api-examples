#!/usr/bin/env python3
"""Run a mission while the OutdoorNav logger and front camera are both recording.

  ./mission_with_recording.py
  ./mission_with_recording.py --camera oak_d_pro_w_rear   # use rear camera instead
  ./mission_with_recording.py --skip-mission              # test recording plumbing only
  ./mission_with_recording.py --mission-uuid <u> --map-uuid <u>   # skip menus

Sequence:
  1. log_manager/start_recording  (Trigger) — opens an EventLog in the UI
  2. <camera>/start_recording     (StartRecording action) — begins video to disk
  3. autonomy/mission             (ExecuteMission) — runs the mission
  4. <camera>/stop_recording      (StopRecording action) — finalises the video file
  5. log_manager/stop_recording   (Trigger) — closes the log

Both recordings stop in the finally block, so Ctrl-C or a rejected mission goal
still closes them cleanly.

Touches:
  service <namespace>/log_manager/start_recording         (std_srvs/Trigger)
  service <namespace>/log_manager/stop_recording          (std_srvs/Trigger)
  action  <namespace>/<camera>/start_recording            (video_recorder_msgs/StartRecording)
  action  <namespace>/<camera>/stop_recording             (video_recorder_msgs/StopRecording)
  action  <namespace>/autonomy/mission                    (ExecuteMission)
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
from video_recorder_msgs.action import StartRecording, StopRecording

from clearpath_mission_manager_msgs.srv import GetAllMaps, GetAllNetworkMissions

from examples.common.argparse_base import make_parser
from examples.common.config import map_id as default_map_id, mission_id as default_mission_id
from examples.common.ros_helpers import wait_for_service, wait_for_action, call_service
from examples.common.onav import select_map, select_mission

DEFAULT_CAMERA = "oak_d_pro_w_front"


class MissionWithRecording(Node):
    def __init__(self, namespace: str, camera: str, skip_mission: bool):
        super().__init__("mission_with_recording")
        self.mission_uuid = ""
        self.map_uuid = ""
        self.skip_mission = skip_mission
        self.log_start_srv = f"{namespace}/log_manager/start_recording"
        self.log_stop_srv = f"{namespace}/log_manager/stop_recording"
        self.vid_start_action = f"{namespace}/{camera}/start_recording"
        self.vid_stop_action = f"{namespace}/{camera}/stop_recording"
        self.mission_action = f"{namespace}/autonomy/mission"
        self.maps_srv = f"{namespace}/mission_manager/get_all_maps"
        self.missions_srv = f"{namespace}/mission_manager/get_all_network_missions"

        self.log_start_client = self.create_client(Trigger, self.log_start_srv)
        self.log_stop_client = self.create_client(Trigger, self.log_stop_srv)
        self.vid_start_client = ActionClient(self, StartRecording, self.vid_start_action)
        self.vid_stop_client = ActionClient(self, StopRecording, self.vid_stop_action)
        self.mission_client = ActionClient(self, ExecuteMission, self.mission_action) if not skip_mission else None
        self.maps_client = self.create_client(GetAllMaps, self.maps_srv) if not skip_mission else None
        self.missions_client = self.create_client(GetAllNetworkMissions, self.missions_srv) if not skip_mission else None
        self._mission_handle = None
        self._video_started = False

    def wait(self) -> None:
        wait_for_service(self, self.log_start_client, self.log_start_srv)
        wait_for_service(self, self.log_stop_client, self.log_stop_srv)
        wait_for_action(self, self.vid_start_client, self.vid_start_action)
        wait_for_action(self, self.vid_stop_client, self.vid_stop_action)
        if self.mission_client:
            wait_for_action(self, self.mission_client, self.mission_action)
        if self.maps_client:
            wait_for_service(self, self.maps_client, self.maps_srv)
        if self.missions_client:
            wait_for_service(self, self.missions_client, self.missions_srv)

    def _trigger(self, client, srv_name: str) -> bool:
        resp = call_service(self, client, Trigger.Request())
        ok = bool(resp and resp.success)
        msg = getattr(resp, "message", "") or "(no message)"
        self.get_logger().info(f"{srv_name}: {'OK' if ok else 'FAILED'} — {msg}")
        return ok

    def start_log(self) -> bool:
        return self._trigger(self.log_start_client, self.log_start_srv)

    def stop_log(self) -> None:
        self._trigger(self.log_stop_client, self.log_stop_srv)

    def start_video(self) -> bool:
        goal = StartRecording.Goal(filename="", duration=0)
        f = self.vid_start_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, f)
        handle = f.result()
        if not handle or not handle.accepted:
            self.get_logger().error(f"video start rejected on {self.vid_start_action}")
            return False
        self.get_logger().info(f"video recording started")
        self._video_started = True
        return True

    def stop_video(self) -> None:
        if not self._video_started:
            return
        goal = StopRecording.Goal(arg=True)
        f = self.vid_stop_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, f)
        handle = f.result()
        if not handle or not handle.accepted:
            self.get_logger().warn("video stop goal rejected")
            return
        result_f = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_f)
        result = result_f.result()
        if result and result.result.success:
            path = result.result.path
            size_mb = result.result.size / 1_000_000
            dur_s = result.result.duration
            self.get_logger().info(f"video saved: {path} ({size_mb:.1f} MB, {dur_s} s)")
        self._video_started = False

    def run_mission(self) -> bool:
        if not self.mission_client:
            self.get_logger().info("--skip-mission: sleeping 5 s")
            time.sleep(5.0)
            return True
        goal = ExecuteMission.Goal(mission_uuid=self.mission_uuid, map_uuid=self.map_uuid)
        f = self.mission_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, f)
        self._mission_handle = f.result()
        if not self._mission_handle or not self._mission_handle.accepted:
            self.get_logger().error("ExecuteMission goal rejected")
            return False
        self.get_logger().info("mission accepted — blocking on result")
        result_f = self._mission_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_f)
        self._mission_handle = None
        status = result_f.result().status if result_f.result() else None
        self.get_logger().info(f"mission ended with status {status} (4=SUCCEEDED)")
        return status == 4

    def cancel_in_flight(self) -> None:
        if self._mission_handle is None:
            return
        self.get_logger().warn("cancelling in-flight mission")
        f = self._mission_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, f)


def main(argv=None):
    parser = make_parser(doc=__doc__)
    parser.add_argument("--mission-uuid", default=default_mission_id() or None,
                        help="Mission UUID (or $ONAV_MISSION_ID). Omit for interactive menu.")
    parser.add_argument("--map-uuid", default=default_map_id() or None,
                        help="Map UUID (or $ONAV_MAP_ID). Omit for interactive menu.")
    parser.add_argument("--camera", default=DEFAULT_CAMERA,
                        help=f"Camera node name for video recording (default: {DEFAULT_CAMERA}).")
    parser.add_argument("--skip-mission", action="store_true",
                        help="Start recordings, sleep 5 s, stop. Useful to confirm plumbing.")
    args = parser.parse_args(argv)

    if args.dry_run:
        ns = args.namespace
        print(f"[dry-run] start log      Trigger  {ns}/log_manager/start_recording")
        print(f"[dry-run] start video    Action   {ns}/{args.camera}/start_recording")
        if args.skip_mission:
            print("[dry-run] sleep 5 s")
        else:
            print(f"[dry-run] run mission    Action   {ns}/autonomy/mission  (interactive select)")
        print(f"[dry-run] stop video     Action   {ns}/{args.camera}/stop_recording")
        print(f"[dry-run] stop log       Trigger  {ns}/log_manager/stop_recording")
        return

    rclpy.init()
    node = MissionWithRecording(args.namespace, args.camera, args.skip_mission)
    log_started = False
    try:
        node.wait()
        if not args.skip_mission:
            node.map_uuid, _ = select_map(node, node.maps_client, args.map_uuid or "")
            node.mission_uuid, _ = select_mission(node, node.missions_client, args.mission_uuid or "")
        log_started = node.start_log()
        if not log_started:
            return
        if not node.start_video():
            return
        try:
            node.run_mission()
        except KeyboardInterrupt:
            node.cancel_in_flight()
    finally:
        node.stop_video()
        if log_started:
            node.stop_log()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
