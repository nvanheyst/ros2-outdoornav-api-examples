#!/usr/bin/env python3
"""Run a mission with automatic resume: if the robot aborts, pick up from
the last waypoint rather than starting over.

  ./mission_with_resume.py                             # up to 3 resume attempts
  ./mission_with_resume.py --max-retries 5 --backoff 10
  ONAV_MAP_ID=<u> ONAV_MISSION_ID=<u> ./mission_with_resume.py

How it works:
  1. Fires ExecuteMission and watches `current_goal_id` to track the active
     waypoint throughout the run.
  2. On SUCCEEDED → done.
  3. On ABORTED → wait `--backoff` seconds, then call ExecuteMissionFromGoal
     with the last-seen waypoint. Repeat up to `--max-retries` times.
  4. On CANCELLED (operator stop) → exits without retrying.

  `run_on_start_tasks=False` on resume so start-of-mission tasks only fire
  once. Abort details (error code, per-waypoint states) are logged before
  each retry so you can see why it stopped.

Touches:
  action <namespace>/autonomy/mission             (ExecuteMission)
  action <namespace>/autonomy/mission_from_goal   (ExecuteMissionFromGoal)
  topic  <namespace>/navigation/current_goal_id   (String, subscribe)
"""

from __future__ import annotations
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from std_msgs.msg import String
from clearpath_navigation_msgs.action import ExecuteMission, ExecuteMissionFromGoal
from clearpath_navigation_msgs.msg import MapGoalState

from clearpath_mission_manager_msgs.srv import GetAllMaps, GetAllNetworkMissions

from examples.common.argparse_base import make_parser
from examples.common.config import map_id as default_map_id, mission_id as default_mission_id
from examples.common.ros_helpers import wait_for_action, wait_for_service
from examples.common.onav import select_map, select_mission


STATE_NAMES = {
    MapGoalState.PENDING: "PENDING",
    MapGoalState.ACTIVE: "ACTIVE",
    MapGoalState.SUCCEEDED: "SUCCEEDED",
    MapGoalState.NAV_FAILURE: "NAV_FAILURE",
    MapGoalState.TASK_WARNING: "TASK_WARNING",
    MapGoalState.TASK_FAILURE: "TASK_FAILURE",
}

RESULT_CODE_NAMES = {
    0: "UNKNOWN", 1: "CANCELLED", 2: "COLLISION", 3: "PLANNING",
    4: "START_TASKS", 5: "END_TASKS", 6: "MISSION_TASKS",
}


class MissionWithResume(Node):
    def __init__(self, namespace: str):
        super().__init__("mission_with_resume")
        self.namespace = namespace
        self.mission_uuid = ""
        self.map_uuid = ""
        self._current_goal_id = ""

        self.mission_action = f"{namespace}/autonomy/mission"
        self.resume_action = f"{namespace}/autonomy/mission_from_goal"
        self.maps_srv = f"{namespace}/mission_manager/get_all_maps"
        self.missions_srv = f"{namespace}/mission_manager/get_all_missions"
        self.mission_client = ActionClient(self, ExecuteMission, self.mission_action)
        self.resume_client = ActionClient(self, ExecuteMissionFromGoal, self.resume_action)
        self.maps_client = self.create_client(GetAllMaps, self.maps_srv)
        self.missions_client = self.create_client(GetAllNetworkMissions, self.missions_srv)

        self.create_subscription(String, f"{namespace}/navigation/current_goal_id",
                                 self._goal_id_cb, 10)

    def _goal_id_cb(self, msg: String) -> None:
        if msg.data:
            self._current_goal_id = msg.data

    def wait(self) -> None:
        wait_for_action(self, self.mission_client, self.mission_action)
        wait_for_action(self, self.resume_client, self.resume_action)
        wait_for_service(self, self.maps_client, self.maps_srv)
        wait_for_service(self, self.missions_client, self.missions_srv)

    def _send_mission(self):
        goal = ExecuteMission.Goal(mission_uuid=self.mission_uuid, map_uuid=self.map_uuid)
        send_future = self.mission_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        return send_future.result()

    def _send_resume(self, goal_uuid: str):
        goal = ExecuteMissionFromGoal.Goal(
            mission_uuid=self.mission_uuid,
            map_uuid=self.map_uuid,
            goal_uuid=goal_uuid,
            run_on_start_tasks=False,
        )
        send_future = self.resume_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        return send_future.result()

    def _wait_result(self, handle):
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result()  # has .status, .result

    def _log_abort_detail(self, result) -> None:
        code = RESULT_CODE_NAMES.get(result.code, f"raw={result.code}")
        self.get_logger().error(f"  abort code: {code}  message: {result.message or '(none)'}")
        if result.error_msg:
            self.get_logger().error(f"  error_code={result.error_code}  {result.error_msg}")
        states = list(getattr(result, "goal_states", []) or [])
        for s in states:
            name = STATE_NAMES.get(s.state, f"raw={s.state}")
            uuid = getattr(s.waypoint, "uuid", "?")[:8]
            wp_name = getattr(s.waypoint, "name", "")
            self.get_logger().info(f"    [{name}] {uuid}  {wp_name}")

    def run(self, max_retries: int, backoff_sec: float) -> bool:
        attempt = 0
        handle = self._send_mission()
        if not handle or not handle.accepted:
            self.get_logger().error("ExecuteMission goal rejected on first attempt")
            return False
        self.get_logger().info(f"mission started (attempt {attempt + 1})")

        while True:
            wrapped = self._wait_result(handle)
            status = wrapped.status if wrapped else None
            result = wrapped.result if wrapped else None
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f"mission succeeded after {attempt + 1} attempt(s)")
                return True
            if status == GoalStatus.STATUS_CANCELED:
                self.get_logger().warn("mission cancelled by operator - no retry")
                return False
            if status != GoalStatus.STATUS_ABORTED:
                self.get_logger().warn(f"unexpected status {status} - stopping")
                return False

            self.get_logger().error(f"mission ABORTED on attempt {attempt + 1}")
            if result is not None:
                self._log_abort_detail(result)

            if attempt >= max_retries:
                self.get_logger().error(f"exhausted {max_retries} retries - giving up")
                return False

            resume_from = self._current_goal_id
            if not resume_from:
                self.get_logger().error("no current_goal_id seen - can't resume")
                return False

            self.get_logger().info(f"sleeping {backoff_sec:.1f}s before retry")
            time.sleep(backoff_sec)
            attempt += 1
            self.get_logger().info(f"resuming from goal {resume_from[:8]} (attempt {attempt + 1})")
            handle = self._send_resume(resume_from)
            if not handle or not handle.accepted:
                self.get_logger().error("ExecuteMissionFromGoal rejected - giving up")
                return False


def main(argv=None):
    parser = make_parser(doc=__doc__)
    parser.add_argument("--mission-uuid", default=default_mission_id() or None,
                        help="Mission UUID (or $ONAV_MISSION_ID).")
    parser.add_argument("--map-uuid", default=default_map_id() or None,
                        help="Map UUID (or $ONAV_MAP_ID).")
    parser.add_argument("--max-retries", type=int, default=3,
                        help="Resume attempts after the first abort (default 3).")
    parser.add_argument("--backoff", type=float, default=5.0,
                        help="Seconds to wait between retries (default 5).")
    args = parser.parse_args(argv)

    if args.dry_run:
        print(f"[dry-run] would run ExecuteMission on {args.namespace}/autonomy/mission")
        print(f"[dry-run] on abort, would call ExecuteMissionFromGoal up to {args.max_retries}x")
        print(f"[dry-run] backoff between retries: {args.backoff} s")
        print(f"[dry-run] map/mission: {'provided' if args.map_uuid and args.mission_uuid else 'interactive menu'}")
        return

    rclpy.init()
    node = MissionWithResume(args.namespace)
    try:
        node.wait()
        node.map_uuid, _ = select_map(node, node.maps_client, args.map_uuid or "")
        node.mission_uuid, _ = select_mission(node, node.missions_client, args.mission_uuid or "")
        node.run(max_retries=args.max_retries, backoff_sec=args.backoff)
    except KeyboardInterrupt:
        node.get_logger().warn("interrupted")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
