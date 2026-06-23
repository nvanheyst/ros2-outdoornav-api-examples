"""
Wait until a target wall-clock time then fire ExecuteMission.

  python schedule_mission.py 2026-06-22T18:00:00Z
  python schedule_mission.py +30s

For a real schedule you'd wrap this in a systemd timer + chrony, not loop
in Python. This is the inside of one fire.
"""

import sys
import time
import re
from datetime import datetime, timezone, timedelta

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from clearpath_navigation_msgs.action import ExecuteMission


ROBOT_NAMESPACE = '/a300_00003'
ACTION_EXECUTE_MISSION = f'{ROBOT_NAMESPACE}/autonomy/mission'

MISSION_ID = "REPLACE_WITH_MISSION_UUID"
MAP_ID = "REPLACE_WITH_MAP_UUID"

HEARTBEAT_S = 5.0


def parse_target(arg: str) -> datetime:
    """Accept '+30s' shortcut or ISO 8601 with timezone."""
    m = re.match(r'^\+(\d+)([smh])$', arg)
    if m:
        n, unit = int(m.group(1)), m.group(2)
        seconds = n * {'s': 1, 'm': 60, 'h': 3600}[unit]
        return datetime.now(timezone.utc) + timedelta(seconds=seconds)
    # fromisoformat handles 'Z' suffix on Python 3.11+.
    return datetime.fromisoformat(arg).astimezone(timezone.utc)


class ScheduledMission(Node):
    def __init__(self, target: datetime):
        super().__init__('scheduled_mission')
        self.target = target
        self.mission_client = ActionClient(self, ExecuteMission, ACTION_EXECUTE_MISSION)
        self._goal_handle = None

    def wait_for_server(self):
        self.get_logger().info(f'Waiting for {ACTION_EXECUTE_MISSION}...')
        while not self.mission_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('  still waiting...')

    def countdown(self):
        while True:
            now = datetime.now(timezone.utc)
            remaining = (self.target - now).total_seconds()
            if remaining <= 0:
                self.get_logger().info('Target time reached — firing mission.')
                return
            self.get_logger().info(
                f'  T-{int(remaining)}s  (target {self.target.isoformat()})'
            )
            time.sleep(min(HEARTBEAT_S, remaining))

    def fire(self) -> bool:
        goal = ExecuteMission.Goal(mission_uuid=MISSION_ID, map_uuid=MAP_ID)
        send_future = self.mission_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        self._goal_handle = send_future.result()

        if not self._goal_handle or not self._goal_handle.accepted:
            self.get_logger().error('ExecuteMission goal rejected.')
            return False

        self.get_logger().info('Mission accepted — blocking on result.')
        result_future = self._goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        status = result_future.result().status if result_future.result() else None
        if status == 4:
            self.get_logger().info('Mission SUCCEEDED.')
            return True
        self.get_logger().error(f'Mission ended with status {status}.')
        return False

    def cancel_in_flight(self):
        if self._goal_handle is None:
            return
        self.get_logger().warn('Cancelling in-flight mission...')
        cancel_future = self._goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future)


def main(args=None):
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(1)
    target = parse_target(sys.argv[1])
    if target <= datetime.now(timezone.utc):
        print('Target time is in the past.')
        sys.exit(1)

    rclpy.init(args=args)
    node = ScheduledMission(target)
    try:
        node.wait_for_server()
        node.countdown()
        node.fire()
    except KeyboardInterrupt:
        node.cancel_in_flight()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
