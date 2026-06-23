"""
Loop a mission until battery drops below threshold or max loops hit.

  python loop_mission_battery_aware.py 30 5    # 30%, 5 loops

Spins the executor manually while the mission action is in flight so the
BMS subscription keeps draining. Set MISSION_ID/MAP_ID below.
"""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import BatteryState
from clearpath_navigation_msgs.action import ExecuteMission


ROBOT_NAMESPACE = '/a300_00003'
ACTION_EXECUTE_MISSION = f'{ROBOT_NAMESPACE}/autonomy/mission'
TOPIC_BATTERY = f'{ROBOT_NAMESPACE}/platform/bms/state'

MISSION_ID = "REPLACE_WITH_MISSION_UUID"
MAP_ID = "REPLACE_WITH_MAP_UUID"


class BatteryAwareLoop(Node):
    def __init__(self, threshold_percent: float, max_loops: int):
        super().__init__('battery_aware_loop')
        self.threshold = threshold_percent
        self.max_loops = max_loops

        self.latest_percent: float | None = None
        self.create_subscription(BatteryState, TOPIC_BATTERY, self._bms_cb, 10)
        self.mission_client = ActionClient(self, ExecuteMission, ACTION_EXECUTE_MISSION)
        self._goal_handle = None

    def _bms_cb(self, msg: BatteryState):
        # BatteryState.percentage is 0.0-1.0.
        self.latest_percent = float(msg.percentage) * 100.0

    def wait_for_initial(self):
        self.get_logger().info('Waiting for action server and first BMS reading...')
        while not self.mission_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('  action server still pending...')
        while self.latest_percent is None:
            rclpy.spin_once(self, timeout_sec=1.0)
        self.get_logger().info(f'Initial battery: {self.latest_percent:.1f}%')

    def run_mission_blocking(self) -> bool:
        goal = ExecuteMission.Goal(mission_uuid=MISSION_ID, map_uuid=MAP_ID)
        send_future = self.mission_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        self._goal_handle = send_future.result()

        if not self._goal_handle or not self._goal_handle.accepted:
            self.get_logger().error('Mission goal rejected.')
            return False

        result_future = self._goal_handle.get_result_async()
        # Spin manually so BMS subscription keeps draining during the mission.
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.5)
        self._goal_handle = None

        status = result_future.result().status if result_future.result() else None
        return status == 4

    def cancel_in_flight(self):
        if self._goal_handle is None:
            return
        self.get_logger().warn('Cancelling in-flight mission...')
        cancel_future = self._goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future)

    def loop(self):
        for i in range(1, self.max_loops + 1):
            # Drain a couple of callbacks so latest_percent is current.
            rclpy.spin_once(self, timeout_sec=0.1)
            rclpy.spin_once(self, timeout_sec=0.1)
            pct = self.latest_percent
            self.get_logger().info(
                f'Loop {i}/{self.max_loops}  battery={pct:.1f}%  threshold={self.threshold:.1f}%'
            )
            if pct < self.threshold:
                self.get_logger().warn(
                    f'Battery {pct:.1f}% below threshold {self.threshold:.1f}% — stopping.'
                )
                return
            ok = self.run_mission_blocking()
            if not ok:
                self.get_logger().error(f'Loop {i} mission failed — stopping.')
                return
        self.get_logger().info(f'Completed all {self.max_loops} loops.')


def main(args=None):
    threshold = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    max_loops = int(sys.argv[2]) if len(sys.argv) > 2 else 10

    rclpy.init(args=args)
    node = BatteryAwareLoop(threshold, max_loops)
    try:
        node.wait_for_initial()
        node.loop()
    except KeyboardInterrupt:
        node.cancel_in_flight()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
