"""
Delete every event log via logger/delete_log.

Ported from the onav SDK ROS 1 examples and updated for ROS 2.

  python delete_logs.py
  python delete_logs.py --keep-recent 24    # leave logs from the last 24h
  python delete_logs.py --dry-run

purge_record=true also removes the bag chunks; without it only the entry
disappears from the UI and disk isn't reclaimed.
"""

import sys
import time

import rclpy
from rclpy.node import Node
from clearpath_logger_msgs.srv import GetAllLogs, DeleteLog


ROBOT_NAMESPACE = '/a300_00003'
SERVICE_GET_ALL_LOGS = f'{ROBOT_NAMESPACE}/logger/get_all_logs'
SERVICE_DELETE_LOG = f'{ROBOT_NAMESPACE}/logger/delete_log'


class DeleteLogs(Node):
    def __init__(self):
        super().__init__('delete_logs')
        self.get_all_client = self.create_client(GetAllLogs, SERVICE_GET_ALL_LOGS)
        self.delete_client = self.create_client(DeleteLog, SERVICE_DELETE_LOG)

    def wait(self):
        for c, n in [(self.get_all_client, SERVICE_GET_ALL_LOGS),
                     (self.delete_client, SERVICE_DELETE_LOG)]:
            while not c.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for {n}...')

    def fetch_logs(self):
        future = self.get_all_client.call_async(GetAllLogs.Request())
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        return getattr(resp, 'logs', [])

    def delete(self, log_uuid: str) -> bool:
        req = DeleteLog.Request()
        req.uuid = log_uuid
        req.delete_media = True
        req.purge_record = True
        future = self.delete_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        return bool(getattr(resp, 'success', True))


def main(args=None):
    keep_recent_h = None
    dry_run = '--dry-run' in sys.argv
    if '--keep-recent' in sys.argv:
        keep_recent_h = float(sys.argv[sys.argv.index('--keep-recent') + 1])

    rclpy.init(args=args)
    node = DeleteLogs()
    try:
        node.wait()
        logs = node.fetch_logs()
        node.get_logger().info(f'Found {len(logs)} log(s).')

        cutoff = (time.time() - keep_recent_h * 3600) if keep_recent_h else None
        to_delete = []
        for log in logs:
            # EventLog has `start_time` (clearpath_logger_msgs/EventTime) — sec is unix seconds.
            ts = getattr(getattr(log, 'start_time', None), 'sec', None)
            if cutoff is not None and ts is not None and ts >= cutoff:
                continue
            to_delete.append(log)

        node.get_logger().info(f'Will delete {len(to_delete)} log(s)'
                               + (' (dry run)' if dry_run else ''))

        for log in to_delete:
            uid = getattr(log, 'uuid', '???')
            if dry_run:
                node.get_logger().info(f'  would delete {uid}')
                continue
            ok = node.delete(uid)
            node.get_logger().info(f'  {"deleted" if ok else "FAILED"} {uid}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
