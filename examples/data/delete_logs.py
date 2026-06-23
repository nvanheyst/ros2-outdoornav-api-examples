#!/usr/bin/env python3
"""Delete event logs via logger/delete_log.

  ./delete_logs.py                           # delete all logs
  ./delete_logs.py --keep-recent 24          # keep last 24 hours
  ./delete_logs.py --dry-run                 # list what would be deleted

Touches:
  service <namespace>/logger/get_all_logs   (GetAllLogs)
  service <namespace>/logger/delete_log     (DeleteLog)

purge_record=true also removes bag chunks; without it only the entry
disappears from the UI and disk isn't reclaimed.
"""

from __future__ import annotations
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import rclpy
from rclpy.node import Node
from clearpath_logger_msgs.srv import GetAllLogs, DeleteLog

from common.argparse_base import make_parser
from common.ros_helpers import wait_for_service, call_service


class DeleteLogs(Node):
    def __init__(self, namespace: str):
        super().__init__("delete_logs")
        self.get_all_srv = f"{namespace}/logger/get_all_logs"
        self.delete_srv = f"{namespace}/logger/delete_log"
        self.get_all_client = self.create_client(GetAllLogs, self.get_all_srv)
        self.delete_client = self.create_client(DeleteLog, self.delete_srv)

    def wait(self) -> None:
        wait_for_service(self, self.get_all_client, self.get_all_srv)
        wait_for_service(self, self.delete_client, self.delete_srv)

    def fetch_logs(self) -> list:
        resp = call_service(self, self.get_all_client, GetAllLogs.Request())
        return getattr(resp, "logs", [])

    def delete(self, log_uuid: str) -> bool:
        req = DeleteLog.Request()
        req.uuid = log_uuid
        req.delete_media = True
        req.purge_record = True
        resp = call_service(self, self.delete_client, req)
        return bool(getattr(resp, "success", True))


def main(argv=None):
    parser = make_parser(doc=__doc__)
    parser.add_argument("--keep-recent", type=float, default=None,
                        help="Keep logs from the last N hours.")
    args = parser.parse_args(argv)

    rclpy.init()
    node = DeleteLogs(args.namespace)
    try:
        node.wait()
        logs = node.fetch_logs()
        node.get_logger().info(f"found {len(logs)} log(s)")

        cutoff = (time.time() - args.keep_recent * 3600) if args.keep_recent else None
        to_delete = []
        for log in logs:
            ts = getattr(getattr(log, "start_time", None), "sec", None)
            if cutoff is not None and ts is not None and ts >= cutoff:
                continue
            to_delete.append(log)

        node.get_logger().info(
            f"will delete {len(to_delete)} log(s)" + (" (dry-run)" if args.dry_run else "")
        )
        for log in to_delete:
            uid = getattr(log, "uuid", "???")
            if args.dry_run:
                node.get_logger().info(f"  would delete {uid}")
                continue
            ok = node.delete(uid)
            node.get_logger().info(f"  {'deleted' if ok else 'FAILED'} {uid}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
