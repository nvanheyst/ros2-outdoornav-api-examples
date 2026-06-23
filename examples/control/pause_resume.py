#!/usr/bin/env python3
"""Pause then resume the running autonomy. Minimal demo.

  ./pause_resume.py                          # default: /autonomy/pause SetBool
  ./pause_resume.py --hold 30                # 30 s pause
  ./pause_resume.py --variant control_setbool   # legacy /control_selection/pause SetBool
  ./pause_resume.py --variant autonomy_trigger  # legacy /autonomy/pause Trigger

Three service interfaces exist across OnAV releases — observed on
current OutdoorNav 2.3 stacks:
  autonomy_setbool (default):
    <ns>/autonomy/pause              (std_srvs/SetBool, data=true to pause)
    <ns>/autonomy/resume             (std_srvs/SetBool, data=true to resume)
  control_setbool (older integration layer):
    <ns>/control_selection/pause     (std_srvs/SetBool)
    <ns>/control_selection/resume    (std_srvs/SetBool)
  autonomy_trigger (older inner autonomy):
    <ns>/autonomy/pause              (std_srvs/Trigger)
    <ns>/autonomy/resume             (std_srvs/Trigger)

Run `service_inventory.py | grep -E 'pause|resume'` on your stack first
to confirm both the path and the type — same path can carry different
types between releases.
"""

from __future__ import annotations
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger

from common.argparse_base import make_parser
from common.ros_helpers import wait_for_service, call_service


class PauseResume(Node):
    def __init__(self, namespace: str, variant: str):
        super().__init__("pause_resume")
        if variant == "autonomy_setbool":
            self.pause_srv = f"{namespace}/autonomy/pause"
            self.resume_srv = f"{namespace}/autonomy/resume"
            self.pause_client = self.create_client(SetBool, self.pause_srv)
            self.resume_client = self.create_client(SetBool, self.resume_srv)
            self._call_pause = lambda: call_service(self, self.pause_client, SetBool.Request(data=True))
            self._call_resume = lambda: call_service(self, self.resume_client, SetBool.Request(data=True))
        elif variant == "control_setbool":
            self.pause_srv = f"{namespace}/control_selection/pause"
            self.resume_srv = f"{namespace}/control_selection/resume"
            self.pause_client = self.create_client(SetBool, self.pause_srv)
            self.resume_client = self.create_client(SetBool, self.resume_srv)
            self._call_pause = lambda: call_service(self, self.pause_client, SetBool.Request(data=True))
            self._call_resume = lambda: call_service(self, self.resume_client, SetBool.Request(data=True))
        else:  # autonomy_trigger
            self.pause_srv = f"{namespace}/autonomy/pause"
            self.resume_srv = f"{namespace}/autonomy/resume"
            self.pause_client = self.create_client(Trigger, self.pause_srv)
            self.resume_client = self.create_client(Trigger, self.resume_srv)
            self._call_pause = lambda: call_service(self, self.pause_client, Trigger.Request())
            self._call_resume = lambda: call_service(self, self.resume_client, Trigger.Request())

    def wait(self) -> None:
        wait_for_service(self, self.pause_client, self.pause_srv)
        wait_for_service(self, self.resume_client, self.resume_srv)

    def pause_then_resume(self, hold_s: float) -> None:
        self.get_logger().info(f"calling {self.pause_srv}")
        resp = self._call_pause()
        self.get_logger().info(f"  -> {getattr(resp, 'success', '?')}")
        self.get_logger().info(f"holding paused {hold_s:.1f} s")
        time.sleep(hold_s)
        self.get_logger().info(f"calling {self.resume_srv}")
        resp = self._call_resume()
        self.get_logger().info(f"  -> {getattr(resp, 'success', '?')}")


def main(argv=None):
    parser = make_parser(doc=__doc__)
    parser.add_argument("--hold", type=float, default=5.0,
                        help="Seconds to remain paused (default 5).")
    parser.add_argument("--variant",
                        choices=["autonomy_setbool", "control_setbool", "autonomy_trigger"],
                        default="autonomy_setbool",
                        help="Service variant. Default matches OutdoorNav 2.3 (autonomy/pause SetBool).")
    args = parser.parse_args(argv)

    if args.dry_run:
        ns = args.namespace
        if args.variant == "autonomy_setbool":
            path = f"{ns}/autonomy/pause"
            print(f"[dry-run] would call {path} (SetBool, data=true)")
        elif args.variant == "control_setbool":
            path = f"{ns}/control_selection/pause"
            print(f"[dry-run] would call {path} (SetBool, data=true)")
        else:
            path = f"{ns}/autonomy/pause"
            print(f"[dry-run] would call {path} (Trigger)")
        print(f"[dry-run] then sleep {args.hold}s")
        print(f"[dry-run] then call resume on the matching path")
        return

    rclpy.init()
    node = PauseResume(args.namespace, args.variant)
    try:
        node.wait()
        node.pause_then_resume(args.hold)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
