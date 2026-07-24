#!/usr/bin/env python3
"""Pause/resume autonomy based on a Bool topic (the perception gate pattern).

Subscribes to any std_msgs/Bool topic. When the message is False, calls
autonomy/pause to halt the robot. When True, calls autonomy/resume. Runs as a
passive background watcher alongside any mission.

The signal source — ML model, sensor, operator button — is the user's concern.
See mini-projects/IDEAS.md "Vision integration" for an example publisher.

  ./perception_gate.py                              # listens on /perception_gate
  ./perception_gate.py --gate-topic /my/bool/topic

Touches:
  topic   <gate_topic>        (std_msgs/Bool, subscribe)
  service autonomy/pause      (SetBool)
  service autonomy/resume     (SetBool)
"""

from __future__ import annotations
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from examples.common.argparse_base import make_parser
from examples.common.ros_helpers import wait_for_service

DEFAULT_GATE_TOPIC = "/perception_gate"


class PerceptionGate(Node):
    def __init__(self, namespace: str, gate_topic: str):
        super().__init__("perception_gate")
        self.pause_srv = f"{namespace}/autonomy/pause"
        self.resume_srv = f"{namespace}/autonomy/resume"
        self.pause_client = self.create_client(SetBool, self.pause_srv)
        self.resume_client = self.create_client(SetBool, self.resume_srv)
        self._paused = False
        self.create_subscription(Bool, gate_topic, self._cb, 10)
        self.get_logger().info(
            f"perception gate: listening on {gate_topic}  |  "
            f"pause → {self.pause_srv}  |  resume → {self.resume_srv}"
        )

    def wait(self) -> None:
        wait_for_service(self, self.pause_client, self.pause_srv)
        wait_for_service(self, self.resume_client, self.resume_srv)

    def _cb(self, msg: Bool) -> None:
        clear = bool(msg.data)
        if not clear and not self._paused:
            self._set_pause(True)
        elif clear and self._paused:
            self._set_pause(False)

    def _set_pause(self, pause: bool) -> None:
        client = self.pause_client if pause else self.resume_client
        srv = self.pause_srv if pause else self.resume_srv
        # pause and resume are separate services; both expect data=True to trigger
        future = client.call_async(SetBool.Request(data=True))
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        resp = future.result()
        ok = bool(resp and resp.success) if resp else False
        action = "paused" if pause else "resumed"
        if ok:
            self._paused = pause
            self.get_logger().info(f"autonomy {action}")
        else:
            self.get_logger().warn(f"{srv} call failed — autonomy may not have {action}")


def main(argv=None):
    parser = make_parser(doc=__doc__, add_dry_run=False)
    parser.add_argument("--gate-topic", default=DEFAULT_GATE_TOPIC,
                        help=f"Bool topic to watch (default: {DEFAULT_GATE_TOPIC}).")
    args = parser.parse_args(argv)

    rclpy.init()
    node = PerceptionGate(args.namespace, args.gate_topic)
    try:
        node.wait()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
