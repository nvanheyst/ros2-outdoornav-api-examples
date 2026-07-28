#!/usr/bin/env python3
"""Watch the ExecuteMission action and notify on abort, with optional context.

  ./notify_on_mission_failure.py                              # prints to stdout
  ./notify_on_mission_failure.py --camera-topic \\
      /a300_00003/sensors/camera_0/color/compressed           # attach latest frame
  NOTIFY_WEBHOOK_URL=https://hooks.slack.com/... \\
      ./notify_on_mission_failure.py --via webhook
  NOTIFY_SMTP_HOST=smtp.gmail.com NOTIFY_SMTP_PORT=587 \\
      NOTIFY_SMTP_USER=foo NOTIFY_SMTP_PASS=bar \\
      NOTIFY_EMAIL_FROM=robot@x.com NOTIFY_EMAIL_TO=ops@x.com,oncall@x.com \\
      TWILIO_SID=... TWILIO_TOKEN=... TWILIO_FROM=+1... TWILIO_TO=+1... \\
      ./notify_on_mission_failure.py --via email,sms --camera-topic \\
        /a300_00003/sensors/camera_0/color/compressed

Passive: catches any mission abort whether it was launched from the web UI or a
script. Rich failure detail (which waypoint failed, why) is only delivered to
the client that *sent* the goal - see the active-variant note near the bottom
if you need that.

The notifier collects a FailureContext at the moment of abort: namespace, goal
uuid, latest GPS fix, latest autonomy state, latest camera frame. Email gets
the full body + JPEG attached; SMS is a one-line "check email" heads-up;
webhook gets the structured payload (no binary).

Touches:
  topic <namespace>/autonomy/mission/_action/status            (GoalStatusArray, subscribe)
  topic <namespace>/autonomy/mission_from_goal/_action/status  (--include-from-goal)
  topic <namespace>/autonomy/status                            (AutonomyStatus, subscribe)
  topic <namespace>/localization/fix                           (NavSatFix, subscribe)
  topic <camera-topic>                                         (CompressedImage, subscribe)
"""

from __future__ import annotations
import json
import os
import smtplib
import sys
import urllib.request
from dataclasses import dataclass
from datetime import datetime, timezone
from email.message import EmailMessage
from pathlib import Path
from typing import Callable

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy,
    qos_profile_sensor_data,
)
from action_msgs.msg import GoalStatusArray, GoalStatus
from sensor_msgs.msg import NavSatFix, CompressedImage
from clearpath_navigation_msgs.msg import AutonomyStatus

from examples.common.argparse_base import make_parser


@dataclass
class FailureContext:
    namespace: str
    goal_uuid: str
    status: str
    when_iso: str
    autonomy_state: str
    current_waypoint: str
    lat: float | None
    lon: float | None
    image_jpeg: bytes | None
    image_topic: str

    def subject(self) -> str:
        return f"OutdoorNav: mission {self.status.lower()} (ns={self.namespace})"

    def short_text(self) -> str:
        loc = f" @ {self.lat:.5f},{self.lon:.5f}" if self.lat is not None else ""
        return f"OutdoorNav {self.namespace}: mission {self.status.lower()}{loc}. Check email."

    def text_body(self) -> str:
        loc = (f"  position        : {self.lat:.6f}, {self.lon:.6f}\n"
               if self.lat is not None else "  position        : (no fix)\n")
        img = (f"  camera frame    : attached ({len(self.image_jpeg)} bytes from {self.image_topic})\n"
               if self.image_jpeg else "")
        return (
            f"Mission goal transitioned to {self.status}.\n"
            f"  namespace       : {self.namespace}\n"
            f"  goal uuid       : {self.goal_uuid}\n"
            f"  stamped (UTC)   : {self.when_iso}\n"
            f"  autonomy state  : {self.autonomy_state}\n"
            f"  current waypoint: {self.current_waypoint or '(none)'}\n"
            f"{loc}{img}"
        )


def send_stdout(ctx: FailureContext) -> None:
    print(f"\n--- NOTIFY ---\nSubject: {ctx.subject()}\n\n{ctx.text_body()}", end="")
    if ctx.image_jpeg:
        print(f"  (would attach: {len(ctx.image_jpeg)} bytes JPEG)")
    print("--------------\n")


def send_email(ctx: FailureContext) -> None:
    host = os.environ["NOTIFY_SMTP_HOST"]
    port = int(os.environ.get("NOTIFY_SMTP_PORT", "587"))
    user = os.environ["NOTIFY_SMTP_USER"]
    password = os.environ["NOTIFY_SMTP_PASS"]
    sender = os.environ["NOTIFY_EMAIL_FROM"]
    recipients = [r.strip() for r in os.environ["NOTIFY_EMAIL_TO"].split(",") if r.strip()]

    msg = EmailMessage()
    msg["Subject"] = ctx.subject()
    msg["From"] = sender
    msg["To"] = ", ".join(recipients)
    msg.set_content(ctx.text_body())
    if ctx.image_jpeg:
        msg.add_attachment(ctx.image_jpeg, maintype="image", subtype="jpeg",
                           filename="latest_frame.jpg")

    with smtplib.SMTP(host, port, timeout=10) as smtp:
        smtp.starttls()
        smtp.login(user, password)
        smtp.send_message(msg)


def send_webhook(ctx: FailureContext) -> None:
    # Binary frames blow up Slack-style webhooks; ship URLs instead. If you
    # want the frame inline, base64-encode it before adding to the payload.
    payload = {
        "text": f"{ctx.subject()}\n\n{ctx.text_body()}",
        "namespace": ctx.namespace,
        "goal_uuid": ctx.goal_uuid,
        "status": ctx.status,
        "when_iso": ctx.when_iso,
        "lat": ctx.lat,
        "lon": ctx.lon,
        "autonomy_state": ctx.autonomy_state,
        "current_waypoint": ctx.current_waypoint,
    }
    req = urllib.request.Request(
        os.environ["NOTIFY_WEBHOOK_URL"],
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
    )
    with urllib.request.urlopen(req, timeout=10) as resp:
        resp.read()


def send_sms(ctx: FailureContext) -> None:
    # Deferred so the script's --help works without `twilio` installed.
    from twilio.rest import Client
    client = Client(os.environ["TWILIO_SID"], os.environ["TWILIO_TOKEN"])
    client.messages.create(
        from_=os.environ["TWILIO_FROM"],
        to=os.environ["TWILIO_TO"],
        body=ctx.short_text(),
    )


TRANSPORTS: dict[str, Callable[[FailureContext], None]] = {
    "stdout": send_stdout,
    "email": send_email,
    "webhook": send_webhook,
    "sms": send_sms,
}


STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
    GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "EXECUTING",
    GoalStatus.STATUS_CANCELING: "CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
}

ACTIVITY_NAMES = {
    AutonomyStatus.IDLE: "IDLE",
    AutonomyStatus.MISSION: "MISSION",
    AutonomyStatus.MISSION_FROM_GOAL: "MISSION_FROM_GOAL",
    AutonomyStatus.GOTO: "GOTO",
    AutonomyStatus.GOTO_POI: "GOTO_POI",
    AutonomyStatus.DOCKING_LOCAL: "DOCKING_LOCAL",
    AutonomyStatus.UNDOCKING_LOCAL: "UNDOCKING_LOCAL",
    AutonomyStatus.DOCKING_MAP: "DOCKING_MAP",
}


def action_status_qos() -> QoSProfile:
    # Action servers publish status with TRANSIENT_LOCAL + RELIABLE; match it
    # so late-joining subscribers still see the last latched status.
    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


class MissionFailureWatcher(Node):
    def __init__(self, namespace: str, transports: list[Callable], also_on_cancel: bool,
                 include_from_goal: bool, camera_topic: str):
        super().__init__("notify_on_mission_failure")
        self.namespace = namespace
        self.transports = transports
        self.also_on_cancel = also_on_cancel
        self.camera_topic = camera_topic
        self._last_status: dict[str, int] = {}
        self._activity = "?"
        self._current_waypoint = ""
        self._latest_fix: NavSatFix | None = None
        self._latest_image: bytes | None = None

        status_topic = f"{namespace}/autonomy/mission/_action/status"
        self.create_subscription(GoalStatusArray, status_topic,
                                 self._status_cb, action_status_qos())
        self.get_logger().info(f"watching {status_topic}")

        if include_from_goal:
            from_goal_topic = f"{namespace}/autonomy/mission_from_goal/_action/status"
            self.create_subscription(GoalStatusArray, from_goal_topic,
                                     self._status_cb, action_status_qos())
            self.get_logger().info(f"watching {from_goal_topic}")

        self.create_subscription(AutonomyStatus, f"{namespace}/autonomy/status",
                                 self._autonomy_cb, 10)
        self.create_subscription(NavSatFix, f"{namespace}/localization/fix",
                                 self._fix_cb, 10)

        if camera_topic:
            self.create_subscription(CompressedImage, camera_topic,
                                     self._image_cb, qos_profile_sensor_data)
            self.get_logger().info(f"will attach frames from {camera_topic}")

    def _autonomy_cb(self, msg: AutonomyStatus) -> None:
        self._activity = ACTIVITY_NAMES.get(msg.state, f"raw={msg.state}")
        self._current_waypoint = msg.current_goal or ""

    def _fix_cb(self, msg: NavSatFix) -> None:
        self._latest_fix = msg

    def _image_cb(self, msg: CompressedImage) -> None:
        self._latest_image = bytes(msg.data)

    def _status_cb(self, msg: GoalStatusArray) -> None:
        for entry in msg.status_list:
            uuid = bytes(entry.goal_info.goal_id.uuid).hex()
            prev = self._last_status.get(uuid)
            self._last_status[uuid] = entry.status
            if prev == entry.status:
                continue
            if entry.status == GoalStatus.STATUS_ABORTED:
                self._fire(uuid, "ABORTED", entry)
            elif entry.status == GoalStatus.STATUS_CANCELED and self.also_on_cancel:
                self._fire(uuid, "CANCELED", entry)

    def _fire(self, uuid: str, status_name: str, entry) -> None:
        stamp = entry.goal_info.stamp
        when_iso = datetime.fromtimestamp(
            stamp.sec + stamp.nanosec * 1e-9, tz=timezone.utc,
        ).isoformat(timespec="seconds")
        ctx = FailureContext(
            namespace=self.namespace,
            goal_uuid=uuid,
            status=status_name,
            when_iso=when_iso,
            autonomy_state=self._activity,
            current_waypoint=self._current_waypoint,
            lat=self._latest_fix.latitude if self._latest_fix else None,
            lon=self._latest_fix.longitude if self._latest_fix else None,
            image_jpeg=self._latest_image,
            image_topic=self.camera_topic,
        )
        self.get_logger().warn(f"{status_name}: {uuid}")
        for transport in self.transports:
            try:
                transport(ctx)
            except Exception as e:
                self.get_logger().error(f"{transport.__name__} failed: {e}")


def dry_run(namespace: str, via: list[str]) -> None:
    ctx = FailureContext(
        namespace=namespace,
        goal_uuid="0" * 32,
        status="ABORTED",
        when_iso="1970-01-01T00:00:00+00:00",
        autonomy_state="MISSION",
        current_waypoint="dry-run-waypoint-uuid",
        lat=50.10940,
        lon=-97.31870,
        image_jpeg=None,
        image_topic="",
    )
    for name in via:
        print(f"[dry-run] via={name}")
        if name == "stdout":
            send_stdout(ctx)
        elif name == "webhook":
            payload = {
                "text": f"{ctx.subject()}\n\n{ctx.text_body()}",
                "namespace": ctx.namespace,
                "goal_uuid": ctx.goal_uuid,
                "status": ctx.status,
                "when_iso": ctx.when_iso,
                "lat": ctx.lat, "lon": ctx.lon,
                "autonomy_state": ctx.autonomy_state,
                "current_waypoint": ctx.current_waypoint,
            }
            print(f"  would POST to $NOTIFY_WEBHOOK_URL:")
            print(json.dumps(payload, indent=2))
        elif name == "email":
            print(f"  would send via SMTP using $NOTIFY_SMTP_HOST/$NOTIFY_EMAIL_TO")
            print(f"  Subject: {ctx.subject()}")
            print(ctx.text_body())
        elif name == "sms":
            print(f"  would send via Twilio using $TWILIO_FROM -> $TWILIO_TO")
            print(f"  Body: {ctx.short_text()}")


# Active variant: if your own script launched the mission, get the rich result
# via goal_handle.get_result_async(). result.code is one of
# UNKNOWN/CANCELLED/COLLISION/PLANNING/START_TASKS/END_TASKS/MISSION_TASKS;
# result.error_msg is human-readable; result.goal_states is a list of
# MapGoalState with NAV_FAILURE / TASK_FAILURE per waypoint. Build a
# FailureContext from those and reuse the transports above.


def main(argv=None):
    parser = make_parser(doc=__doc__)
    parser.add_argument("--via", default="stdout",
                        help="Comma-separated backends: stdout,email,webhook,sms. Default stdout.")
    parser.add_argument("--also-on-cancel", action="store_true",
                        help="Also notify on STATUS_CANCELED (default: only aborts).")
    parser.add_argument("--include-from-goal", action="store_true",
                        help="Also watch the mission_from_goal action.")
    parser.add_argument("--camera-topic", default="",
                        help="CompressedImage topic to attach to email (default: no attachment).")
    args = parser.parse_args(argv)

    via_list = [v.strip() for v in args.via.split(",") if v.strip()]
    unknown = [v for v in via_list if v not in TRANSPORTS]
    if unknown:
        parser.error(f"unknown --via backend(s): {unknown}; pick from {sorted(TRANSPORTS)}")

    if args.dry_run:
        dry_run(args.namespace, via_list)
        return

    rclpy.init()
    node = MissionFailureWatcher(
        namespace=args.namespace,
        transports=[TRANSPORTS[v] for v in via_list],
        also_on_cancel=args.also_on_cancel,
        include_from_goal=args.include_from_goal,
        camera_topic=args.camera_topic,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
