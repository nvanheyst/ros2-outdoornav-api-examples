#!/usr/bin/env python3
"""Boustrophedon coverage map at the robot's CURRENT position + heading - no input.

Same idea as row_generator_square.py, but instead of --lat/--lon/--bearing it
reads the live GPS fix and the map->base_link heading, so it runs with zero
required arguments. Rows run ALONG the robot's current heading. Handy for a quick
demo: just `./row_generator_from_here.py`.

  ./row_generator_from_here.py
  ./row_generator_from_here.py --width 40 --height 30 --spacing 5 --name demo_field
  ./row_generator_from_here.py --dry-run
  ./row_generator_from_here.py --bearing 90        # override the heading

Touches:
  topic   <ns>/localization/fix                 (NavSatFix, subscribe)
  tf      <ns>/tf : map -> base_link            (heading)
  service <ns>/mission_manager/create_map       (CreateMap)
"""

from __future__ import annotations
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import (
    QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy,
)
from sensor_msgs.msg import NavSatFix
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer

from clearpath_mission_manager_msgs.srv import CreateMap
from clearpath_mission_manager_msgs.msg import MapEdgeReq
from clearpath_navigation_msgs.msg import MapPoint

from examples.common.argparse_base import make_parser
from examples.common.ros_helpers import wait_for_service, call_service
# Reuse the geometry from the fixed-coordinate version - this script only swaps
# where the centre + bearing come from.
from row_generator_square import (
    boustrophedon, offset_ll, DEFAULT_EDGE_RADIUS_M, DEFAULT_SPEED_LIMIT_M_S,
)


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    """Yaw (rotation about Z) in radians from a quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def anchor_center(lat, lon, w, h, bearing_deg, anchor):
    """boustrophedon() centres the field on (lat, lon). For anchor='start' shift
    the centre so the robot sits at the field's start corner and the rows run
    forward from it; 'center' leaves the robot in the middle."""
    if anchor == "center":
        return lat, lon
    b = math.radians(bearing_deg)
    along = (math.sin(b + math.pi / 2), math.cos(b + math.pi / 2))  # row direction
    across = (math.sin(b), math.cos(b))                            # spacing direction
    ce = along[0] * (w / 2) + across[0] * (h / 2)
    cn = along[1] * (w / 2) + across[1] * (h / 2)
    return offset_ll(lat, lon, ce, cn)


class RowGenFromHere(Node):
    def __init__(self, namespace: str, name: str, map_frame: str, base_frame: str):
        super().__init__("row_generator_from_here")
        self.map_name = name
        self.map_frame = map_frame
        self.base_frame = base_frame
        self.fix: NavSatFix | None = None
        self.create_subscription(
            NavSatFix, f"{namespace}/localization/fix", self._fix_cb, 10)
        # The robot publishes tf on <ns>/tf, not global /tf, so subscribe there
        # explicitly and feed the buffer rather than using TransformListener.
        self.tf_buffer = Buffer()
        self.create_subscription(TFMessage, f"{namespace}/tf", self._tf_cb, 10)
        tf_static_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=100,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            TFMessage, f"{namespace}/tf_static", self._tf_static_cb, tf_static_qos)
        self.srv = f"{namespace}/mission_manager/create_map"
        self.create_map_client = self.create_client(CreateMap, self.srv)

    def _fix_cb(self, msg: NavSatFix) -> None:
        if not math.isnan(msg.latitude) and not math.isnan(msg.longitude):
            self.fix = msg

    def _tf_cb(self, msg: TFMessage) -> None:
        for t in msg.transforms:
            self.tf_buffer.set_transform(t, "row_gen_from_here")

    def _tf_static_cb(self, msg: TFMessage) -> None:
        for t in msg.transforms:
            self.tf_buffer.set_transform_static(t, "row_gen_from_here")

    def _have_tf(self) -> bool:
        return self.tf_buffer.can_transform(self.map_frame, self.base_frame, Time())

    def collect(self, timeout_sec: float) -> bool:
        """Spin until we have both a GPS fix and a map->base_link transform."""
        deadline = self.get_clock().now().nanoseconds * 1e-9 + timeout_sec
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.fix is not None and self._have_tf():
                return True
            if self.get_clock().now().nanoseconds * 1e-9 > deadline:
                return False
        return False

    def heading_deg(self) -> float:
        """Robot heading in the map (ENU) frame, degrees CCW from East."""
        tfm = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        q = tfm.transform.rotation
        return math.degrees(yaw_from_quat(q.x, q.y, q.z, q.w))

    def create_map(self, pts) -> str:
        points = []
        for i, (lat, lon) in enumerate(pts):
            p = MapPoint()
            p.uuid = str(i); p.latitude = lat; p.longitude = lon
            points.append(p)
        edges = []
        for a, b in zip(points[:-1], points[1:]):
            e = MapEdgeReq()
            e.start_point_id = a.uuid
            e.end_point_id = b.uuid
            e.speed_limit = DEFAULT_SPEED_LIMIT_M_S
            e.radius = DEFAULT_EDGE_RADIUS_M
            edges.append(e)
        req = CreateMap.Request()
        req.name = self.map_name
        req.default_radius = DEFAULT_EDGE_RADIUS_M
        req.default_speed_limit = DEFAULT_SPEED_LIMIT_M_S
        req.points = points
        req.connections = edges
        resp = call_service(self, self.create_map_client, req)
        map_uuid = resp.result.uuid if resp and resp.result else None
        if not map_uuid:
            raise RuntimeError(f"CreateMap returned no uuid: {resp}")
        return map_uuid


def main(argv=None):
    parser = make_parser(doc=__doc__)
    parser.add_argument("--width", type=float, default=20.0,
                        help="Field width in metres (default 20).")
    parser.add_argument("--height", type=float, default=20.0,
                        help="Field height in metres (default 20).")
    parser.add_argument("--spacing", type=float, default=5.0,
                        help="Row spacing in metres (default 5).")
    parser.add_argument("--name", default="row_from_here", help="Map name.")
    parser.add_argument("--anchor", choices=["start", "center"], default="start",
                        help="Robot at the field's start corner (default) or its centre.")
    parser.add_argument("--bearing", type=float, default=None,
                        help="Override row bearing (deg); default = robot heading from tf.")
    parser.add_argument("--map-frame", default="map", help="Localization frame (default map).")
    parser.add_argument("--base-frame", default="base_link", help="Robot base frame (default base_link).")
    parser.add_argument("--timeout", type=float, default=15.0,
                        help="Seconds to wait for fix + tf (default 15).")
    args = parser.parse_args(argv)

    if not args.namespace:
        print("error: no namespace - set ONAV_NAMESPACE (docker/*.env) or pass --namespace",
              file=sys.stderr)
        return 2

    rclpy.init()
    node = RowGenFromHere(args.namespace, args.name, args.map_frame, args.base_frame)
    try:
        node.get_logger().info(
            f"reading current fix + heading (tf {args.map_frame}->{args.base_frame})...")
        if not node.collect(args.timeout):
            missing = []
            if node.fix is None:
                missing.append(f"{args.namespace}/localization/fix")
            if not node._have_tf():
                missing.append(f"tf {args.map_frame}->{args.base_frame}")
            node.get_logger().error(f"timed out waiting for: {', '.join(missing)}")
            return 1

        lat, lon = node.fix.latitude, node.fix.longitude
        if args.bearing is not None:
            bearing, src = args.bearing, "override"
        else:
            heading = node.heading_deg()
            # map-frame (ENU) yaw -> boustrophedon bearing; negating makes the rows
            # run ALONG the heading rather than across it.
            bearing, src = -heading, f"robot heading {heading:.1f} deg"
        clat, clon = anchor_center(lat, lon, args.width, args.height, bearing, args.anchor)
        pts = boustrophedon(clat, clon, args.width, args.height, args.spacing, bearing)
        node.get_logger().info(
            f"origin {lat:.6f},{lon:.6f} ({args.anchor})  bearing {bearing:.1f} deg [{src}]  "
            f"{args.width:g}x{args.height:g} m @ {args.spacing:g} m -> {len(pts)} waypoints")
        node.get_logger().info(f"first waypoint {pts[0][0]:.6f},{pts[0][1]:.6f}")

        if args.dry_run:
            print(f"[dry-run] {len(pts)} waypoints, map {args.name!r}, would call {node.srv}")
            return 0

        wait_for_service(node, node.create_map_client, node.srv)
        map_uuid = node.create_map(pts)
        node.get_logger().info(f"created map {args.name!r} ({map_uuid}) with {len(pts)} nodes")
        node.get_logger().info(
            f"to drive it: ./examples/missions/generate_traversal_mission.py "
            f"--map-uuid {map_uuid} --run")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main() or 0)
