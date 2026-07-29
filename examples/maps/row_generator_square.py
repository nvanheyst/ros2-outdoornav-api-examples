#!/usr/bin/env python3
"""Boustrophedon coverage map over a rectangle. Three ways to specify the zone:

  # explicit coordinates (lat/lon of the centre)
  ./row_generator_square.py --lat 50.10940 --lon -97.31870 \
      --width 60 --height 40 --spacing 6

  # at the robot's current position and heading (no coordinates needed)
  ./row_generator_square.py --around-me --width 60 --height 40 --spacing 6

  # two POIs tagged 'sq-field' mark opposite corners; spacing required
  ./row_generator_square.py --poi-tag sq-field --spacing 5

Exactly one of --lat/--lon, --around-me, or --poi-tag must be given.
`--bearing` overrides the row direction for any mode (default: 0 = E-W rows,
N-S traversal; --around-me uses the robot's heading unless overridden).

One-way chain edges. Pair with generate_traversal_mission.py --run to drive it.
Compare with row_generator_polygon.py: POI-driven with two-way edges and a
perimeter border — better for irregular shapes. Use this one for rectangles.

Touches:
  topic   <ns>/localization/fix                          (NavSatFix, --around-me)
  tf      <ns>/tf : map -> base_link                     (heading, --around-me)
  service <ns>/mission_manager/get_all_points_of_interest (GetAllPointsOfInterest, --poi-tag)
  service <ns>/mission_manager/create_map                (CreateMap)
"""

from __future__ import annotations
import math
import sys
import time
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
from clearpath_mission_manager_msgs.srv import CreateMap, GetAllPointsOfInterest
from clearpath_mission_manager_msgs.msg import MapEdgeReq
from clearpath_navigation_msgs.msg import MapPoint

from examples.common.argparse_base import make_parser
from examples.common.ros_helpers import wait_for_service, call_service
from examples.common.geo import haversine_m, yaw_from_quat


DEFAULT_EDGE_RADIUS_M = 1.5
DEFAULT_SPEED_LIMIT_M_S = 1.0


def offset_ll(lat: float, lon: float, east_m: float, north_m: float) -> tuple[float, float]:
    dlat = north_m / 111_320.0
    dlon = east_m / (111_320.0 * math.cos(math.radians(lat)))
    return lat + dlat, lon + dlon


def boustrophedon(center_lat, center_lon, w, h, spacing, bearing_deg):
    b = math.radians(bearing_deg)
    along = (math.sin(b + math.pi / 2), math.cos(b + math.pi / 2))
    across = (math.sin(b), math.cos(b))
    num_rows = max(2, int(h / spacing) + 1)
    actual_spacing = h / (num_rows - 1)
    half_w, half_h = w / 2, h / 2
    pts = []
    for i in range(num_rows):
        ofs = -half_h + i * actual_spacing
        row_e = across[0] * ofs
        row_n = across[1] * ofs
        start = offset_ll(center_lat, center_lon,
                          row_e + along[0] * -half_w, row_n + along[1] * -half_w)
        end = offset_ll(center_lat, center_lon,
                        row_e + along[0] * half_w, row_n + along[1] * half_w)
        pts.extend([start, end] if i % 2 == 0 else [end, start])
    return pts


class RowGenSquare(Node):
    def __init__(self, namespace: str, name: str, mode: str,
                 map_frame: str = "map", base_frame: str = "base_link"):
        super().__init__("row_generator_square")
        self.map_name = name
        self.mode = mode
        self.map_frame = map_frame
        self.base_frame = base_frame
        self._fix: NavSatFix | None = None

        self.create_srv = f"{namespace}/mission_manager/create_map"
        self.create_map_client = self.create_client(CreateMap, self.create_srv)

        self.pois_client = None
        self.tf_buffer = None

        if mode == "around_me":
            self.create_subscription(
                NavSatFix, f"{namespace}/localization/fix", self._fix_cb, 10)
            self.tf_buffer = Buffer()
            self.create_subscription(TFMessage, f"{namespace}/tf", self._tf_cb, 10)
            tf_static_qos = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST, depth=100,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(
                TFMessage, f"{namespace}/tf_static", self._tf_static_cb, tf_static_qos)

        if mode == "poi_tag":
            self.pois_srv = f"{namespace}/mission_manager/get_all_points_of_interest"
            self.pois_client = self.create_client(GetAllPointsOfInterest, self.pois_srv)

    def _fix_cb(self, msg: NavSatFix) -> None:
        if not math.isnan(msg.latitude) and not math.isnan(msg.longitude):
            self._fix = msg

    def _tf_cb(self, msg: TFMessage) -> None:
        for t in msg.transforms:
            self.tf_buffer.set_transform(t, "row_gen_square")

    def _tf_static_cb(self, msg: TFMessage) -> None:
        for t in msg.transforms:
            self.tf_buffer.set_transform_static(t, "row_gen_square")

    def _have_tf(self) -> bool:
        return self.tf_buffer.can_transform(self.map_frame, self.base_frame, Time())

    def wait(self) -> None:
        wait_for_service(self, self.create_map_client, self.create_srv)
        if self.pois_client:
            wait_for_service(self, self.pois_client, self.pois_srv)

    def collect_fix_and_tf(self, timeout_s: float) -> bool:
        deadline = time.time() + timeout_s
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self._fix is not None and self._have_tf():
                return True
        return False

    def heading_deg(self) -> float:
        tfm = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        q = tfm.transform.rotation
        return math.degrees(yaw_from_quat(q.x, q.y, q.z, q.w))

    def fetch_poi_corners(self, tag: str) -> tuple[tuple, tuple]:
        """Return two (lat, lon) pairs for POIs carrying `tag`. Raises if not exactly 2."""
        resp = call_service(self, self.pois_client, GetAllPointsOfInterest.Request())
        all_pois = getattr(resp, "points_of_interest", None) or []
        matched = [p for p in all_pois if tag in (p.tags or [])]
        if len(matched) != 2:
            raise RuntimeError(
                f"--poi-tag {tag!r}: need exactly 2 POIs with that tag, found {len(matched)}"
                f" — pass --lat/--lon or --around-me to skip POI lookup"
            )
        return (matched[0].latitude, matched[0].longitude), \
               (matched[1].latitude, matched[1].longitude)

    def create_map(self, pts: list) -> str:
        points = []
        for i, (lat, lon) in enumerate(pts):
            p = MapPoint()
            p.uuid = str(i)
            p.latitude = lat
            p.longitude = lon
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
        self.get_logger().info(
            f"created map {self.map_name!r} ({map_uuid}) "
            f"with {len(points)} nodes, {len(edges)} edges"
        )
        return map_uuid


def main(argv=None):
    parser = make_parser(doc=__doc__)

    # Geometry mode (mutually exclusive)
    mode_group = parser.add_mutually_exclusive_group()
    mode_group.add_argument("--around-me", action="store_true",
                            help="Use the robot's current GPS fix + heading as the zone centre.")
    mode_group.add_argument("--poi-tag",
                            help="Tag string shared by exactly 2 POIs that mark opposite corners.")

    # Explicit coord mode
    parser.add_argument("--lat", type=float, default=None,
                        help="Zone centre latitude (--lat/--lon mode).")
    parser.add_argument("--lon", type=float, default=None,
                        help="Zone centre longitude (--lat/--lon mode).")

    # Dimensions — required for lat/lon and around-me; derived from POIs in poi-tag
    parser.add_argument("--width", type=float, default=None,
                        help="Width in metres (E-W when bearing=0).")
    parser.add_argument("--height", type=float, default=None,
                        help="Height in metres (N-S when bearing=0).")
    parser.add_argument("--spacing", type=float, required=True,
                        help="Row spacing in metres.")
    parser.add_argument("--bearing", type=float, default=None,
                        help="Row direction in degrees (default: 0 for lat/lon + poi-tag, "
                             "robot heading for --around-me). 0 = E-W rows.")
    parser.add_argument("--name", default="row_coverage_square", help="Map name.")
    parser.add_argument("--anchor", choices=["start", "center"], default="start",
                        help="(--around-me only) robot at field start corner or centre.")
    parser.add_argument("--map-frame", default="map",
                        help="(--around-me only) localisation frame.")
    parser.add_argument("--base-frame", default="base_link",
                        help="(--around-me only) robot base frame.")
    parser.add_argument("--timeout", type=float, default=15.0,
                        help="(--around-me only) seconds to wait for fix + tf.")
    args = parser.parse_args(argv)

    # Determine mode and validate
    if args.around_me:
        mode = "around_me"
        if args.width is None or args.height is None:
            parser.error("--around-me requires --width and --height")
    elif args.poi_tag:
        mode = "poi_tag"
        # width/height inferred from POIs; warn if user passed them
    else:
        if args.lat is None or args.lon is None:
            parser.error("specify exactly one of --lat/--lon, --around-me, or --poi-tag")
        mode = "lat_lon"
        if args.width is None or args.height is None:
            parser.error("--lat/--lon mode requires --width and --height")

    if args.dry_run:
        ns = args.namespace
        if mode == "lat_lon":
            bearing = args.bearing or 0.0
            pts = boustrophedon(args.lat, args.lon, args.width, args.height,
                                args.spacing, bearing)
            print(f"[dry-run] {len(pts)} waypoints, "
                  f"{args.width}x{args.height} m @ {args.spacing} m spacing, "
                  f"bearing {bearing} deg")
        elif mode == "around_me":
            print(f"[dry-run] would read fix from {ns}/localization/fix "
                  f"and heading from tf {args.map_frame}->{args.base_frame}")
            print(f"[dry-run] {args.width}x{args.height} m @ {args.spacing} m spacing")
        else:
            print(f"[dry-run] would fetch POIs tagged {args.poi_tag!r} from "
                  f"{ns}/mission_manager/get_all_points_of_interest")
            print(f"[dry-run] bounding box from 2 matching POIs @ {args.spacing} m spacing")
        print(f"[dry-run] map name: {args.name!r}")
        print(f"[dry-run] service: {ns}/mission_manager/create_map")
        return

    rclpy.init()
    node = RowGenSquare(args.namespace, args.name, mode, args.map_frame, args.base_frame)
    try:
        node.wait()

        if mode == "lat_lon":
            lat, lon = args.lat, args.lon
            bearing = args.bearing if args.bearing is not None else 0.0
            width, height = args.width, args.height
            node.get_logger().info(
                f"centre ({lat:.6f}, {lon:.6f})  bearing {bearing:.1f} deg  "
                f"{width:g}x{height:g} m @ {args.spacing:g} m"
            )

        elif mode == "around_me":
            node.get_logger().info(
                f"collecting fix and tf {args.map_frame}->{args.base_frame} …")
            if not node.collect_fix_and_tf(args.timeout):
                missing = []
                if node._fix is None:
                    missing.append(f"{args.namespace}/localization/fix")
                if not node._have_tf():
                    missing.append(f"tf {args.map_frame}->{args.base_frame}")
                node.get_logger().error(f"timed out waiting for: {', '.join(missing)}")
                return 1
            lat, lon = node._fix.latitude, node._fix.longitude
            if args.bearing is not None:
                bearing = args.bearing
                src = "override"
            else:
                heading = node.heading_deg()
                bearing = -heading
                src = f"robot heading {heading:.1f} deg"
            width, height = args.width, args.height
            # shift centre so robot sits at the start corner when anchor='start'
            if args.anchor == "start":
                b = math.radians(bearing)
                along = (math.sin(b + math.pi / 2), math.cos(b + math.pi / 2))
                across = (math.sin(b), math.cos(b))
                lat, lon = offset_ll(lat, lon,
                                     along[0] * (width / 2) + across[0] * (height / 2),
                                     along[1] * (width / 2) + across[1] * (height / 2))
            node.get_logger().info(
                f"origin ({node._fix.latitude:.6f}, {node._fix.longitude:.6f}) "
                f"[{args.anchor}]  bearing {bearing:.1f} deg [{src}]  "
                f"{width:g}x{height:g} m @ {args.spacing:g} m"
            )

        else:  # poi_tag
            c1, c2 = node.fetch_poi_corners(args.poi_tag)
            lat = (c1[0] + c2[0]) / 2
            lon = (c1[1] + c2[1]) / 2
            # axis-aligned bounding box
            width = haversine_m((lat, c1[1]), (lat, c2[1]))
            height = haversine_m((c1[0], lon), (c2[0], lon))
            bearing = args.bearing if args.bearing is not None else 0.0
            node.get_logger().info(
                f"POI corners: {c1} and {c2}")
            node.get_logger().info(
                f"centre ({lat:.6f}, {lon:.6f})  "
                f"{width:.1f}x{height:.1f} m @ {args.spacing:g} m  bearing {bearing:.1f} deg"
            )

        pts = boustrophedon(lat, lon, width, height, args.spacing, bearing)
        node.get_logger().info(f"generated {len(pts)} waypoints")
        map_uuid = node.create_map(pts)
        node.get_logger().info(
            f"to drive it: ./examples/missions/generate_traversal_mission.py "
            f"--map-uuid {map_uuid} --run"
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
