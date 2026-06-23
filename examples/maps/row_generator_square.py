#!/usr/bin/env python3
"""Boustrophedon coverage map over a rectangle (centre + w/h + bearing).

Pushes the map via mission_manager/create_map. Pair with
generate_traversal_mission.py to drive it.

  ./row_generator_square.py --lat 50.10940 --lon -97.31870 \
      --width 60 --height 40 --spacing 6 --name row_coverage_square
  ./row_generator_square.py --lat 50.1094 --lon -97.3187 --width 60 --height 40 \
      --spacing 6 --bearing 30 --dry-run

Touches: service <namespace>/mission_manager/create_map (CreateMap).
"""

from __future__ import annotations
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import rclpy
from rclpy.node import Node
from clearpath_mission_manager_msgs.srv import CreateMap
from clearpath_mission_manager_msgs.msg import MapEdgeReq
from clearpath_navigation_msgs.msg import MapPoint

from common.argparse_base import make_parser
from common.ros_helpers import wait_for_service, call_service


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
    def __init__(self, namespace: str, name: str):
        super().__init__("row_generator_square")
        self.map_name = name
        self.srv = f"{namespace}/mission_manager/create_map"
        self.create_map_client = self.create_client(CreateMap, self.srv)

    def wait(self) -> None:
        wait_for_service(self, self.create_map_client, self.srv)

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
        self.get_logger().info(
            f"created map {self.map_name!r} ({map_uuid}) with {len(points)} nodes, {len(edges)} edges"
        )
        return map_uuid


def main(argv=None):
    parser = make_parser(doc=__doc__)
    parser.add_argument("--lat", type=float, required=True, help="Centre latitude.")
    parser.add_argument("--lon", type=float, required=True, help="Centre longitude.")
    parser.add_argument("--width", type=float, required=True, help="Width in metres.")
    parser.add_argument("--height", type=float, required=True, help="Height in metres.")
    parser.add_argument("--spacing", type=float, required=True, help="Row spacing in metres.")
    parser.add_argument("--bearing", type=float, default=0.0,
                        help="Row direction in degrees (0 = E-W). Default 0.")
    parser.add_argument("--name", default="row_coverage_square", help="Map name.")
    args = parser.parse_args(argv)

    pts = boustrophedon(args.lat, args.lon, args.width, args.height, args.spacing, args.bearing)

    if args.dry_run:
        print(f"[dry-run] would generate {len(pts)} waypoints across {args.width}×{args.height} m")
        print(f"[dry-run] map name: {args.name!r}")
        print(f"[dry-run] service: {args.namespace}/mission_manager/create_map")
        return

    rclpy.init()
    node = RowGenSquare(args.namespace, args.name)
    try:
        node.wait()
        node.get_logger().info(f"generated {len(pts)} waypoints across {args.width}×{args.height} m")
        map_uuid = node.create_map(pts)
        node.get_logger().info(
            f"done. to drive it: ./examples/missions/generate_traversal_mission.py "
            f"--map-uuid {map_uuid} --run"
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
