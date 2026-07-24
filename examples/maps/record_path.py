#!/usr/bin/env python3
"""Record the robot's lat/lon while it drives, simplify, push as a map.

Subscribes to <namespace>/localization/fix and accumulates points; on Ctrl-C,
applies Ramer-Douglas-Peucker simplification and optionally pushes the result
as a new map.

  ./record_path.py                           # print points on Ctrl-C, don't push
  ./record_path.py east_perimeter            # push as map 'east_perimeter' (one-way)
  ./record_path.py east_perimeter --two-way  # bidirectional edges (forward + reverse)
  ./record_path.py east_perimeter --min-distance 0.25 --max-deviation 0.3

Drive however you like in another terminal while this runs.

Touches:
  topic   <namespace>/localization/fix          (NavSatFix, subscribe)
  service <namespace>/mission_manager/create_map (CreateMap, only when saving)
"""

from __future__ import annotations
import math
import signal
import sys
import threading
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from clearpath_mission_manager_msgs.srv import CreateMap
from clearpath_mission_manager_msgs.msg import MapEdgeReq
from clearpath_navigation_msgs.msg import MapPoint

from examples.common.argparse_base import make_parser
from examples.common.ros_helpers import wait_for_service


EDGE_RADIUS_M = 1.5
SPEED_LIMIT_M_S = 1.0


def haversine_m(a, b):
    lat1, lon1 = math.radians(a[0]), math.radians(a[1])
    lat2, lon2 = math.radians(b[0]), math.radians(b[1])
    dlat, dlon = lat2 - lat1, lon2 - lon1
    h = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * 6_371_000.0 * math.asin(math.sqrt(h))


def rdp_simplify(points, max_dev_m):
    if len(points) < 3:
        return list(points)
    lat0, lon0 = points[0]
    m_per_deg_lat = 111_320.0
    m_per_deg_lon = 111_320.0 * math.cos(math.radians(lat0))
    xy = [((p[1] - lon0) * m_per_deg_lon, (p[0] - lat0) * m_per_deg_lat) for p in points]

    def perp(i, s, e):
        x, y = xy[i]; x1, y1 = xy[s]; x2, y2 = xy[e]
        dx, dy = x2 - x1, y2 - y1
        if dx == 0 and dy == 0:
            return math.hypot(x - x1, y - y1)
        t = ((x - x1) * dx + (y - y1) * dy) / (dx * dx + dy * dy)
        t = max(0.0, min(1.0, t))
        return math.hypot(x - (x1 + t * dx), y - (y1 + t * dy))

    keep = [False] * len(points)
    keep[0] = keep[-1] = True
    stack = [(0, len(points) - 1)]
    while stack:
        s, e = stack.pop()
        if e - s < 2:
            continue
        d_max, i_max = 0.0, -1
        for i in range(s + 1, e):
            d = perp(i, s, e)
            if d > d_max:
                d_max, i_max = d, i
        if d_max > max_dev_m:
            keep[i_max] = True
            stack += [(s, i_max), (i_max, e)]
    return [points[i] for i, k in enumerate(keep) if k]


class PathRecorder(Node):
    def __init__(self, namespace: str, min_dist: float):
        super().__init__("record_path")
        self.min_dist = min_dist
        self.points: list[tuple[float, float]] = []
        self.fix_topic = f"{namespace}/localization/fix"
        self.create_map_srv = f"{namespace}/mission_manager/create_map"
        self.create_subscription(NavSatFix, self.fix_topic, self._cb, 10)
        self.create_map_client = self.create_client(CreateMap, self.create_map_srv)

    def _cb(self, msg: NavSatFix) -> None:
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        pt = (msg.latitude, msg.longitude)
        if not self.points or haversine_m(self.points[-1], pt) >= self.min_dist:
            self.points.append(pt)

    def push_as_map(self, name: str, simplified, two_way: bool = False) -> str:
        wait_for_service(self, self.create_map_client, self.create_map_srv)
        points = []
        for i, (lat, lon) in enumerate(simplified):
            p = MapPoint(); p.uuid = str(i); p.latitude = lat; p.longitude = lon
            points.append(p)
        edges = []
        for a, b in zip(points[:-1], points[1:]):
            def make_edge(src, dst):
                e = MapEdgeReq()
                e.start_point_id = src.uuid
                e.end_point_id = dst.uuid
                e.speed_limit = SPEED_LIMIT_M_S
                e.radius = EDGE_RADIUS_M
                return e
            edges.append(make_edge(a, b))
            if two_way:
                edges.append(make_edge(b, a))
        req = CreateMap.Request()
        req.name = name
        req.default_radius = EDGE_RADIUS_M
        req.default_speed_limit = SPEED_LIMIT_M_S
        req.points = points
        req.connections = edges
        future = self.create_map_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().result.uuid


def main(argv=None):
    parser = make_parser(doc=__doc__)
    parser.add_argument("map_name", nargs="?", default=None,
                        help="If set, save the simplified path as a map of this name.")
    parser.add_argument("--min-distance", type=float, default=0.5,
                        help="Minimum metres between sampled points (default 0.5).")
    parser.add_argument("--max-deviation", type=float, default=0.5,
                        help="Max RDP simplification deviation in metres (default 0.5).")
    parser.add_argument("--two-way", action="store_true",
                        help="Emit both a→b and b→a edges (bidirectional). Default: one-way chain.")
    args = parser.parse_args(argv)

    rclpy.init()
    node = PathRecorder(args.namespace, args.min_distance)
    edge_mode = "two-way" if args.two_way else "one-way"
    node.get_logger().info(
        f"recording from {node.fix_topic} [{edge_mode}]. Ctrl-C to stop"
        + (f" and save as {args.map_name!r}." if args.map_name else " (no save - will print points).")
    )

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())

    last_logged = 0
    try:
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.5)
            n = len(node.points)
            if n != last_logged and n % 10 == 0:
                node.get_logger().info(f"  {n} points...")
                last_logged = n

        node.get_logger().info(f"captured {len(node.points)} raw points")
        if len(node.points) >= 2:
            simplified = rdp_simplify(node.points, args.max_deviation)
            node.get_logger().info(
                f"simplified to {len(simplified)} (max_dev={args.max_deviation} m)"
            )
            if args.map_name and not args.dry_run:
                map_uuid = node.push_as_map(args.map_name, simplified, two_way=args.two_way)
                node.get_logger().info(
                    f"map {args.map_name!r} created ({edge_mode}): {map_uuid}"
                )
            elif args.map_name and args.dry_run:
                n_edges = (len(simplified) - 1) * (2 if args.two_way else 1)
                node.get_logger().info(
                    f"[dry-run] would create map {args.map_name!r} ({edge_mode}, "
                    f"{len(simplified)} nodes, {n_edges} edges) via {node.create_map_srv}"
                )
            else:
                for lat, lon in simplified:
                    print(f"  {lat:.6f}, {lon:.6f}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
