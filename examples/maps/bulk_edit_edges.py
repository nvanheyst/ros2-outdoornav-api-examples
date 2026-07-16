#!/usr/bin/env python3
"""Bulk-edit every edge inside a circular zone (centre + radius) in one shot.

This is the slow-zone + bulk-path-radius tool. `--speed` sets the edge
speed limit (slow zone); `--path-radius` sets the edge path radius
(corridor widening / tightening). Either or both can be applied.

  ./bulk_edit_edges.py <map_uuid> <lat> <lon> <radius_m> --speed 0.5
  ./bulk_edit_edges.py <map_uuid> <lat> <lon> <radius_m> --path-radius 0.8
  ./bulk_edit_edges.py <map_uuid> <lat> <lon> 15 --speed 0.5 --path-radius 0.8 \
      --clone "test_map_slow_near_well"
  ./bulk_edit_edges.py <map_uuid> --around-me 15 --speed 0.3   # zone centred on robot

An edge is "in the zone" if either endpoint is within radius_m of the
centre. With `--clone`, the source map is copied first and edits land on
the clone; the original is preserved. With `--around-me`, the centre is
the robot's current GPS fix instead of an explicit lat/lon.

The `-1.0` sentinel for `speed_limit`, `radius`, and `cost_factor` in
the `UpdateMapEdges` request means "leave unchanged" - passing only
`--speed` updates speed and leaves radius/cost intact.

Touches:
  service <namespace>/mission_manager/get_map          (GetMap)
  service <namespace>/mission_manager/clone_map        (CloneMap, only with --clone)
  service <namespace>/mission_manager/update_map_edges (UpdateMapEdges)
  topic   <namespace>/localization/fix                 (NavSatFix, only with --around-me)
"""

from __future__ import annotations
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from clearpath_mission_manager_msgs.srv import GetMap, CloneMap, UpdateMapEdges

from examples.common.argparse_base import make_parser
from examples.common.config import map_id as default_map_id
from examples.common.ros_helpers import wait_for_service, call_service


EARTH_R = 6_371_000.0


def haversine_m(a, b):
    lat1, lon1 = math.radians(a[0]), math.radians(a[1])
    lat2, lon2 = math.radians(b[0]), math.radians(b[1])
    dlat, dlon = lat2 - lat1, lon2 - lon1
    h = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * EARTH_R * math.asin(math.sqrt(h))


class BulkEdit(Node):
    def __init__(self, namespace: str):
        super().__init__("bulk_edit_edges")
        self.ns = namespace
        self.fix_topic = f"{namespace}/localization/fix"
        self.get_map_srv = f"{namespace}/mission_manager/get_map"
        self.clone_srv = f"{namespace}/mission_manager/clone_map"
        self.update_srv = f"{namespace}/mission_manager/update_map_edges"
        self.get_map_client = self.create_client(GetMap, self.get_map_srv)
        self.clone_client = self.create_client(CloneMap, self.clone_srv)
        self.update_client = self.create_client(UpdateMapEdges, self.update_srv)
        self._latest_fix = None

    def wait(self, need_clone: bool) -> None:
        wait_for_service(self, self.get_map_client, self.get_map_srv)
        wait_for_service(self, self.update_client, self.update_srv)
        if need_clone:
            wait_for_service(self, self.clone_client, self.clone_srv)

    def fetch_fix(self, timeout_s: float = 10.0) -> tuple[float, float]:
        sub = self.create_subscription(
            NavSatFix, self.fix_topic, lambda m: setattr(self, "_latest_fix", m), 10,
        )
        try:
            self.get_logger().info(f"waiting for fix on {self.fix_topic}…")
            deadline = time.time() + timeout_s
            while self._latest_fix is None and time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.5)
            if self._latest_fix is None:
                raise TimeoutError(f"no fix within {timeout_s} s on {self.fix_topic}")
            m = self._latest_fix
            if math.isnan(m.latitude) or math.isnan(m.longitude):
                raise RuntimeError("fix contains NaN - robot may not have a valid fix yet")
            return (m.latitude, m.longitude)
        finally:
            self.destroy_subscription(sub)

    def fetch_map(self, map_uuid: str):
        return call_service(self, self.get_map_client, GetMap.Request(uuid=map_uuid)).map

    def clone(self, source_uuid: str, new_name: str) -> str:
        resp = call_service(
            self, self.clone_client, CloneMap.Request(uuid=source_uuid, new_name=new_name)
        )
        return resp.map.uuid

    def update(self, map_uuid: str, edge_uuids: list, speed: float, path_radius: float):
        req = UpdateMapEdges.Request()
        req.map_id = map_uuid
        req.uuids = edge_uuids
        req.radii = [path_radius] * len(edge_uuids)
        req.speed_limits = [speed] * len(edge_uuids)
        req.cost_factors = [-1.0] * len(edge_uuids)
        return call_service(self, self.update_client, req)


def edges_in_zone(map_msg, center, radius_m):
    matches = []
    for edge in map_msg.connections:
        a = (edge.start_point.latitude, edge.start_point.longitude)
        b = (edge.end_point.latitude, edge.end_point.longitude)
        if haversine_m(a, center) <= radius_m or haversine_m(b, center) <= radius_m:
            matches.append(edge.uuid)
    return matches


def main(argv=None):
    parser = make_parser(doc=__doc__)
    parser.add_argument("map_uuid", nargs="?", default=default_map_id() or None,
                        help="Map UUID (or $ONAV_MAP_ID).")
    parser.add_argument("lat", type=float, nargs="?",
                        help="Zone centre latitude (omit with --around-me).")
    parser.add_argument("lon", type=float, nargs="?",
                        help="Zone centre longitude (omit with --around-me).")
    parser.add_argument("radius_m", type=float, help="Zone radius in metres.")
    parser.add_argument("--around-me", action="store_true",
                        help="Use the robot's current GPS fix as the centre (ignores lat/lon args).")
    parser.add_argument("--speed", type=float, default=-1.0,
                        help="New edge speed limit (negative = unchanged).")
    parser.add_argument("--path-radius", type=float, default=-1.0,
                        help="New edge path radius (negative = unchanged).")
    parser.add_argument("--clone", default=None,
                        help="If set, clone the map under this name and edit the clone.")
    args = parser.parse_args(argv)

    if not args.map_uuid:
        parser.error("map_uuid is required (positional or via $ONAV_MAP_ID)")
    if not args.around_me and (args.lat is None or args.lon is None):
        parser.error("lat and lon required (or pass --around-me)")
    if args.speed < 0 and args.path_radius < 0 and not args.dry_run:
        parser.error("Need at least one of --speed / --path-radius (or --dry-run).")

    rclpy.init()
    node = BulkEdit(args.namespace)
    try:
        node.wait(need_clone=args.clone is not None)
        if args.around_me:
            lat, lon = node.fetch_fix()
            node.get_logger().info(f"centring on robot fix ({lat:.6f}, {lon:.6f})")
        else:
            lat, lon = args.lat, args.lon

        m = node.fetch_map(args.map_uuid)
        node.get_logger().info(
            f"source map {m.name!r}: {len(m.points)} pts, {len(m.connections)} edges"
        )
        target_edges = edges_in_zone(m, (lat, lon), args.radius_m)
        node.get_logger().info(
            f"{len(target_edges)} edges within {args.radius_m:.1f} m of "
            f"({lat:.6f}, {lon:.6f})"
        )
        if args.dry_run or not target_edges:
            return

        target_uuid = args.map_uuid
        if args.clone:
            target_uuid = node.clone(args.map_uuid, args.clone)
            node.get_logger().info(f"cloned to {args.clone!r} ({target_uuid})")
            m = node.fetch_map(target_uuid)
            target_edges = edges_in_zone(m, (lat, lon), args.radius_m)

        node.update(target_uuid, target_edges, args.speed, args.path_radius)
        node.get_logger().info(
            f"updated {len(target_edges)} edges"
            f"{f' speed={args.speed:.2f} m/s' if args.speed >= 0 else ''}"
            f"{f' radius={args.path_radius:.2f} m' if args.path_radius >= 0 else ''}"
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
