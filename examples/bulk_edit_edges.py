"""
Edit every edge in a zone (center + radius) in one shot.

  python bulk_edit_edges.py <map_uuid> <lat> <lon> <radius_m> --speed 0.5
  python bulk_edit_edges.py <map_uuid> <lat> <lon> <radius_m> --path-radius 0.8
  python bulk_edit_edges.py <map_uuid> <lat> <lon> 15 --speed 0.5 --path-radius 0.8 --clone "test_map_slow_near_well"

An edge is in the zone if either of its endpoints is within radius_m of the
centre. With --clone, the source map is copied first and the edits land on
the clone (lets you revert by switching maps).

Pass --dry-run to list the matching edges without writing anything.
"""

import math
import sys

import rclpy
from rclpy.node import Node
from clearpath_mission_manager_msgs.srv import (
    GetMap, CloneMap, UpdateMapEdges,
)


ROBOT_NAMESPACE = '/a300_00003'
SERVICE_GET_MAP        = f'{ROBOT_NAMESPACE}/mission_manager/get_map'
SERVICE_CLONE_MAP      = f'{ROBOT_NAMESPACE}/mission_manager/clone_map'
SERVICE_UPDATE_EDGES   = f'{ROBOT_NAMESPACE}/mission_manager/update_map_edges'

EARTH_R = 6371000.0


def haversine_m(a, b):
    lat1, lon1 = math.radians(a[0]), math.radians(a[1])
    lat2, lon2 = math.radians(b[0]), math.radians(b[1])
    dlat, dlon = lat2 - lat1, lon2 - lon1
    h = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * EARTH_R * math.asin(math.sqrt(h))


def parse_args(argv):
    if len(argv) < 5:
        print(__doc__)
        sys.exit(1)
    map_uuid = argv[1]
    lat = float(argv[2])
    lon = float(argv[3])
    radius_m = float(argv[4])
    speed, path_radius, clone_name, dry_run = -1.0, -1.0, None, False
    i = 5
    while i < len(argv):
        if argv[i] == '--speed':
            speed = float(argv[i + 1]); i += 2
        elif argv[i] == '--path-radius':
            path_radius = float(argv[i + 1]); i += 2
        elif argv[i] == '--clone':
            clone_name = argv[i + 1]; i += 2
        elif argv[i] == '--dry-run':
            dry_run = True; i += 1
        else:
            i += 1
    if speed < 0 and path_radius < 0 and not dry_run:
        print('Need at least one of --speed / --path-radius (or --dry-run).')
        sys.exit(1)
    return map_uuid, (lat, lon), radius_m, speed, path_radius, clone_name, dry_run


class BulkEdit(Node):
    def __init__(self):
        super().__init__('bulk_edit_edges')
        self.get_map_client = self.create_client(GetMap, SERVICE_GET_MAP)
        self.clone_client = self.create_client(CloneMap, SERVICE_CLONE_MAP)
        self.update_client = self.create_client(UpdateMapEdges, SERVICE_UPDATE_EDGES)

    def wait(self, need_clone: bool):
        for c, n in [(self.get_map_client, SERVICE_GET_MAP),
                     (self.update_client, SERVICE_UPDATE_EDGES)]:
            while not c.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for {n}...')
        if need_clone:
            while not self.clone_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for {SERVICE_CLONE_MAP}...')

    def fetch_map(self, map_uuid: str):
        req = GetMap.Request(uuid=map_uuid)
        future = self.get_map_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().map

    def clone(self, source_uuid: str, new_name: str) -> str:
        req = CloneMap.Request(uuid=source_uuid, new_name=new_name)
        future = self.clone_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().map.uuid

    def update(self, map_uuid: str, edge_uuids: list, speed: float, path_radius: float):
        req = UpdateMapEdges.Request()
        req.map_id = map_uuid
        req.uuids = edge_uuids
        req.radii = [path_radius] * len(edge_uuids)
        req.speed_limits = [speed] * len(edge_uuids)
        req.cost_factors = [-1.0] * len(edge_uuids)   # negative = leave unchanged
        future = self.update_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def edges_in_zone(map_msg, center, radius_m):
    in_zone = []
    for edge in map_msg.connections:
        a = (edge.start_point.latitude, edge.start_point.longitude)
        b = (edge.end_point.latitude, edge.end_point.longitude)
        if haversine_m(a, center) <= radius_m or haversine_m(b, center) <= radius_m:
            in_zone.append(edge.uuid)
    return in_zone


def main(args=None):
    map_uuid, center, radius_m, speed, path_radius, clone_name, dry_run = parse_args(sys.argv)
    rclpy.init(args=args)
    node = BulkEdit()
    try:
        node.wait(need_clone=clone_name is not None)
        m = node.fetch_map(map_uuid)
        node.get_logger().info(f'Source map {m.name!r}: {len(m.points)} pts, {len(m.connections)} edges.')

        target_edges = edges_in_zone(m, center, radius_m)
        node.get_logger().info(
            f'{len(target_edges)} edges within {radius_m:.1f}m of ({center[0]:.6f}, {center[1]:.6f}).'
        )
        if dry_run or not target_edges:
            return

        target_uuid = map_uuid
        if clone_name:
            target_uuid = node.clone(map_uuid, clone_name)
            node.get_logger().info(f'Cloned to {clone_name!r} ({target_uuid}).')
            m = node.fetch_map(target_uuid)
            target_edges = edges_in_zone(m, center, radius_m)

        node.update(target_uuid, target_edges, speed, path_radius)
        node.get_logger().info(
            f'Updated {len(target_edges)} edges'
            f'{f" speed={speed:.2f}m/s" if speed >= 0 else ""}'
            f'{f" radius={path_radius:.2f}m" if path_radius >= 0 else ""}.'
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
