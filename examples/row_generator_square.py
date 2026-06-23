"""
Boustrophedon coverage map over a rectangle (center + w/h + bearing).
Pushes the map via mission_manager/create_map; pair with
generate_traversal_mission.py to actually drive it.

Edit the constants block below for your field. For polygon-shaped fields
use row_generator_polygon.py.
"""

import math
import uuid

import rclpy
from rclpy.node import Node
from clearpath_mission_manager_msgs.srv import CreateMap
from clearpath_mission_manager_msgs.msg import MapEdgeReq
from clearpath_navigation_msgs.msg import MapPoint


ROBOT_NAMESPACE = '/a300_00003'
SERVICE_CREATE_MAP = f'{ROBOT_NAMESPACE}/mission_manager/create_map'

# -- Coverage area ---------------------------------------------------------
CENTER_LAT = 50.10940
CENTER_LON = -97.31870
WIDTH_M = 60.0
HEIGHT_M = 40.0
SPACING_M = 6.0
BEARING_DEG = 0.0   # 0 = rows run E-W, sweep N→S

MAP_NAME = 'row_coverage_square'
DEFAULT_EDGE_RADIUS_M = 1.5
DEFAULT_SPEED_LIMIT_M_S = 1.0


def offset_ll(lat: float, lon: float, east_m: float, north_m: float) -> tuple[float, float]:
    dlat = north_m / 111320.0
    dlon = east_m / (111320.0 * math.cos(math.radians(lat)))
    return lat + dlat, lon + dlon


def boustrophedon(center_lat, center_lon, w, h, spacing, bearing_deg) -> list[tuple[float, float]]:
    b = math.radians(bearing_deg)
    along = (math.sin(b + math.pi / 2), math.cos(b + math.pi / 2))   # (east, north)
    across = (math.sin(b), math.cos(b))

    num_rows = max(2, int(h / spacing) + 1)
    actual_spacing = h / (num_rows - 1)
    half_w, half_h = w / 2, h / 2

    pts = []
    for i in range(num_rows):
        ofs = -half_h + i * actual_spacing
        row_e = across[0] * ofs
        row_n = across[1] * ofs
        start = offset_ll(center_lat, center_lon, row_e + along[0] * -half_w, row_n + along[1] * -half_w)
        end   = offset_ll(center_lat, center_lon, row_e + along[0] *  half_w, row_n + along[1] *  half_w)
        pts.extend([start, end] if i % 2 == 0 else [end, start])
    return pts


class RowGenSquare(Node):
    def __init__(self):
        super().__init__('row_generator_square')
        self.create_map_client = self.create_client(CreateMap, SERVICE_CREATE_MAP)

    def wait(self):
        while not self.create_map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for {SERVICE_CREATE_MAP}...')

    def create_map(self, pts: list[tuple[float, float]]) -> str:
        # MapPoint UUIDs in the request are local IDs only — the server
        # regenerates them. We re-use these IDs in the connections array.
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
        req.name = MAP_NAME
        req.default_radius = DEFAULT_EDGE_RADIUS_M
        req.default_speed_limit = DEFAULT_SPEED_LIMIT_M_S
        req.points = points
        req.connections = edges
        future = self.create_map_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        map_uuid = resp.result.uuid if resp and resp.result else None
        if not map_uuid:
            raise RuntimeError(f'CreateMap returned no uuid: {resp}')
        self.get_logger().info(
            f'Created map {MAP_NAME!r} ({map_uuid}) with {len(points)} nodes, {len(edges)} edges.'
        )
        return map_uuid


def main(args=None):
    rclpy.init(args=args)
    node = RowGenSquare()
    try:
        node.wait()
        pts = boustrophedon(CENTER_LAT, CENTER_LON, WIDTH_M, HEIGHT_M, SPACING_M, BEARING_DEG)
        node.get_logger().info(f'Generated {len(pts)} waypoints across {WIDTH_M}×{HEIGHT_M}m.')
        map_uuid = node.create_map(pts)
        node.get_logger().info(
            f'Done. To drive it:  python generate_traversal_mission.py {map_uuid} --run'
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
