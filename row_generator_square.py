"""
Generate a boustrophedon (lawnmower) coverage plan over an axis-aligned
rectangular area defined by a GPS center + width + height + bearing, then push
it to OutdoorNav as a map + mission and optionally start it.

Usage:
    python row_generator_square.py
        (edit CENTER / WIDTH_M / HEIGHT_M / SPACING_M / BEARING_DEG below)
    python row_generator_square.py --run        # start the mission after creating

Flow:
    1. Build the row endpoints in local meters (offset N/E from the GPS center).
    2. Convert each endpoint back to (lat, lon) with a flat-earth approximation
       (good to centimetres at site scale).
    3. Call mission_manager/create_map with NetworkPoint + NetworkEdgeReq lists.
    4. Call mission_manager/create_mission referencing the new map.
    5. Optionally fire ExecuteMission.

Notes:
    - This is the "rectangle" form. For a polygon defined by POIs, see
      row_generator_polygon.py.
    - Edges get a `radius` (path tolerance). Tune for the field margin you
      actually need — a smaller radius forces tighter row tracking.
"""

import math
import sys
import uuid

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from clearpath_mission_manager_msgs.srv import CreateMap, CreateMission
from clearpath_mission_manager_msgs.msg import NetworkEdgeReq, MissionPlan, MissionTask
from clearpath_navigation_msgs.msg import NetworkPoint
from clearpath_navigation_msgs.action import ExecuteMission


ROBOT_NAMESPACE = '/a300_00003'
SERVICE_CREATE_MAP = f'{ROBOT_NAMESPACE}/mission_manager/create_map'
SERVICE_CREATE_MISSION = f'{ROBOT_NAMESPACE}/mission_manager/create_mission'
ACTION_EXECUTE_MISSION = f'{ROBOT_NAMESPACE}/mission_manager/execute_mission'

# -- Coverage area ---------------------------------------------------------
CENTER_LAT = 50.10940
CENTER_LON = -97.31870
WIDTH_M = 60.0
HEIGHT_M = 40.0
SPACING_M = 6.0
BEARING_DEG = 0.0   # 0 = rows run E-W, sweep N→S

MAP_NAME = 'row_coverage_square'
EDGE_RADIUS_M = 1.5
SPEED_LIMIT_M_S = 1.0


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
    def __init__(self, do_run: bool):
        super().__init__('row_generator_square')
        self.do_run = do_run
        self.create_map_client = self.create_client(CreateMap, SERVICE_CREATE_MAP)
        self.create_mission_client = self.create_client(CreateMission, SERVICE_CREATE_MISSION)
        self.execute_mission_client = ActionClient(self, ExecuteMission, ACTION_EXECUTE_MISSION)

    def wait_for_servers(self):
        for c, n in [(self.create_map_client, SERVICE_CREATE_MAP),
                     (self.create_mission_client, SERVICE_CREATE_MISSION)]:
            while not c.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for {n}...')
        if self.do_run:
            while not self.execute_mission_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for {ACTION_EXECUTE_MISSION}...')

    def create_map(self, pts: list[tuple[float, float]]) -> str:
        points = []
        for lat, lon in pts:
            p = NetworkPoint()
            p.uuid = str(uuid.uuid4())
            p.latitude = lat
            p.longitude = lon
            points.append(p)

        edges = []
        for a, b in zip(points[:-1], points[1:]):
            e = NetworkEdgeReq()
            e.start_point_id = a.uuid
            e.end_point_id = b.uuid
            e.speed_limit = SPEED_LIMIT_M_S
            e.radius = EDGE_RADIUS_M
            edges.append(e)

        req = CreateMap.Request()
        req.name = MAP_NAME
        req.points = points
        req.connections = edges
        future = self.create_map_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        map_uuid = getattr(getattr(resp, 'result', None), 'uuid', None) or getattr(resp, 'uuid', None)
        if not map_uuid:
            raise RuntimeError(f'CreateMap returned no uuid: {resp}')
        self.get_logger().info(f'Created map {MAP_NAME!r} ({map_uuid[:8]}...) with {len(points)} points.')
        return map_uuid, [p.uuid for p in points]

    def create_mission(self, map_uuid: str, point_uuids: list[str]) -> str:
        plan = MissionPlan()
        plan.tasks = []
        for pid in point_uuids:
            t = MissionTask()
            t.uuid = str(uuid.uuid4())
            t.type = 'goto_waypoint'
            t.waypoint_uuid = pid
            plan.tasks.append(t)

        req = CreateMission.Request()
        req.name = MAP_NAME
        req.map_uuid = map_uuid
        req.mission_plan = plan
        future = self.create_mission_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        mission_uuid = getattr(getattr(resp, 'result', None), 'uuid', None) or getattr(resp, 'uuid', None)
        if not mission_uuid:
            raise RuntimeError(f'CreateMission returned no uuid: {resp}')
        self.get_logger().info(f'Created mission {MAP_NAME!r} ({mission_uuid[:8]}...).')
        return mission_uuid

    def execute(self, mission_uuid: str, map_uuid: str):
        goal = ExecuteMission.Goal(mission_uuid=mission_uuid, map_uuid=map_uuid)
        send_future = self.execute_mission_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if not handle or not handle.accepted:
            self.get_logger().error('ExecuteMission rejected.')
            return
        self.get_logger().info('ExecuteMission accepted — blocking on result.')
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        status = result_future.result().status if result_future.result() else None
        self.get_logger().info(f'Mission ended with status {status}.')


def main(args=None):
    do_run = '--run' in sys.argv
    rclpy.init(args=args)
    node = RowGenSquare(do_run)
    try:
        node.wait_for_servers()
        pts = boustrophedon(CENTER_LAT, CENTER_LON, WIDTH_M, HEIGHT_M, SPACING_M, BEARING_DEG)
        node.get_logger().info(f'Generated {len(pts)} waypoints across {WIDTH_M}×{HEIGHT_M}m.')
        map_uuid, point_uuids = node.create_map(pts)
        mission_uuid = node.create_mission(map_uuid, point_uuids)
        if do_run:
            node.execute(mission_uuid, map_uuid)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
