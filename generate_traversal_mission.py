"""
Generate a mission that traverses every point of an existing map.

Given a map UUID, this script:
    1. Pulls the map (points + connections) via mission_manager/get_map.
    2. Builds an ordering — sequential for a chain map (every node has at
       most two neighbours, e.g. row_generator_square output), or greedy
       nearest-neighbour walking the connection graph for a polygon mesh
       (row_generator_polygon output).
    3. Pushes a mission of goto_waypoint tasks via
       mission_manager/create_mission.
    4. Optionally fires ExecuteMission.

Usage:
    python generate_traversal_mission.py <map_uuid>
    python generate_traversal_mission.py <map_uuid> --run
    python generate_traversal_mission.py <map_uuid> --name "my_traversal"

Notes:
    - "Chain" detection: if no node in the connection graph has degree > 2,
      the map is treated as an ordered chain and the points are visited in
      that natural order (boustrophedon survives intact).
    - For mesh maps the greedy walk visits each node once; it does NOT
      cover every edge. For a true Chinese-postman over the boundary +
      rows, extend this script with networkx.algorithms.euler.
"""

import sys
import uuid

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from clearpath_mission_manager_msgs.srv import GetMap, CreateMission
from clearpath_mission_manager_msgs.msg import MissionPlan, MissionTask
from clearpath_navigation_msgs.action import ExecuteMission


ROBOT_NAMESPACE = '/a300_00003'
SERVICE_GET_MAP        = f'{ROBOT_NAMESPACE}/mission_manager/get_map'
SERVICE_CREATE_MISSION = f'{ROBOT_NAMESPACE}/mission_manager/create_mission'
ACTION_EXECUTE_MISSION = f'{ROBOT_NAMESPACE}/mission_manager/execute_mission'


def haversine_m(a: tuple[float, float], b: tuple[float, float]) -> float:
    import math
    lat1, lon1 = math.radians(a[0]), math.radians(a[1])
    lat2, lon2 = math.radians(b[0]), math.radians(b[1])
    dlat, dlon = lat2 - lat1, lon2 - lon1
    h = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * 6371000.0 * math.asin(math.sqrt(h))


def detect_chain(points: list, connections: list) -> bool:
    """A 'chain' = every node has at most two neighbours in the connection graph."""
    deg: dict[str, int] = {}
    for c in connections:
        deg[c.start_point_id] = deg.get(c.start_point_id, 0) + 1
        deg[c.end_point_id]   = deg.get(c.end_point_id, 0) + 1
    return all(d <= 2 for d in deg.values())


def order_chain(points: list, connections: list) -> list[str]:
    """Walk the chain from an endpoint (degree 1) through every node."""
    adj: dict[str, list[str]] = {p.uuid: [] for p in points}
    for c in connections:
        adj[c.start_point_id].append(c.end_point_id)
        adj[c.end_point_id].append(c.start_point_id)
    endpoints = [pid for pid, ns in adj.items() if len(ns) == 1]
    start = endpoints[0] if endpoints else next(iter(adj))

    order = [start]
    visited = {start}
    while True:
        nxts = [n for n in adj[order[-1]] if n not in visited]
        if not nxts:
            break
        order.append(nxts[0])
        visited.add(nxts[0])
    return order


def order_greedy(points: list) -> list[str]:
    """Nearest-neighbour walk over haversine distance, ignoring connections."""
    coords = {p.uuid: (p.latitude, p.longitude) for p in points}
    remaining = set(coords.keys())
    start = next(iter(remaining))
    order = [start]
    remaining.remove(start)
    while remaining:
        here = coords[order[-1]]
        nxt = min(remaining, key=lambda u: haversine_m(here, coords[u]))
        order.append(nxt)
        remaining.remove(nxt)
    return order


class TraversalMission(Node):
    def __init__(self, map_uuid: str, mission_name: str | None, do_run: bool):
        super().__init__('generate_traversal_mission')
        self.map_uuid = map_uuid
        self.mission_name = mission_name or f'traversal_{map_uuid[:8]}'
        self.do_run = do_run
        self.get_map_client = self.create_client(GetMap, SERVICE_GET_MAP)
        self.create_mission_client = self.create_client(CreateMission, SERVICE_CREATE_MISSION)
        self.execute_client = ActionClient(self, ExecuteMission, ACTION_EXECUTE_MISSION)

    def wait(self):
        for c, n in [(self.get_map_client, SERVICE_GET_MAP),
                     (self.create_mission_client, SERVICE_CREATE_MISSION)]:
            while not c.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for {n}...')
        if self.do_run:
            while not self.execute_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for {ACTION_EXECUTE_MISSION}...')

    def fetch_map(self):
        req = GetMap.Request(uuid=self.map_uuid)
        future = self.get_map_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        if resp is None or not resp.map.points:
            raise RuntimeError('GetMap returned no points.')
        return resp.map.points, resp.map.connections

    def create_mission(self, ordered_uuids: list[str]) -> str:
        plan = MissionPlan()
        plan.tasks = []
        for pid in ordered_uuids:
            t = MissionTask()
            t.uuid = str(uuid.uuid4())
            t.type = 'goto_waypoint'
            t.waypoint_uuid = pid
            plan.tasks.append(t)

        req = CreateMission.Request()
        req.name = self.mission_name
        req.map_uuid = self.map_uuid
        req.mission_plan = plan
        future = self.create_mission_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        mission_uuid = getattr(getattr(resp, 'result', None), 'uuid', None) or getattr(resp, 'uuid', None)
        if not mission_uuid:
            raise RuntimeError(f'CreateMission returned no uuid: {resp}')
        self.get_logger().info(
            f'Mission {self.mission_name!r} created ({mission_uuid[:8]}...) '
            f'with {len(ordered_uuids)} tasks.'
        )
        return mission_uuid

    def execute(self, mission_uuid: str):
        goal = ExecuteMission.Goal(mission_uuid=mission_uuid, map_uuid=self.map_uuid)
        send_future = self.execute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if not handle or not handle.accepted:
            self.get_logger().error('ExecuteMission rejected.')
            return
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        status = result_future.result().status if result_future.result() else None
        self.get_logger().info(f'Mission ended with status {status}.')

    def run(self):
        points, connections = self.fetch_map()
        if detect_chain(points, connections):
            self.get_logger().info(
                f'Chain map detected ({len(points)} points) — using sequential order.'
            )
            order = order_chain(points, connections)
        else:
            self.get_logger().info(
                f'Mesh map detected ({len(points)} points, {len(connections)} edges) — '
                'using greedy nearest-neighbour.'
            )
            order = order_greedy(points)
        mission_uuid = self.create_mission(order)
        if self.do_run:
            self.execute(mission_uuid)


def main(args=None):
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    map_uuid = sys.argv[1]
    do_run = '--run' in sys.argv
    name = None
    if '--name' in sys.argv:
        name = sys.argv[sys.argv.index('--name') + 1]

    rclpy.init(args=args)
    node = TraversalMission(map_uuid, name, do_run)
    try:
        node.wait()
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
