"""
Build a runnable mission that traverses every node of an existing map.

NetworkMissions reference Waypoints (not raw map points), so this calls
CreateNetworkMission then CreateWaypoint(...assign_to=[mission_uuid]) for
each point.

  python generate_traversal_mission.py <map_uuid>
  python generate_traversal_mission.py <map_uuid> --run
  python generate_traversal_mission.py <map_uuid> --name foo --tolerance 1.5

Chain map (all degrees <= 2) is walked from an endpoint. Mesh maps use
greedy nearest-neighbour over haversine — visits every node once, doesn't
cover every edge.
"""

import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from clearpath_mission_manager_msgs.srv import GetMap, CreateNetworkMission, CreateWaypoint
from clearpath_navigation_msgs.action import ExecuteMission


ROBOT_NAMESPACE = '/a300_00003'
SERVICE_GET_MAP         = f'{ROBOT_NAMESPACE}/mission_manager/get_map'
SERVICE_CREATE_MISSION  = f'{ROBOT_NAMESPACE}/mission_manager/create_mission'
SERVICE_CREATE_WAYPOINT = f'{ROBOT_NAMESPACE}/mission_manager/create_waypoint'
ACTION_EXECUTE_MISSION  = f'{ROBOT_NAMESPACE}/autonomy/mission'

DEFAULT_POSITION_TOLERANCE_M = 1.0
# Negative disables the heading constraint — robot flows through the waypoint
# without spinning to match. Tangent heading is still set as a hint.
DEFAULT_YAW_TOLERANCE_DEG = -1.0


def haversine_m(a: tuple[float, float], b: tuple[float, float]) -> float:
    import math
    lat1, lon1 = math.radians(a[0]), math.radians(a[1])
    lat2, lon2 = math.radians(b[0]), math.radians(b[1])
    dlat, dlon = lat2 - lat1, lon2 - lon1
    h = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * 6371000.0 * math.asin(math.sqrt(h))


def bearing_deg(a: tuple[float, float], b: tuple[float, float]) -> float:
    """Initial compass bearing from a to b. 0=N, 90=E."""
    import math
    lat1, lat2 = math.radians(a[0]), math.radians(b[0])
    dlon = math.radians(b[1] - a[1])
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0


def detect_chain(points, connections) -> bool:
    """A 'chain' = every node has at most two neighbours in the connection graph."""
    deg: dict[str, int] = {}
    for c in connections:
        # MapEdge has start_point and end_point sub-messages, each with a uuid
        deg[c.start_point.uuid] = deg.get(c.start_point.uuid, 0) + 1
        deg[c.end_point.uuid]   = deg.get(c.end_point.uuid, 0) + 1
    return all(d <= 2 for d in deg.values())


def order_chain(points, connections) -> list[int]:
    """Walk the chain from a degree-1 endpoint through every point."""
    uuid_to_idx = {p.uuid: i for i, p in enumerate(points)}
    adj: dict[str, list[str]] = {p.uuid: [] for p in points}
    for c in connections:
        adj[c.start_point.uuid].append(c.end_point.uuid)
        adj[c.end_point.uuid].append(c.start_point.uuid)
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
    return [uuid_to_idx[u] for u in order]


def order_graph_walk(points, connections) -> list[int]:
    """Walk the connection graph: at each step prefer the unvisited adjacent
    node that requires the least turn from the current heading. Falls back to
    greedy haversine only when the current node has no unvisited neighbours.

    This keeps consecutive waypoints actually connected by an edge, which
    makes tangent headings meaningful (no 180-degree flips between collinear
    points the way pure nearest-neighbour produces).
    """
    uuid_to_idx = {p.uuid: i for i, p in enumerate(points)}
    coords = [(p.latitude, p.longitude) for p in points]
    adj: dict[int, list[int]] = {i: [] for i in range(len(points))}
    for c in connections:
        a = uuid_to_idx.get(c.start_point.uuid)
        b = uuid_to_idx.get(c.end_point.uuid)
        if a is not None and b is not None:
            adj[a].append(b)
            adj[b].append(a)

    visited = set()
    order = [0]
    visited.add(0)
    prev_heading = None
    while len(order) < len(points):
        here = order[-1]
        unvisited_neighbours = [n for n in adj[here] if n not in visited]
        if unvisited_neighbours:
            if prev_heading is None:
                # First step: just take the first available neighbour.
                nxt = unvisited_neighbours[0]
            else:
                def turn_cost(n):
                    b = bearing_deg(coords[here], coords[n])
                    diff = abs((b - prev_heading + 540) % 360 - 180)
                    return diff
                nxt = min(unvisited_neighbours, key=turn_cost)
        else:
            # Stuck — jump to nearest unvisited.
            remaining = [i for i in range(len(points)) if i not in visited]
            nxt = min(remaining, key=lambda i: haversine_m(coords[here], coords[i]))
        prev_heading = bearing_deg(coords[here], coords[nxt])
        order.append(nxt)
        visited.add(nxt)
    return order


class TraversalMission(Node):
    def __init__(self, map_uuid: str, mission_name: str | None, do_run: bool,
                 position_tolerance: float):
        super().__init__('generate_traversal_mission')
        self.map_uuid = map_uuid
        self.mission_name = mission_name or f'traversal_{map_uuid[:8]}'
        self.do_run = do_run
        self.position_tolerance = position_tolerance
        self.get_map_client = self.create_client(GetMap, SERVICE_GET_MAP)
        self.create_mission_client = self.create_client(CreateNetworkMission, SERVICE_CREATE_MISSION)
        self.create_waypoint_client = self.create_client(CreateWaypoint, SERVICE_CREATE_WAYPOINT)
        self.execute_client = ActionClient(self, ExecuteMission, ACTION_EXECUTE_MISSION) if do_run else None
        self._goal_handle = None

    def wait(self):
        for c, n in [(self.get_map_client, SERVICE_GET_MAP),
                     (self.create_mission_client, SERVICE_CREATE_MISSION),
                     (self.create_waypoint_client, SERVICE_CREATE_WAYPOINT)]:
            while not c.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for {n}...')
        if self.execute_client:
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

    def create_empty_mission(self) -> str:
        req = CreateNetworkMission.Request()
        req.name = self.mission_name
        req.waypoint_ids = []
        req.replan_enabled = True
        req.return_to_dock_enabled = False
        future = self.create_mission_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        mission_uuid = resp.result.uuid if resp and resp.result else None
        if not mission_uuid:
            raise RuntimeError(f'CreateNetworkMission returned no uuid: {resp}')
        self.get_logger().info(f'Mission {self.mission_name!r} created ({mission_uuid[:8]}…).')
        return mission_uuid

    def append_waypoint(self, mission_uuid: str, idx: int, lat: float, lon: float,
                        heading: float) -> str:
        req = CreateWaypoint.Request()
        req.name = f'{self.mission_name}_wp{idx:03d}'
        req.latitude = lat
        req.longitude = lon
        req.heading = heading
        req.position_tolerance = self.position_tolerance
        req.yaw_tolerance = DEFAULT_YAW_TOLERANCE_DEG
        req.task_ids = []
        req.assign_to = [mission_uuid]
        future = self.create_waypoint_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        wp_uuid = resp.result.uuid if resp and resp.result else None
        if not wp_uuid:
            raise RuntimeError(f'CreateWaypoint returned no uuid: {resp}')
        return wp_uuid

    def execute(self, mission_uuid: str):
        goal = ExecuteMission.Goal(mission_uuid=mission_uuid, map_uuid=self.map_uuid)
        send_future = self.execute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        self._goal_handle = send_future.result()
        if not self._goal_handle or not self._goal_handle.accepted:
            self.get_logger().error('ExecuteMission rejected.')
            return
        result_future = self._goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self._goal_handle = None
        status = result_future.result().status if result_future.result() else None
        self.get_logger().info(f'Mission ended with status {status} (4=SUCCEEDED).')

    def cancel_in_flight(self):
        if self._goal_handle is None:
            return
        cancel_future = self._goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future)

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
                'walking the connection graph with least-turn preference.'
            )
            order = order_graph_walk(points, connections)

        # Tangent heading: face the next waypoint. Quality depends on the
        # ordering produced above. Clean for chain maps (row generators,
        # recorded paths) and boustrophedon meshes; arbitrary mesh maps
        # with many cross-connections can still produce zigzags here
        # because the walker has many locally-equivalent choices.
        ordered_pts = [(points[idx].latitude, points[idx].longitude) for idx in order]
        headings = []
        for i, p in enumerate(ordered_pts):
            if i + 1 < len(ordered_pts):
                headings.append(bearing_deg(p, ordered_pts[i + 1]))
            elif headings:
                headings.append(headings[-1])
            else:
                headings.append(0.0)

        mission_uuid = self.create_empty_mission()
        for i, idx in enumerate(order):
            p = points[idx]
            self.append_waypoint(mission_uuid, i, p.latitude, p.longitude, headings[i])
            if (i + 1) % 10 == 0 or i + 1 == len(order):
                self.get_logger().info(f'  appended {i + 1}/{len(order)} waypoints')

        if self.do_run:
            self.execute(mission_uuid)


def main(args=None):
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    map_uuid = sys.argv[1]
    do_run = '--run' in sys.argv
    name = None
    position_tolerance = DEFAULT_POSITION_TOLERANCE_M
    if '--name' in sys.argv:
        name = sys.argv[sys.argv.index('--name') + 1]
    if '--tolerance' in sys.argv:
        position_tolerance = float(sys.argv[sys.argv.index('--tolerance') + 1])

    rclpy.init(args=args)
    node = TraversalMission(map_uuid, name, do_run, position_tolerance)
    try:
        node.wait()
        node.run()
    except KeyboardInterrupt:
        node.cancel_in_flight()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
