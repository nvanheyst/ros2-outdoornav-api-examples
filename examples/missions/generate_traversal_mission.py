#!/usr/bin/env python3
"""Build a runnable mission that traverses every node of an existing map.

NetworkMissions reference Waypoints (not raw map points), so this calls
CreateNetworkMission then CreateWaypoint(..., assign_to=[mission_uuid]) for
each point.

  ./generate_traversal_mission.py
  ./generate_traversal_mission.py --run
  ./generate_traversal_mission.py --name foo --tolerance 1.5
  ./generate_traversal_mission.py --map-uuid <uuid>   # skip the map menu

Chain map (all node degrees ≤ 2) walks from a degree-1 endpoint. Mesh maps
use a least-turn graph walk over the connection graph.

Compare with `traverse_entire_map_gotos.py`: that one walks the nodes
ad-hoc via ExecuteGoTo (no mission stored, free-GPS path, greedy NN
from current fix). This one is the persistent, edge-following version:
the resulting NetworkMission is re-runnable from the UI and the robot
sticks to the network corridors.

Touches:
  service <namespace>/mission_manager/get_map           (GetMap)
  service <namespace>/mission_manager/create_mission    (CreateNetworkMission)
  service <namespace>/mission_manager/create_waypoint   (CreateWaypoint)
  action  <namespace>/autonomy/mission                  (ExecuteMission, only with --run)
"""

from __future__ import annotations
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from clearpath_mission_manager_msgs.srv import GetMap, GetAllMaps, CreateNetworkMission, CreateWaypoint
from clearpath_navigation_msgs.action import ExecuteMission

from examples.common.argparse_base import make_parser
from examples.common.config import map_id as default_map_id
from examples.common.ros_helpers import wait_for_service, wait_for_action, call_service
from examples.common.onav import select_map


DEFAULT_POSITION_TOLERANCE_M = 1.0
# Negative disables the heading constraint - robot flows through the waypoint
# without spinning to match. Tangent heading is still set as a hint.
DEFAULT_YAW_TOLERANCE_DEG = -1.0


def haversine_m(a, b):
    lat1, lon1 = math.radians(a[0]), math.radians(a[1])
    lat2, lon2 = math.radians(b[0]), math.radians(b[1])
    dlat, dlon = lat2 - lat1, lon2 - lon1
    h = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * 6_371_000.0 * math.asin(math.sqrt(h))


def bearing_deg(a, b):
    """Initial compass bearing from a to b. 0=N, 90=E."""
    lat1, lat2 = math.radians(a[0]), math.radians(b[0])
    dlon = math.radians(b[1] - a[1])
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0


def detect_chain(points, connections) -> bool:
    deg: dict[str, int] = {}
    for c in connections:
        deg[c.start_point.uuid] = deg.get(c.start_point.uuid, 0) + 1
        deg[c.end_point.uuid] = deg.get(c.end_point.uuid, 0) + 1
    return all(d <= 2 for d in deg.values())


def order_chain(points, connections) -> list[int]:
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
    """Walk the connection graph preferring the least-turn neighbour."""
    uuid_to_idx = {p.uuid: i for i, p in enumerate(points)}
    coords = [(p.latitude, p.longitude) for p in points]
    adj: dict[int, list[int]] = {i: [] for i in range(len(points))}
    for c in connections:
        a = uuid_to_idx.get(c.start_point.uuid)
        b = uuid_to_idx.get(c.end_point.uuid)
        if a is not None and b is not None:
            adj[a].append(b)
            adj[b].append(a)

    visited = {0}
    order = [0]
    prev_heading = None
    while len(order) < len(points):
        here = order[-1]
        unvisited = [n for n in adj[here] if n not in visited]
        if unvisited:
            if prev_heading is None:
                nxt = unvisited[0]
            else:
                def turn_cost(n):
                    b = bearing_deg(coords[here], coords[n])
                    return abs((b - prev_heading + 540) % 360 - 180)
                nxt = min(unvisited, key=turn_cost)
        else:
            remaining = [i for i in range(len(points)) if i not in visited]
            nxt = min(remaining, key=lambda i: haversine_m(coords[here], coords[i]))
        prev_heading = bearing_deg(coords[here], coords[nxt])
        order.append(nxt)
        visited.add(nxt)
    return order


class TraversalMission(Node):
    def __init__(self, namespace: str, mission_name: str, do_run: bool,
                 position_tolerance: float):
        super().__init__("generate_traversal_mission")
        self.map_uuid = ""
        self.mission_name = mission_name
        self.do_run = do_run
        self.position_tolerance = position_tolerance
        self.get_map_srv = f"{namespace}/mission_manager/get_map"
        self.maps_srv = f"{namespace}/mission_manager/get_all_maps"
        self.create_mission_srv = f"{namespace}/mission_manager/create_mission"
        self.create_waypoint_srv = f"{namespace}/mission_manager/create_waypoint"
        self.mission_action = f"{namespace}/autonomy/mission"
        self.get_map_client = self.create_client(GetMap, self.get_map_srv)
        self.maps_client = self.create_client(GetAllMaps, self.maps_srv)
        self.create_mission_client = self.create_client(CreateNetworkMission, self.create_mission_srv)
        self.create_waypoint_client = self.create_client(CreateWaypoint, self.create_waypoint_srv)
        self.execute_client = ActionClient(self, ExecuteMission, self.mission_action) if do_run else None
        self._goal_handle = None

    def wait(self) -> None:
        wait_for_service(self, self.get_map_client, self.get_map_srv)
        wait_for_service(self, self.maps_client, self.maps_srv)
        wait_for_service(self, self.create_mission_client, self.create_mission_srv)
        wait_for_service(self, self.create_waypoint_client, self.create_waypoint_srv)
        if self.execute_client:
            wait_for_action(self, self.execute_client, self.mission_action)

    def fetch_map(self):
        resp = call_service(self, self.get_map_client, GetMap.Request(uuid=self.map_uuid))
        if resp is None or not resp.map.points:
            raise RuntimeError("GetMap returned no points.")
        return resp.map.points, resp.map.connections

    def create_empty_mission(self) -> str:
        req = CreateNetworkMission.Request()
        req.name = self.mission_name
        req.waypoint_ids = []
        req.replan_enabled = True
        req.return_to_dock_enabled = False
        resp = call_service(self, self.create_mission_client, req)
        mission_uuid = resp.result.uuid if resp and resp.result else None
        if not mission_uuid:
            raise RuntimeError(f"CreateNetworkMission returned no uuid: {resp}")
        self.get_logger().info(f"mission {self.mission_name!r} created ({mission_uuid[:8]}…)")
        return mission_uuid

    def append_waypoint(self, mission_uuid: str, idx: int, lat: float, lon: float,
                        heading: float) -> str:
        req = CreateWaypoint.Request()
        req.name = f"{self.mission_name}_wp{idx:03d}"
        req.latitude = lat
        req.longitude = lon
        req.heading = heading
        req.position_tolerance = self.position_tolerance
        req.yaw_tolerance = DEFAULT_YAW_TOLERANCE_DEG
        req.task_ids = []
        req.assign_to = [mission_uuid]
        resp = call_service(self, self.create_waypoint_client, req)
        wp_uuid = resp.result.uuid if resp and resp.result else None
        if not wp_uuid:
            raise RuntimeError(f"CreateWaypoint returned no uuid: {resp}")
        return wp_uuid

    def execute(self, mission_uuid: str) -> None:
        goal = ExecuteMission.Goal(mission_uuid=mission_uuid, map_uuid=self.map_uuid)
        send_future = self.execute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        self._goal_handle = send_future.result()
        if not self._goal_handle or not self._goal_handle.accepted:
            self.get_logger().error("ExecuteMission rejected")
            return
        result_future = self._goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self._goal_handle = None
        status = result_future.result().status if result_future.result() else None
        self.get_logger().info(f"mission ended with status {status} (4=SUCCEEDED)")

    def cancel_in_flight(self) -> None:
        if self._goal_handle is None:
            return
        cancel_future = self._goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future)

    def run(self) -> None:
        points, connections = self.fetch_map()
        if detect_chain(points, connections):
            self.get_logger().info(
                f"chain map detected ({len(points)} points) - using sequential order"
            )
            order = order_chain(points, connections)
        else:
            self.get_logger().info(
                f"mesh map detected ({len(points)} points, {len(connections)} edges) - "
                "walking the connection graph with least-turn preference"
            )
            order = order_graph_walk(points, connections)

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
                self.get_logger().info(f"  appended {i + 1}/{len(order)} waypoints")

        if self.do_run:
            self.execute(mission_uuid)


def main(argv=None):
    parser = make_parser(doc=__doc__)
    parser.add_argument("--map-uuid", default=default_map_id() or None,
                        help="Map UUID (or $ONAV_MAP_ID).")
    parser.add_argument("--name", default=None, help="Mission name (default: traversal_<map-id-prefix>).")
    parser.add_argument("--tolerance", type=float, default=DEFAULT_POSITION_TOLERANCE_M,
                        help=f"Position tolerance in metres (default {DEFAULT_POSITION_TOLERANCE_M}).")
    parser.add_argument("--run", action="store_true",
                        help="Also fire ExecuteMission after creating the mission.")
    args = parser.parse_args(argv)

    if args.dry_run:
        print(f"[dry-run] would build traversal mission over selected map")
        print(f"[dry-run] services: get_map, create_mission, create_waypoint under {args.namespace}/mission_manager/")
        if args.run:
            print(f"[dry-run] would then fire ExecuteMission on {args.namespace}/autonomy/mission")
        return

    rclpy.init()
    node = TraversalMission(args.namespace, args.name or "", args.run, args.tolerance)
    try:
        node.wait()
        node.map_uuid, map_name = select_map(node, node.maps_client, args.map_uuid or "")
        if not node.mission_name:
            node.mission_name = f"traversal_{map_name}"
        node.run()
    except KeyboardInterrupt:
        node.cancel_in_flight()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
