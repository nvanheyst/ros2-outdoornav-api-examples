"""
Visit every POI on a map in nearest-neighbor order (greedy shortest-path).

Usage:
    python traverse_entire_map_shortest.py            # start from current GPS fix
    python traverse_entire_map_shortest.py --naive    # visit in fetched order
                                                       (baseline comparison)

Flow:
    1. Call mission_manager/get_map to fetch all POIs.
    2. Get current GPS via /navsat/fix (sensor_msgs/NavSatFix).
    3. Build a visit order:
       - greedy: repeatedly hop to the nearest unvisited POI;
       - naive: keep the order returned by the service.
    4. Print both route lengths so the operator can see the savings.
    5. Drive the chosen order via ExecuteNetworkGoToPOI, one POI at a time.

Notes:
    - Distance is haversine over (lat, lon) — close enough at site scale.
    - Greedy nearest-neighbor is not optimal TSP, but it is O(n^2) and
      reliably beats arbitrary order. For maps with <30 POIs the difference
      is usually 30-50%, which is a useful gut-check vs naive.
"""

import sys
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import NavSatFix
from clearpath_mission_manager_msgs.srv import GetMap
from clearpath_navigation_msgs.action import ExecuteNetworkGoToPOI as GoToPoi


ROBOT_NAMESPACE = '/a300_00003'
SERVICE_GET_MAP = f'{ROBOT_NAMESPACE}/mission_manager/get_map'
ACTION_GO_TO_POI = f'{ROBOT_NAMESPACE}/autonomy/network_goto_poi'
TOPIC_FIX = f'{ROBOT_NAMESPACE}/sensors/gps_0/fix'

MAP_ID = 'REPLACE_WITH_MAP_UUID'

EARTH_R = 6371000.0


def haversine_m(a: tuple[float, float], b: tuple[float, float]) -> float:
    lat1, lon1 = math.radians(a[0]), math.radians(a[1])
    lat2, lon2 = math.radians(b[0]), math.radians(b[1])
    dlat, dlon = lat2 - lat1, lon2 - lon1
    h = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * EARTH_R * math.asin(math.sqrt(h))


def route_length(order: list[tuple[float, float]]) -> float:
    return sum(haversine_m(order[i], order[i + 1]) for i in range(len(order) - 1))


def greedy_nearest(start: tuple[float, float], pts: list[tuple[float, float]]) -> list[int]:
    remaining = list(range(len(pts)))
    here = start
    order: list[int] = []
    while remaining:
        nxt = min(remaining, key=lambda i: haversine_m(here, pts[i]))
        order.append(nxt)
        here = pts[nxt]
        remaining.remove(nxt)
    return order


class TraverseShortest(Node):
    def __init__(self, naive: bool):
        super().__init__('traverse_shortest')
        self.naive = naive
        self.fix: tuple[float, float] | None = None
        self.create_subscription(NavSatFix, TOPIC_FIX, self._fix_cb, 10)
        self.get_map_client = self.create_client(GetMap, SERVICE_GET_MAP)
        self.poi_client = ActionClient(self, GoToPoi, ACTION_GO_TO_POI)
        self._goal_handle = None

    def _fix_cb(self, msg: NavSatFix):
        if not math.isnan(msg.latitude) and not math.isnan(msg.longitude):
            self.fix = (msg.latitude, msg.longitude)

    def wait_for_inputs(self):
        while not self.get_map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for {SERVICE_GET_MAP}...')
        while not self.poi_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for {ACTION_GO_TO_POI}...')
        self.get_logger().info(f'Waiting for first fix on {TOPIC_FIX}...')
        while self.fix is None:
            rclpy.spin_once(self, timeout_sec=1.0)
        self.get_logger().info(f'Current fix: {self.fix[0]:.6f}, {self.fix[1]:.6f}')

    def fetch_pois(self) -> list[tuple[str, str, float, float]]:
        req = GetMap.Request(uuid=MAP_ID)
        future = self.get_map_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        if resp is None or not resp.map.points:
            return []
        return [
            (p.uuid, p.name, p.latitude, p.longitude)
            for p in resp.map.points if p.name
        ]

    def go_to(self, uuid: str) -> bool:
        goal = GoToPoi.Goal(poi_uuid=uuid, map_uuid=MAP_ID)
        send_future = self.poi_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        self._goal_handle = send_future.result()
        if not self._goal_handle or not self._goal_handle.accepted:
            return False
        result_future = self._goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self._goal_handle = None
        status = result_future.result().status if result_future.result() else None
        return status == 4

    def cancel_in_flight(self):
        if self._goal_handle is None:
            return
        cancel_future = self._goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future)

    def run(self):
        pois = self.fetch_pois()
        if not pois:
            self.get_logger().error('No POIs on map.')
            return

        coords = [(p[2], p[3]) for p in pois]
        naive_order = list(range(len(pois)))
        greedy_order = greedy_nearest(self.fix, coords)

        naive_len = route_length([self.fix] + [coords[i] for i in naive_order])
        greedy_len = route_length([self.fix] + [coords[i] for i in greedy_order])

        self.get_logger().info(f'POIs: {len(pois)}')
        self.get_logger().info(f'  naive  route: {naive_len:7.1f} m')
        self.get_logger().info(f'  greedy route: {greedy_len:7.1f} m'
                               f'  ({100 * (1 - greedy_len / naive_len):.0f}% shorter)')

        order = naive_order if self.naive else greedy_order
        for i, idx in enumerate(order, 1):
            uuid, name, _, _ = pois[idx]
            self.get_logger().info(f'[{i}/{len(order)}] -> {name!r}')
            ok = self.go_to(uuid)
            self.get_logger().info(f'    {"OK" if ok else "FAILED"}')


def main(args=None):
    naive = '--naive' in sys.argv
    rclpy.init(args=args)
    node = TraverseShortest(naive=naive)
    try:
        node.wait_for_inputs()
        node.run()
    except KeyboardInterrupt:
        node.cancel_in_flight()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
