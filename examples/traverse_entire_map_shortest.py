"""
Visit every map node in greedy nearest-neighbour order from the current GPS fix.

  python traverse_entire_map_shortest.py            # greedy
  python traverse_entire_map_shortest.py --naive    # fetched order, for comparison

Map nodes aren't POIs, so this uses ExecuteGoTo with a synthetic Waypoint
per node rather than ExecuteGoToPOI.
"""

import math
import sys
import uuid

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import NavSatFix
from clearpath_mission_manager_msgs.srv import GetMap
from clearpath_navigation_msgs.msg import Waypoint
from clearpath_navigation_msgs.action import ExecuteGoTo


ROBOT_NAMESPACE = '/a300_00003'
SERVICE_GET_MAP     = f'{ROBOT_NAMESPACE}/mission_manager/get_map'
ACTION_EXECUTE_GOTO = f'{ROBOT_NAMESPACE}/autonomy/goto'
TOPIC_FIX           = f'{ROBOT_NAMESPACE}/localization/fix'

MAP_ID = "REPLACE_WITH_MAP_UUID"

POSITION_TOLERANCE_M = 1.0
# Negative disables the heading constraint — pass-through, no spin to match.
YAW_TOLERANCE_RAD = -1.0

EARTH_R = 6371000.0


def haversine_m(a: tuple[float, float], b: tuple[float, float]) -> float:
    lat1, lon1 = math.radians(a[0]), math.radians(a[1])
    lat2, lon2 = math.radians(b[0]), math.radians(b[1])
    dlat, dlon = lat2 - lat1, lon2 - lon1
    h = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * EARTH_R * math.asin(math.sqrt(h))


def bearing_deg(a: tuple[float, float], b: tuple[float, float]) -> float:
    lat1, lat2 = math.radians(a[0]), math.radians(b[0])
    dlon = math.radians(b[1] - a[1])
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0


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
        self.goto_client = ActionClient(self, ExecuteGoTo, ACTION_EXECUTE_GOTO)
        self._goal_handle = None

    def _fix_cb(self, msg: NavSatFix):
        if not math.isnan(msg.latitude) and not math.isnan(msg.longitude):
            self.fix = (msg.latitude, msg.longitude)

    def wait_for_inputs(self):
        while not self.get_map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for {SERVICE_GET_MAP}...')
        while not self.goto_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for {ACTION_EXECUTE_GOTO}...')
        self.get_logger().info(f'Waiting for first fix on {TOPIC_FIX}...')
        while self.fix is None:
            rclpy.spin_once(self, timeout_sec=1.0)
        self.get_logger().info(f'Current fix: {self.fix[0]:.6f}, {self.fix[1]:.6f}')

    def fetch_points(self) -> list[tuple[str, float, float]]:
        req = GetMap.Request(uuid=MAP_ID)
        future = self.get_map_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        if resp is None or not resp.map.points:
            return []
        return [(p.uuid, p.latitude, p.longitude) for p in resp.map.points]

    def go_to(self, lat: float, lon: float, heading: float = 0.0) -> bool:
        wp = Waypoint()
        wp.uuid = str(uuid.uuid4())
        wp.name = f'shortest_{wp.uuid[:8]}'
        wp.latitude = lat
        wp.longitude = lon
        wp.heading = heading
        wp.position_tolerance = POSITION_TOLERANCE_M
        wp.yaw_tolerance = YAW_TOLERANCE_RAD

        goal = ExecuteGoTo.Goal()
        goal.map_uuid = MAP_ID
        goal.waypoint = wp
        send_future = self.goto_client.send_goal_async(goal)
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
        points = self.fetch_points()
        if not points:
            self.get_logger().error('No points on map.')
            return

        coords = [(p[1], p[2]) for p in points]
        naive_order = list(range(len(points)))
        greedy_order = greedy_nearest(self.fix, coords)

        naive_len = route_length([self.fix] + [coords[i] for i in naive_order])
        greedy_len = route_length([self.fix] + [coords[i] for i in greedy_order])

        self.get_logger().info(f'Points: {len(points)}')
        self.get_logger().info(f'  naive  route: {naive_len:7.1f} m')
        savings = 100 * (1 - greedy_len / naive_len) if naive_len > 0 else 0.0
        self.get_logger().info(f'  greedy route: {greedy_len:7.1f} m  ({savings:.0f}% shorter)')

        order = naive_order if self.naive else greedy_order
        ordered_pts = [coords[i] for i in order]
        for i, idx in enumerate(order, 1):
            point_uuid, lat, lon = points[idx]
            # Face the next waypoint (last one keeps the prior bearing).
            if i < len(order):
                heading = bearing_deg(ordered_pts[i - 1], ordered_pts[i])
            elif i > 1:
                heading = bearing_deg(ordered_pts[i - 2], ordered_pts[i - 1])
            else:
                heading = 0.0
            self.get_logger().info(
                f'[{i}/{len(order)}] -> ({lat:.6f}, {lon:.6f}) hdg {heading:.0f} node={point_uuid[:8]}'
            )
            ok = self.go_to(lat, lon, heading)
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
