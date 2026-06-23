"""
Record the robot's lat/lon while it drives, simplify, push as a map.

Operator-side: subscribes to /localization/fix directly. No autonomy action
server involved, so this works on any robot publishing a NavSatFix.

  python record_path.py                     # print points on Ctrl-C, don't push
  python record_path.py east_perimeter      # push as a map named 'east_perimeter'
  python record_path.py east_perimeter --min-distance 0.25 --max-deviation 0.3

Drive however you like while this runs (joystick, OutdoorNav teleop,
drive_robot_forward.py in another shell). Ctrl-C stops the recording.
"""

import math
import signal
import sys
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from clearpath_mission_manager_msgs.srv import CreateMap
from clearpath_mission_manager_msgs.msg import MapEdgeReq
from clearpath_navigation_msgs.msg import MapPoint


ROBOT_NAMESPACE = '/a300_00003'
TOPIC_FIX = f'{ROBOT_NAMESPACE}/localization/fix'
SERVICE_CREATE_MAP = f'{ROBOT_NAMESPACE}/mission_manager/create_map'

EDGE_RADIUS_M = 1.5
SPEED_LIMIT_M_S = 1.0


def haversine_m(a, b):
    lat1, lon1 = math.radians(a[0]), math.radians(a[1])
    lat2, lon2 = math.radians(b[0]), math.radians(b[1])
    dlat, dlon = lat2 - lat1, lon2 - lon1
    h = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * 6371000.0 * math.asin(math.sqrt(h))


def rdp_simplify(points, max_dev_m):
    if len(points) < 3:
        return list(points)
    lat0, lon0 = points[0]
    m_per_deg_lat = 111320.0
    m_per_deg_lon = 111320.0 * math.cos(math.radians(lat0))
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


def parse_args(argv):
    map_name = None
    min_dist = 0.5
    max_dev = 0.5
    i = 1
    while i < len(argv):
        a = argv[i]
        if a == '--min-distance':
            min_dist = float(argv[i + 1]); i += 2
        elif a == '--max-deviation':
            max_dev = float(argv[i + 1]); i += 2
        elif not a.startswith('-') and map_name is None:
            map_name = a; i += 1
        else:
            i += 1
    return map_name, min_dist, max_dev


class PathRecorder(Node):
    def __init__(self, min_dist):
        super().__init__('record_path')
        self.min_dist = min_dist
        self.points: list[tuple[float, float]] = []
        self.create_subscription(NavSatFix, TOPIC_FIX, self._cb, 10)
        self.create_map_client = self.create_client(CreateMap, SERVICE_CREATE_MAP)

    def _cb(self, msg):
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        pt = (msg.latitude, msg.longitude)
        if not self.points or haversine_m(self.points[-1], pt) >= self.min_dist:
            self.points.append(pt)

    def push_as_map(self, name, simplified):
        while not self.create_map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for {SERVICE_CREATE_MAP}...')
        points = []
        for i, (lat, lon) in enumerate(simplified):
            p = MapPoint(); p.uuid = str(i); p.latitude = lat; p.longitude = lon
            points.append(p)
        edges = []
        for a, b in zip(points[:-1], points[1:]):
            e = MapEdgeReq()
            e.start_point_id = a.uuid
            e.end_point_id = b.uuid
            e.speed_limit = SPEED_LIMIT_M_S
            e.radius = EDGE_RADIUS_M
            edges.append(e)
        req = CreateMap.Request()
        req.name = name
        req.default_radius = EDGE_RADIUS_M
        req.default_speed_limit = SPEED_LIMIT_M_S
        req.points = points
        req.connections = edges
        future = self.create_map_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().result.uuid


def main(args=None):
    map_name, min_dist, max_dev = parse_args(sys.argv)
    rclpy.init(args=args)
    node = PathRecorder(min_dist)
    node.get_logger().info(
        f'Recording from {TOPIC_FIX}. Drive the robot. Ctrl-C to stop'
        + (f' and save as {map_name!r}.' if map_name else ' (no save — will print points).')
    )

    # Manual stop flag: we want to finish recording AND push the map before
    # rclpy is shut down. Python's default SIGINT handler raises into
    # rclpy.shutdown() before our finally block can call services.
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())

    last_logged = 0
    try:
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.5)
            n = len(node.points)
            if n != last_logged and n % 10 == 0:
                node.get_logger().info(f'  {n} points...')
                last_logged = n

        node.get_logger().info(f'Captured {len(node.points)} raw points.')
        if len(node.points) >= 2:
            simplified = rdp_simplify(node.points, max_dev)
            node.get_logger().info(f'Simplified to {len(simplified)} (max_dev={max_dev}m).')
            if map_name:
                map_uuid = node.push_as_map(map_name, simplified)
                node.get_logger().info(f'Map {map_name!r} created: {map_uuid}')
            else:
                for lat, lon in simplified:
                    print(f'  {lat:.6f}, {lon:.6f}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
