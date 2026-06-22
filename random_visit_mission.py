"""
Loop random GoTo destinations inside a map's bounding box, forever or N times.

Usage:
    python random_visit_mission.py                            # 10 hops, default map
    python random_visit_mission.py 20                         # 20 hops
    python random_visit_mission.py 0 42                       # infinite, seed=42
    python random_visit_mission.py --map-uuid <uuid> 5

Flow:
    1. mission_manager/get_map → all points.
    2. Compute the lat/lon bounding box from those points.
    3. Loop:
       a. Roll a random (lat, lon) inside the box.
       b. Fire ExecuteGoTo with that pose.
       c. Block on the result, log success/failure, repeat.
    4. Ctrl-C cancels the in-flight goto cleanly.

Notes:
    - Uses ExecuteGoTo (free GPS pose), NOT GoToPOI. The destinations are
      random — they have nothing to do with whatever POIs the map has.
    - The bounding box is just for picking a plausible target inside the
      mapped area; OutdoorNav still runs its own drivability checks and
      will reject targets it can't reach.
    - Set count=0 to loop until interrupted.
"""

import math
import random
import sys
import uuid

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from clearpath_mission_manager_msgs.srv import GetMap
from clearpath_navigation_msgs.msg import Waypoint
from clearpath_navigation_msgs.action import ExecuteGoTo


ROBOT_NAMESPACE = '/a300_00003'
SERVICE_GET_MAP   = f'{ROBOT_NAMESPACE}/mission_manager/get_map'
ACTION_EXECUTE_GOTO = f'{ROBOT_NAMESPACE}/autonomy/goto'

DEFAULT_MAP_ID = 'REPLACE_WITH_MAP_UUID'

POSITION_TOLERANCE_M = 1.0
YAW_TOLERANCE_RAD = 0.4


def bbox(points) -> tuple[float, float, float, float]:
    lats = [p.latitude for p in points]
    lons = [p.longitude for p in points]
    return min(lats), max(lats), min(lons), max(lons)


def haversine_m(a: tuple[float, float], b: tuple[float, float]) -> float:
    lat1, lon1 = math.radians(a[0]), math.radians(a[1])
    lat2, lon2 = math.radians(b[0]), math.radians(b[1])
    dlat, dlon = lat2 - lat1, lon2 - lon1
    h = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * 6371000.0 * math.asin(math.sqrt(h))


class RandomVisit(Node):
    def __init__(self, map_uuid: str, count: int, seed: int | None):
        super().__init__('random_visit')
        self.map_uuid = map_uuid
        self.count = count    # 0 = infinite
        self.rng = random.Random(seed)
        self.get_map_client = self.create_client(GetMap, SERVICE_GET_MAP)
        self.goto_client = ActionClient(self, ExecuteGoTo, ACTION_EXECUTE_GOTO)
        self._goal_handle = None
        self.bbox: tuple[float, float, float, float] | None = None

    def wait(self):
        while not self.get_map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for {SERVICE_GET_MAP}...')
        while not self.goto_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for {ACTION_EXECUTE_GOTO}...')

    def fetch_bbox(self):
        req = GetMap.Request(uuid=self.map_uuid)
        future = self.get_map_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        if resp is None or not resp.map.points:
            raise RuntimeError('GetMap returned no points.')
        self.bbox = bbox(resp.map.points)
        lat_min, lat_max, lon_min, lon_max = self.bbox
        diag = haversine_m((lat_min, lon_min), (lat_max, lon_max))
        self.get_logger().info(
            f'Map bbox: lat [{lat_min:.6f}, {lat_max:.6f}] '
            f'lon [{lon_min:.6f}, {lon_max:.6f}]  (diagonal {diag:.0f} m)'
        )

    def random_target(self) -> tuple[float, float, float]:
        lat_min, lat_max, lon_min, lon_max = self.bbox
        lat = self.rng.uniform(lat_min, lat_max)
        lon = self.rng.uniform(lon_min, lon_max)
        heading = self.rng.uniform(-math.pi, math.pi)
        return lat, lon, heading

    def go_to(self, lat: float, lon: float, heading: float) -> bool:
        wp = Waypoint()
        wp.uuid = str(uuid.uuid4())
        wp.name = 'random_visit'
        wp.latitude = lat
        wp.longitude = lon
        wp.heading = heading
        wp.position_tolerance = POSITION_TOLERANCE_M
        wp.yaw_tolerance = YAW_TOLERANCE_RAD

        goal = ExecuteGoTo.Goal()
        goal.map_uuid = self.map_uuid
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
        self.get_logger().warn('Cancelling in-flight GoTo...')
        cancel_future = self._goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future)

    def run(self):
        self.fetch_bbox()
        i = 0
        while self.count == 0 or i < self.count:
            i += 1
            lat, lon, heading = self.random_target()
            self.get_logger().info(
                f'[{i}{"" if self.count == 0 else "/" + str(self.count)}] '
                f'GoTo ({lat:.6f}, {lon:.6f}) hdg {math.degrees(heading):+.0f}°'
            )
            ok = self.go_to(lat, lon, heading)
            self.get_logger().info(f'  {"OK" if ok else "FAILED — continuing"}')


def main(args=None):
    map_uuid = DEFAULT_MAP_ID
    argv = list(sys.argv[1:])
    if '--map-uuid' in argv:
        idx = argv.index('--map-uuid')
        map_uuid = argv.pop(idx + 1)
        argv.pop(idx)
    count = int(argv[0]) if len(argv) > 0 else 10
    seed = int(argv[1]) if len(argv) > 1 else None

    rclpy.init(args=args)
    node = RandomVisit(map_uuid, count, seed)
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
