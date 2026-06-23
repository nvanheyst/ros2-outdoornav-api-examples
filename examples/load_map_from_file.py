"""
Load an OutdoorNav map from JSON via mission_manager/create_map.

Ported from the onav SDK ROS 1 examples and updated for ROS 2.

  python load_map_from_file.py data/map_Greenspace.json
  python load_map_from_file.py data/map_Greenspace.json "Greenspace v2"

JSON shape (matches the UI export):
    {
      "name": "Greenspace",
      "default_radius": 1.5,
      "default_speed_limit": 1.0,
      "points":      [{"id": "p1", "latitude": 50.109, "longitude": -97.318}, ...],
      "connections": [{"start_point_id": "p1", "end_point_id": "p2",
                       "speed_limit": 1.0, "radius": 1.5}, ...]
    }

The `id`s in the request are local; the server generates real UUIDs.
"""

import json
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from clearpath_mission_manager_msgs.srv import CreateMap
from clearpath_mission_manager_msgs.msg import MapEdgeReq
from clearpath_navigation_msgs.msg import MapPoint


ROBOT_NAMESPACE = '/a300_00003'
SERVICE_CREATE_MAP = f'{ROBOT_NAMESPACE}/mission_manager/create_map'


def build_map_request(data: dict, override_name: str | None) -> CreateMap.Request:
    name = override_name or data.get('name')
    if not name:
        raise ValueError('No map name in JSON or on CLI.')

    points = []
    local_ids: set[str] = set()
    for i, raw in enumerate(data.get('points', [])):
        p = MapPoint()
        p.uuid = str(raw.get('id') or raw.get('uuid') or i)
        p.latitude = float(raw['latitude'])
        p.longitude = float(raw['longitude'])
        points.append(p)
        local_ids.add(p.uuid)

    if not points:
        raise ValueError('No points in JSON.')

    connections = []
    for raw in data.get('connections', []):
        start_id = str(raw.get('start_point_id') or raw.get('start'))
        end_id = str(raw.get('end_point_id') or raw.get('end'))
        if start_id not in local_ids or end_id not in local_ids:
            print(f'  WARN: edge {start_id}->{end_id} references unknown point id, skipping.')
            continue
        e = MapEdgeReq()
        e.start_point_id = start_id
        e.end_point_id = end_id
        e.speed_limit = float(raw.get('speed_limit', 1.0))
        e.radius = float(raw.get('radius', 1.5))
        connections.append(e)

    req = CreateMap.Request()
    req.name = name
    req.default_radius = float(data.get('default_radius', 1.5))
    req.default_speed_limit = float(data.get('default_speed_limit', 1.0))
    req.points = points
    req.connections = connections
    return req


class LoadMap(Node):
    def __init__(self):
        super().__init__('load_map_from_file')
        self.client = self.create_client(CreateMap, SERVICE_CREATE_MAP)

    def wait(self):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for {SERVICE_CREATE_MAP}...')

    def create(self, req: CreateMap.Request) -> str:
        self.get_logger().info(
            f'Loading map {req.name!r}: {len(req.points)} points, {len(req.connections)} edges.'
        )
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        map_uuid = resp.result.uuid if resp and resp.result else None
        if not map_uuid:
            raise RuntimeError(f'CreateMap returned no uuid: {resp}')
        self.get_logger().info(f'OK: map_uuid={map_uuid}')
        return map_uuid


def main(args=None):
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    path = Path(sys.argv[1]).expanduser()
    override_name = sys.argv[2] if len(sys.argv) > 2 else None
    data = json.loads(path.read_text())
    req = build_map_request(data, override_name)

    rclpy.init(args=args)
    node = LoadMap()
    try:
        node.wait()
        node.create(req)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
