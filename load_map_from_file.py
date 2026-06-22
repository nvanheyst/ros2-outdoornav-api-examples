"""
Load an OutdoorNav map from a JSON file via mission_manager/create_map.

Usage:
    python load_map_from_file.py data/map_Greenspace.json
    python load_map_from_file.py data/map_Greenspace.json "Greenspace v2"

JSON schema (matches the shape OutdoorNav's UI exports):
    {
      "name": "Greenspace",
      "points": [
        {"uuid": "...", "name": "p1", "latitude": 50.109, "longitude": -97.318},
        ...
      ],
      "connections": [
        {"start_point_id": "...", "end_point_id": "...",
         "speed_limit": 1.0, "radius": 1.5},
        ...
      ]
    }

If `uuid` fields are missing on points, new ones are generated and stitched
into the connections by name. If `name` is not provided on the CLI, the file's
top-level "name" is used.
"""

import json
import sys
import uuid
from pathlib import Path

import rclpy
from rclpy.node import Node
from clearpath_mission_manager_msgs.srv import CreateMap
from clearpath_mission_manager_msgs.msg import NetworkEdgeReq
from clearpath_navigation_msgs.msg import NetworkPoint


ROBOT_NAMESPACE = '/a300_00003'
SERVICE_CREATE_MAP = f'{ROBOT_NAMESPACE}/mission_manager/create_map'


def build_map_request(data: dict, override_name: str | None) -> CreateMap.Request:
    name = override_name or data.get('name')
    if not name:
        raise ValueError('No map name in JSON or on CLI.')

    points = []
    name_to_uuid: dict[str, str] = {}
    for raw in data.get('points', []):
        p = NetworkPoint()
        p.uuid = raw.get('uuid') or str(uuid.uuid4())
        p.latitude = float(raw['latitude'])
        p.longitude = float(raw['longitude'])
        if hasattr(p, 'name') and raw.get('name'):
            p.name = raw['name']
        points.append(p)
        if raw.get('name'):
            name_to_uuid[raw['name']] = p.uuid
        if raw.get('uuid'):
            name_to_uuid[raw['uuid']] = p.uuid  # uuid → uuid identity

    if not points:
        raise ValueError('No points in JSON.')

    connections = []
    for raw in data.get('connections', []):
        e = NetworkEdgeReq()
        start_id = raw.get('start_point_id') or raw.get('start')
        end_id = raw.get('end_point_id') or raw.get('end')
        e.start_point_id = name_to_uuid.get(start_id, start_id)
        e.end_point_id = name_to_uuid.get(end_id, end_id)
        e.speed_limit = float(raw.get('speed_limit', 1.0))
        e.radius = float(raw.get('radius', 1.5))
        connections.append(e)

    req = CreateMap.Request()
    req.name = name
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
        map_uuid = getattr(getattr(resp, 'result', None), 'uuid', None) or getattr(resp, 'uuid', None)
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
