#!/usr/bin/env python3
"""Load an OutdoorNav map from JSON via mission_manager/create_map.

  ./load_map_from_file.py data/map_Greenspace.json
  ./load_map_from_file.py data/map_Greenspace.json --name "Greenspace v2"
  ./load_map_from_file.py data/map_Greenspace.json --dry-run

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

Touches: service <namespace>/mission_manager/create_map (CreateMap).
"""

from __future__ import annotations
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import rclpy
from rclpy.node import Node
from clearpath_mission_manager_msgs.srv import CreateMap
from clearpath_mission_manager_msgs.msg import MapEdgeReq
from clearpath_navigation_msgs.msg import MapPoint

from examples.common.argparse_base import make_parser
from examples.common.ros_helpers import wait_for_service, call_service


def build_map_request(data: dict, override_name: str | None) -> CreateMap.Request:
    name = override_name or data.get("name")
    if not name:
        raise ValueError("No map name in JSON or on CLI.")

    points = []
    local_ids: set[str] = set()
    for i, raw in enumerate(data.get("points", [])):
        p = MapPoint()
        p.uuid = str(raw.get("id") or raw.get("uuid") or i)
        p.latitude = float(raw["latitude"])
        p.longitude = float(raw["longitude"])
        points.append(p)
        local_ids.add(p.uuid)
    if not points:
        raise ValueError("No points in JSON.")

    connections = []
    for raw in data.get("connections", []):
        start_id = str(raw.get("start_point_id") or raw.get("start"))
        end_id = str(raw.get("end_point_id") or raw.get("end"))
        if start_id not in local_ids or end_id not in local_ids:
            print(f"  WARN: edge {start_id}->{end_id} references unknown point id, skipping.")
            continue
        e = MapEdgeReq()
        e.start_point_id = start_id
        e.end_point_id = end_id
        e.speed_limit = float(raw.get("speed_limit", 1.0))
        e.radius = float(raw.get("radius", 1.5))
        connections.append(e)

    req = CreateMap.Request()
    req.name = name
    req.default_radius = float(data.get("default_radius", 1.5))
    req.default_speed_limit = float(data.get("default_speed_limit", 1.0))
    req.points = points
    req.connections = connections
    return req


class LoadMap(Node):
    def __init__(self, namespace: str):
        super().__init__("load_map_from_file")
        self.srv = f"{namespace}/mission_manager/create_map"
        self.client = self.create_client(CreateMap, self.srv)

    def wait(self) -> None:
        wait_for_service(self, self.client, self.srv)

    def create(self, req: CreateMap.Request) -> str:
        self.get_logger().info(
            f"loading map {req.name!r}: {len(req.points)} points, {len(req.connections)} edges"
        )
        resp = call_service(self, self.client, req)
        map_uuid = resp.result.uuid if resp and resp.result else None
        if not map_uuid:
            raise RuntimeError(f"CreateMap returned no uuid: {resp}")
        self.get_logger().info(f"OK: map_uuid={map_uuid}")
        return map_uuid


def main(argv=None):
    parser = make_parser(doc=__doc__)
    parser.add_argument("path", type=Path, help="Path to map JSON file.")
    parser.add_argument("--name", default=None, help="Override the map name from the JSON.")
    args = parser.parse_args(argv)

    data = json.loads(args.path.expanduser().read_text())
    req = build_map_request(data, args.name)

    if args.dry_run:
        print(f"[dry-run] would create map {req.name!r} via {args.namespace}/mission_manager/create_map")
        print(f"[dry-run]   points={len(req.points)}, connections={len(req.connections)}")
        return

    rclpy.init()
    node = LoadMap(args.namespace)
    try:
        node.wait()
        node.create(req)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
