"""
Build a row-coverage map from a polygon defined by tagged POIs.

ROS 2 port of the original onav_sdk/rospy version. The polygon is defined by
POIs sharing a tag (e.g. 'cov-2'). Vertices are ordered by POI name suffix
(cov-2-1, cov-2-2, ...). The first POI carries a JSON blob in
`custom_fields_json` with row spacing + width:

    {"spacing": 6.0, "width": 1.5}

The script:
    1. Fetches all POIs, filters by tag, sorts by name.
    2. Pulls spacing + width from the first POI's custom_fields_json.
    3. Builds the polygon in UTM (zone derived from the centroid longitude).
    4. Generates parallel rows perpendicular to the first edge, clipped to
       the polygon.
    5. Integrates the polygon's own edges so the boundary is part of the
       drivable graph.
    6. Deletes any existing map with the same name.
    7. Pushes the new map as NetworkPoint + NetworkEdgeReq.
    8. (No execution — pair with generate_traversal_mission.py to drive it.)

Usage:
    python row_generator_polygon.py             # tag default 'cov-2'
    python row_generator_polygon.py cov-3

Dependencies:
    pip install shapely networkx pyproj
"""

import json
import sys
import uuid

import networkx as nx
from pyproj import CRS, Transformer
from shapely.affinity import scale
from shapely.geometry import Polygon, LineString, Point, MultiPoint
from shapely.ops import unary_union, snap

import rclpy
from rclpy.node import Node
from clearpath_mission_manager_msgs.srv import (
    CreateMap, GetAllMaps, DeleteMap, GetAllPois,
)
from clearpath_mission_manager_msgs.msg import NetworkEdgeReq
from clearpath_navigation_msgs.msg import NetworkPoint


ROBOT_NAMESPACE = '/a300_00003'
SERVICE_CREATE_MAP   = f'{ROBOT_NAMESPACE}/mission_manager/create_map'
SERVICE_GET_ALL_MAPS = f'{ROBOT_NAMESPACE}/mission_manager/get_all_maps'
SERVICE_DELETE_MAP   = f'{ROBOT_NAMESPACE}/mission_manager/delete_map'
SERVICE_GET_ALL_POIS = f'{ROBOT_NAMESPACE}/mission_manager/get_all_pois'

DEFAULT_POI_TAG = 'cov-2'
SPEED_LIMIT_M_S = 1.0


# ---- geometry helpers -----------------------------------------------------

def utm_zone_for(lon: float) -> int:
    return int((lon + 180) / 6) + 1


def make_transformers(lon: float, lat: float) -> tuple[Transformer, Transformer]:
    zone = utm_zone_for(lon)
    hemi = 'north' if lat >= 0 else 'south'
    utm_crs = CRS.from_proj4(f'+proj=utm +zone={zone} +{hemi} +ellps=WGS84 +datum=WGS84 +units=m +no_defs')
    ll = CRS.from_epsg(4326)
    return (Transformer.from_crs(ll, utm_crs, always_xy=True),
            Transformer.from_crs(utm_crs, ll, always_xy=True))


def generate_parallel_lines(polygon: Polygon, start_edge, spacing: float):
    line = LineString([start_edge[0], start_edge[1]])
    line = scale(line, xfact=30.0, yfact=30.0, origin='centroid')

    parallels = []
    for side in ('right', 'left'):
        d = spacing
        while True:
            par = line.parallel_offset(distance=d, side=side)
            if not polygon.intersects(par):
                break
            inter = polygon.intersection(par)
            if isinstance(inter, LineString):
                parallels.append(inter)
            else:
                break
            d += spacing
    return parallels


def integrate_polygon_edges(graph: nx.Graph, polygon: Polygon, node_map: dict, parallels: list):
    merged = unary_union(parallels)
    coords = list(polygon.exterior.coords)
    for i in range(len(coords) - 1):
        line = LineString([coords[i], coords[i + 1]])
        snapped = snap(line, merged, 1e-6)
        inter = snapped.intersection(merged)
        pts = list(line.coords)
        if isinstance(inter, Point):
            pts.append(inter.coords[0])
        elif isinstance(inter, MultiPoint):
            pts.extend(p.coords[0] for p in inter.geoms)
        unique_sorted = sorted(set(pts), key=lambda p: line.project(Point(p)))
        segs = [LineString(unique_sorted[i:i + 2]) for i in range(len(unique_sorted) - 1)]
        add_edges(graph, segs, node_map)


def add_edges(graph: nx.Graph, lines: list, node_map: dict):
    for line in lines:
        coords = list(line.coords)
        for a, b in zip(coords[:-1], coords[1:]):
            for pt in (a, b):
                if pt not in node_map:
                    node_map[pt] = len(node_map)
                    graph.add_node(node_map[pt], pos=pt)
            graph.add_edge(node_map[a], node_map[b], weight=line.length)


# ---- node ------------------------------------------------------------------

class RowGenPolygon(Node):
    def __init__(self, tag: str):
        super().__init__('row_generator_polygon')
        self.tag = tag
        self.create_map_client   = self.create_client(CreateMap,   SERVICE_CREATE_MAP)
        self.get_all_maps_client = self.create_client(GetAllMaps,  SERVICE_GET_ALL_MAPS)
        self.delete_map_client   = self.create_client(DeleteMap,   SERVICE_DELETE_MAP)
        self.get_all_pois_client = self.create_client(GetAllPois,  SERVICE_GET_ALL_POIS)

    def wait(self):
        for c, n in [(self.create_map_client,   SERVICE_CREATE_MAP),
                     (self.get_all_maps_client, SERVICE_GET_ALL_MAPS),
                     (self.delete_map_client,   SERVICE_DELETE_MAP),
                     (self.get_all_pois_client, SERVICE_GET_ALL_POIS)]:
            while not c.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for {n}...')

    def call(self, client, req):
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def fetch_polygon_pois(self) -> list:
        resp = self.call(self.get_all_pois_client, GetAllPois.Request())
        pois = [p for p in getattr(resp, 'points_of_interest', []) if self.tag in getattr(p, 'tags', [])]
        pois.sort(key=lambda p: p.name)
        return pois

    def delete_existing_map(self, name: str):
        resp = self.call(self.get_all_maps_client, GetAllMaps.Request())
        for m in getattr(resp, 'maps', []):
            if m.name == name:
                req = DeleteMap.Request()
                req.uuid = m.uuid
                self.call(self.delete_map_client, req)
                self.get_logger().info(f'Deleted existing map {name!r}.')
                return

    def push_map(self, name: str, graph: nx.Graph, utm_to_ll: Transformer, edge_radius: float) -> str:
        points = []
        node_uuid = {}
        for node, attrs in graph.nodes(data=True):
            x, y = attrs['pos']
            lon, lat = utm_to_ll.transform(x, y)
            p = NetworkPoint()
            p.uuid = str(uuid.uuid4())
            p.latitude = lat
            p.longitude = lon
            points.append(p)
            node_uuid[node] = p.uuid

        edges = []
        for u, v in graph.edges():
            for s, e in ((u, v), (v, u)):   # bidirectional
                edge = NetworkEdgeReq()
                edge.start_point_id = node_uuid[s]
                edge.end_point_id = node_uuid[e]
                edge.speed_limit = SPEED_LIMIT_M_S
                edge.radius = edge_radius
                edges.append(edge)

        req = CreateMap.Request()
        req.name = name
        req.points = points
        req.connections = edges
        resp = self.call(self.create_map_client, req)
        map_uuid = getattr(getattr(resp, 'result', None), 'uuid', None) or getattr(resp, 'uuid', None)
        self.get_logger().info(
            f'Created map {name!r} ({map_uuid[:8] if map_uuid else "?"}) '
            f'with {len(points)} points and {len(edges)} edges.'
        )
        return map_uuid

    def run(self):
        pois = self.fetch_polygon_pois()
        if len(pois) < 3:
            self.get_logger().error(f'Need >=3 POIs tagged {self.tag!r}; got {len(pois)}.')
            return

        cfg = json.loads(pois[0].custom_fields_json or '{}')
        spacing = float(cfg.get('spacing', 6.0))
        width = float(cfg.get('width', 1.5))
        map_name = self.tag

        lon0 = sum(p.longitude for p in pois) / len(pois)
        lat0 = sum(p.latitude  for p in pois) / len(pois)
        ll_to_utm, utm_to_ll = make_transformers(lon0, lat0)

        utm_vertices = [ll_to_utm.transform(p.longitude, p.latitude) for p in pois]
        polygon = Polygon(utm_vertices)
        start_edge = (polygon.exterior.coords[0], polygon.exterior.coords[1])

        lines = generate_parallel_lines(polygon, start_edge, spacing)
        graph = nx.Graph()
        node_map: dict = {}
        add_edges(graph, lines, node_map)
        integrate_polygon_edges(graph, polygon, node_map, lines)

        self.delete_existing_map(map_name)
        self.push_map(map_name, graph, utm_to_ll, edge_radius=width)


def main(args=None):
    tag = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_POI_TAG
    rclpy.init(args=args)
    node = RowGenPolygon(tag)
    try:
        node.wait()
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
