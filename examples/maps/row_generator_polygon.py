#!/usr/bin/env python3
"""Row-coverage map from a polygon defined by tagged POIs.

POIs sharing a tag (default 'cov-2') define polygon vertices, ordered by
name suffix (cov-2-1, cov-2-2, ...). The first POI's custom_fields_json
carries the row spacing + width:
    {"spacing": 6.0, "width": 1.5}

  ./row_generator_polygon.py                  # tag 'cov-2'
  ./row_generator_polygon.py --tag cov-3
  ./row_generator_polygon.py --tag cov-3 --dry-run
  ./row_generator_polygon.py --tag cov-3 --replace   # overwrite existing

By default this errors if a map with the same name (the tag) already
exists. Pass --replace to delete the existing map first.

Needs: pip install shapely networkx pyproj

Touches:
  service <namespace>/mission_manager/get_all_points_of_interest (GetAllPointsOfInterest)
  service <namespace>/mission_manager/get_all_maps               (GetAllMaps)
  service <namespace>/mission_manager/delete_map                 (DeleteById, only with --replace)
  service <namespace>/mission_manager/create_map                 (CreateMap)
"""

from __future__ import annotations
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import networkx as nx
from pyproj import CRS, Transformer
from shapely.affinity import scale
from shapely.geometry import Polygon, LineString, Point, MultiPoint
from shapely.ops import unary_union, snap

import rclpy
from rclpy.node import Node
from clearpath_mission_manager_msgs.srv import (
    CreateMap, GetAllMaps, DeleteById, GetAllPointsOfInterest,
)
from clearpath_mission_manager_msgs.msg import MapEdgeReq
from clearpath_navigation_msgs.msg import MapPoint

from common.argparse_base import make_parser
from common.ros_helpers import wait_for_service, call_service


SPEED_LIMIT_M_S = 1.0


def utm_zone_for(lon: float) -> int:
    return int((lon + 180) / 6) + 1


def make_transformers(lon: float, lat: float):
    zone = utm_zone_for(lon)
    hemi = "north" if lat >= 0 else "south"
    utm_crs = CRS.from_proj4(
        f"+proj=utm +zone={zone} +{hemi} +ellps=WGS84 +datum=WGS84 +units=m +no_defs"
    )
    ll = CRS.from_epsg(4326)
    return (
        Transformer.from_crs(ll, utm_crs, always_xy=True),
        Transformer.from_crs(utm_crs, ll, always_xy=True),
    )


def generate_parallel_lines(polygon: Polygon, start_edge, spacing: float):
    line = LineString([start_edge[0], start_edge[1]])
    line = scale(line, xfact=30.0, yfact=30.0, origin="centroid")
    parallels = []
    for side in ("right", "left"):
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


def integrate_polygon_edges(graph, polygon, node_map, parallels):
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


def add_edges(graph, lines, node_map):
    for line in lines:
        coords = list(line.coords)
        for a, b in zip(coords[:-1], coords[1:]):
            for pt in (a, b):
                if pt not in node_map:
                    node_map[pt] = len(node_map)
                    graph.add_node(node_map[pt], pos=pt)
            graph.add_edge(node_map[a], node_map[b], weight=line.length)


class RowGenPolygon(Node):
    def __init__(self, namespace: str, tag: str):
        super().__init__("row_generator_polygon")
        self.tag = tag
        self.create_map_srv = f"{namespace}/mission_manager/create_map"
        self.get_all_maps_srv = f"{namespace}/mission_manager/get_all_maps"
        self.delete_map_srv = f"{namespace}/mission_manager/delete_map"
        self.get_all_pois_srv = f"{namespace}/mission_manager/get_all_points_of_interest"
        self.create_map_client = self.create_client(CreateMap, self.create_map_srv)
        self.get_all_maps_client = self.create_client(GetAllMaps, self.get_all_maps_srv)
        self.delete_map_client = self.create_client(DeleteById, self.delete_map_srv)
        self.get_all_pois_client = self.create_client(GetAllPointsOfInterest, self.get_all_pois_srv)

    def wait(self) -> None:
        for c, n in [
            (self.create_map_client, self.create_map_srv),
            (self.get_all_maps_client, self.get_all_maps_srv),
            (self.delete_map_client, self.delete_map_srv),
            (self.get_all_pois_client, self.get_all_pois_srv),
        ]:
            wait_for_service(self, c, n)

    def fetch_polygon_pois(self):
        resp = call_service(self, self.get_all_pois_client, GetAllPointsOfInterest.Request())
        pois = [p for p in getattr(resp, "points_of_interest", [])
                if self.tag in getattr(p, "tags", [])]
        pois.sort(key=lambda p: p.name)
        return pois

    def find_existing_map(self, name: str) -> str | None:
        resp = call_service(self, self.get_all_maps_client, GetAllMaps.Request())
        for m in getattr(resp, "maps", []):
            if m.name == name:
                return m.uuid
        return None

    def delete_map(self, map_uuid: str, name: str) -> None:
        req = DeleteById.Request(); req.uuid = map_uuid
        call_service(self, self.delete_map_client, req)
        self.get_logger().info(f"deleted existing map {name!r} ({map_uuid})")

    def push_map(self, name: str, graph, utm_to_ll, edge_radius: float) -> str:
        points = []
        node_local_id: dict = {}
        for node, attrs in graph.nodes(data=True):
            x, y = attrs["pos"]
            lon, lat = utm_to_ll.transform(x, y)
            p = MapPoint(); p.uuid = str(node); p.latitude = lat; p.longitude = lon
            points.append(p)
            node_local_id[node] = p.uuid
        edges = []
        for u, v in graph.edges():
            for s, e in ((u, v), (v, u)):
                edge = MapEdgeReq()
                edge.start_point_id = node_local_id[s]
                edge.end_point_id = node_local_id[e]
                edge.speed_limit = SPEED_LIMIT_M_S
                edge.radius = edge_radius
                edges.append(edge)
        req = CreateMap.Request()
        req.name = name
        req.default_radius = edge_radius
        req.default_speed_limit = SPEED_LIMIT_M_S
        req.points = points
        req.connections = edges
        resp = call_service(self, self.create_map_client, req)
        map_uuid = resp.result.uuid if resp and resp.result else None
        self.get_logger().info(
            f"created map {name!r} ({map_uuid}) with {len(points)} points, {len(edges)} edges"
        )
        return map_uuid

    def run(self, dry_run: bool = False, replace: bool = False) -> int:
        pois = self.fetch_polygon_pois()
        if len(pois) < 3:
            self.get_logger().error(f"need >=3 POIs tagged {self.tag!r}; got {len(pois)}")
            return 1

        cfg = json.loads(pois[0].custom_fields_json or "{}")
        spacing = float(cfg.get("spacing", 6.0))
        width = float(cfg.get("width", 1.5))
        map_name = self.tag

        existing_uuid = self.find_existing_map(map_name)
        if existing_uuid and not replace and not dry_run:
            self.get_logger().error(
                f"map {map_name!r} already exists ({existing_uuid}). "
                "Pass --replace to overwrite, or change --tag."
            )
            return 1

        lon0 = sum(p.longitude for p in pois) / len(pois)
        lat0 = sum(p.latitude for p in pois) / len(pois)
        ll_to_utm, utm_to_ll = make_transformers(lon0, lat0)

        utm_vertices = [ll_to_utm.transform(p.longitude, p.latitude) for p in pois]
        polygon = Polygon(utm_vertices)
        start_edge = (polygon.exterior.coords[0], polygon.exterior.coords[1])

        lines = generate_parallel_lines(polygon, start_edge, spacing)
        graph = nx.Graph()
        node_map: dict = {}
        add_edges(graph, lines, node_map)
        integrate_polygon_edges(graph, polygon, node_map, lines)

        self.get_logger().info(
            f"{len(graph.nodes)} nodes, {len(graph.edges)} edges generated from polygon"
        )
        if dry_run:
            verb = "would replace" if existing_uuid else "would create"
            self.get_logger().info(f"[dry-run] {verb} map {map_name!r}")
            return 0

        if existing_uuid:
            self.delete_map(existing_uuid, map_name)
        self.push_map(map_name, graph, utm_to_ll, edge_radius=width)
        return 0


def main(argv=None):
    parser = make_parser(doc=__doc__)
    parser.add_argument("--tag", default="cov-2", help="POI tag identifying polygon vertices.")
    parser.add_argument("--replace", action="store_true",
                        help="If a map with the same name exists, delete it first.")
    args = parser.parse_args(argv)

    rc = 1
    rclpy.init()
    node = RowGenPolygon(args.namespace, args.tag)
    try:
        node.wait()
        rc = node.run(dry_run=args.dry_run, replace=args.replace)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(rc)


if __name__ == "__main__":
    main()
