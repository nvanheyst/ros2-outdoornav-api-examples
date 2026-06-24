#!/usr/bin/env python3
"""Wipe every map, mission, and POI from the OnAV database.

Useful as cleanup after running these API examples — they accumulate
test maps, missions, and POIs that clutter the UI. One service call
nukes them all.

  ./delete_all.py --dry-run     # counts only, no state change
  ./delete_all.py --confirm     # actually fires the service
  ./delete_all.py               # refuses; reminds you to pick one

Safety: the underlying service request carries a
`yes_i_am_absolutely_sure_i_want_to_do_this` bool. This script only
sets it when you pass `--confirm`. Does NOT delete docks (separate
database — see `<ns>/docking/dock_manager/clear_data`).

Touches:
  service <namespace>/mission_manager/delete_all                     (DeleteEverything)
  service <namespace>/mission_manager/get_all_maps                   (GetAllMaps, --dry-run only)
  service <namespace>/mission_manager/get_all_network_missions       (GetAllNetworkMissions, --dry-run only)
  service <namespace>/mission_manager/get_all_points_of_interest     (GetAllPointsOfInterest, --dry-run only)
"""

from __future__ import annotations
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import rclpy
from rclpy.node import Node
from clearpath_mission_manager_msgs.srv import (
    DeleteEverything, GetAllMaps, GetAllNetworkMissions, GetAllPointsOfInterest,
)

from common.argparse_base import make_parser
from common.ros_helpers import wait_for_service, call_service


class DeleteAll(Node):
    def __init__(self, namespace: str):
        super().__init__("delete_all")
        self.delete_srv = f"{namespace}/mission_manager/delete_all"
        self.get_maps_srv = f"{namespace}/mission_manager/get_all_maps"
        self.get_missions_srv = f"{namespace}/mission_manager/get_all_network_missions"
        self.get_pois_srv = f"{namespace}/mission_manager/get_all_points_of_interest"

        self.delete_client = self.create_client(DeleteEverything, self.delete_srv)
        self.get_maps_client = self.create_client(GetAllMaps, self.get_maps_srv)
        self.get_missions_client = self.create_client(GetAllNetworkMissions, self.get_missions_srv)
        self.get_pois_client = self.create_client(GetAllPointsOfInterest, self.get_pois_srv)

    def wait_for_counts(self) -> None:
        wait_for_service(self, self.get_maps_client, self.get_maps_srv)
        wait_for_service(self, self.get_missions_client, self.get_missions_srv)
        wait_for_service(self, self.get_pois_client, self.get_pois_srv)

    def wait_for_delete(self) -> None:
        wait_for_service(self, self.delete_client, self.delete_srv)

    def counts(self) -> tuple[int, int, int]:
        maps = call_service(self, self.get_maps_client, GetAllMaps.Request())
        missions = call_service(self, self.get_missions_client, GetAllNetworkMissions.Request())
        pois = call_service(self, self.get_pois_client, GetAllPointsOfInterest.Request())
        return (
            len(getattr(maps, "maps", []) or []),
            len(getattr(missions, "network_missions", []) or getattr(missions, "missions", []) or []),
            len(getattr(pois, "points_of_interest", []) or []),
        )

    def fire(self) -> bool:
        req = DeleteEverything.Request()
        req.yes_i_am_absolutely_sure_i_want_to_do_this = True
        resp = call_service(self, self.delete_client, req)
        ok = bool(resp and getattr(resp, "ok", False))
        self.get_logger().info(f"delete_all: {'OK — database wiped' if ok else 'FAILED'}")
        return ok


def main(argv=None):
    parser = make_parser(doc=__doc__, add_dry_run=True)
    parser.add_argument("--confirm", action="store_true",
                        help="Actually fire the service. Without this (and without --dry-run), refuses.")
    args = parser.parse_args(argv)

    if not args.dry_run and not args.confirm:
        parser.error("refusing — pass --dry-run for counts or --confirm to actually wipe")

    rclpy.init()
    node = DeleteAll(args.namespace)
    try:
        if args.dry_run:
            node.wait_for_counts()
            n_maps, n_missions, n_pois = node.counts()
            print(f"[dry-run] {n_maps} maps, {n_missions} missions, {n_pois} POIs would be deleted")
            print(f"[dry-run] service: {node.delete_srv}")
            return
        node.wait_for_delete()
        node.fire()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
