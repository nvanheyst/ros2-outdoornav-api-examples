#!/usr/bin/env python3
"""OutdoorNav MCP server: 9 tools via streamable-HTTP.

Run inside the dev container on the machine with ROS 2 access to the robot.
Requires the OutdoorNav stack running (sim or real).

Usage:
  python mini-projects/onav-console/server.py [--host 0.0.0.0] [--port 8091]
  ONAV_NAMESPACE=/a300_00070 python mini-projects/onav-console/server.py
"""
import argparse
import os
import sys
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from clearpath_dock_msgs.action import Dock, MapDock, Undock
from clearpath_dock_msgs.srv import GetDockDatabase
from clearpath_mission_manager_msgs.srv import (
    GetAllMaps,
    GetAllNetworkMissions,
    GetAllPointsOfInterest,
)
from clearpath_navigation_msgs.action import ExecuteGoToPOI, ExecuteMission
from clearpath_navigation_msgs.msg import AutonomyStatus
from mcp.server.fastmcp import FastMCP
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, NavSatFix
from std_srvs.srv import Trigger

def normalize_namespace(ns: str) -> str:
    ns = (ns or "").strip()
    if not ns:
        return ""
    if not ns.startswith("/"):
        ns = "/" + ns
    return ns.rstrip("/")


def _detect_robot_namespaces(node) -> list[str]:
    topics = {t for t, _ in node.get_topic_names_and_types()}
    roots: set[str] = set()
    for _name, ns in node.get_node_names_and_namespaces():
        ns = normalize_namespace(ns)
        if ns:
            roots.add("/" + ns.lstrip("/").split("/")[0])
    anchored = sorted(
        r for r in roots
        if f"{r}/localization/fix" in topics or f"{r}/platform/bms/state" in topics
    )
    return anchored or sorted(roots)


SVC_TIMEOUT = 30.0

TOOL_TIMEOUT = {
    "sync_data": 30.0,
    "get_state": 30.0,
    "go_to_poi": 600.0,
    "run_mission": 3600.0,
    "dock": 300.0,
    "dock_local": 300.0,
    "undock": 120.0,
    "stop": 10.0,
}

STATE_NAMES = {
    AutonomyStatus.IDLE: "idle",
    AutonomyStatus.MISSION: "running mission",
    AutonomyStatus.MISSION_FROM_GOAL: "running mission from goal",
    AutonomyStatus.GOTO: "navigating to location",
    AutonomyStatus.GOTO_POI: "navigating to POI",
    AutonomyStatus.DOCKING_LOCAL: "docking (local sensor)",
    AutonomyStatus.UNDOCKING_LOCAL: "undocking",
    AutonomyStatus.DOCKING_MAP: "docking (map-based)",
}


def _translate_result(status: int, success: bool, message: str, error_code) -> str:
    if status == GoalStatus.STATUS_CANCELED:
        return "cancelled by operator"
    if status == GoalStatus.STATUS_ABORTED:
        if error_code == 2:
            return "blocked by obstacle"
        msg = message or (f"code {error_code}" if error_code else "see robot log")
        return f"aborted — {msg}"
    if status == GoalStatus.STATUS_SUCCEEDED:
        if success:
            return "succeeded"
        return f"failed — {message or 'see robot log'}"
    return f"ended with status {status}"


class OnavNode(Node):
    def __init__(self, ns: str):
        super().__init__("onav_mcp")
        self._ns = ns
        self._cancel = threading.Event()
        self._active_handle = None
        self._active_lock = threading.Lock()
        self._data: dict = {}
        self._data_ready = threading.Event()

        self._autonomy_status: AutonomyStatus | None = None
        self._battery: BatteryState | None = None
        self._gps: NavSatFix | None = None

        self.create_subscription(AutonomyStatus, f"{ns}/autonomy/status", self._on_autonomy, 10)
        self.create_subscription(BatteryState, f"{ns}/platform/bms/state", self._on_bms, 10)
        self.create_subscription(NavSatFix, f"{ns}/localization/fix", self._on_gps, 10)

        self._maps_client = self.create_client(GetAllMaps, f"{ns}/mission_manager/get_all_maps")
        self._pois_client = self.create_client(GetAllPointsOfInterest, f"{ns}/mission_manager/get_all_points_of_interest")
        self._missions_client = self.create_client(GetAllNetworkMissions, f"{ns}/mission_manager/get_all_missions")
        self._dock_db_client = self.create_client(GetDockDatabase, f"{ns}/docking/get_dock_database")
        self._stop_client = self.create_client(Trigger, f"{ns}/autonomy/stop")

        self._goto_poi_client = ActionClient(self, ExecuteGoToPOI, f"{ns}/autonomy/goto_poi")
        self._mission_client = ActionClient(self, ExecuteMission, f"{ns}/autonomy/mission")
        self._dock_map_client = ActionClient(self, MapDock, f"{ns}/autonomy/dock_map")
        self._dock_local_client = ActionClient(self, Dock, f"{ns}/autonomy/dock_local")
        self._undock_client = ActionClient(self, Undock, f"{ns}/autonomy/undock")

    def _on_autonomy(self, msg: AutonomyStatus) -> None:
        self._autonomy_status = msg

    def _on_bms(self, msg: BatteryState) -> None:
        self._battery = msg

    def _on_gps(self, msg: NavSatFix) -> None:
        self._gps = msg

    def _wait(self, future, timeout: float):
        """Poll future.done() in a tight loop; return None on cancel or timeout."""
        deadline = time.monotonic() + timeout
        while not future.done():
            if self._cancel.is_set():
                return None
            if time.monotonic() > deadline:
                return None
            time.sleep(0.05)
        return future.result()

    def _call_srv(self, client, request, timeout: float = SVC_TIMEOUT):
        if not client.wait_for_service(timeout_sec=5.0):
            return None
        return self._wait(client.call_async(request), timeout)

    def _run_action(self, action_client, goal, timeout: float, label: str):
        if not action_client.wait_for_server(timeout_sec=10.0):
            return None, "action server not available — is the stack up?"

        elapsed_ref = [0.0]

        def _feedback(msg):
            elapsed_ref[0] = getattr(msg.feedback, "elapsed_time", 0.0)

        send_fut = action_client.send_goal_async(goal, feedback_callback=_feedback)
        handle = self._wait(send_fut, 15.0)
        if handle is None:
            return None, "timed out or cancelled waiting for goal acceptance"
        if not handle.accepted:
            return None, "goal rejected by action server"

        with self._active_lock:
            self._active_handle = handle

        result_fut = handle.get_result_async()
        t0 = time.monotonic()
        last_print = 0.0

        while not result_fut.done():
            if self._cancel.is_set():
                with self._active_lock:
                    self._active_handle = None
                return None, "cancelled by kill"
            elapsed = time.monotonic() - t0
            if elapsed > timeout:
                with self._active_lock:
                    self._active_handle = None
                return None, f"timed out after {timeout:.0f}s"
            if elapsed - last_print >= 10.0:
                t = elapsed_ref[0] or elapsed
                print(f"  {label}: {t:.0f}s elapsed", flush=True)
                last_print = elapsed
            time.sleep(0.05)

        with self._active_lock:
            self._active_handle = None
        return result_fut.result(), None

    def cancel_active(self) -> None:
        self._cancel.set()
        with self._active_lock:
            h = self._active_handle
        if h is not None:
            try:
                cancel_fut = h.cancel_goal_async()
                deadline = time.monotonic() + 5.0
                while not cancel_fut.done() and time.monotonic() < deadline:
                    time.sleep(0.05)
            except Exception:
                pass

    def clear_cancel(self) -> None:
        self._cancel.clear()

    def _find_by_name(self, items: list[dict], name: str) -> dict | None:
        name_low = name.lower()
        exact = [i for i in items if i["name"].lower() == name_low]
        if exact:
            return exact[0]
        fuzzy = [i for i in items if name_low in i["name"].lower()]
        return fuzzy[0] if len(fuzzy) == 1 else None

    def _map_uuid(self, map_name: str = "") -> str | None:
        maps = self._data.get("maps", [])
        if not maps:
            return None
        if map_name:
            m = self._find_by_name(maps, map_name)
            return m["uuid"] if m else None
        return maps[0]["uuid"]

    # ---- tool implementations -----------------------------------------------

    def do_sync(self) -> dict:
        maps_resp = self._call_srv(self._maps_client, GetAllMaps.Request())
        pois_resp = self._call_srv(self._pois_client, GetAllPointsOfInterest.Request())
        missions_resp = self._call_srv(self._missions_client, GetAllNetworkMissions.Request())
        docks_resp = self._call_srv(self._dock_db_client, GetDockDatabase.Request())

        maps = [{"name": m.name, "uuid": m.uuid}
                for m in (getattr(maps_resp, "maps", None) or [])]
        pois = [{"name": p.name, "uuid": p.uuid}
                for p in (getattr(pois_resp, "points_of_interest", None) or [])]
        missions = [{"name": m.name, "uuid": m.uuid}
                    for m in (getattr(missions_resp, "missions", None) or [])]
        db = getattr(docks_resp, "database", None)
        raw_docks = (getattr(db, "docks", None) or []) if db else []
        docks = [{"name": d.name} for d in raw_docks]

        self._data = {"namespace": self._ns, "maps": maps, "pois": pois, "missions": missions, "docks": docks}
        self._data_ready.set()
        return self._data

    def do_get_state(self) -> dict:
        result: dict = {}
        if self._autonomy_status is not None:
            st = self._autonomy_status
            result["state"] = STATE_NAMES.get(st.state, str(st.state))
            result["paused"] = bool(st.paused)
            if st.current_goal:
                result["current_goal"] = st.current_goal
        if self._battery is not None:
            result["battery_percent"] = round(self._battery.percentage * 100, 1)
        if self._gps is not None:
            result["position"] = {
                "lat": round(self._gps.latitude, 7),
                "lon": round(self._gps.longitude, 7),
            }
        if not result:
            return {"error": "no state data yet — is the OutdoorNav stack running?"}
        return result

    def do_goto_poi(self, poi_name: str, map_name: str = "") -> dict:
        data = self._data
        if not data.get("maps"):
            return {"error": "no maps loaded — call sync_data first or create a map in the web UI"}
        poi = self._find_by_name(data.get("pois", []), poi_name)
        if poi is None:
            known = [p["name"] for p in data.get("pois", [])]
            return {"error": f"POI {poi_name!r} not found — known: {known}"}
        map_uuid = self._map_uuid(map_name)
        if map_uuid is None:
            return {"error": f"map {map_name!r} not found"}
        goal = ExecuteGoToPOI.Goal(poi_uuid=poi["uuid"], map_uuid=map_uuid)
        result, err = self._run_action(self._goto_poi_client, goal,
                                       TOOL_TIMEOUT["go_to_poi"], f"go_to_poi({poi_name!r})")
        if err:
            return {"error": err}
        r = result.result
        msg = _translate_result(result.status, getattr(r, "success", False),
                                getattr(r, "message", ""), getattr(r, "error_code", None))
        return {"status": msg, "poi": poi_name}

    def do_run_mission(self, mission_name: str, map_name: str = "") -> dict:
        data = self._data
        if not data.get("maps"):
            return {"error": "no maps loaded — call sync_data first or create a map in the web UI"}
        mission = self._find_by_name(data.get("missions", []), mission_name)
        if mission is None:
            known = [m["name"] for m in data.get("missions", [])]
            return {"error": f"mission {mission_name!r} not found — known: {known}"}
        map_uuid = self._map_uuid(map_name)
        if map_uuid is None:
            return {"error": f"map {map_name!r} not found"}
        goal = ExecuteMission.Goal(mission_uuid=mission["uuid"], map_uuid=map_uuid)
        result, err = self._run_action(self._mission_client, goal,
                                       TOOL_TIMEOUT["run_mission"], f"run_mission({mission_name!r})")
        if err:
            return {"error": err}
        r = result.result
        msg = _translate_result(result.status, getattr(r, "success", False),
                                getattr(r, "message", ""), getattr(r, "error_code", None))
        return {"status": msg, "mission": mission_name}

    def do_dock(self, dock_name: str, map_name: str = "") -> dict:
        map_uuid = self._map_uuid(map_name)
        if map_uuid is None:
            return {"error": "no maps loaded — call sync_data first"}
        goal = MapDock.Goal(dock_name=dock_name, map_uuid=map_uuid)
        result, err = self._run_action(self._dock_map_client, goal,
                                       TOOL_TIMEOUT["dock"], f"dock({dock_name!r})")
        if err:
            return {"error": err}
        r = result.result
        msg = _translate_result(result.status, getattr(r, "success", False),
                                getattr(r, "message", ""), getattr(r, "error_code", None))
        return {"status": msg, "dock": dock_name}

    def do_dock_local(self, dock_name: str) -> dict:
        goal = Dock.Goal(dock_name=dock_name)
        result, err = self._run_action(self._dock_local_client, goal,
                                       TOOL_TIMEOUT["dock_local"], f"dock_local({dock_name!r})")
        if err:
            return {"error": err}
        r = result.result
        msg = _translate_result(result.status, getattr(r, "success", False),
                                getattr(r, "message", ""), getattr(r, "error_code", None))
        return {"status": msg, "dock": dock_name}

    def do_undock(self, dock_name: str) -> dict:
        goal = Undock.Goal(dock_name=dock_name)
        result, err = self._run_action(self._undock_client, goal,
                                       TOOL_TIMEOUT["undock"], f"undock({dock_name!r})")
        if err:
            return {"error": err}
        r = result.result
        msg = _translate_result(result.status, getattr(r, "success", False),
                                getattr(r, "message", ""), getattr(r, "error_code", None))
        return {"status": msg, "dock": dock_name}

    def do_stop(self) -> dict:
        resp = self._call_srv(self._stop_client, Trigger.Request(), TOOL_TIMEOUT["stop"])
        if resp is None:
            return {"error": "autonomy/stop service not available"}
        return {"success": resp.success, "message": resp.message}


def _detect_ns(ros_args: list[str]) -> str:
    ns_env = normalize_namespace(os.environ.get("ONAV_NAMESPACE", ""))
    if ns_env:
        print(f"Namespace: {ns_env} (from $ONAV_NAMESPACE)", flush=True)
        return ns_env

    print("Detecting robot namespace...", flush=True)
    rclpy.init(args=ros_args)
    probe = Node("onav_mcp_probe")
    exec_ = SingleThreadedExecutor()
    exec_.add_node(probe)
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        exec_.spin_once(timeout_sec=0.1)
    detected = _detect_robot_namespaces(probe)
    probe.destroy_node()
    rclpy.shutdown()

    if not detected:
        print("ERROR: no robot namespace found — is the stack up?", file=sys.stderr, flush=True)
        sys.exit(1)
    if len(detected) > 1:
        print(f"ERROR: multiple namespaces {detected} — set $ONAV_NAMESPACE to pick one",
              file=sys.stderr, flush=True)
        sys.exit(1)
    ns = detected[0]
    print(f"Namespace: {ns}", flush=True)
    return ns


def build_server(node: OnavNode, host: str, port: int) -> FastMCP:
    mcp = FastMCP("onav", host=host, port=port)

    @mcp.tool()
    def sync_data() -> dict:
        """Fetch maps, missions, POIs, and docks from the robot.

        Call at startup and after creating new maps or missions in the web UI.
        Returns {"namespace":"...", "maps":[{"name","uuid"}], "missions":[...], "pois":[...], "docks":[{"name"}]}.
        """
        return node.do_sync()

    @mcp.tool()
    def get_state() -> dict:
        """Current robot state: activity, paused flag, battery percent, GPS position."""
        return node.do_get_state()

    @mcp.tool()
    def go_to_poi(poi_name: str, map_name: str = "") -> dict:
        """Drive the robot to a named point of interest. Blocks until arrival.

        poi_name: name (or partial name) of the POI as shown in the OutdoorNav UI.
        map_name: optional map to use. Defaults to the first synced map.
        """
        return node.do_goto_poi(poi_name, map_name)

    @mcp.tool()
    def run_mission(mission_name: str, map_name: str = "") -> dict:
        """Run a named autonomous mission. Blocks until the mission completes.

        mission_name: name (or partial name) of the mission as shown in the UI.
        map_name: optional map to use. Defaults to the first synced map.
        """
        return node.do_run_mission(mission_name, map_name)

    @mcp.tool()
    def dock(dock_name: str, map_name: str = "") -> dict:
        """Map-based docking at a named dock. Robot must be at the pre-dock position.

        dock_name: name of the dock as it appears in the docking database.
        map_name: optional map override.
        """
        return node.do_dock(dock_name, map_name)

    @mcp.tool()
    def dock_local(dock_name: str) -> dict:
        """Local-sensor docking. Robot must be facing the dock.

        dock_name: name of the dock as it appears in the docking database.
        Use this when dock_map is not available or the robot is already facing the dock.
        """
        return node.do_dock_local(dock_name)

    @mcp.tool()
    def undock(dock_name: str) -> dict:
        """Undock from the named dock.

        dock_name: name of the dock the robot is currently docked at.
        """
        return node.do_undock(dock_name)

    @mcp.tool()
    def stop() -> dict:
        """Immediately stop the autonomy stack. Robot brakes and becomes idle.

        Pauses an in-flight mission without cancelling the ROS 2 action goal.
        For full preemption of a blocking tool call, the operator presses Ctrl-C.
        """
        return node.do_stop()

    @mcp.tool()
    def kill() -> dict:
        """Cancel the active goal and stop the robot. Use when stuck or unsafe.

        Sets the cancel flag (interrupts any blocking tool), cancels the in-flight ROS 2 action,
        and calls autonomy/stop. Resets the cancel flag so the next command works normally.
        """
        node.cancel_active()
        node.clear_cancel()  # clear before stop so _wait() doesn't short-circuit
        stop_result = node.do_stop()
        return {"cancelled": True, "stop": stop_result}

    return mcp


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8091)
    args, ros_args = parser.parse_known_args()

    ns = _detect_ns(ros_args)

    rclpy.init(args=ros_args)
    node = OnavNode(ns)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    # Fire startup sync in background so server accepts connections immediately.
    threading.Thread(target=node.do_sync, daemon=True, name="startup-sync").start()

    print(f"onav-console server listening on {args.host}:{args.port}", flush=True)
    try:
        build_server(node, args.host, args.port).run(transport="streamable-http")
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
