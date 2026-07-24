# missions/ - build, schedule, loop, and traverse missions

Standalone `rclpy` examples for turning maps into missions and running them. Dev
container + connection setup are in the [top-level README](../../README.md).

| File | API surface | What it does |
|---|---|---|
| `generate_traversal_mission.py` | `mission_manager/{get_map, create_mission, create_waypoint}`, optional `autonomy/mission` | **Persistent**, edge-following traversal. Builds a NetworkMission + Waypoints in the database so you can re-run it from the UI. Sequential for chain maps, least-turn graph walk for mesh. Optional `--run`. |
| `traverse_entire_map_gotos.py` | `mission_manager/get_map`, `autonomy/goto` | **Ephemeral**, free-GPS traversal. Fires one ExecuteGoTo per node in greedy nearest-neighbour order from the current fix. No mission stored. Ignores edges. |
| `random_visit_mission.py` | `mission_manager/get_map`, `autonomy/goto` | Random GoTo goals inside the map bbox. Soak test. |
| `go_to_poi.py` | `mission_manager/{get_all_maps, get_all_points_of_interest}`, `autonomy/goto_poi` | Drive to a single named POI (ExecuteGoToPOI). Interactive map + POI menu, or provide UUIDs directly. |
| `mission_feedback.py` | `mission_manager/{get_all_maps, get_all_network_missions}`, `autonomy/mission` | Run ExecuteMission with a `feedback_callback` and a manual spin loop so per-waypoint progress arrives during execution. |
| `loop_mission_battery_aware.py` | `autonomy/mission`, `platform/bms/state` | Loop a mission until battery drops below threshold. For robots without a dock. |
| `loop_mission_dock_charge.py` | `autonomy/mission`, `autonomy/{dock_local, undock}`, `platform/bms/state` | Loop indefinitely, docking to charge when battery falls below `--dock-threshold` and resuming when it reaches `--resume-threshold`. For robots with a dock. |
| `visit_pois_by_tag.py` | `mission_manager/{get_all_maps, get_all_points_of_interest}`, `autonomy/goto_poi` | Visit every POI matching a tag in order (ExecuteGoToPOI), for `--loops` iterations. Useful for inspection routes tagged in the UI. |
| `schedule_mission.py` | `autonomy/mission` | Wait until a wall-clock target (ISO 8601 or `+30s/+5m`) then fire ExecuteMission. |
| `run_mission.py` | `autonomy/mission`, optional `log_manager`, optional `<camera>/start_recording` | Run a mission with optional extras: prompted Y/N for log and camera video. The simplest way to fire a mission from a script with progressively more features. |

Recording a *path* by driving lives in [`maps/record_path.py`](../maps/record_path.py);
abort-recovery is in [`ops/mission_with_resume.py`](../ops/mission_with_resume.py).
