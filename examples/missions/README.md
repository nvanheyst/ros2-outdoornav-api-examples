# missions/ - build, schedule, loop, and traverse missions

Standalone `rclpy` examples for turning maps into missions and running them. Dev
container + connection setup are in the [top-level README](../../README.md).

| File | API surface | What it does |
|---|---|---|
| `generate_traversal_mission.py` | `mission_manager/{get_map, create_mission, create_waypoint}`, optional `autonomy/mission` | **Persistent**, edge-following traversal. Builds a NetworkMission + Waypoints in the database so you can re-run it from the UI. Sequential for chain maps, least-turn graph walk for mesh. Optional `--run`. |
| `traverse_entire_map_gotos.py` | `mission_manager/get_map`, `autonomy/goto` | **Ephemeral**, free-GPS traversal. Fires one ExecuteGoTo per node in greedy nearest-neighbour order from the current fix. No mission stored. Ignores edges. |
| `random_visit_mission.py` | `mission_manager/get_map`, `autonomy/goto` | Random GoTo goals inside the map bbox. Soak test. |
| `loop_mission_battery_aware.py` | `autonomy/mission`, `platform/bms/state` | Loop a mission until battery drops below threshold. For robots without a dock. |
| `schedule_mission.py` | `autonomy/mission` | Wait until a wall-clock target (ISO 8601 or `+30s/+5m`) then fire ExecuteMission. |
| `run_mission.py` | `autonomy/mission`, optional `log_manager`, optional `<camera>/start_recording` | Run a mission with optional extras: prompted Y/N for log and camera video. The simplest way to fire a mission from a script with progressively more features. |

Recording a *path* by driving lives in [`maps/record_path.py`](../maps/README.md);
abort-recovery is in [`ops/mission_with_resume.py`](../ops/README.md).
