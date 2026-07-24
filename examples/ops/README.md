# ops/ - readiness, diagnostics, cleanup, notifications

Operational examples: check the robot is ready to run, snapshot its state, watch
for failures, and clean up test data. Dev container + connection setup are in the
[top-level README](../../README.md).

| File | API surface | What it does |
|---|---|---|
| `connect_real_robot.py` | `localization/fix`, `autonomy/status`, ROS graph | Read-only connection check for a real robot: confirms transport (Fast DDS discovery server vs CycloneDDS), auto-detects namespace, and shows live fix + autonomy state. First step when switching from sim to a live AMP. |
| `preflight.py` | `diagnostics_{toplevel_state,agg}`, `localization/fix`, tf, `platform/{emergency_stop,bms/state}`, `sensors/lidar3d_0/nonground_filtered`, `collision_monitor/get_parameters`, `autonomy/{mission,dock_map}` (wait_for_server) | **Go / no-go gate.** Checks the things that stop autonomy - system-health ERROR, localization, collision detection actually fed (not just enabled), docking + mission action servers, e-stop, battery - and prints READY / NOT READY. Read-only. Fails only on functional blockers; diagnostics WARN and low-but-ok battery are notes, not failures. |
| `where_am_i.py` | `localization/fix` | Print the latest GPS fix and exit. |
| `doctor.py` | `autonomy/status`, `localization/fix`, `platform/bms/state`, `autonomy/mission/_action/status`, optional `<lifecycle_node>/get_state` | One-call diagnostic snapshot: env, autonomy state, battery, GPS fix age, last mission status, key topic liveness, and (with `--include-lifecycle`) every managed-node state. |
| `service_inventory.py` | ROS graph | List all services live on the graph; `--grep` filters. First stop when a wait-for-service hangs - paths and types differ across OutdoorNav releases. |
| `mission_with_resume.py` | `autonomy/mission`, `autonomy/mission_from_goal`, `navigation/current_goal_id` | Background resilience process: run ExecuteMission, and on abort retry from the last in-flight waypoint via ExecuteMissionFromGoal up to `--max-retries` times, logging the rich abort detail (code, message, per-waypoint goal_states) each time. |
| `notify_on_mission_failure.py` | `autonomy/mission/_action/status`, `autonomy/status`, `localization/fix`, optional `sensors/camera_*/color/compressed` | Passive watcher: ping a notifier when any mission goal aborts, regardless of who launched it. `--via stdout,email,webhook,sms`; email attaches the latest camera frame if `--camera-topic` is set. |
| `delete_logs.py` | `logger/{get_all_logs, delete_log}` | Enumerate event logs and delete each. `--keep-recent`, `--dry-run`, `--keep-media` / `--keep-record`. |
| `delete_all.py` | `mission_manager/{delete_all, get_all_maps, get_all_network_missions, get_all_points_of_interest}` | Wipe every map, mission, POI - cleanup after these examples litter the UI. `--dry-run` lists counts; `--confirm` fires. |
