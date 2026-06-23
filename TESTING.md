# Manual smoke sweep

Sequenced walk-through that exercises every example at least once. Each
entry: setup, command, expected output, failure modes. Stop at the first
failure and triage before moving on.

## Prerequisites

1. OnAV stack up and reachable. From the OnAV web UI confirm:
   - Robot publishes a stable `/localization/fix` (green status badge).
   - At least one map exists; note its UUID.
   - At least one mission exists; note its UUID.
   - At least one POI exists; note its UUID.
2. ROS env sourced: `source /opt/ros/jazzy/setup.bash` plus the
   clearpath overlay.
3. Export defaults to avoid retyping:
   ```bash
   export ONAV_NAMESPACE=/a300_00003
   export ONAV_MAP_ID=<...>
   export ONAV_MISSION_ID=<...>
   export ONAV_POI_ID=<...>
   ```
4. Run `./scripts/smoke_all.sh` and confirm 0 failures. Skipped is OK
   when run outside the dev container.

## Live sanity (before the sweep)

| Command | Expected | Failure mode |
|---|---|---|
| `./examples/diagnostics/service_inventory.py --grep mission_manager` | Lists at least `create_map`, `create_mission`, `create_waypoint`, `get_map`, `get_all_maps`, etc. | Empty output → wrong namespace or wrong DDS config; `autonomy/pause` only → you're talking to an OnAV release that doesn't expose `control_selection/pause`. |
| `./examples/diagnostics/where_am_i.py` | Prints `lat,lon,alt` line on stdout, exit 0. | Timeout → robot doesn't have a fix. |

## Maps

| Step | Command | Expected | Failure mode |
|---|---|---|---|
| Load from file | `./examples/maps/load_map_from_file.py path/to/map.json --name "smoke_load"` | Logs `map_uuid=…`, new map visible in UI under "smoke_load". | "No map name" → JSON missing `name`; "no uuid" → service returned empty response. |
| Square row coverage | `./examples/maps/row_generator_square.py --lat 50.1094 --lon -97.3187 --width 30 --height 20 --spacing 5 --name smoke_rows` | Logs `created map 'smoke_rows' (…) with N nodes, N-1 edges`. UI shows snake of nodes. | "no GPS fix" → not applicable here (no fix needed); errors during create → check service is up. |
| Polygon row coverage | `./examples/maps/row_generator_polygon.py --tag cov-2` | Requires ≥3 POIs tagged `cov-2`. Logs node + edge counts. | "need >=3 POIs" → tag mismatch; pyproj/shapely import error → `pip install -r requirements.txt`. |
| Bulk edit edges | `./examples/maps/bulk_edit_edges.py --speed 0.5 -- $ONAV_MAP_ID 50.1094 -97.3187 15` then `./examples/maps/bulk_edit_edges.py $ONAV_MAP_ID 50.1094 -97.3187 15 --dry-run` | Dry-run lists matching edges; live call updates them. | Edges array empty → centre is too far from the map; permission denied → wrong namespace. |

## Missions

| Step | Command | Expected | Failure mode |
|---|---|---|---|
| Build traversal mission | `./examples/missions/generate_traversal_mission.py --name smoke_traversal` | "chain map detected …" or "mesh map detected …", then waypoints appended in batches of 10. Mission appears in UI. | "GetMap returned no points" → wrong map id; "CreateNetworkMission returned no uuid" → service issue. |
| Build + run | `./examples/missions/generate_traversal_mission.py --name smoke_traversal --run` | As above, then mission goes through ExecuteMission and the robot drives. | Goal rejected → autonomy not in idle; sim/HW issue. |
| Random visit | `./examples/missions/random_visit_mission.py --count 3` | Three GoTo goals fire; each prints `OK` or `FAILED — continuing`. | Many `FAILED` → tolerance / map bbox too generous; goal rejected → mission already in flight. |
| Traverse entire map (greedy) | `./examples/missions/traverse_entire_map_shortest.py` | Prints naive vs greedy length, then ExecuteGoTo per node. | "no points on map" → bad map id; "still waiting" loop → action server down. |
| Schedule a mission | `./examples/missions/schedule_mission.py +30s` | Countdown, then mission fires at T0. | "target time is in the past" → date parsing issue; permission denied → namespace. |
| Battery loop | `./examples/missions/loop_mission_battery_aware.py --threshold 30 --loops 2` | Two loop iterations (assuming battery > 30%). | Battery never drops — fine; mission rejected at each loop — autonomy state. |
| Record path | `./examples/missions/record_path.py smoke_recorded --min-distance 1.0` | Subscribes to fix, prints "N points…" every 10 samples. Ctrl-C → "captured N raw points → simplified to M → map smoke_recorded created". | No points accumulate → no fix; `create_map` errors → map service. |

## Control

| Step | Command | Expected | Failure mode |
|---|---|---|---|
| Forward 1 m at 0.2 m/s | `./examples/control/drive_robot_forward.py --distance 1 --velocity 0.2` | Robot rolls ~1 m, stops; "stopped" log. | No motion → controller disabled, e-stop pressed. |
| Service inventory (paused state) | `./examples/diagnostics/service_inventory.py --grep pause` | Lists `pause`/`resume` services your stack exposes; pick variant. | Empty → autonomy isn't running. |
| Pause/resume (SetBool variant) | `./examples/control/pause_resume.py --hold 3` | Pause OK → 3 s → resume OK. | "service not available" → try `--variant trigger`. |
| Pause/resume (Trigger variant) | `./examples/control/pause_resume.py --variant trigger --hold 3` | Same as above, opposite variant. | Same. |
| Pause + teleop in-mission | `./examples/control/pause_and_teleop.py --hold 5` | GoToPOI fires → after 10 s pauses → 180° turn + 1 m drive while paused → resume → completes. | POI rejected → wrong POI uuid; autonomy/stop fails → ignore (best-effort cleanup). |
| Stop autonomy | `./examples/control/stop_autonomy.py` | `OK` log, robot brakes. | "service not available" → wrong namespace. |

## Data

| Step | Command | Expected | Failure mode |
|---|---|---|---|
| List logs (dry) | `./examples/data/delete_logs.py --dry-run` | "found N log(s) … will delete N (dry-run)" | Service unavailable → logger not running. |
| Purge oldest only | `./examples/data/delete_logs.py --keep-recent 1 --dry-run` then drop `--dry-run` | Lists then deletes everything > 1 h old. | Same. |

## Diagnostics

Both already exercised above. Re-run anytime to confirm the stack still
matches the assumptions in your other commands.

## What "failure" means here

Any of:
- Script crashes (non-zero exit, traceback).
- Script "succeeds" but the UI shows the wrong state (e.g. mission
  created but contains zero waypoints).
- A service path printed in the log is wrong for your release.

For the last case, update the script's service constant or pass
`--namespace …`. Don't paper over it: if the API path has moved, the
example is stale and the README's index needs updating.
