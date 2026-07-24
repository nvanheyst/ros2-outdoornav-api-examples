# Manual smoke sweep

Sequenced walk-through that exercises every example at least once. Each
entry: setup, command, expected output, failure modes. Stop at the first
failure and triage before moving on.

## Prerequisites

1. OutdoorNav stack up and reachable. From the OutdoorNav web UI confirm:
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
4. Run `./docker/ci/smoke_all.sh` and confirm 0 failures. Skipped is OK
   when run outside the dev container.

## Live sanity (before the sweep)

| Command | Expected | Failure mode |
|---|---|---|
| `./examples/ops/service_inventory.py --grep mission_manager` | Lists at least `create_map`, `create_mission`, `create_waypoint`, `get_map`, `get_all_maps`, etc. | Empty output → wrong namespace or wrong DDS config; `autonomy/pause` only → you're talking to an OutdoorNav release that doesn't expose `control_selection/pause`. |
| `./examples/ops/where_am_i.py` | Prints `lat,lon,alt` line on stdout, exit 0. | Timeout → robot doesn't have a fix. |

## Maps

| Step | Command | Expected | Failure mode |
|---|---|---|---|
| Load from file | `./examples/maps/load_map_from_file.py path/to/map.json --name "smoke_load"` | Logs `map_uuid=…`, new map visible in UI under "smoke_load". | "No map name" → JSON missing `name`; "no uuid" → service returned empty response. |
| Square row coverage | `./examples/maps/row_generator_square.py --lat 50.1094 --lon -97.3187 --width 30 --height 20 --spacing 5 --name smoke_rows` | Logs `created map 'smoke_rows' (…) with N nodes, N-1 edges`. UI shows snake of nodes. | "no GPS fix" → not applicable here (no fix needed); errors during create → check service is up. |
| Polygon row coverage | `./examples/maps/row_generator_polygon.py --tag cov-2` | Requires ≥3 POIs tagged `cov-2`. Logs node + edge counts. | "need >=3 POIs" → tag mismatch; pyproj/shapely import error → `pip install -r docker/requirements.txt`. |
| Coverage from here | `./examples/maps/row_generator_from_here.py --dry-run` then `./examples/maps/row_generator_from_here.py --width 30 --height 20 --spacing 5 --name smoke_from_here` | Dry-run prints the GPS fix + heading it would use. Live: reads current position + heading, logs node + edge counts, map appears in UI rows aligned to robot heading. | "no fix within N s" → robot not localized; "no TF for heading" → TF not publishing map→base_link. |
| Bulk edit edges | `./examples/maps/bulk_edit_edges.py 50.1094 -97.3187 --radius 15 --map-uuid $ONAV_MAP_ID --dry-run` then drop `--dry-run` and add `--speed 0.5` | Dry-run lists matching edges; live call updates them. | Edges array empty → centre is too far from the map; permission denied → wrong namespace. |
| Slow zone around me | `./examples/maps/bulk_edit_edges.py --around-me --radius 15 --speed 0.3 --map-uuid $ONAV_MAP_ID --dry-run` then drop `--dry-run` | Waits for fix, then lists/updates edges within 15 m of the robot. | "no fix within 10 s" → robot not localized. |

## Missions

| Step | Command | Expected | Failure mode |
|---|---|---|---|
| Build traversal mission | `./examples/missions/generate_traversal_mission.py --name smoke_traversal` | "chain map detected …" or "mesh map detected …", then waypoints appended in batches of 10. Mission appears in UI. | "GetMap returned no points" → wrong map id; "CreateNetworkMission returned no uuid" → service issue. |
| Build + run | `./examples/missions/generate_traversal_mission.py --name smoke_traversal --run` | As above, then mission goes through ExecuteMission and the robot drives. | Goal rejected → autonomy not in idle; sim/HW issue. |
| Random visit | `./examples/missions/random_visit_mission.py --count 3` | Three GoTo goals fire; each prints `OK` or `FAILED - continuing`. | Many `FAILED` → tolerance / map bbox too generous; goal rejected → mission already in flight. |
| Traverse entire map via gotos | `./examples/missions/traverse_entire_map_gotos.py` | Prints naive vs greedy length, then ExecuteGoTo per node. | "no points on map" → bad map id; "still waiting" loop → action server down. |
| Schedule a mission | `./examples/missions/schedule_mission.py +30s` | Countdown, then mission fires at T0. | "target time is in the past" → date parsing issue; permission denied → namespace. |
| Battery loop | `./examples/missions/loop_mission_battery_aware.py --threshold 30 --loops 2` | Two loop iterations (assuming battery > 30%). | Battery never drops - fine; mission rejected at each loop - autonomy state. |
| Loop + dock to charge | `./examples/missions/loop_mission_dock_charge.py --loops 1 --dock-threshold 5` | One loop; battery > 5% so it skips the charge wait and finishes cleanly. Set `--dock-threshold` above current battery to exercise the dock-and-wait path. | "dock not found" → pass `--dock-name`; "undock rejected" → docking stack not running. |
| Visit POIs by tag | `./examples/missions/visit_pois_by_tag.py --tag goto --loops 1` | Lists POIs tagged `goto`, drives to each in alphabetical order, logs result per POI. | "no POIs with tag 'goto'" → assign the tag in the UI or use a different tag; "GoToPOI rejected" → autonomy busy. |
| Go to POI (dry) | `./examples/missions/go_to_poi.py --dry-run` | Prints the three API paths (maps service, POIs service, goto_poi action). | None. |
| Go to POI (live) | `./examples/missions/go_to_poi.py` | Interactive map + POI menu → ExecuteGoToPOI accepted → "arrived at <name>" log. | "GoToPOI goal rejected" → autonomy busy; "no POIs found" → no POIs on this map. |
| Mission with feedback (dry) | `./examples/missions/mission_feedback.py --dry-run` | Prints the three API paths. | None. |
| Mission with feedback (live) | `./examples/missions/mission_feedback.py` | Interactive map + mission menu → "waypoint N/M (X%)" lines printed as the robot drives; "mission succeeded" at the end. | No feedback lines but mission runs → confirm the RMW isn't swallowing feedback (spin loop should drain it). |
| Record path | `./examples/maps/record_path.py smoke_recorded --min-distance 1.0` | Subscribes to fix, prints "N points…" every 10 samples. Ctrl-C → "captured N raw points → simplified to M → map smoke_recorded created". | No points accumulate → no fix; `create_map` errors → map service. |
| Run mission with log (plumbing) | `./examples/missions/run_mission.py --skip-mission --log` | start_recording OK → 5 s sleep → stop_recording OK. New log shows up in the UI. | "service not available" → run `service_inventory.py --grep log_manager`. |
| Run mission with log (full run) | `./examples/missions/run_mission.py --log` | start_recording → pick map + mission → ExecuteMission runs to completion → stop_recording. UI shows a single bracketed log. | "Goal rejected" → autonomy busy; stop_recording still fires in the finally block. |

## Control

| Step | Command | Expected | Failure mode |
|---|---|---|---|
| Forward 1 m at 0.2 m/s | `./examples/control/drive_robot_forward.py --distance 1 --velocity 0.2` | Robot rolls ~1 m, stops; "stopped" log. | No motion → controller disabled, e-stop pressed. |
| Service inventory (paused state) | `./examples/ops/service_inventory.py --grep pause` | Lists `pause`/`resume` services your stack exposes; pick variant. | Empty → autonomy isn't running. |
| Pause/resume (autonomy SetBool - default, 2.3) | `./examples/control/pause_resume.py --hold 3` | Pause OK → 3 s → resume OK. | "service not available" → try `--variant control_setbool` or `--variant autonomy_trigger`. |
| Pause/resume (control_selection SetBool - legacy) | `./examples/control/pause_resume.py --variant control_setbool --hold 3` | Same lifecycle on the older path. | Same. |
| Pause/resume (autonomy Trigger - legacy) | `./examples/control/pause_resume.py --variant autonomy_trigger --hold 3` | Same lifecycle on the older type. | Same. |
| Stop autonomy | `./examples/control/stop_autonomy.py` | `OK` log, robot brakes. | "service not available" → wrong namespace. |
| Dock at named dock | `./examples/control/dock_now.py --dock-name smoke_dock` | Picks dock from DB, fires Dock action to completion. | "dock not found" → list available docks with `service_inventory.py --grep dock_database`; "action unavailable" → docking stack not started. |
| Dock via map (dry) | `./examples/control/dock_map.py` with no dock in DB → interactive menu; with `--dock-name smoke_dock --map-uuid <uuid>` → immediate send | MapDock goal accepted → "docked at 'smoke_dock'" log. | "dock goal rejected" → MapDock action server unavailable (run `preflight.py` to check); dock result success=False → dock not reachable or wrong name. |

## Ops

| Step | Command | Expected | Failure mode |
|---|---|---|---|
| Preflight | `./examples/ops/preflight.py` | READY with all checks green, or NOT READY + named blockers. | NOT READY → fix the listed blockers before running a mission; `wait_for_server` hangs → action server not up (check `service_inventory.py --grep dock_map`). |
| Connect (real robot) | From `~/onav-lab/docker/` on the robot: `docker compose --env-file real-amp.env run --rm dev python3 examples/ops/connect_real_robot.py` | Detects Fast DDS transport, auto-detects namespace, prints live GPS fix + autonomy state. | "no namespace found" → discovery server not reachable; wrong `ROS_DOMAIN_ID` → check `robot.yaml`. |
| List logs (dry) | `./examples/ops/delete_logs.py --dry-run` | "found N log(s) … will delete N (dry-run)" | Service unavailable → logger not running. |
| Purge oldest only | `./examples/ops/delete_logs.py --keep-recent 1 --dry-run` then drop `--dry-run` | Lists then deletes everything > 1 h old. | Same. |
| Delete entry but keep media | `./examples/ops/delete_logs.py --keep-media --keep-record --dry-run` then drop `--dry-run` | UI entries vanish, files stay on disk. | Same. |
| Database wipe (dry) | `./examples/ops/delete_all.py --dry-run` | Prints `N maps, M missions, K POIs would be deleted`. | get_all_* hangs → mission_manager backend not responsive on this stack. |
| Database wipe (live) | `./examples/ops/delete_all.py --confirm` | `OK` from delete_all; UI now empty. **Irreversible.** | Refused without `--confirm` - that's the safety. |
| Doctor snapshot | `./examples/ops/doctor.py --include-lifecycle` | Sections: env, autonomy, localization, power, topic liveness, Nav2 lifecycle. Each non-empty when the stack is healthy. | `(no message)` rows → topic isn't publishing in this namespace; lifecycle empty → `--include-lifecycle` was omitted or no managed nodes match the ns filter. |
| Doctor with short collect | `./examples/ops/doctor.py --collect 1` | Same shape, may miss low-rate topics. | If battery / fix sections are empty, bump `--collect`. |
| Notify watcher (dry) | `./examples/ops/notify_on_mission_failure.py --dry-run --via stdout,email,webhook,sms` | Prints the formatted body for every backend; no network. | Body missing fields → `FailureContext` dataclass out of sync with template. |
| Notify watcher (live, stdout) | `./examples/ops/notify_on_mission_failure.py` then in another shell start a mission and Ctrl-C `./examples/missions/run_mission.py`, or trigger a real abort (e-stop, blocked path) | After the cancel, watcher prints nothing (cancels are silent by default). Re-run with `--also-on-cancel` to confirm; for a real abort, drop an e-stop or block the path. | No status seen → topic name mismatch; subscribed before any mission ran → expected. |
| Notify watcher (live, email + sms) | `NOTIFY_SMTP_* … TWILIO_* … ./examples/ops/notify_on_mission_failure.py --via email,sms --camera-topic <ns>/sensors/camera_0/color/compressed` then trigger an abort | Email arrives with body + JPEG attachment; SMS arrives with one-line summary. | Email auth fail → SMTP creds / 2FA app password; SMS fail → Twilio creds / verified number. |
| `where_am_i`, `service_inventory` | already exercised in **Live sanity** at the top. | - | - |

## Patterns

| Step | Command | Expected | Failure mode |
|---|---|---|---|
| Graceful shutdown (live, Ctrl-C) | `./examples/patterns/graceful_shutdown.py` then Ctrl-C while the mission is in flight | "Ctrl-C - cancelling in-flight goal" log; mission status returns 5 (CANCELED); robot brakes. | Status returns 4 (SUCCEEDED) → you waited too long, mission finished before the interrupt; status missing → cancel ack timed out (check `cancel_goal_blocking` timeout). |
| Param list | `./examples/patterns/parameter_runtime.py --node <ns>/controller_server list` | Prints parameter names, one per line, then a `-- N parameter(s) --` footer. | Empty → wrong node name or service not exposed. |
| Param get | `./examples/patterns/parameter_runtime.py --node <ns>/controller_server get max_vel_x` | `max_vel_x = 0.5  (double)` style output. | "parameter not declared" → name typo or not exposed. |
| Param set | `./examples/patterns/parameter_runtime.py --node <ns>/controller_server set max_vel_x 0.3` | `OK: max_vel_x <- 0.3 (double)` | "FAILED: parameter is read-only" → declared with `read_only=True`; "rejected" → outside declared range. |
| Perception gate (dry) | `./examples/patterns/perception_gate.py --dry-run` — not applicable; no dry-run. Use: `./examples/patterns/perception_gate.py` in background, then `ros2 topic pub -1 /perception_gate std_msgs/Bool '{data: false}'` and `'{data: true}'` | Log shows "autonomy paused" / "autonomy resumed" on each message. | "service not available" → wrong namespace; no log → Bool topic mismatch. |

## Recovery

| Step | Command | Expected | Failure mode |
|---|---|---|---|
| Mission with resume (dry) | `./examples/ops/mission_with_resume.py --dry-run` | Lists the action paths and retry params without firing anything. | None expected. |
| Mission with resume (live, no failure) | `./examples/ops/mission_with_resume.py --max-retries 2` | Runs ExecuteMission, hits SUCCEEDED, logs "mission succeeded after 1 attempt(s)" and exits. | If autonomy is busy → goal rejected on first attempt; check stack state. |
| Mission with resume (live, induced failure) | Same command, then drop an obstacle in the path so the first attempt aborts | After abort: rich detail printed (code, goal_states), `--backoff` sleep, then ExecuteMissionFromGoal from the last in-flight waypoint. Up to `--max-retries` retries. | Robot can't replan → exhausts retries; that's the intended terminal behaviour. |

## What "failure" means here

Any of:
- Script crashes (non-zero exit, traceback).
- Script "succeeds" but the UI shows the wrong state (e.g. mission
  created but contains zero waypoints).
- A service path printed in the log is wrong for your release.

For the last case, update the script's service constant or pass
`--namespace …`. Don't paper over it: if the API path has moved, the
example is stale and the README's index needs updating.
