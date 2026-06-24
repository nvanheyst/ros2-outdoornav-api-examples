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
4. Run `./ci/smoke_all.sh` and confirm 0 failures. Skipped is OK
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
| Polygon row coverage | `./examples/maps/row_generator_polygon.py --tag cov-2` | Requires ≥3 POIs tagged `cov-2`. Logs node + edge counts. | "need >=3 POIs" → tag mismatch; pyproj/shapely import error → `pip install -r requirements.txt`. |
| Bulk edit edges | `./examples/maps/bulk_edit_edges.py --speed 0.5 -- $ONAV_MAP_ID 50.1094 -97.3187 15` then `./examples/maps/bulk_edit_edges.py $ONAV_MAP_ID 50.1094 -97.3187 15 --dry-run` | Dry-run lists matching edges; live call updates them. | Edges array empty → centre is too far from the map; permission denied → wrong namespace. |
| Slow zone around me | `./examples/maps/bulk_edit_edges.py $ONAV_MAP_ID 15 --around-me --speed 0.3 --dry-run` then drop `--dry-run` | Waits for fix, then lists/updates edges within 15 m of the robot. | "no fix within 10 s" → robot not localized. |

## Missions

| Step | Command | Expected | Failure mode |
|---|---|---|---|
| Build traversal mission | `./examples/missions/generate_traversal_mission.py --name smoke_traversal` | "chain map detected …" or "mesh map detected …", then waypoints appended in batches of 10. Mission appears in UI. | "GetMap returned no points" → wrong map id; "CreateNetworkMission returned no uuid" → service issue. |
| Build + run | `./examples/missions/generate_traversal_mission.py --name smoke_traversal --run` | As above, then mission goes through ExecuteMission and the robot drives. | Goal rejected → autonomy not in idle; sim/HW issue. |
| Random visit | `./examples/missions/random_visit_mission.py --count 3` | Three GoTo goals fire; each prints `OK` or `FAILED — continuing`. | Many `FAILED` → tolerance / map bbox too generous; goal rejected → mission already in flight. |
| Traverse entire map via gotos | `./examples/missions/traverse_entire_map_gotos.py` | Prints naive vs greedy length, then ExecuteGoTo per node. | "no points on map" → bad map id; "still waiting" loop → action server down. |
| Schedule a mission | `./examples/missions/schedule_mission.py +30s` | Countdown, then mission fires at T0. | "target time is in the past" → date parsing issue; permission denied → namespace. |
| Battery loop | `./examples/missions/loop_mission_battery_aware.py --threshold 30 --loops 2` | Two loop iterations (assuming battery > 30%). | Battery never drops — fine; mission rejected at each loop — autonomy state. |
| Record path | `./examples/missions/record_path.py smoke_recorded --min-distance 1.0` | Subscribes to fix, prints "N points…" every 10 samples. Ctrl-C → "captured N raw points → simplified to M → map smoke_recorded created". | No points accumulate → no fix; `create_map` errors → map service. |
| Mission with recording (plumbing) | `./examples/missions/mission_with_recording.py --skip-mission` | start_recording OK → 5 s sleep → stop_recording OK. New log shows up in the UI. | "service not available" → run `service_inventory.py --grep log_manager`; if your stack uses `StartRecording`/`StopRecording` (not Trigger), swap the import. |
| Mission with recording (full run) | `./examples/missions/mission_with_recording.py` | start_recording → ExecuteMission runs to completion → stop_recording. UI shows a single bracketed log. | "Goal rejected" → autonomy busy; stop_recording still fires in the finally block. |

## Control

| Step | Command | Expected | Failure mode |
|---|---|---|---|
| Forward 1 m at 0.2 m/s | `./examples/control/drive_robot_forward.py --distance 1 --velocity 0.2` | Robot rolls ~1 m, stops; "stopped" log. | No motion → controller disabled, e-stop pressed. |
| Service inventory (paused state) | `./examples/ops/service_inventory.py --grep pause` | Lists `pause`/`resume` services your stack exposes; pick variant. | Empty → autonomy isn't running. |
| Pause/resume (autonomy SetBool — default, 2.3) | `./examples/control/pause_resume.py --hold 3` | Pause OK → 3 s → resume OK. | "service not available" → try `--variant control_setbool` or `--variant autonomy_trigger`. |
| Pause/resume (control_selection SetBool — legacy) | `./examples/control/pause_resume.py --variant control_setbool --hold 3` | Same lifecycle on the older path. | Same. |
| Pause/resume (autonomy Trigger — legacy) | `./examples/control/pause_resume.py --variant autonomy_trigger --hold 3` | Same lifecycle on the older type. | Same. |
| Pause + teleop in-mission | `./examples/control/pause_and_teleop.py --hold 5` | GoToPOI fires → after 10 s pauses → 180° turn + 1 m drive while paused → resume → completes. | POI rejected → wrong POI uuid; autonomy/stop fails → ignore (best-effort cleanup). |
| Stop autonomy | `./examples/control/stop_autonomy.py` | `OK` log, robot brakes. | "service not available" → wrong namespace. |
| Cancel mission (self-started) | `./examples/control/cancel_mission.py --after 5` | Mission starts, runs ~5 s, cancels via goal handle, status returns 5 (CANCELED). | "goal rejected" → autonomy busy; "status=4" → mission completed faster than `--after`. |
| Dock workflow (plumbing) | `./examples/control/dock_workflow.py --dry-run` | Lists the 5 calls it would make. | None. |
| Dock workflow (live) | `./examples/control/dock_workflow.py --dock-name smoke_dock --template default_dock --backup-distance 2 --hold 5` | AddDockCurrentPose OK → 2 m back-up → Dock action succeeds → 5 s hold → Undock action succeeds → RemoveDock OK. | "AddDockCurrentPose failed" → wrong template name (check robot config); Dock action rejected → dock target not visible to the sensor; in sim, try `--template a300_default_dock` or whatever your sim ships. |

## Ops

| Step | Command | Expected | Failure mode |
|---|---|---|---|
| List logs (dry) | `./examples/ops/delete_logs.py --dry-run` | "found N log(s) … will delete N (dry-run)" | Service unavailable → logger not running. |
| Purge oldest only | `./examples/ops/delete_logs.py --keep-recent 1 --dry-run` then drop `--dry-run` | Lists then deletes everything > 1 h old. | Same. |
| Delete entry but keep media | `./examples/ops/delete_logs.py --keep-media --keep-record --dry-run` then drop `--dry-run` | UI entries vanish, files stay on disk. | Same. |
| Database wipe (dry) | `./examples/ops/delete_all.py --dry-run` | Prints `N maps, M missions, K POIs would be deleted`. | get_all_* hangs → mission_manager backend not responsive on this stack. |
| Database wipe (live) | `./examples/ops/delete_all.py --confirm` | `OK` from delete_all; UI now empty. **Irreversible.** | Refused without `--confirm` — that's the safety. |
| Doctor snapshot | `./examples/ops/doctor.py --include-lifecycle` | Sections: env, autonomy, localization, power, topic liveness, Nav2 lifecycle. Each non-empty when the stack is healthy. | `(no message)` rows → topic isn't publishing in this namespace; lifecycle empty → `--include-lifecycle` was omitted or no managed nodes match the ns filter. |
| Doctor with short collect | `./examples/ops/doctor.py --collect 1` | Same shape, may miss low-rate topics. | If battery / fix sections are empty, bump `--collect`. |
| Notify watcher (dry) | `./examples/ops/notify_on_mission_failure.py --dry-run --via stdout,email,webhook,sms` | Prints the formatted body for every backend; no network. | Body missing fields → `FailureContext` dataclass out of sync with template. |
| Notify watcher (live, stdout) | `./examples/ops/notify_on_mission_failure.py` then in another shell start and abort a mission (e.g. `./examples/control/cancel_mission.py --after 2`) | After the cancel, watcher prints nothing (cancels are silent by default). Re-run with `--also-on-cancel` to confirm; for a real abort, drop an e-stop or block the path. | No status seen → topic name mismatch; subscribed before any mission ran → expected. |
| Notify watcher (live, email + sms) | `NOTIFY_SMTP_* … TWILIO_* … ./examples/ops/notify_on_mission_failure.py --via email,sms --camera-topic <ns>/sensors/camera_0/color/compressed` then trigger an abort | Email arrives with body + JPEG attachment; SMS arrives with one-line summary. | Email auth fail → SMTP creds / 2FA app password; SMS fail → Twilio creds / verified number. |
| `where_am_i`, `service_inventory` | already exercised in **Live sanity** at the top. | — | — |

## Patterns

| Step | Command | Expected | Failure mode |
|---|---|---|---|
| Graceful shutdown (live, Ctrl-C) | `./patterns/graceful_shutdown.py` then Ctrl-C while the mission is in flight | "Ctrl-C — cancelling in-flight goal" log; mission status returns 5 (CANCELED); robot brakes. | Status returns 4 (SUCCEEDED) → you waited too long, mission finished before the interrupt; status missing → cancel ack timed out (check `cancel_goal_blocking` timeout). |
| Param list | `./patterns/parameter_runtime.py --node <ns>/controller_server list` | Prints parameter names, one per line, then a `-- N parameter(s) --` footer. | Empty → wrong node name or service not exposed. |
| Param get | `./patterns/parameter_runtime.py --node <ns>/controller_server get max_vel_x` | `max_vel_x = 0.5  (double)` style output. | "parameter not declared" → name typo or not exposed. |
| Param set | `./patterns/parameter_runtime.py --node <ns>/controller_server set max_vel_x 0.3` | `OK: max_vel_x <- 0.3 (double)` | "FAILED: parameter is read-only" → declared with `read_only=True`; "rejected" → outside declared range. |

## Recovery

| Step | Command | Expected | Failure mode |
|---|---|---|---|
| Recover from abort (dry) | `./examples/missions/recover_from_abort.py --dry-run` | Lists the action paths and retry params without firing anything. | None expected. |
| Recover from abort (live, no failure) | `./examples/missions/recover_from_abort.py --max-retries 2` | Runs ExecuteMission, hits SUCCEEDED, logs "mission succeeded after 1 attempt(s)" and exits. | If autonomy is busy → goal rejected on first attempt; check stack state. |
| Recover from abort (live, induced failure) | Same command, then drop an obstacle in the path so the first attempt aborts | After abort: rich detail printed (code, goal_states), `--backoff` sleep, then ExecuteMissionFromGoal from the last in-flight waypoint. Up to `--max-retries` retries. | Robot can't replan → exhausts retries; that's the intended terminal behaviour. |

## What "failure" means here

Any of:
- Script crashes (non-zero exit, traceback).
- Script "succeeds" but the UI shows the wrong state (e.g. mission
  created but contains zero waypoints).
- A service path printed in the log is wrong for your release.

For the last case, update the script's service constant or pass
`--namespace …`. Don't paper over it: if the API path has moved, the
example is stale and the README's index needs updating.
