# Clearpath OutdoorNav — unofficial API examples

A personal collection of ROS-2 examples and notes I've put together
while ramping up on Clearpath's OutdoorNav API. The official API reference,
message definitions, and supported examples live in the upstream
[clearpath_outdoornav](https://github.com/clearpathrobotics/clearpath_outdoornav)
repo — start there. This repo is the stuff I wished I'd had in my first
month: a working dev container and a few patterns the docs don't cover.

Pure Python, single-file examples. Each script uses raw `rclpy` plus the
`clearpath_*` message packages — no SDK wrappers — so the service / topic
/ action path is visible at every call site and `ros2 service list`,
`topic echo`, and `grep` are enough to audit them. Works against any
robot running OutdoorNav 2.x (Jackal / Husky / Warthog AMP).

## If you're starting out

Read [docs/getting-started-insights.md](docs/getting-started-insights.md)
first, then try a couple of examples to get connected and reporting:

- [examples/ops/where_am_i.py](examples/ops/where_am_i.py) — confirm you can talk to the robot and read its GPS fix.
- [examples/ops/doctor.py](examples/ops/doctor.py) — one-call snapshot of autonomy state, battery, and topic liveness.

## Quick start

In the dev container (recommended — see [docker/README.md](docker/README.md)
for what the container is and isn't):

```bash
cd docker
docker compose pull
docker compose run --rm dev        # interactive shell at /repo
# inside:
python3 examples/ops/where_am_i.py --timeout 5
```

Without Docker, source `/opt/ros/jazzy/setup.bash` plus an overlay that
ships `clearpath_*` messages, then run scripts directly from the repo
root:

```bash
./examples/ops/where_am_i.py --timeout 5
./examples/control/drive_robot_forward.py --distance 1 --velocity 0.2
```

The polygon row-generator needs `shapely`, `networkx`, `pyproj`; see
`requirements.txt`.

### Env-var defaults

Every script reads namespace + UUIDs from env vars (overridable per call
with `--namespace`, `--map-uuid`, etc.):

```bash
export ONAV_NAMESPACE=/a300_00003
export ONAV_MAP_ID=<map-uuid-from-the-ui>
export ONAV_MISSION_ID=<mission-uuid>
export ONAV_POI_ID=<poi-uuid>
```

### Sanity check

```bash
./ci/smoke_all.sh         # --help on every example, no live ROS needed
./ci/scenario_smoke.sh    # representative dry-run flows
./ci/live_dryrun.sh       # --dry-run sweep against a running OutdoorNav stack
```

## Layout

```
common/         shared argparse / config / rclpy boilerplate
docs/           getting-started insights
examples/
  maps/         create / load / edit maps
  missions/     build / schedule / loop / traverse missions
  control/      teleop, pause, resume, stop
  ops/          diagnostics, log cleanup, notifications
patterns/       small reusable recipes (graceful shutdown, runtime params)
mini-projects/  ideas for larger end-to-end examples (not built yet)
ci/             offline smoke + live dry-run harnesses
docker/         dev environment + drop-in API client for any OS
```

For each table below, upstream's
[clearpath_outdoornav](https://github.com/clearpathrobotics/clearpath_outdoornav)
repo and the official docs are the authoritative reference for the API
surface listed. The descriptions here are how I think about them while
writing scripts.

## Examples — index

### maps

| File | API surface | What it does |
|---|---|---|
| `maps/load_map_from_file.py` | `mission_manager/create_map` | JSON → CreateMap. Tolerates missing point ids. |
| `maps/row_generator_square.py` | `mission_manager/create_map` | Boustrophedon over a centre + w/h + bearing rectangle. **One-way** edges (chain). All geometry from CLI args — repeating requires retyping. |
| `maps/row_generator_polygon.py` | `mission_manager/{get_all_points_of_interest, get_all_maps, delete_map, create_map}` | Boustrophedon inside a polygon defined by tagged POIs, with the polygon perimeter included as a border. **Two-way** edges (graph). POI-driven means you can move the vertices in the UI and re-run. Still one-shot though, not reactive. `--replace` overwrites an existing map of the same name. |
| `maps/bulk_edit_edges.py` | `mission_manager/{get_map, clone_map, update_map_edges}`, `localization/fix` | Bulk edit edge speed limit (slow zone) and/or path radius inside a centre+radius zone. `--around-me` uses the robot's current fix as the centre. Optional `--clone`. |

### missions

| File | API surface | What it does |
|---|---|---|
| `missions/generate_traversal_mission.py` | `mission_manager/{get_map, create_mission, create_waypoint}`, optional `autonomy/mission` | **Persistent**, edge-following traversal. Builds a NetworkMission + Waypoints in the database so you can re-run it from the UI. Sequential for chain maps, least-turn graph walk for mesh. Robot follows map edges. Optional `--run`. |
| `missions/traverse_entire_map_gotos.py` | `mission_manager/get_map`, `autonomy/goto` | **Ephemeral**, free-GPS traversal. Fires one ExecuteGoTo per node in greedy nearest-neighbour order from the current fix. No mission stored. Robot picks its own path between nodes — ignores edges. |
| `missions/random_visit_mission.py` | `mission_manager/get_map`, `autonomy/goto` | Random GoTo goals inside the map bbox. Soak test. |
| `missions/loop_mission_battery_aware.py` | `autonomy/mission`, `platform/bms/state` | Loop a mission until battery drops below threshold. |
| `missions/schedule_mission.py` | `autonomy/mission` | Wait until a wall-clock target (ISO 8601 or `+30s/+5m`) then fire ExecuteMission. |
| `missions/record_path.py` | `localization/fix`, `mission_manager/create_map` | Subscribe to fix while the operator drives, simplify with Douglas-Peucker on Ctrl-C, optionally push as a map. |
| `missions/mission_with_recording.py` | `log_manager/{start_recording, stop_recording}`, `autonomy/mission` | Bracket an ExecuteMission call with start_recording / stop_recording so the run shows up as a single log in the UI. |

### control

| File | API surface | What it does |
|---|---|---|
| `control/drive_robot_forward.py` | `ui_teleop/cmd_vel` | Publish TwistStamped for N metres at M m/s. Open-loop. |
| `control/pause_resume.py` | `autonomy/{pause,resume}` (SetBool — default) **or** `control_selection/{pause,resume}` (SetBool) **or** `autonomy/{pause,resume}` (Trigger) | Minimal pause→hold→resume. Three release variants supported via `--variant`. Run `service_inventory.py | grep -E 'pause\|resume'` first to confirm path + type. |
| `control/pause_and_teleop.py` | `autonomy/{pause,resume}` (SetBool), `autonomy/goto_poi`, `ui_teleop/cmd_vel`, `autonomy/stop` | Drive to a POI, pause mid-route, teleop a turn + drive, resume, wait for completion. |
| `control/stop_autonomy.py` | `autonomy/stop` | Hard-stop the autonomy stack (Trigger). |
| `control/cancel_mission.py` | `autonomy/mission` | Start a mission from this process, sleep `--after` seconds, then cancel via the goal handle. Demonstrates the action-cancel pattern. To stop a mission started elsewhere, use `stop_autonomy.py`. |
| `control/dock_workflow.py` | `docking/dock_localizer/add_dock_current_pose`, `docking/dock_manager/delete_dock`, `ui_teleop/cmd_vel`, `autonomy/{dock_local, undock}` | Add a dock at the robot's current pose, back up, dock, hold, undock, clean up. End-to-end docking demo. |

### ops

| File | API surface | What it does |
|---|---|---|
| `ops/where_am_i.py` | `localization/fix` | Print the latest GPS fix and exit. |
| `ops/service_inventory.py` | ROS graph | List all services live on the graph; `--grep` filters. First stop when a wait-for-service hangs — paths and types differ across OutdoorNav releases. |
| `ops/delete_logs.py` | `logger/{get_all_logs, delete_log}` | Enumerate event logs and delete each. `--keep-recent`, `--dry-run` supported. `--keep-media` / `--keep-record` opt out of scorched earth. |
| `ops/delete_all.py` | `mission_manager/{delete_all, get_all_maps, get_all_network_missions, get_all_points_of_interest}` | Wipe every map, mission, POI. Useful as cleanup after running these examples leaves test data littering the UI. `--dry-run` lists counts; `--confirm` actually fires. |
| `ops/notify_on_mission_failure.py` | `autonomy/mission/_action/status`, `autonomy/status`, `localization/fix`, optional `sensors/camera_*/color/compressed` | Passive watcher: subscribe to the ExecuteMission action's status topic, ping a notifier when a goal aborts. Catches any abort regardless of who launched the mission. `--via stdout,email,webhook,sms` (comma-separated, configs via env vars). Email attaches the latest camera frame if `--camera-topic` is set; SMS is a one-line heads-up. |
| `ops/doctor.py` | `autonomy/status`, `localization/fix`, `platform/bms/state`, `autonomy/mission/_action/status`, optional `<lifecycle_node>/get_state` | One-call diagnostic snapshot: env, autonomy state, battery, GPS fix age, last mission status, key topic liveness, and (with `--include-lifecycle`) every managed-node state. Replaces the "run a half-dozen tools to figure out what's up" scavenger hunt. |

### missions (cont.)

| File | API surface | What it does |
|---|---|---|
| `missions/recover_from_abort.py` | `autonomy/mission`, `autonomy/mission_from_goal`, `navigation/current_goal_id` | Run ExecuteMission. On abort, retry from the last in-flight waypoint via ExecuteMissionFromGoal, up to `--max-retries` times. Logs the rich abort detail (code, message, per-waypoint goal_states) before each retry. |

### patterns

Small reusable recipes you'll copy into your own scripts.

| File | API surface | What it does |
|---|---|---|
| `patterns/graceful_shutdown.py` | any action `goal_handle.cancel_goal_async` | Cancel an in-flight action cleanly on Ctrl-C. Provides `spin_until_done_or_cancel()` and a `CancelOnShutdown` context manager. Demo `main()` runs ExecuteMission and cancels cleanly on interrupt. |
| `patterns/parameter_runtime.py` | `<node>/{list,get,set,describe}_parameters` | `ros2 param`-equivalent in script form. List / get / set / describe parameters on a live remote node so you can tune Nav2 params (or anything else with a parameter server) without restart. |

### mini-projects

Larger end-to-end examples that span multiple files. See
[mini-projects/IDEAS.md](mini-projects/IDEAS.md) for the backlog
(perception gate, long-running supervisor, GPU perception, observability
sink, in-depth security). Nothing here is implemented yet — opening an
issue with the use case is the best way to bump one up the list.

## Conventions

Every script:

- Has `#!/usr/bin/env python3` and is executable.
- Uses `argparse` via `common.argparse_base.make_parser` for consistent
  `--namespace`, `--dry-run`, `-v`.
- Accepts UUIDs from env vars (`$ONAV_MAP_ID`, etc.) as fallback for
  CLI flags.
- Carries a `Touches:` block in its docstring listing every service /
  topic / action it hits.

No `clearpath_outdoornav_api_lib` imports anywhere — keep the call
sites grep-able. `ci/smoke_all.sh` enforces this.

## Notably missing

Contributions welcome:

- Mission-side cancel via `mission_manager` (vs the action-handle path
  shown in `cancel_mission.py`)
- Per-mission video / data-sampling configuration (beyond the basic
  start/stop bracket in `mission_with_recording.py`)
- MapDock action (`autonomy/dock_map`) — dock via map coordinates rather
  than the local target tracker
- The mini-projects in [mini-projects/IDEAS.md](mini-projects/IDEAS.md)
  — none built yet, open to suggestions
