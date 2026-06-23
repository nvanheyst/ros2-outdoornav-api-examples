# Clearpath OutdoorNav 2.x — raw ROS 2 API examples

Personal supplementary examples for OutdoorNav. Each script is one
runnable file using raw rclpy + clearpath/clearpath_outdoornav messages.

**Why raw ROS?** So `ros2 service list`, `ros2 topic echo`, and `grep` for
a service path are all the audit tools you need. There are no SDK
wrappers in this folder — no `clearpath_outdoornav_api_lib` imports
anywhere. The `scripts/smoke_all.sh` harness enforces this with a grep.

Pure-Python on CPU. No VLM, no vision. Should work on any AMP that has
OutdoorNav up.

A few files (`row_generator_polygon`, `load_map_from_file`, `delete_logs`)
started from the onav SDK ROS 1 examples and were rewritten for ROS 2.

## Layout

```
common/        shared helpers — argparse base, env-var config, rclpy boilerplate
examples/
  maps/        create / load / edit maps
  missions/    build / schedule / loop / traverse missions
  control/     teleop, pause, resume, stop
  data/        logs, recordings
  diagnostics/ where am I, what services are live
scripts/       smoke (offline) and live-dryrun harnesses
docker/        dev environment (ROS 2 Jazzy + clearpath overlay)
```

## Quick start

### Configure once

Every script reads namespace + UUIDs from env vars (overridable per call
with `--namespace`, `--map-uuid`, etc.):

```bash
export ONAV_NAMESPACE=/a300_00003
export ONAV_MAP_ID=<map-uuid-from-the-ui>
export ONAV_MISSION_ID=<mission-uuid>
export ONAV_POI_ID=<poi-uuid>
```

### Inside the dev container (recommended)

```bash
cd docker
docker compose pull
docker compose run --rm dev
# inside: ROS sourced, /examples mounted
python3 /examples/control/drive_robot_forward.py --help
```

The image lives at `ghcr.io/nvanheyst/onav-api-examples:latest`.
`latest` is only published from `main`. PR builds publish temporary
`pr-<number>` tags. Docker CI also runs `./scripts/smoke_all.sh` inside
the built container before push. It also runs
`./scripts/scenario_smoke.sh` for a basic dry-run scenario pass. The
image defaults to FastDDS to match OutdoorNav. For Cyclone or Zenoh see
`docker/README.md`.

### Without Docker

Source `/opt/ros/jazzy/setup.bash` plus your overlay containing
`clearpath_*` messages, then run scripts directly:

```bash
./examples/control/drive_robot_forward.py --distance 5 --velocity 0.4
```

The polygon row-generator needs three Python deps; see `requirements.txt`.

### Sanity check

```bash
./scripts/smoke_all.sh         # --help on every example, no live ROS needed
./scripts/scenario_smoke.sh    # a few representative dry-run flows
./scripts/live_dryrun.sh       # --dry-run sweep against a running OnAV stack
```

## Branch protection

Before you merge, set branch protection on `main` and require these checks:
- `ros2-jazzy-ci / python-lint`
- `ros2-jazzy-ci / shell-lint`
- `ros2-jazzy-ci / jazzy-offline-smoke`
- `docker / pr-build-test-and-push-temp` (or `docker / pr-build-and-test-fork` for fork PRs)

Also make sure Actions can write packages, or PR image push and cleanup
will fail.

## Examples — index

### maps

| File | API surface | What it does |
|---|---|---|
| `maps/load_map_from_file.py` | `mission_manager/create_map` | JSON → CreateMap. Tolerates missing point ids. |
| `maps/row_generator_square.py` | `mission_manager/create_map` | Boustrophedon over a centre + w/h + bearing rectangle. Map only. |
| `maps/row_generator_polygon.py` | `mission_manager/{get_all_pois, get_all_maps, delete_map, create_map}` | Boustrophedon over a polygon defined by tagged POIs. UTM + shapely + networkx. |
| `maps/bulk_edit_edges.py` | `mission_manager/{get_map, clone_map, update_map_edges}` | Edit speed and/or radius on every edge in a centre+radius zone. Optional `--clone`. |

### missions

| File | API surface | What it does |
|---|---|---|
| `missions/generate_traversal_mission.py` | `mission_manager/{get_map, create_mission, create_waypoint}`, optional `autonomy/mission` | Given a map UUID, build a mission via CreateNetworkMission + CreateWaypoint. Sequential for chain maps, least-turn graph walk for mesh. Optional `--run`. |
| `missions/traverse_entire_map_shortest.py` | `mission_manager/get_map`, `autonomy/goto`, `localization/fix` | Visit every map node in greedy NN order from the current fix. ExecuteGoTo per node. |
| `missions/random_visit_mission.py` | `mission_manager/get_map`, `autonomy/goto` | Random ExecuteGoTo inside the map bbox, loop forever or N times. |
| `missions/loop_mission_battery_aware.py` | `autonomy/mission`, `platform/bms/state` | Loop a mission until battery drops below threshold. Manually spins so the BMS sub stays alive while the mission is in flight. |
| `missions/schedule_mission.py` | `autonomy/mission` | Wait until a wall-clock target (ISO 8601 or `+30s/+5m`), fire ExecuteMission. |
| `missions/record_path.py` | `localization/fix`, `mission_manager/create_map` | Subscribe to fix while the operator drives, simplify with Douglas-Peucker on Ctrl-C, optionally push as a map. |

### control

| File | API surface | What it does |
|---|---|---|
| `control/drive_robot_forward.py` | `ui_teleop/cmd_vel` | Publish TwistStamped for N metres at M m/s. Open-loop, no odom feedback. |
| `control/pause_resume.py` | `control_selection/{pause,resume}` (SetBool) **or** `autonomy/{pause,resume}` (Trigger) | Minimal pause→hold→resume demo. Picks the service variant at the CLI; **run `service_inventory.py --grep pause` first to confirm which variant your stack exposes**. |
| `control/pause_and_teleop.py` | `autonomy/goto_poi`, `control_selection/{pause,resume}`, `ui_teleop/cmd_vel`, `autonomy/stop` | Drive to a POI, pause mid-route, teleop a turn + drive, resume, wait for completion. |
| `control/stop_autonomy.py` | `autonomy/stop` | Hard-stop the autonomy stack (Trigger). |
| `control/cancel_mission.py` | `autonomy/mission` | Documents how to cancel an in-flight mission action goal from the process that owns the goal handle. |

### data

| File | API surface | What it does |
|---|---|---|
| `data/delete_logs.py` | `logger/{get_all_logs, delete_log}` | Enumerate event logs and delete each. `--keep-recent`, `--dry-run` supported. |

### diagnostics

| File | API surface | What it does |
|---|---|---|
| `diagnostics/where_am_i.py` | `localization/fix` | Print the latest GPS fix and exit. |
| `diagnostics/service_inventory.py` | ROS graph | List all services live on the graph; `--grep` filters. Use this before any pause/dock/etc work to confirm the service path exists. |

## Conventions

Every script:
- Has `#!/usr/bin/env python3` and is executable.
- Uses `argparse` (no manual `sys.argv` parsing) via `common.argparse_base.make_parser`.
- Accepts `--namespace` (defaults to `$ONAV_NAMESPACE` or `/a300_00003`).
- Accepts `--dry-run` if it mutates state, and prints what it *would* do.
- Accepts UUID args from env vars (`$ONAV_MAP_ID`, etc.) as fallback for CLI flags.

## Two pause interfaces — short version

OutdoorNav exposes pause through two different layers depending on
release. Both are demonstrated:

- `<ns>/control_selection/pause` / `…/resume` — `std_srvs/SetBool`,
  newer integration layer. Used by `pause_and_teleop.py` and the default
  variant of `pause_resume.py`.
- `<ns>/autonomy/pause` / `…/resume` — `std_srvs/Trigger`, older inner
  autonomy layer. Available via `pause_resume.py --variant trigger`.

`stop` is consistent across both: `<ns>/autonomy/stop` (Trigger).

Run `diagnostics/service_inventory.py --grep pause` to see what your
stack actually exposes; pick the matching variant.

## Notably missing from this folder

These API surfaces aren't represented yet. PRs welcome.

- Docking (`mission_manager/add_dock`, `autonomy/dock`, `autonomy/undock`)
- Return-to-dock missions
- Slow zones
- Mission data sampling / video recording configuration
- Mission-side cancellation via `mission_manager` (vs the goal-handle path
  shown in `cancel_mission.py`)
