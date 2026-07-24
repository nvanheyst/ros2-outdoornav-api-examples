# OutdoorNav API examples

Single-file Python examples for the Clearpath OutdoorNav API. Each script uses raw `rclpy` and
the `clearpath_*` message packages, with no SDK wrappers, so the service, topic, and action
calls are visible at every call site and `ros2 service list`, `topic echo`, and `grep` are
enough to read them. They work against any robot running OutdoorNav 2.x (Jackal, Husky, or
Warthog AMP). For the authoritative reference, see the
[OutdoorNav user manual](https://docs.clearpathrobotics.com/docs_outdoornav_user_manual/) and
the [clearpath_outdoornav](https://github.com/cpr-application/clearpath_outdoornav) repo.

First get the dev container running and connected to a robot or sim (see the
[top-level README](../README.md)). Everything below runs from `/repo`.

## If you're starting out

Run these in order when connecting to a robot for the first time:

1. [ops/where_am_i.py](ops/where_am_i.py) — can I see the robot at all? Subscribes to one topic and prints the GPS fix. Times out immediately if the connection or namespace is wrong.
2. [ops/preflight.py](ops/preflight.py) — is autonomy ready to run? READY or NOT READY + blockers. Run this before every mission send.
3. [ops/doctor.py](ops/doctor.py) — when something feels off: full snapshot of env, autonomy state, topic liveness, last mission result, and (with `--include-lifecycle`) every Nav2 node's state. Not a pre-mission check — a troubleshooting tool.

## Env-var defaults

Every script reads namespace + UUIDs from env vars (overridable per call with `--namespace`,
`--map-uuid`, etc.):

```bash
export ONAV_NAMESPACE=/a300_00003
export ONAV_MAP_ID=<map-uuid-from-the-ui>
export ONAV_MISSION_ID=<mission-uuid>
export ONAV_POI_ID=<poi-uuid>
```

The polygon row-generator needs `shapely`, `networkx`, `pyproj` (see `docker/requirements.txt`).

## Sanity check

```bash
./docker/ci/smoke_all.sh         # --help on every example, no live ROS needed
./docker/ci/scenario_smoke.sh    # representative dry-run flows
./docker/ci/live_dryrun.sh       # --dry-run sweep against a running OutdoorNav stack
```

## Layout

```
common/     shared argparse / config / rclpy boilerplate
maps/       create / load / edit maps
missions/   build / schedule / loop / traverse missions
control/    teleop, pause, resume, stop, dock
ops/        diagnostics, readiness, cleanup, notifications
patterns/   small reusable recipes (graceful shutdown, runtime params)
```

## Examples

Grouped by folder. Each has its own README with the full table:

- **[maps/](maps/README.md)** - create, load, and edit maps (incl. `maps/row_generator_from_here.py`: a zero-arg coverage field at the robot's current position + heading).
- **[missions/](missions/README.md)** - build, schedule, loop, and traverse missions.
- **[control/](control/README.md)** - teleop, pause/resume, stop, dock.
- **[ops/](ops/README.md)** - readiness, diagnostics, cleanup, notifications.

Two good first runs against a real robot:

- **[ops/preflight.py](ops/preflight.py)** - go/no-go: is autonomy actually ready (missions, docking, collision detection, e-stop, battery)? Read-only.
- **[maps/row_generator_from_here.py](maps/row_generator_from_here.py)** - drop a coverage field where the robot is standing, no coordinates to type.

### patterns

Small reusable recipes you'll copy into your own scripts.

| File | API surface | What it does |
|---|---|---|
| `patterns/graceful_shutdown.py` | any action `goal_handle.cancel_goal_async` | Cancel an in-flight action cleanly on Ctrl-C. Provides `spin_until_done_or_cancel()` and a `CancelOnShutdown` context manager. Demo `main()` runs ExecuteMission and cancels cleanly on interrupt. |
| `patterns/parameter_runtime.py` | `<node>/{list,get,set,describe}_parameters` | `ros2 param`-equivalent in script form. List / get / set / describe parameters on a live remote node so you can tune Nav2 params without a restart. |

## Conventions

Every script:

- Has `#!/usr/bin/env python3` and is executable.
- Uses `argparse` via `examples.common.argparse_base.make_parser` for consistent
  `--namespace`, `--dry-run`, `-v`.
- Accepts UUIDs from env vars (`$ONAV_MAP_ID`, etc.) as a fallback for CLI flags.
- Carries a `Touches:` block in its docstring listing every service / topic / action it hits.

No `clearpath_outdoornav_api_lib` imports anywhere, which keeps the call sites grep-able.
`docker/ci/smoke_all.sh` enforces this.

## Requests and gaps

Open an issue if you have an example request. Some known gaps:

- Mission-side cancel via `mission_manager` (vs the action-handle path in `control/cancel_mission.py`).
- Per-mission video / data-sampling config (beyond the start/stop bracket in `missions/run_mission.py`).
- MapDock action (`autonomy/dock_map`) - dock via map coordinates rather than the local target tracker.
- The larger examples sketched in [../mini-projects/IDEAS.md](../mini-projects/IDEAS.md) - none built yet.
