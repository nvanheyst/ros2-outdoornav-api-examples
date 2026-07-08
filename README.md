# Clearpath OutdoorNav — unofficial API examples

A personal collection of ROS-2 examples and notes I've put together
while ramping up on Clearpath's OutdoorNav API. The official API reference,
message definitions, and supported examples live in the upstream
[clearpath_outdoornav](https://github.com/clearpathrobotics/clearpath_outdoornav)
repo — start there. This repo is the stuff I wished I'd had in my first
month: a general-purpose ROS 2 dev container — useful on its own — and a
few patterns the docs don't cover.

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
- [examples/ops/preflight.py](examples/ops/preflight.py) — go/no-go readiness gate before you send a mission.

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

### Sim vs real robot (transport profiles)

The examples are namespace-agnostic (no baked-in serial) and work against both the
onav-lab sim and a real robot — the difference is only the DDS transport. Pick a
committed profile instead of memorising env vars:

```bash
docker compose --env-file sim.env      run --rm dev   # sim: CycloneDDS, domain 25
docker compose --env-file real-amp.env run --rm dev   # real: FastDDS + discovery server, domain 0
```

|                | sim (`sim.env`)     | real amp (`real-amp.env`)                 |
|----------------|---------------------|-------------------------------------------|
| RMW            | `rmw_cyclonedds_cpp`| `rmw_fastrtps_cpp`                         |
| Domain         | 25                  | 0                                         |
| Discovery      | multicast           | Fast DDS Discovery Server `127.0.0.1:11811` (super-client) |
| Namespace      | `/a300_00003`       | auto-detected (pin with `ONAV_NAMESPACE`) |

Confirm the link first — this prints the transport, auto-detects the namespace, and
shows live signals:

```bash
python3 examples/ops/connect_real_robot.py
```

Real robots run a discovery server on the robot's loopback, so run against a real robot
**on the robot**. Off-robot needs a bridge (e.g. a Zenoh router); a Cloudflare TCP
tunnel is not allowed (ToS).

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
docker/         general-purpose ROS 2 dev container — its own project; the examples just use it
```

Upstream's [clearpath_outdoornav](https://github.com/clearpathrobotics/clearpath_outdoornav)
repo and the official docs are the authoritative reference for the API surface each
example uses; the folder READMEs below are how I think about them while writing scripts.

## Examples

Grouped by folder — each has its own README with the full table:

- **[examples/maps/](examples/maps/README.md)** — create, load, and edit maps (incl. `row_generator_from_here.py`: a zero-arg coverage field at the robot's current position + heading).
- **[examples/missions/](examples/missions/README.md)** — build, schedule, loop, and traverse missions.
- **[examples/control/](examples/control/README.md)** — teleop, pause/resume, stop, dock.
- **[examples/ops/](examples/ops/README.md)** — readiness, diagnostics, cleanup, notifications.

Two good first runs against a real robot:

- **[examples/ops/preflight.py](examples/ops/preflight.py)** — go/no-go: is autonomy actually ready (missions, docking, collision detection, e-stop, battery)? Read-only.
- **[examples/maps/row_generator_from_here.py](examples/maps/row_generator_from_here.py)** — drop a coverage field where the robot is standing, no coordinates to type.

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
