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

## Workflows

**Record and replay a path:**
1. `./maps/record_path.py east_perimeter` — drive while it records; Ctrl-C simplifies and pushes the map.
2. `./missions/generate_traversal_mission.py --name east_patrol` — convert map nodes into a stored mission.
3. `./missions/run_mission.py` — run it on demand (with optional log and video).

## Env-var defaults

Every script reads namespace + UUIDs from env vars (overridable per call with `--namespace`,
`--map-uuid`, etc.):

```bash
export ONAV_NAMESPACE=/a300_00003
export ONAV_MAP_ID=<map-uuid-from-the-ui>
export ONAV_MISSION_ID=<mission-uuid>
export ONAV_POI_ID=<poi-uuid>
```

The polygon row-generator needs `shapely`, `networkx`, `pyproj` (included in the dev container; see `docker/requirements.txt` if running outside it).

## Examples

- **[maps/](maps/README.md)** — create, load, and edit maps.
- **[missions/](missions/README.md)** — build, schedule, loop, and traverse missions.
- **[control/](control/README.md)** — teleop, pause/resume, stop, dock.
- **[ops/](ops/README.md)** — readiness, diagnostics, cleanup, notifications.
- **[patterns/](patterns/README.md)** — reusable recipes: graceful shutdown, perception gate, runtime parameter tuning.

## Requests and gaps

Open an issue if you have an example request. One known gap:

- Per-mission video / data-sampling config (beyond the start/stop bracket in `missions/run_mission.py`).
