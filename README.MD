# Clearpath OutdoorNav 2.x — ROS 2 API examples

Personal supplementary examples. Each file is one runnable script, no
shared package, no SDK wrapper.

Pure-Python on CPU. No VLM, no vision. Should run on any AMP that has
OutdoorNav up.

A few files (`row_generator_polygon.py`, `load_map_from_file.py`,
`delete_logs.py`) started from the onav SDK ROS 1 examples and were
updated for ROS 2. The rest are written from scratch.

## Layout

    examples/   the runnable example scripts
    docker/     dev environment — image with ROS 2 Jazzy + clearpath overlay

## Run

Easiest path is the dev container. The image lives at
`ghcr.io/nvanheyst/onav-api-examples:latest` — **only `:latest` exists**, CI
overwrites it on every push to `main`.

    cd docker
    docker compose pull        # always :latest
    docker compose run --rm dev
    # inside container, ROS sourced; examples/ mounted at /examples
    python3 /examples/random_visit_mission.py 5

OutdoorNav uses FastDDS by default; the container does the same. For
Cyclone or Zenoh see `docker/README.md`.

Without Docker: source `/opt/ros/jazzy/setup.bash` plus your own overlay
build of `clearpath_msgs` + `clearpath_outdoornav`, then `python3
examples/<name>.py`.

Before any example runs, edit the constants at the top of the file:

- `ROBOT_NAMESPACE` (defaults to `/a300_00003`)
- `MAP_ID` / `MISSION_ID` (pull from `mission_manager/get_all_maps` or the
  OutdoorNav UI)

## Examples

| File | What it does |
|---|---|
| `pause.py` | GoToPOI started non-blocking, paused mid-route, teleop'd while paused, then resumed. |
| `drive_robot_forward.py` | Publish TwistStamped on `ui_teleop/cmd_vel` for N meters at M m/s. Open-loop, no odom feedback. |
| `schedule_mission.py` | Wait until a wall-clock target (ISO 8601 or `+30s`), fire ExecuteMission. Wrap in systemd for real use. |
| `loop_mission_battery_aware.py` | Loop a mission until battery drops below threshold. Manually spins so the BMS sub stays alive while the mission action is in flight. |
| `row_generator_square.py` | Boustrophedon over a center + w/h + bearing rectangle. Creates the map only. |
| `row_generator_polygon.py` | Boustrophedon over a polygon defined by POIs tagged `cov-2`. UTM + shapely + networkx. |
| `load_map_from_file.py` | JSON → CreateMap. Tolerates missing point ids. |
| `generate_traversal_mission.py` | Given a map UUID, build a mission via CreateNetworkMission + CreateWaypoint loop. Sequential for chain maps, greedy NN for mesh. Optional `--run`. |
| `traverse_entire_map_shortest.py` | Visit every map node in greedy NN order from the current fix. ExecuteGoTo per node — nodes aren't POIs. |
| `random_visit_mission.py` | Random ExecuteGoTo inside the map bbox, loop forever or N times. |
| `bulk_edit_edges.py` | Update speed and/or path radius on every edge in a centre+radius zone in one shot. Optional `--clone` so the source map is preserved. |
| `record_path.py` | Subscribe to `/localization/fix` while the operator drives, simplify via Douglas-Peucker on Ctrl-C, push as a map. Operator-side; no autonomy action server needed. |
| `delete_logs.py` | Enumerate via `logger/get_all_logs`, delete each. `--keep-recent` / `--dry-run` supported. |

