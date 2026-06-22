# Clearpath OutdoorNav 2.x — ROS 2 API examples

Personal supplementary examples. Each file is a single runnable script —
no shared package, no SDK wrapper — that exercises one OutdoorNav API
pattern more fully than the stock docs do.

All examples are **pure-Python on CPU**: no VLM, no vision pipeline, no GPU.
They should run on any AMP that already has the OutdoorNav stack up.

## Before running

Edit the constants block at the top of each file:

- `ROBOT_NAMESPACE` — defaults to `/a300_00003`; change to match your robot.
- `MAP_ID` / `MISSION_ID` — UUIDs of the map and (where applicable) mission
  you want to drive. Pull them from `mission_manager/get_all_maps` or the
  OutdoorNav UI.

Source your ROS 2 environment, then:

```bash
python3 <example>.py
```

Most scripts accept a few CLI args for tuning — see the docstring at the top
of each file.

## Examples

### Autonomy control

| File | What it shows |
|---|---|
| `pause.py` | GoToPOI action started non-blocking, paused mid-route, teleop'd while paused, then resumed. Exercises the autonomy pause/resume contract. |
| `drive_robot_forward.py` | Open-loop teleop: publish `TwistStamped` on `ui_teleop/cmd_vel` for a computed duration, then stop hard. Useful for one-off positioning without spinning up a mission. |

### Scheduling and looping

| File | What it shows |
|---|---|
| `schedule_mission.py` | Wait until a wall-clock target (ISO 8601 or `+30s` shortcut), then fire an `ExecuteMission` action. Heartbeat countdown + clean Ctrl-C cancel. Docstring covers wrapping with systemd timers + chrony for production scheduling. |
| `loop_mission_battery_aware.py` | Repeat a mission until battery drops below a threshold or a loop cap is hit. Subscribes to `sensor_msgs/BatteryState` and keeps it draining via a manual spin loop while the mission action is in flight. |

### Map generation

| File | What it shows |
|---|---|
| `row_generator_square.py` | Rectangular boustrophedon. Center + width + height + bearing + spacing → flat-earth waypoint list → `mission_manager/create_map`. Optional `--run` also creates and starts the mission. |
| `row_generator_polygon.py` | Polygon boustrophedon. Reads vertices from POIs tagged `cov-2` (custom_fields_json on the first POI carries spacing + width), builds a UTM polygon, generates parallel lines with shapely, integrates the boundary into a networkx graph, deletes any old map with the same name, and pushes a new map. Pair with `generate_traversal_mission.py` to drive it. Needs `shapely`, `networkx`, `pyproj`. |
| `load_map_from_file.py` | Read a JSON map definition from disk and push it via `mission_manager/create_map`. Tolerates missing point UUIDs (generates them and stitches the connections by name). |

### Mission planning over an existing map

| File | What it shows |
|---|---|
| `generate_traversal_mission.py` | Given an existing map UUID, fetches points + connections and creates a traversal mission — sequential for chain maps (square row output), greedy nearest-neighbour for mesh maps (polygon row output). Optional `--run`. |
| `traverse_entire_map_shortest.py` | Visit every POI in greedy nearest-neighbour order from the current GPS fix. Prints route length vs naive ordering so the savings are obvious. |
| `random_visit_mission.py` | Loop forever (or N times) firing `ExecuteGoTo` at random GPS points inside the map's bounding box. Free poses, not POIs — the destinations have nothing to do with whatever POIs the map happens to have. Seedable. |

### Maintenance

| File | What it shows |
|---|---|
| `delete_logs.py` | Enumerate event logs via `logger/get_all_logs` and delete each via `logger/delete_log` with `purge_record=true`. Supports `--keep-recent <hours>` and `--dry-run`. |
