# maps/ - create, load, and edit OutdoorNav maps

Standalone `rclpy` examples for building maps (coverage rows, recorded paths),
loading them from disk, and editing edges. Dev container + connection setup are in
the [top-level README](../../README.md).

| File | API surface | What it does |
|---|---|---|
| `load_map_from_file.py` | `mission_manager/create_map` | JSON → CreateMap. Tolerates missing point ids. |
| `row_generator_square.py` | `mission_manager/create_map` | Boustrophedon over a centre + w/h + bearing rectangle. **One-way** edges (chain). All geometry from CLI args - repeating requires retyping. |
| `row_generator_from_here.py` | `localization/fix`, tf `map->base_link`, `mission_manager/create_map` | Same boustrophedon as `row_generator_square`, but anchored at the robot's **live fix + heading** - zero required args. Rows run along the current heading. Good for a one-command demo. |
| `row_generator_polygon.py` | `mission_manager/{get_all_points_of_interest, get_all_maps, delete_map, create_map}` | Boustrophedon inside a polygon defined by tagged POIs, with the polygon perimeter as a border. **Two-way** edges (graph). Move the vertices in the UI and re-run. `--replace` overwrites a map of the same name. |
| `record_path.py` | `localization/fix`, `mission_manager/create_map` | Map generator by **driving**: subscribe to fix while the operator drives, simplify with Douglas-Peucker on Ctrl-C, push as a map. |
| `bulk_edit_edges.py` | `mission_manager/{get_map, clone_map, update_map_edges}`, `localization/fix` | Bulk edit edge speed limit (slow zone) and/or path radius inside a centre+radius zone. `--around-me` uses the robot's current fix as the centre. Optional `--clone`. |

The polygon generator needs `shapely`, `networkx`, `pyproj` (see `docker/requirements.txt`).
