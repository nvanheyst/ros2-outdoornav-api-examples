# control/ - teleop, pause/resume, stop, dock

Direct-control examples: drive the robot, pause and resume autonomy, hard-stop, and
dock. Dev container + connection setup are in the
[top-level README](../../README.md).

> These send real motion / autonomy commands - mind a live robot.

| File | API surface | What it does |
|---|---|---|
| `drive_robot_forward.py` | `ui_teleop/cmd_vel` | Publish TwistStamped for N metres at M m/s. Open-loop. |
| `pause_resume.py` | `autonomy/{pause,resume}` (SetBool, default) **or** `control_selection/{pause,resume}` **or** `autonomy/{pause,resume}` (Trigger) | Minimal pause → hold → resume. Three release variants via `--variant`. Run `../ops/service_inventory.py --grep 'pause\|resume'` first to confirm path + type. |
| `stop_autonomy.py` | `autonomy/stop` | Hard-stop the autonomy stack (Trigger). |
| `dock_now.py` | `autonomy/dock_local` (Dock), `docking/get_dock_database` (GetDockDatabase) | Dock the robot at a named dock already in the database. Interactive menu if `--dock-name` is omitted. |
| `dock_map.py` | `autonomy/dock_map` (MapDock), `docking/get_dock_database` (GetDockDatabase) | Dock via map coordinates (pose stored in the map). Interactive menu if `--dock-name` is omitted. |
