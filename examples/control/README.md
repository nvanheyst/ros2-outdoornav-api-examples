# control/ - teleop, pause/resume, stop, dock

Direct-control examples: drive the robot, pause and resume autonomy, hard-stop, and
run a docking workflow. Dev container + connection setup are in the
[top-level README](../../README.md).

> These send real motion / autonomy commands - mind a live robot.

| File | API surface | What it does |
|---|---|---|
| `drive_robot_forward.py` | `ui_teleop/cmd_vel` | Publish TwistStamped for N metres at M m/s. Open-loop. |
| `pause_resume.py` | `autonomy/{pause,resume}` (SetBool, default) **or** `control_selection/{pause,resume}` **or** `autonomy/{pause,resume}` (Trigger) | Minimal pause → hold → resume. Three release variants via `--variant`. Run `../ops/service_inventory.py --grep 'pause\|resume'` first to confirm path + type. |
| `pause_and_teleop.py` | `autonomy/{pause,resume}` (SetBool), `autonomy/goto_poi`, `ui_teleop/cmd_vel`, `autonomy/stop` | Drive to a POI, pause mid-route, teleop a turn + drive, resume, wait for completion. |
| `stop_autonomy.py` | `autonomy/stop` | Hard-stop the autonomy stack (Trigger). |
| `cancel_mission.py` | `autonomy/mission` | Start a mission from this process, sleep `--after` seconds, then cancel via the goal handle - the action-cancel pattern. To stop a mission started elsewhere, use `stop_autonomy.py`. |
| `dock_workflow.py` | `docking/dock_localizer/add_dock_current_pose`, `docking/dock_manager/delete_dock`, `ui_teleop/cmd_vel`, `autonomy/{dock_local, undock}` | Add a dock at the robot's current pose, back up, dock, hold, undock, clean up. End-to-end docking demo. |
