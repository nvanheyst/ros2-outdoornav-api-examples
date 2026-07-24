# patterns/ - reusable recipes

Cross-cutting patterns that combine with examples in other directories. Each file
is a self-contained, importable template. Dev container + connection setup are in
the [top-level README](../../README.md).

| File | API surface | What it does |
|---|---|---|
| `graceful_shutdown.py` | `autonomy/mission` (ExecuteMission) | Catch Ctrl-C and SIGTERM, cancel the in-flight goal cleanly, wait for the cancel ack before shutting down. Drop-in signal handler template. |
| `parameter_runtime.py` | `<node>/list_parameters`, `<node>/get_parameters`, `<node>/set_parameters` | List, get, and set parameters on any running node at runtime — without restarting it. Good for live-tuning controller speed limits or collision-monitor thresholds. |
| `perception_gate.py` | `std_msgs/Bool` (subscribe), `autonomy/{pause,resume}` (SetBool) | Pause autonomy when a Bool topic goes False, resume when it returns True. Runs alongside any mission. Wire any signal source — ML model, safety sensor, operator toggle — to the Bool topic. |
