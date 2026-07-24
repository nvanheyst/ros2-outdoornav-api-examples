# Mini-project ideas

Application demos and larger multi-file examples. Not required to run on every AMP — custom hardware, optional software, and non-standard config are fine here. Open an issue if you want to see one built or want to expand an existing one further.

## Security patrol demo

A long-running guard-route supervisor: `systemd` unit wraps `loop_mission_dock_charge.py`, survives crashes, logs to journald, and exposes a `/health` endpoint. On an external alert signal it breaks out of the patrol loop, fires `go_to_poi.py` at the alert location, and calls `notify_on_mission_failure.py` to page the operator.

## Fleet monitoring

Multi-robot observability sink: each AMP runs `notify_on_mission_failure.py`, which POSTs to a shared FastAPI receiver. The receiver aggregates status, mission results, and GPS positions into a Grafana-compatible endpoint — one dashboard for a 2–3 robot fleet.

## Fleet coordination

Thin task dispatcher that assigns guard-route or inspection missions to whichever AMP is idle (or lowest battery). Could plug into Open-RMF for traffic management, or run as a standalone scheduler. Shows how to manage multiple `autonomy/mission` clients from one process.

## Inspection and data collection

Row or polygon traversal that pauses at each waypoint, fires a camera capture (or sensor trigger), and writes a timestamped record — GPS position, camera frame, custom sensor reading — to a JSON log and an optional cloud bucket. Shows the robot as a data-collection platform, not just transport.

## Vision integration

Connect a vision model running on the robot's onboard compute (Jetson or GPU AMP) to the `patterns/perception_gate.py` pattern. Covers building the `std_msgs/Bool` publisher that drives pause/resume from a real classifier — object detection, anomaly, safety zone.

## OutdoorNav AI console (prototype)

Straight Ollama calls with a thin wrapper: user types a natural-language command, the script sends it to a local model, maps the response to an OutdoorNav API call (get state, send mission, go to POI, dock), and handles errors and misses without crashing. No TUI, no MCP protocol overhead — same pattern as the kuka console in the iiwa7-lab.
