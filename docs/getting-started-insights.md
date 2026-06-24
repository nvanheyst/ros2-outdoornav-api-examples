# Getting started — things I'd tell my past self

Informal notes from my first weeks with the OutdoorNav API. Not a tutorial
or a replacement for the official docs — those will take you further on
the API surface. This is the stuff that wasn't obvious to me on day one.

## Mental model: map, mission, goal, POI

The four terms get used a lot. Holding them straight up-front saves a lot
of confused calls.

- **Map** — a graph of waypoints connected by edges, with metadata
  (speed limits per edge, radii, default behaviours). Persistent; lives
  in the mission manager DB and survives restarts. Identified by a UUID.
  Create with `mission_manager/create_map` or load from file with
  `examples/maps/load_map_from_file.py`.
- **Mission** — a saved recipe that names a map and an ordered list of
  waypoints to visit. Persistent. Identified by a UUID. Run with the
  `autonomy/mission` action (`ExecuteMission`). Build one with
  `examples/missions/generate_traversal_mission.py`.
- **Goal** — one in-flight execution. Comes from sending an action goal
  (mission action, goto action, dock action). Ephemeral — has a status
  while running, no DB record. Identified by a goal UUID assigned by the
  action server.
- **POI** — a tagged point on the map. Persistent. Useful as an anchor
  for "go to that thing" or "patrol around this tag." Used by
  `row_generator_polygon.py` (POI tags drive the boustrophedon) and
  `pause_and_teleop.py` (GoToPOI).

Quick test: if it's persistent and reusable, it's a map / mission / POI.
If it's an in-flight thing the action server is tracking, it's a goal.

## Namespacing — pick one and stick with it

Every script in this repo takes `--namespace` and defaults to whatever
`$ONAV_NAMESPACE` is set to (or `/a300_00003` if nothing's set). The
canonical pattern:

```bash
export ONAV_NAMESPACE=/a300_00003   # leading slash, no trailing slash
```

Everything composes from there. Topics live at
`<ns>/autonomy/status`, services at `<ns>/mission_manager/get_all_maps`,
actions at `<ns>/autonomy/mission`. If a service path in the docs
doesn't resolve on your robot, run
`./examples/ops/service_inventory.py --grep <module>` — service paths
drift between releases and the live graph is the source of truth.

## DDS discovery: ROS_DOMAIN_ID matters

If you can't see the robot's topics from your laptop, 9 times out of 10
it's a domain ID mismatch:

```bash
echo $ROS_DOMAIN_ID                 # your laptop
ssh robot 'ros2 daemon stop; ros2 topic list | head'  # robot side
```

OutdoorNav defaults to a non-zero domain (commonly 25 in this lab).
Match it on both ends. If you're on different networks, you also need
the RMW config — see the `docker/README.md` in this repo for FastDDS
vs Cyclone vs Zenoh.

## The docker container has three roles

`docker/` in this repo is not just for CI. The image
(`ghcr.io/nvanheyst/onav-api-examples:latest`) is the easiest path to
a working API environment:

1. **Run the smoke and live-dryrun in CI** — what it does on GitHub.
2. **Iterate on examples without polluting your host install** —
   `docker compose run --rm dev` mounts the repo at `/repo`. Source is
   already overlaid; rclpy and clearpath messages just import. No need
   to install ROS 2 Jazzy on your laptop.
3. **Talk to the API from a different OS or directly on the robot** —
   the image runs on Linux, macOS (under Docker Desktop), or Windows
   (WSL2). Point it at the robot's domain ID and matching RMW config,
   and your scripts run as if you were on the robot. Useful when the
   onboard compute is busy and you want a thin client elsewhere.

## Persistent vs ephemeral, quickly

| Thing | Persistent? | How to inspect |
|---|---|---|
| Map | yes | `mission_manager/get_all_maps`, web UI map list |
| Mission | yes | `mission_manager/get_all_network_missions` |
| POI | yes | `mission_manager/get_all_points_of_interest` |
| Mission goal | no | `autonomy/mission/_action/status` (see latest only) |
| GoTo goal | no | watch the action status, no DB record |
| Logs | yes (until you delete them) | `examples/ops/delete_logs.py --dry-run` |

If you accidentally created a hundred test maps while iterating, see
`examples/ops/delete_all.py` (requires `--confirm`).

## The doctor is your first move when something feels off

When the robot is doing nothing and you don't know why:

```bash
./examples/ops/doctor.py --include-lifecycle
```

One pass reports namespace, env, autonomy state, battery, GPS fix age,
last mission status, key topic publishers, and Nav2 lifecycle node
states. Cheaper than the `where_am_i → service_inventory → ros2 topic
echo → ros2 lifecycle list` chase.

## Action result is delivered only to the client that sent the goal

Subtle but important. If you launch a mission from your script and want
the rich result (which waypoint failed, why), call
`goal_handle.get_result_async()` on the handle your `send_goal_async`
returned. If the operator launched the mission from the web UI, your
script can passively watch the action status topic (see
`examples/ops/notify_on_mission_failure.py`) but won't get the result
detail — that's a ROS 2 action semantics thing, not an OutdoorNav one.

## Catch-all: when the docs and reality disagree

Read the message definitions. They're in
`clearpath_outdoornav_msgs` (and `clearpath_navigation_msgs`,
`clearpath_mission_manager_msgs`, etc. depending on what you're
calling). Once you've got the .msg/.srv/.action file open, the script
that uses it is a small wrapper.
