# Mini-project ideas

Things I'd build next if there's interest. Nothing here is implemented yet -
it's a backlog, not a promise. If you want to see one of these first, open an
issue or DM me.

Mini-projects (vs. the single-file examples in `examples/`) are multi-file,
have their own README and dependencies, and demonstrate an end-to-end pattern
rather than one API call. Each lives in `mini-projects/<name>/`.

## Perception gate

A wrapper that pauses autonomy at a chosen waypoint, waits for an external
classifier to publish "go" / "no-go", and resumes (or aborts) accordingly.
The classifier itself is out of scope - the project ships a dummy publisher
so the pattern is runnable without any real perception stack.

Shape:
- `perception_gate.py` - subscribes to autonomy status + classifier topic, calls pause / resume / stop.
- `dummy_classifier.py` - publishes randomly or on a timer, for demos.
- `README.md` - wiring diagram and how to plug in a real classifier.

Why it matters: every customer who runs autonomy alongside their own ML asks
"how do I make the robot wait for my model?" There's no one-size answer, but
there is a clean pattern.

## Long-running supervisor

A `systemd` unit (or a docker-compose `restart: always` recipe) that wraps
`loop_mission_battery_aware.py` so it survives crashes, logs to journald,
and exposes a minimal health endpoint. The kind of thing you actually want
once you stop running missions from a tmux session.

Shape:
- `mission-supervisor.service` - systemd unit template.
- `wrapper.py` - adds a small HTTP `/health` endpoint, structured logging, graceful shutdown signal handling.
- `README.md` - install, enable, journalctl recipes.

Why it matters: the difference between "I ran a mission" and "missions are
running" is operational. Customers hit this in week 3 or 4 and roll their own.

## GPU perception node

A torch / ONNX node that subscribes to the camera stream, publishes
classifications, and optionally drives the perception gate above. Separate
project because GPU setup (CUDA base image, model packaging, Jetson vs x86)
has enough of its own moving parts that bundling it would muddy the
perception-gate example.

Shape:
- `Dockerfile` - CUDA base, model artifacts.
- `perception_node.py` - image subscriber, inference loop, classification publisher.
- `README.md` - model swap guide, performance notes.

Why it matters: most customers already have a model. They want to know how
to wire it into autonomy, not how to train one. This shows the wiring.

## Observability sink

A small webhook receiver (FastAPI) plus a dashboard-friendly JSON endpoint
that consumes events from `notify_on_mission_failure.py`. Closes the loop
from "the robot pinged us" to "the dashboard shows the abort and where it
happened."

Shape:
- `receiver.py` - webhook ingestion + in-memory store.
- `metrics.py` - Grafana-style JSON-API endpoint.
- `docker-compose.yml` - receiver + a minimal Grafana with the JSON datasource preconfigured.
- `README.md` - point your notifier at `http://localhost:8000/hook`, see events on the dashboard.

Why it matters: notifications without context are noise. A dashboard turns
them into something you can debug a week later.

## In-depth security example

What hardening a fleet actually requires beyond default ROS 2. Probably
covers SROS2 / DDS-level access controls, namespace isolation, and audit
logging of who sent which action goal. Not "ROS 2 security 101" - focused
on the specific surface a customer node touches when talking to OutdoorNav.

Shape:
- `enclave/` - example SROS2 enclave with permissions XML.
- `audit_sidecar.py` - wraps an action client, logs goal sender + payload to journald.
- `README.md` - threat model, what this covers, what it doesn't.

Why it matters: customers deploying outside a lab need to answer "what
stops a bad actor on the network from sending a stop_autonomy goal?" There
isn't a clean answer in upstream docs.

## Adding to this list

Open an issue with the use case. The goal here is to build the ones people
actually need, not all of them.
