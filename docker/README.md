# Dev environment

Docker image with ROS 2 Jazzy + the Clearpath message + API library overlay
pre-built. Three roles:

1. **CI / smoke testing** — `docker compose run --rm dev ./ci/smoke_all.sh`.
   What it does on every push to GitHub.
2. **Dev environment for the examples** — `docker compose run --rm dev`
   gives you an interactive shell with everything sourced, no host install
   needed. The repo is bind-mounted at `/repo`, so edits on the host are
   live inside the container.
3. **A drop-in API client for offboard machines or the robot itself** —
   the image runs on Linux, macOS (Docker Desktop), or Windows (WSL2).
   Set `ROS_DOMAIN_ID` and pick an RMW (FastDDS / Cyclone / Zenoh) to
   match the robot, and your scripts talk to OutdoorNav as if you were on the
   robot. Useful when:
   - your laptop is a different OS than the robot,
   - you don't want to install the overlay on the host you're using,
   - the robot's onboard compute is busy and you want a thin client
     elsewhere on the same network.

The image is published to GHCR as `ghcr.io/nvanheyst/onav-api-examples:latest`.
`latest` is only published from `main`. PR builds publish temporary
`pr-<number>` tags.

Docker CI runs `./ci/smoke_all.sh` and `./ci/scenario_smoke.sh` inside the
built container before it pushes an image.

In scenario smoke, you should see three `==> scenario:` lines and dry-run
output for each command.

## Pull

The image is public — no GHCR login needed. Pulling the prebuilt `:latest`
is the preferred path; you only need to build locally if you're changing the
Dockerfile.

```bash
cd docker
docker compose pull        # always gets :latest
```

## (Optional) build locally

If you're iterating on the Dockerfile itself:

```bash
docker compose build
```

## RMW / DDS

OutdoorNav defaults to FastDDS (same as ROS 2 Jazzy). This container does
the same. Copy `.env.example` to `.env`: it ships a complete, ready-to-run
CycloneDDS + namespace block you can use as-is (or edit the three values to
match your robot). To stay on FastDDS instead, comment the CycloneDDS lines
and uncomment the FastDDS block.

- **FastDDS** — default. Optionally point `FASTRTPS_DEFAULT_PROFILES_FILE`
  at your own XML for custom transport / discovery tuning.
- **CycloneDDS** — uses `docker/cyclonedds.xml`. Edit the
  `<NetworkInterface name=…/>` line to the interface that reaches your
  robot (`eth0`, `enp4s0`, etc.). Wired ethernet only — RELIABLE QoS
  retransmits for service requests don't tolerate wifi packet loss.
- **Zenoh** — `rmw_zenoh_cpp` is not pre-installed; add it to the
  Dockerfile if you want to use it, then point `ZENOH_CONFIG_OVERRIDE` at
  your config.

If your robot uses a non-default ROS domain, set `ROS_DOMAIN_ID` in `.env`
(default `25` to match the local gz sim stack). Set `ONAV_NAMESPACE` in the
same file to point every script at your robot (default `/a300_00003`); the
value is normalized, so `a300_00003`, `/a300_00003`, and `/a300_00003/` all
work.

## Shell

```bash
docker compose run --rm dev
```

ROS and the overlay are already sourced in `~/.bashrc`. The repo is
mounted at `/repo` and the shell starts there. Run any example:

```bash
python3 examples/missions/random_visit_mission.py --count 5
python3 examples/maps/row_generator_square.py --lat 50.1094 --lon -97.3187 \
    --width 30 --height 20 --spacing 5 --name smoke_rows
```

## One-shot

```bash
docker compose run --rm dev python3 examples/maps/load_map_from_file.py \
    path/to/your_map.json
```

## Teardown

```bash
docker compose down
```
