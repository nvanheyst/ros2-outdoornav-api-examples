# Dev environment

Docker image with ROS 2 Jazzy + the Clearpath message + API library overlay
pre-built. Saves you from installing the overlay on the host.

The image is published to GHCR as `ghcr.io/nvanheyst/onav-api-examples:latest`.
`latest` is only published from `main`. PR builds publish temporary
`pr-<number>` tags.

Docker CI runs `./scripts/smoke_all.sh` and `./scripts/scenario_smoke.sh`
inside the built container before it pushes an image.
In scenario smoke, you should see three `==> scenario:` lines and dry-run
output for each command.

## Pull

    cd docker
    docker compose pull        # always gets :latest

## (Optional) build locally

If you're iterating on the Dockerfile itself:

    docker compose build

## RMW / DDS

OutdoorNav defaults to FastDDS (same as ROS 2 Jazzy). This container does
the same. If you run a different RMW on your robot, copy `.env.example` to
`.env` and uncomment the block that matches.

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
(default `25` to match the local gz sim stack).

## Shell

    docker compose run --rm dev

ROS and the overlay are already sourced in `~/.bashrc`. The repo's
`examples/` directory is mounted at `/examples`. Run any example:

    python3 /examples/random_visit_mission.py 5
    python3 /examples/row_generator_square.py

## One-shot

    docker compose run --rm dev python3 /examples/load_map_from_file.py /examples/data/my_map.json

## Teardown

    docker compose down
