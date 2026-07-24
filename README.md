# onav-lab

OutdoorNav 2.0 usage examples and tools. A personal extension of Clearpath's official
OutdoorNav documentation, meant to help people get started and get more out of OutdoorNav.

Open an issue if you have an example request.

## Unofficial

This is a personal project, kept on my own time and account. It isn't official Clearpath
material or a supported product, and it doesn't replace the documentation. For the
authoritative reference, use the official
[OutdoorNav user manual](https://docs.clearpathrobotics.com/docs_outdoornav_user_manual/) and
the [clearpath_outdoornav](https://github.com/cpr-application/clearpath_outdoornav) repository.
I keep it public because it has saved me time and might do the same for someone else.

## What's here

- **[docker/](docker/README.md)** — ROS 2 Jazzy dev container. Pull it, point it at a real robot or sim, and everything in `examples/` runs inside it.
- **[examples/](examples/README.md)** — single-file OutdoorNav API examples, written against raw `rclpy` with no SDK wrappers. Grouped by maps, missions, control, ops, and patterns.
- **[mini-projects/](mini-projects/IDEAS.md)** — notes toward larger demos. Not built yet; open an issue if you want to see one.

## Quick start

```bash
cd docker
docker compose pull
docker compose run --rm dev
```

See [docker/README.md](docker/README.md) for connecting to a real robot or sim, and
[examples/README.md](examples/README.md) to start running examples.
