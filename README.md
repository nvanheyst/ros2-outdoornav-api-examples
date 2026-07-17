# onav-lab

OutdoorNav 2.0 usage examples and tools. This is a personal extension of Clearpath's official
OutdoorNav documentation, meant to help people get started and get more out of OutdoorNav. It
began as a dev container with a handful of API examples and has grown from there.

Open an issue if you have an example request.

## Unofficial

This is a personal project, kept on my own time and account. It isn't official Clearpath
material or a supported product, and it doesn't replace the documentation. For the
authoritative reference, use the official
[OutdoorNav user manual](https://docs.clearpathrobotics.com/docs_outdoornav_user_manual/) and
the [clearpath_outdoornav](https://github.com/cpr-application/clearpath_outdoornav) repository.
I keep it public because it has saved me time and might do the same for someone else.

## What's here

`docker/` is a general-purpose ROS 2 (Jazzy) dev container for talking to a real robot or a
sim. It carries committed transport profiles, runs on its own, and is where the examples run.

`examples/` holds single-file OutdoorNav API examples written against raw rclpy, grouped by
maps, missions, control, and ops, along with a few reusable patterns. See
[examples/README.md](examples/README.md).

`mini-projects/` collects notes toward larger, multi-file examples that aren't built yet.

## Quick start

```bash
cd docker
docker compose pull
docker compose run --rm dev
```

## Connecting

Transport is set by a committed profile instead of environment variables you have to remember.
The primary target is a real robot, which uses Fast DDS with the robot's discovery server.
Switching to sim is one flag.

```bash
# real AMP: run this on the robot, since the discovery server is on the robot's loopback
docker compose --env-file real-amp.env run --rm dev

# sim: CycloneDDS multicast
docker compose --env-file sim.env run --rm dev
```

|           | real amp (`real-amp.env`)                         | sim (`sim.env`)      |
|-----------|---------------------------------------------------|----------------------|
| RMW       | `rmw_fastrtps_cpp`                                | `rmw_cyclonedds_cpp` |
| Discovery | Fast DDS discovery server `127.0.0.1:11811`       | multicast            |
| Domain    | per robot (`robot.yaml`, `system.ros2.domain_id`) | 25                   |

Run `python3 examples/ops/connect_real_robot.py` first to confirm the connection. Since a real
robot runs its discovery server on loopback, run this on the robot. Connecting from another
machine needs a bridge, and a Cloudflare TCP tunnel isn't allowed under the ToS. From there,
see [examples/](examples/).
