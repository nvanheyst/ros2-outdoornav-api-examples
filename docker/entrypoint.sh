#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
if [ -f /opt/ros2_overlay/install/setup.bash ]; then
  source /opt/ros2_overlay/install/setup.bash
fi

exec "$@"
