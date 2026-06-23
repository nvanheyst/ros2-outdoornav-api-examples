#!/usr/bin/env bash
set -eo pipefail

# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash
if [ -f /opt/ros2_overlay/install/setup.bash ]; then
  # shellcheck source=/dev/null
  source /opt/ros2_overlay/install/setup.bash
fi

exec "$@"
