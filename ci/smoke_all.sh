#!/usr/bin/env bash
# Smoke test that does NOT require a live OnAV stack.
# - Every example responds to --help without error.
# - No script imports clearpath_outdoornav_api_lib (the audit hook).
#
# Run from the repo root:
#   ./ci/smoke_all.sh

set -euo pipefail

cd "$(dirname "$0")/.." || exit

# 1. No app-lib imports anywhere in examples/ or common/.
echo "==> grep: no clearpath_outdoornav_api_lib imports"
if grep -rnE "^(from|import)[[:space:]]+clearpath_outdoornav_api_lib" \
       examples/ common/ 2>/dev/null; then
    echo "FAIL: app-lib import detected. This folder is raw-ROS only."
    exit 1
fi
echo "  ok"

# 2. Every example responds to --help.
# If the ROS environment isn't sourced (no clearpath_* msgs / no rclpy) we
# treat ModuleNotFoundError on those modules as 'skipped' rather than failure,
# so the smoke can run on a dev machine without a full OnAV install.
echo "==> --help on every example"
fail=0; skip=0; pass=0
while IFS= read -r script; do
    out=$(python3 "$script" --help 2>&1) && rc=0 || rc=$?
    if [ "$rc" -eq 0 ]; then
        echo "  ok:      $script"; pass=$((pass + 1))
    elif echo "$out" | grep -qE "No module named '(rclpy|clearpath_|sensor_msgs|geometry_msgs|std_srvs|shapely|networkx|pyproj)"; then
        missing=$(echo "$out" | grep -oE "No module named '[^']+'" | head -1)
        echo "  skipped: $script  ($missing)"; skip=$((skip + 1))
    else
        echo "  FAIL:    $script"
        out_indented=${out//$'\n'/$'\n      '}
        echo "      $out_indented"
        fail=$((fail + 1))
    fi
done < <(find examples/ -name "*.py" -not -name "__init__.py")

echo
echo "pass=$pass skipped=$skip fail=$fail"
[ "$fail" -eq 0 ]
