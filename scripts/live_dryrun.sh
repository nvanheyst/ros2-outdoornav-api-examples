#!/usr/bin/env bash
# Live --dry-run sweep against a running OnAV stack.
# - Read-only examples: actually run.
# - Mutating examples:  invoked with --dry-run.
# Use this to verify wiring (ROS env, namespace, IDs) before doing a real sweep.
#
#   ONAV_NAMESPACE=/a300_00003 \
#   ONAV_MAP_ID=... ONAV_MISSION_ID=... ONAV_POI_ID=... \
#   ./scripts/live_dryrun.sh

set -uo pipefail
cd "$(dirname "$0")/.." || exit

PASS=0; FAIL=0

run() {
    local desc="$1"; shift
    echo "==> $desc"
    if "$@"; then
        PASS=$((PASS + 1)); echo "  ok"
    else
        FAIL=$((FAIL + 1)); echo "  FAIL: $*"
    fi
    echo
}

# Read-only — actually run.
run "service inventory" python3 examples/diagnostics/service_inventory.py --grep mission_manager
run "where am I"        python3 examples/diagnostics/where_am_i.py --timeout 5

# Mutating — dry-run only.
run "row gen square (dry)" python3 examples/maps/row_generator_square.py \
    --lat 50.10940 --lon -97.31870 --width 30 --height 20 --spacing 5 --dry-run
if [ -n "${ONAV_MAP_ID:-}" ]; then
    run "traversal mission (dry)" python3 examples/missions/generate_traversal_mission.py --dry-run
    run "random visit (dry)"       python3 examples/missions/random_visit_mission.py --dry-run --count 3
    run "traverse shortest (dry)"  python3 examples/missions/traverse_entire_map_shortest.py --dry-run
fi
if [ -n "${ONAV_MISSION_ID:-}" ] && [ -n "${ONAV_MAP_ID:-}" ]; then
    run "loop mission (dry)"     python3 examples/missions/loop_mission_battery_aware.py --dry-run
    run "schedule mission (dry)" python3 examples/missions/schedule_mission.py +1m --dry-run
fi
run "drive forward (dry)" python3 examples/control/drive_robot_forward.py --dry-run
run "stop autonomy (dry)" python3 examples/control/stop_autonomy.py --dry-run

echo "PASS: $PASS  FAIL: $FAIL"
[ "$FAIL" -eq 0 ]
