#!/usr/bin/env bash
# Quick scenario smoke checks for CI.
# Runs a few real scripts in --dry-run mode.
# Expect three "==> scenario:" lines and dry-run output from each command.

set -euo pipefail

cd "$(dirname "$0")/../.." || exit

if ! python3 -c "import rclpy" >/dev/null 2>&1; then
  echo "==> skipping scenario smoke (rclpy not available)"
  exit 0
fi

tmpdir="$(mktemp -d)"
trap 'rm -rf "$tmpdir"' EXIT

cat > "$tmpdir/ci_map.json" <<'JSON'
{
  "name": "ci-map",
  "default_radius": 1.5,
  "default_speed_limit": 1.0,
  "points": [
    {"id": "p1", "latitude": 50.1094, "longitude": -97.3187},
    {"id": "p2", "latitude": 50.1095, "longitude": -97.3186}
  ],
  "connections": [
    {"start_point_id": "p1", "end_point_id": "p2", "speed_limit": 1.0, "radius": 1.5}
  ]
}
JSON

echo "==> scenario: stop autonomy dry-run"
python3 examples/control/stop_autonomy.py --dry-run

echo "==> scenario: square map generation dry-run"
python3 examples/maps/row_generator_square.py \
  --lat 50.1094 \
  --lon -97.3187 \
  --width 20 \
  --height 12 \
  --spacing 3 \
  --name ci-square \
  --dry-run

echo "==> scenario: load map from JSON dry-run"
python3 examples/maps/load_map_from_file.py "$tmpdir/ci_map.json" --name ci-map --dry-run
