#!/usr/bin/env bash
# Activate the venv and launch the console.
# Pass env vars before the script or export them:
#   ONAV_TARGET=real MCP_URL=http://10.0.0.35:8091/mcp bash run.sh
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
VENV="$SCRIPT_DIR/.venv"

if [ ! -d "$VENV" ]; then
    echo "Run setup first: bash $SCRIPT_DIR/setup.sh"
    exit 1
fi

exec "$VENV/bin/python" "$SCRIPT_DIR/console.py" "$@"
