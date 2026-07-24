#!/usr/bin/env bash
# Create the console venv and install dependencies. Run once on the operator machine.
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
VENV="$SCRIPT_DIR/.venv"

if [ -d "$VENV" ]; then
    echo "venv already exists at $VENV — skipping. Delete it to reinstall."
    exit 0
fi

python3 -m venv "$VENV"
"$VENV/bin/pip" install --quiet --upgrade pip
"$VENV/bin/pip" install --quiet -r "$SCRIPT_DIR/requirements.txt"
echo "venv ready at $VENV"
echo "Run with: bash $SCRIPT_DIR/run.sh"
