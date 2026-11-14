#!/bin/bash
# Arm, dive to a depth, hold it, then disarm and float up.
# Usage: tools/scripts/move/dive.sh --depth 1.0 --hold 10 [--dry-run] (all options: --help)
PYTHON="${BANYU_PYTHON:-$HOME/venv-ardupilot/bin/python3}"
[ -x "$PYTHON" ] || PYTHON=python3
exec "$PYTHON" "$(dirname "$(readlink -f "$0")")/auv_move.py" dive "$@"
