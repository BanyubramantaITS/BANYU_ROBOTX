#!/bin/bash
# Print the GPS fix and heading (no arming), to note waypoints.
# Usage: tools/scripts/move/where.sh (Ctrl+C to stop; all options: --help)
PYTHON="${BANYU_PYTHON:-$HOME/venv-ardupilot/bin/python3}"
[ -x "$PYTHON" ] || PYTHON=python3
exec "$PYTHON" "$(dirname "$(readlink -f "$0")")/auv_move.py" where "$@"
