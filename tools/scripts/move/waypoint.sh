#!/bin/bash
# Drive to RTK GPS waypoints at the surface, then disarm.
# Usage: tools/scripts/move/waypoint.sh --ne 10,0 [--ne 10,10 ...] [--return-home] [--dry-run] (all options: --help)
PYTHON="${BANYU_PYTHON:-$HOME/venv-ardupilot/bin/python3}"
[ -x "$PYTHON" ] || PYTHON=python3
exec "$PYTHON" "$(dirname "$(readlink -f "$0")")/auv_move.py" waypoint "$@"
