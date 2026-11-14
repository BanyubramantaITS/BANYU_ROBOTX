#!/bin/bash
# Turn on the compass by an angle (or --time s open loop), then disarm.
# Usage: tools/scripts/move/yaw.sh --dir left|right --angle 90 [--dry-run] (all options: --help)
PYTHON="${BANYU_PYTHON:-$HOME/venv-ardupilot/bin/python3}"
[ -x "$PYTHON" ] || PYTHON=python3
exec "$PYTHON" "$(dirname "$(readlink -f "$0")")/auv_move.py" yaw "$@"
