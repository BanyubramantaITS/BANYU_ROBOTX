#!/bin/bash
# SAUVC video run: dive, forward, turn 180 deg clockwise, forward back to the start, hover, disarm.
# Usage: tools/scripts/move/sauvc.sh --speed 0.4 [--depth 0|1.0 --distance 5] [--dry-run] (all options: --help)
PYTHON="${BANYU_PYTHON:-$HOME/venv-ardupilot/bin/python3}"
[ -x "$PYTHON" ] || PYTHON=python3
exec "$PYTHON" "$(dirname "$(readlink -f "$0")")/auv_move.py" sauvc "$@"
