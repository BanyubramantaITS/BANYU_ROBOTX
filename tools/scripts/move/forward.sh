#!/bin/bash
# Drive forward for a few seconds (surface, or --depth D), then disarm.
# Usage: tools/scripts/move/forward.sh --time 5 --power 30 [--dry-run] (all options: --help)
PYTHON="${BANYU_PYTHON:-$HOME/venv-ardupilot/bin/python3}"
[ -x "$PYTHON" ] || PYTHON=python3
exec "$PYTHON" "$(dirname "$(readlink -f "$0")")/auv_move.py" forward "$@"
