#!/bin/bash
# YOLO on the Hailo-8 against the front camera; options: yolo_test.sh --help.
PYTHON="${BANYU_YOLO_PYTHON:-$HOME/venv-yolo/bin/python3}"
[ -x "$PYTHON" ] || { echo "Error: $PYTHON missing; run tools/scripts/setup/yolo_setup.sh first." >&2; exit 1; }
exec "$PYTHON" "$(dirname "$(readlink -f "$0")")/yolo_test.py" "$@"
