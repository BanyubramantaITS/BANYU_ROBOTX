#!/bin/bash
# Bench test of the wired AUV hardware (devices, kill switch, cameras, GPS, Pixhawk/TX16S).
# Usage: tools/scripts/debug/test_hardware.sh [devices|kill|cameras|gps|pixhawk ...] [--no-prompt]
# Stop the mission stack first; keep run_mavproxy.sh running for the pixhawk test.
PYTHON="${BANYU_PYTHON:-$HOME/venv-ardupilot/bin/python3}"
[ -x "$PYTHON" ] || PYTHON=python3
exec "$PYTHON" "$(dirname "$(readlink -f "$0")")/test_hardware.py" "$@"
