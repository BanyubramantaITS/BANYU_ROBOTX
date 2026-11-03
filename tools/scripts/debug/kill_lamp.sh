#!/bin/bash
# Set or read the ESP32 kill-switch indicator lamp by hand (the port must be free: no teleop,
# no esp32_trigger_node). Wraps tools/scripts/esp32_kill_cmd.py.
set -e
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
PORT="${BANYU_ESP32_PORT:-/dev/esp32_kill}"
PYTHON="${BANYU_PYTHON:-python3}"

print_usage() {
  cat << 'USAGE'
Usage: tools/scripts/debug/kill_lamp.sh <manual|auto|emergency|status> [--hold]

  manual     MANUAL lamp     (sends its0, = operational_mode 1)
  auto       AUTO lamp       (sends its1, = operational_mode 2)
  emergency  EMERGENCY lamp  (sends its2, = operational_mode 0)
  status     Print the board's reports for 2 s. "its2" while you set manual/auto means the
             physical kill input is active.
  --hold     Keep the port open until Ctrl+C.

The lamp keeps its state after the script exits. Port: $BANYU_ESP32_PORT (default /dev/esp32_kill).
USAGE
}

ACTION="${1:-}"; shift || true
if [ ! -e "$PORT" ]; then
  echo "Error: $PORT not found (ESP32 plugged in? udev rule installed?)" >&2; exit 1
fi
if command -v fuser > /dev/null && fuser "$PORT" > /dev/null 2>&1; then
  echo "Error: $PORT is in use by PID(s):$(fuser "$PORT" 2>/dev/null) (banyu_teleop / esp32_trigger_node?)." >&2
  exit 1
fi

case "$ACTION" in
  manual) exec "$PYTHON" "${SCRIPT_DIR}/../esp32_kill_cmd.py" remote --port "$PORT" "$@" ;;
  auto) exec "$PYTHON" "${SCRIPT_DIR}/../esp32_kill_cmd.py" auto --port "$PORT" "$@" ;;
  emergency) exec "$PYTHON" "${SCRIPT_DIR}/../esp32_kill_cmd.py" kill --port "$PORT" "$@" ;;
  status)
    stty -F "$PORT" 115200 raw -echo -hupcl
    echo "Reports from $PORT (its0 manual, its1 auto, its2 emergency):"
    timeout 2 cat "$PORT" | tr -d '\r' || true ;;
  -h|--help|"") print_usage; [ -n "$ACTION" ] ;;
  *) print_usage; exit 1 ;;
esac
