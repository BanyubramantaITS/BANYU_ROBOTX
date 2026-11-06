#!/bin/bash
# ==============================================================================
# run_gps_bridge.sh — u-blox RTK GPS -> Pixhawk (MAVLink GPS_INPUT, GPS1_TYPE 14)
# ==============================================================================
# Runs gps_forwarder.py (next to this script): UBX NAV-PVT from the receiver -> GPS_INPUT to
# MAVProxy on 127.0.0.1:14553 (run_mavproxy.sh), which forwards it to the Pixhawk, instead of
# opening /dev/pixhawk a second time. The Pixhawk's GPS with GPS1_TYPE 14 (docs/hardware_setup.md
# section 4). Runs on its own, like MAVProxy and MAVROS; robot order: run_mavproxy.sh ->
# run_gps_bridge.sh -> launch_mavros.sh -> launch_task2.sh --real (which refuses to start without it).
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
FORWARDER="${SCRIPT_DIR}/gps_forwarder.py"
PYTHON="${BANYU_GPS_PYTHON:-$HOME/venv-ardupilot/bin/python3}"
GPS_PORT="/dev/gps_rtk"
MAVLINK_URL="udpout:127.0.0.1:14553"
RATE_HZ="5"
DRY_RUN=false

print_usage() {
  cat << 'USAGE'
Usage: ./tools/scripts/run/run_gps_bridge.sh [OPTIONS]

Options:
  --gps-port PATH     u-blox serial port (default /dev/gps_rtk, udev rule tools/udev/99-banyu.rules).
  --mavlink URL       Where GPS_INPUT goes (default udpout:127.0.0.1:14553 = run_mavproxy.sh).
  --rate HZ           GPS_INPUT rate (default 5; Pixhawk GPS1_RATE_MS is 200).
  --dry-run           Print the command without starting the bridge.
  -h, --help          Show this help message.

Environment: BANYU_GPS_PYTHON (default ~/venv-ardupilot/bin/python3, needs pyserial + pymavlink).
On each start the receiver gets the sea dynamic model, a 10 deg elevation mask and static hold off.
Nothing is sent to the Pixhawk until the receiver has a fix (antenna with sky view).
Check on the Pixhawk side: ros2 topic echo /mavros/gpsstatus/gps1/raw --once
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --gps-port) GPS_PORT="$2"; shift 2 ;;
    --mavlink) MAVLINK_URL="$2"; shift 2 ;;
    --rate) RATE_HZ="$2"; shift 2 ;;
    --dry-run) DRY_RUN=true; shift ;;
    -h|--help) print_usage; exit 0 ;;
    *) echo "Error: unknown option $1 (see --help)" >&2; exit 1 ;;
  esac
done

BRIDGE_CMD=(
  "$PYTHON" "$FORWARDER"
  --gps "$GPS_PORT"
  --out "$MAVLINK_URL"
  --rate "$RATE_HZ"
)

echo "=========================================================="
echo " BANYU AUV — RTK GPS bridge (keep this terminal open)"
echo "=========================================================="
if [ -e "$GPS_PORT" ]; then
  echo "GPS Port          : OK ($GPS_PORT -> $(readlink -f "$GPS_PORT"))"
else
  # The forwarder waits for the port, but a missing udev name at start is an operator error.
  echo "Error: GPS port $GPS_PORT not found (u-blox plugged in? udev rule installed?)." >&2
  [ "$DRY_RUN" = true ] || exit 1
fi
if [ ! -x "$PYTHON" ]; then
  echo "Error: $PYTHON not found (set BANYU_GPS_PYTHON to a Python with pyserial + pymavlink)." >&2
  [ "$DRY_RUN" = true ] || exit 1
fi
if pgrep -f "python[0-9.]* [^ ]*gps_(forwarder|to_pixhawk)[.]py" > /dev/null; then
  echo "Error: a GPS forwarder is already running (only one may own $GPS_PORT)." >&2
  [ "$DRY_RUN" = true ] || exit 1
fi
if [[ "$MAVLINK_URL" == udpout:127.0.0.1:14553 ]] && ! pgrep -f "mavproxy.py.*udpin:127.0.0.1:14553" > /dev/null; then
  echo "Warning: no MAVProxy with --out=udpin:127.0.0.1:14553; (re)start ./run_mavproxy.sh first." >&2
fi
echo "MAVLink Out       : $MAVLINK_URL"
echo "=========================================================="
echo "${BRIDGE_CMD[*]}"
echo "=========================================================="

if [ "$DRY_RUN" = true ]; then
  echo "Dry run requested. Exiting without starting the bridge."
  exit 0
fi

exec "${BRIDGE_CMD[@]}"
