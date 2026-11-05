#!/bin/bash
# ==============================================================================
# run_gps_bridge.sh — u-blox RTK GPS -> Pixhawk (MAVLink GPS_INPUT, GPS1_TYPE 14)
# ==============================================================================
# Runs gps_to_pixhawk.py from the team repo BanyubramantaITS/RTK-GPS-GNSS. It sends GPS_INPUT to
# MAVProxy on 127.0.0.1:14553 (run_mavproxy.sh), which forwards it to the Pixhawk, instead of
# opening /dev/pixhawk a second time. Start it after run_mavproxy.sh; leave it running.
# FALLBACK ONLY: the AUV's F9P is wired to the Pixhawk (GPS1_TYPE 1), which ignores GPS_INPUT;
# use this only with a receiver on USB alone and GPS1_TYPE 14 (docs/hardware_setup.md section 4).
# ==============================================================================

set -e

GPS_REPO="${BANYU_GPS_REPO:-$HOME/RTK-GPS-GNSS}"
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

Environment: BANYU_GPS_REPO (default ~/RTK-GPS-GNSS), BANYU_GPS_PYTHON (default ~/venv-ardupilot).
Nothing is sent to the Pixhawk until the receiver has a fix (antenna with sky view).
Check on the Pixhawk side: ros2 topic echo /mavros/global_position/raw/fix --once
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

# --no-anti-drift: the bridge's anti-drift freezes the position and zeroes the velocity below
# 0.35 m/s / 1.6 m, which the slow AUV is during the whole GPS leg. --optimize-gps re-applies the
# receiver config on each start: sea dynamic model, 10 deg elevation mask, rate.
BRIDGE_CMD=(
  "$PYTHON" "${GPS_REPO}/gps_to_pixhawk.py"
  --gps-port "$GPS_PORT"
  --pixhawk-port "$MAVLINK_URL"
  --rate "$RATE_HZ"
  --no-anti-drift
  --no-qgc
  --optimize-gps --dyn-model sea --elev-mask 10
  --mode cli
)

echo "=========================================================="
echo " BANYU AUV — RTK GPS bridge (keep this terminal open)"
echo "=========================================================="
if [ -e "$GPS_PORT" ]; then
  echo "GPS Port          : OK ($GPS_PORT -> $(readlink -f "$GPS_PORT"))"
else
  echo "Error: GPS port $GPS_PORT not found (u-blox plugged in? udev rule installed?)." >&2
  [ "$DRY_RUN" = true ] || exit 1
fi
if [ ! -f "${GPS_REPO}/gps_to_pixhawk.py" ]; then
  echo "Error: ${GPS_REPO}/gps_to_pixhawk.py not found (clone BanyubramantaITS/RTK-GPS-GNSS)." >&2
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
