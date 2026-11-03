#!/bin/bash
# RTK GPS health: rtk-rover (RTCM corrections) service, then the receiver's fix.
# Fix types from the bridge: 3 = 3D, 4 = DGPS, 5 = RTK float, 6 = RTK fixed.
SECONDS_TO_WATCH="${1:-10}"
GPS_PORT="${BANYU_GPS_PORT:-/dev/gps_rtk}"
GPS_REPO="${BANYU_GPS_REPO:-$HOME/RTK-GPS-GNSS}"
PYTHON="${BANYU_GPS_PYTHON:-$HOME/venv-ardupilot/bin/python3}"

if [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
  echo "Usage: tools/scripts/check/rtk_status.sh [seconds=10]"; exit 0
fi

echo "== rtk-rover service (RTCM corrections from the base station)"
if systemctl list-unit-files rtk-rover.service > /dev/null 2>&1 && systemctl cat rtk-rover.service > /dev/null 2>&1; then
  echo "state: $(systemctl is-active rtk-rover.service)"
  grep -o "tcpcli://[^ ]*" /etc/systemd/system/rtk-rover.service | sed 's/^/base:  /'
  journalctl -u rtk-rover -n 5 --no-pager 2>/dev/null | tail -n 5
else
  echo "not installed (run tools/scripts/setup/rtk_setup.sh); no RTK corrections, plain GPS only"
fi

echo; echo "== receiver ($GPS_PORT)"
if [ ! -e "$GPS_PORT" ]; then
  echo "not found (u-blox plugged in? udev rule installed?)"; exit 1
fi
if pgrep -f "gps_to_pixhawk.py" > /dev/null; then
  # The bridge owns the UBX stream; a second reader would split it. Ask the Pixhawk instead.
  echo "run_gps_bridge.sh is running; Pixhawk view:"
  [ -f /opt/ros/jazzy/setup.bash ] && source /opt/ros/jazzy/setup.bash
  timeout 5 ros2 topic echo /mavros/gpsstatus/gps1/raw --once 2>/dev/null \
    | grep -E "fix_type|satellites_visible|eph|h_acc" \
    || echo "no /mavros/gpsstatus/gps1/raw (MAVROS running?); bridge output is in its terminal"
else
  timeout "$SECONDS_TO_WATCH" "$PYTHON" "${GPS_REPO}/gps_to_pixhawk.py" --gps-port "$GPS_PORT" \
    --gps-only --no-anti-drift --mode cli || true
fi
