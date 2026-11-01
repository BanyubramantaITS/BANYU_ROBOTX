#!/bin/bash
set -x
# For forwarding PIXHAWK to GroundControl via IP and local UDP ports for MAVROS (14551) and nogps_odom_beacon (14552)
# 192.168.0.155 is outside minipc device (laptopnya jalu)
MASTER_DEV="${1:-/dev/pixhawk}"
if [ ! -e "$MASTER_DEV" ] && [ -e "/dev/ttyACM0" ]; then
  MASTER_DEV="/dev/ttyACM0"
fi
QGC_IP="${2:-192.168.0.155}"
# One telemetry log per session (MAVProxy truncates its log on start). It holds the QGC
# joystick MANUAL_CONTROL that por_teach_extract.py uses when there is no RC receiver.
LOG_DIR="${BANYU_MAVPROXY_LOG_DIR:-$HOME/banyu_bags/mavproxy}"
mkdir -p "$LOG_DIR"

mavproxy.py --master="${MASTER_DEV}" \
  --logfile="${LOG_DIR}/$(date +%Y%m%d_%H%M%S).tlog" \
  --out="udp:${QGC_IP}:14550" \
  --out=udp:127.0.0.1:14550 \
  --out=udp:127.0.0.1:14551 \
  --out=udp:127.0.0.1:14552 \
  --streamrate=10 \
  --daemon
