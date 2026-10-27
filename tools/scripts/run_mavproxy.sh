#!/bin/bash
set -x
# For forwarding PIXHAWK to GroundControl via IP and local UDP ports for MAVROS (14551) and nogps_odom_beacon (14552)
# 192.168.0.155 is outside minipc device (laptopnya jalu)
MASTER_DEV="${1:-/dev/pixhawk}"
if [ ! -e "$MASTER_DEV" ] && [ -e "/dev/ttyACM0" ]; then
  MASTER_DEV="/dev/ttyACM0"
fi
QGC_IP="${2:-192.168.0.155}"

mavproxy.py --master="${MASTER_DEV}" \
  --out="udp:${QGC_IP}:14550" \
  --out=udp:127.0.0.1:14550 \
  --out=udp:127.0.0.1:14551 \
  --out=udp:127.0.0.1:14552 \
  --streamrate=10 \
  --daemon
