#!/bin/bash
# One-screen check of the AUV hardware before a run: udev devices, who holds them, and the
# background services (MAVProxy, MAVROS, GPS bridge, RTK corrections).
if [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
  echo "Usage: check_devices.sh   (read-only: devices, their users, services, last kill-switch report)"
  exit 0
fi
ok() { printf '  %-16s OK   %s\n' "$1" "$2"; }
bad() { printf '  %-16s FAIL %s\n' "$1" "$2"; }

echo "Devices (tools/udev/99-banyu.rules):"
for dev in pixhawk esp32_kill gps_rtk front_camera bottom_camera; do
  path="/dev/$dev"
  if [ -e "$path" ]; then
    users="$(fuser "$path" 2>/dev/null | xargs)"
    owner=""
    for pid in $users; do owner+=" $(ps -o comm= -p "$pid")($pid)"; done
    ok "$dev" "-> $(readlink -f "$path")${owner:+  used by:$owner}"
  else
    bad "$dev" "missing"
  fi
done

echo "Processes:"
proc() { if pgrep -f "$2" > /dev/null; then ok "$1" "running"; else bad "$1" "not running ($3)"; fi; }
proc mavproxy "mavproxy.py" "run_mavproxy.sh"
if pgrep -f "mavproxy.py.*udpin:127.0.0.1:14553" > /dev/null || ! pgrep -f mavproxy.py > /dev/null; then :; else
  bad "mavproxy:14553" "MAVProxy started without the GPS port; restart run_mavproxy.sh"; fi
proc mavros "mavros_node" "launch_mavros.sh"
proc gps_bridge "gps_to_pixhawk.py" "run_gps_bridge.sh; needed for Task 2"
if systemctl cat rtk-rover.service > /dev/null 2>&1; then
  [ "$(systemctl is-active rtk-rover.service)" = active ] && ok rtk-rover "active" || bad rtk-rover "$(systemctl is-active rtk-rover.service)"
else
  bad rtk-rover "not installed (setup/rtk_setup.sh)"
fi

if [ -e /dev/esp32_kill ] && ! fuser /dev/esp32_kill > /dev/null 2>&1; then
  stty -F /dev/esp32_kill 115200 raw -echo -hupcl 2>/dev/null
  last="$(timeout 1.2 cat /dev/esp32_kill 2>/dev/null | tr -d '\r' | tail -n 1)"
  echo "Kill switch board reports: ${last:-nothing} (its2 with nothing commanded = physical kill active)"
fi
