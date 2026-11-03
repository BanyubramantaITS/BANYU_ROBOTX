#!/bin/bash
# One-time RTK rover setup on the AUV computer: installs str2str (Ubuntu rtklib) if missing, then
# runs the teammate script fix_rtk_rover.sh, which installs the rtk-rover systemd service
# (RTCM from the base station tcpcli://<base>:2002 into the u-blox, restarted on failure, on boot).
set -e
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"

if ! command -v str2str > /dev/null; then
  echo "str2str not found: installing rtklib (sudo)."
  sudo apt-get install -y rtklib
fi
echo "Base station prompt: press ENTER for 192.168.0.1:2002 or type the competition base IP/port."
exec "${SCRIPT_DIR}/fix_rtk_rover.sh"
