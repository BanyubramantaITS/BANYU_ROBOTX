#!/bin/bash
# Vendored from https://github.com/jausyan/RobotX/blob/main/script/fix_rtk_rover.sh (teammate script, unmodified below).
# Run it through tools/scripts/setup/rtk_setup.sh, which installs str2str (rtklib) first.
# ============================================================
#  fix_rtk_rover.sh — RTK Rover Auto-Setup Script
#  Sets up str2str as a persistent systemd service
#  Fixes: ModemManager port-busy, manual restart, port lock
#
#  Usage: chmod +x fix_rtk_rover.sh && ./fix_rtk_rover.sh
# ============================================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

info()    { echo -e "${CYAN}[INFO]${NC} $1"; }
success() { echo -e "${GREEN}[✓]${NC} $1"; }
warn()    { echo -e "${YELLOW}[!]${NC} $1"; }
error()   { echo -e "${RED}[✗]${NC} $1"; exit 1; }
header()  { echo -e "\n${BOLD}${CYAN}=== $1 ===${NC}"; }

if [ "$EUID" -eq 0 ]; then
    error "Do NOT run as root. Run as your normal user."
fi

echo -e "${BOLD}"
echo "  ======================================================"
echo "   RTK ROVER — Persistent Setup Script"
echo "   Fixes: port-busy, ModemManager conflict, auto-restart"
echo "  ======================================================"
echo -e "${NC}"

header "Step 0: Verifying sudo access"
sudo -v || error "Sudo authentication failed."
success "Sudo OK"

# ---------- Step 1: Detect str2str binary ----------
header "Step 1: Locating str2str binary"

STR2STR_PATH=""
for p in "$HOME/RTKLIB/app/str2str/gcc/str2str" "$HOME/rtklib/app/str2str/gcc/str2str" "/usr/local/bin/str2str" "/usr/bin/str2str"; do
    if [ -x "$p" ]; then STR2STR_PATH="$p"; success "Found: $p"; break; fi
done

if [ -z "$STR2STR_PATH" ]; then
    FOUND=$(find "$HOME" /usr/local /opt -name "str2str" -type f -executable 2>/dev/null | head -1)
    [ -n "$FOUND" ] && STR2STR_PATH="$FOUND" && success "Found: $FOUND" || error "str2str not found!"
fi

# ---------- Step 2: Detect F9P serial port ----------
header "Step 2: Detecting u-blox F9P serial port"

F9P_PORT=""
UBLOX_BYID=$(ls /dev/serial/by-id/ 2>/dev/null | grep -i "u-blox\|ublox\|GNSS\|gnss" | head -1)

if [ -n "$UBLOX_BYID" ]; then
    F9P_PORT="serial/by-id/$UBLOX_BYID"
    success "Auto-detected: /dev/$F9P_PORT"
else
    warn "u-blox device not found via by-id. Available ports:"
    ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | while read p; do echo "    - $p"; done || echo "    (none found)"
    echo ""
    read -rp "  Enter full port path (e.g. /dev/ttyUSB0): " F9P_PORT_INPUT
    [ -z "$F9P_PORT_INPUT" ] && error "No port specified. Plug in F9P and re-run."
    F9P_PORT="$F9P_PORT_INPUT"
fi

# ---------- Step 3: Configure base station IP ----------
header "Step 3: Base Station Connection"

BASE_IP="192.168.0.1"
BASE_PORT="2002"
echo -e "  Default base station: ${BOLD}$BASE_IP:$BASE_PORT${NC}"
read -rp "  Press ENTER to use $BASE_IP, or type a different IP: " CUSTOM_IP
[ -n "$CUSTOM_IP" ] && BASE_IP="$CUSTOM_IP"
read -rp "  Press ENTER to use port $BASE_PORT, or type a different port: " CUSTOM_PORT
[ -n "$CUSTOM_PORT" ] && BASE_PORT="$CUSTOM_PORT"
success "Base station: $BASE_IP:$BASE_PORT"

# ---------- Step 4: Backup ----------
header "Step 4: Backing up existing configuration"

BACKUP_DIR="$HOME/backups/rtk_rover_backup_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$BACKUP_DIR"

for f in ~/rtk_script.sh ~/rtk_script.bash ~/rtk_rover.sh; do
    [ -f "$f" ] && cp "$f" "$BACKUP_DIR/" && info "Backed up: $f"
done
[ -f /etc/systemd/system/rtk-rover.service ] && \
    sudo cp /etc/systemd/system/rtk-rover.service "$BACKUP_DIR/rtk-rover.service.bak" && info "Backed up: rtk-rover.service"
[ -f /etc/udev/rules.d/99-f9p-ignore-modemmanager.rules ] && \
    sudo cp /etc/udev/rules.d/99-f9p-ignore-modemmanager.rules "$BACKUP_DIR/" && info "Backed up: udev rule"

success "Backup at: $BACKUP_DIR"

# ---------- Step 5: Cleanup ----------
header "Step 5: Cleaning up running str2str processes"

pgrep -x str2str > /dev/null 2>&1 && sudo pkill -9 str2str 2>/dev/null && sleep 1 && success "Killed old str2str" || info "No str2str running"
systemctl is-active --quiet rtk-rover.service 2>/dev/null && sudo systemctl stop rtk-rover.service && info "Stopped old service"
sudo rm -f /var/lock/LCK..tty* /run/lock/LCK..tty* 2>/dev/null || true
success "Lock files cleared"

# ---------- Step 6: udev rule ----------
header "Step 6: Installing udev rule (block ModemManager from F9P)"

sudo tee /etc/udev/rules.d/99-f9p-ignore-modemmanager.rules > /dev/null << 'EOF'
# Prevent ModemManager from claiming u-blox GNSS receivers (F9P/F94/M8P etc)
ATTRS{idVendor}=="1546", ENV{ID_MM_DEVICE_IGNORE}="1"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
success "udev rule installed"

# ---------- Step 7: systemd service ----------
header "Step 7: Installing rtk-rover systemd service"

if [[ "$F9P_PORT" == serial/by-id/* ]] || [[ "$F9P_PORT" == /dev/serial/* ]]; then
    SERIAL_OUT="serial://${F9P_PORT#/dev/}:115200"
elif [[ "$F9P_PORT" == /dev/* ]]; then
    SERIAL_OUT="serial://${F9P_PORT#/dev/}:115200"
else
    SERIAL_OUT="serial://${F9P_PORT}:115200"
fi

info "str2str input:  tcpcli://$BASE_IP:$BASE_PORT"
info "str2str output: $SERIAL_OUT"

sudo tee /etc/systemd/system/rtk-rover.service > /dev/null << EOF
[Unit]
Description=RTK Rover RTCM Injection via str2str
After=network.target
StartLimitIntervalSec=0

[Service]
Type=simple
User=root

ExecStartPre=/bin/bash -c "pkill -9 str2str 2>/dev/null; sleep 1; rm -f /var/lock/LCK..tty* /run/lock/LCK..tty*"

ExecStart=${STR2STR_PATH} -in tcpcli://${BASE_IP}:${BASE_PORT} -out ${SERIAL_OUT} -s 3000

Restart=always
RestartSec=5

StandardOutput=journal
StandardError=journal
SyslogIdentifier=rtk-rover

[Install]
WantedBy=multi-user.target
EOF

success "Service file written: /etc/systemd/system/rtk-rover.service"

# ---------- Step 8: Enable and start ----------
header "Step 8: Enabling and starting rtk-rover service"

sudo systemctl daemon-reload
sudo systemctl enable rtk-rover.service
success "Service enabled (auto-start on boot)"

sudo systemctl restart ModemManager 2>/dev/null || true
info "ModemManager restarted (now ignoring u-blox devices)"

sudo systemctl start rtk-rover.service
sleep 3

# ---------- Step 9: Status ----------
header "Step 9: Verification"
echo ""
sudo systemctl status rtk-rover.service --no-pager
echo ""

systemctl is-active --quiet rtk-rover.service && \
    echo -e "${GREEN}${BOLD}  RTK Rover is UP — receiving RTCM from $BASE_IP:$BASE_PORT${NC}" || \
    warn "Service retrying (normal if base offline or F9P not connected yet)"

echo ""
echo -e "${BOLD}${GREEN}============================================================${NC}"
echo -e "${BOLD}${GREEN}  SETUP COMPLETE!${NC}"
echo -e "${BOLD}${GREEN}============================================================${NC}"
echo ""
echo "  Installed:"
echo "    - udev rule:        /etc/udev/rules.d/99-f9p-ignore-modemmanager.rules"
echo "    - systemd service:  /etc/systemd/system/rtk-rover.service"
echo "    - Backup at:        $BACKUP_DIR"
echo ""
echo "  Useful commands:"
echo "    sudo systemctl start rtk-rover      # Start"
echo "    sudo systemctl stop rtk-rover       # Stop"
echo "    sudo systemctl status rtk-rover     # Status"
echo "    journalctl -u rtk-rover -f          # Live logs"
echo ""
echo "  To RESTORE:"
echo "    sudo systemctl disable --now rtk-rover.service"
echo "    sudo rm /etc/systemd/system/rtk-rover.service"
echo "    sudo rm /etc/udev/rules.d/99-f9p-ignore-modemmanager.rules"
echo "    # Restore from: $BACKUP_DIR"
echo ""
