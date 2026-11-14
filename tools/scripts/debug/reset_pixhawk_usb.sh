#!/usr/bin/env bash
    # Reset Pixhawk USB device (Vendor: 1209, Product: 5741)
    set -e

    if [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
      echo "Usage: sudo reset_pixhawk_usb.sh   (unbinds and rebinds the Pixhawk 1209:5741 USB device)"
      exit 0
    fi

    VENDOR="1209"
    PRODUCT="5741"

    if [ "$EUID" -ne 0 ]; then
      echo "[-] Please run as root (sudo ./reset_pixhawk_usb.sh)"
      exit 1
    fi

    echo "[*] Searching for Pixhawk ($VENDOR:$PRODUCT)..."

    FOUND=0
    for dev in /sys/bus/usb/devices/*; do
      if [ -f "$dev/idVendor" ] && [ -f "$dev/idProduct" ]; then
        v=$(cat "$dev/idVendor")
        p=$(cat "$dev/idProduct")
        if [ "$v" = "$VENDOR" ] && [ "$p" = "$PRODUCT" ]; then
          DEV_PATH="$dev"
          DEV_NAME=$(basename "$dev")
          FOUND=1
          break
        fi
      fi
    done

    if [ $FOUND -eq 0 ]; then
      echo "[-] Pixhawk ($VENDOR:$PRODUCT) not found on any USB bus."
      exit 1
    fi

    echo "[+] Found Pixhawk at sysfs node: $DEV_NAME ($DEV_PATH)"

    # Method A: De-authorize and re-authorize USB device
    echo "[*] De-authorizing USB device (disconnecting)..."
    echo 0 > "$DEV_PATH/authorized"
    sleep 1

    echo "[*] Re-authorizing USB device (reconnecting)..."
    echo 1 > "$DEV_PATH/authorized"
    sleep 1

    # Method B: Trigger USB bus reset via usbreset if installed
    if command -v usbreset >/dev/null 2>&1; then
      echo "[*] Issuing USBDEVFS_RESET via usbreset..."
      usbreset "$VENDOR:$PRODUCT" || true
    fi

    echo "[+] Pixhawk USB connection reset successfully!"
