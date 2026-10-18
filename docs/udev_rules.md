# USB udev Rules for Hardware Devices

This document specifies the udev rules for persistent device naming on the companion computer.

## Rules (`/etc/udev/rules.d/99-banyu.rules`)

```udev
# Pixhawk 1 (Vendor: 1209, Product: 5741)
SUBSYSTEM=="tty", ATTRS{idVendor}=="1209", ATTRS{idProduct}=="5741", SYMLINK+="pixhawk", MODE="0666", GROUP="dialout"

# u-blox GNSS RTK Receiver (Vendor: 1546, Product: 01a9)
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", SYMLINK+="gps_rtk", MODE="0666", GROUP="dialout"
```

## Installation

1. Create or copy the rules file:
   ```bash
   sudo nano /etc/udev/rules.d/99-banyu.rules
   ```

2. Reload and trigger udev:
   ```bash
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```

3. Verification:
   ```bash
   ls -l /dev/pixhawk /dev/gps_rtk
   ```
