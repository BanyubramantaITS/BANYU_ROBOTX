# USB udev Rules for Hardware Devices

This document specifies the udev rules for persistent device naming on the companion computer.

## Rules (`/etc/udev/rules.d/99-banyu.rules`)

```udev
# Pixhawk 1 (Vendor: 1209, Product: 5741)
SUBSYSTEM=="tty", ATTRS{idVendor}=="1209", ATTRS{idProduct}=="5741", SYMLINK+="pixhawk", MODE="0666", GROUP="dialout"

# u-blox GNSS RTK Receiver (Vendor: 1546, Product: 01a9)
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", SYMLINK+="gps_rtk", MODE="0666", GROUP="dialout"

# Horus B / 2M WebCam (Vendor: 2ce3, Product: 3894)
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="2ce3", ATTRS{idProduct}=="3894", ATTR{index}=="0", SYMLINK+="horus_camera", MODE="0666", GROUP="video"
```

For camera-specific setup, parameters, and troubleshooting, refer to [Horus Camera Guide](horus_camera.md).

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
