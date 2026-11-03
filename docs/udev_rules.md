# USB udev Rules for Hardware Devices

Persistent device names on the AUV computer. The rules file is
[`tools/udev/99-banyu.rules`](../tools/udev/99-banyu.rules):

| Name | Device | USB ID |
|---|---|---|
| `/dev/pixhawk` | Pixhawk 1 | `1209:5741` |
| `/dev/gps_rtk` | u-blox RTK GNSS receiver | `1546:01a9` |
| `/dev/esp32_kill` | ESP32 kill-switch indicator (native USB) | `303a:1001` |
| `/dev/front_camera` | "H264 USB Camera" capture node (index 0) | `32e4:9422` |
| `/dev/bottom_camera` | "2M WebCam" / Horus B capture node (index 0) | `2ce3:3894` |

The serial rules also set `ID_MM_DEVICE_IGNORE` so ModemManager never probes these ports.
What each device is used for: [`hardware_setup.md`](hardware_setup.md).

## Installation

```bash
sudo install -m 644 src/BANYU_ROBOTX/tools/udev/99-banyu.rules /etc/udev/rules.d/99-banyu.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --action=change --subsystem-match=tty
sudo udevadm trigger --action=change --subsystem-match=video4linux
ls -l /dev/pixhawk /dev/gps_rtk /dev/esp32_kill /dev/front_camera /dev/bottom_camera
```

Installed on the competition Pi (banyu@192.168.0.100) on 2026-11-03.
