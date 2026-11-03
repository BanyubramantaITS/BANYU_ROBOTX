# Competition Hardware Setup (Raspberry Pi 5)

The AUV computer is a Raspberry Pi 5 (Ubuntu 24.04, Jazzy, workspace `~/ros_ws`). This page lists
every USB device of the competition stack, how each one is wired into the software, the bring-up
order, and the debug scripts.

## 1. Devices

| Device | USB ID | Name (udev) | Used by |
|---|---|---|---|
| Pixhawk 1 (ArduSub) | `1209:5741` | `/dev/pixhawk` | `run_mavproxy.sh` (only process that opens it) |
| ESP32 kill-switch indicator (Espressif USB JTAG/serial) | `303a:1001` | `/dev/esp32_kill` | `banyu_teleop` (or `esp32_trigger_node` with `--no-rc`) |
| u-blox RTK GNSS receiver | `1546:01a9` | `/dev/gps_rtk` | `run_gps_bridge.sh` (reads), `rtk-rover` service (writes RTCM) |
| Front camera "H264 USB Camera" | `32e4:9422` | `/dev/front_camera` | `front_camera` usb_cam node; Task 2 YOLO collision avoidance (planned) |
| Bottom camera "2M WebCam" (Horus B) | `2ce3:3894` | `/dev/bottom_camera` | `bottom_camera` usb_cam node (Task 2) |

All configs use these names, so the `ttyACM*` / `video*` order after a replug does not matter.
The rules are in [`tools/udev/99-banyu.rules`](../tools/udev/99-banyu.rules); install them with
[`udev_rules.md`](udev_rules.md). The ESP32 rule matches the generic Espressif ID, so keep only one
ESP32 plugged in.

Check everything at once:

```bash
./src/BANYU_ROBOTX/tools/scripts/debug/check_devices.sh
```

## 2. Cameras

`sub_bringup.launch.py` starts `usb_cam` with [`hardware_params.yaml`](../banyu_bringup/config/hardware_params.yaml):

| Camera | Mode | Topic |
|---|---|---|
| Front (Task 2 YOLO, planned) | MJPEG 640x480 @ 30 (measured 17-20 fps indoors: auto exposure) | `/front_camera/image_raw` (+ `/compressed`), info `/camera/camera_info` |
| Bottom (Task 2) | MJPEG 1920x1080 @ 15 (the only size; YUYV at 1080p arrives green over USB 2.0) | `/bottom_camera/image_raw` (+ `/compressed`) |

usb_cam 0.8 opens the udev symlinks directly. It also keeps idle read-only handles on every
`/dev/video*`, so `fuser` lists a running usb_cam on both cameras; those handles do not block the
other camera. Both cameras, the GPS and the ESP32 share one USB 2.0 hub; both cameras streaming at
once works (2026-11-03: bottom 14 fps, front 17 fps; decoding the 1080p bottom stream takes about
half a Pi 5 core).

POR is finished. In Task 2 the front camera will feed YOLO object detection for collision avoidance
(model in training). Until that node exists, `sub_bringup.launch.py` still starts the front camera
only for `mode:=por`; add it to `mode:=task2` together with the YOLO node.

**Front camera orientation:** the image is rotated 90° (on the bench the ceiling is on the right
edge), and the camera has no V4L2 rotate/flip control. Either mount it upright or rotate in the YOLO
node right after decoding, before inference: `cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)`
(C++ `cv::rotate`), well under 1 ms at 640x480. Re-check the direction once it is mounted, and
prefer a `rotation` parameter (0/90/180/270) over hard-coding. The result is a 480x640 portrait
frame; boxes and left/right decisions are then in the rotated frame. Bags record the raw sideways
image, so apply the same rotation when building the training set. `front_camera_info.yaml` still
holds the old Logitech C270 intrinsics: recalibrate (and rotate fx/fy, cx/cy) before using anything
that needs them.
Camera details: [`horus_camera.md`](horus_camera.md).

## 3. Kill switch (ESP32 indicator, `indikator_banyu.ino`)

The new board has **no relay**. The physical kill switch cuts thruster power in hardware; the ESP32
senses that input on ADC pin 0 (below 1000 = emergency) and drives three lamps.

Serial protocol on `/dev/esp32_kill` (115200, native USB, opening the port does not reboot it):

| Direction | Line | Meaning |
|---|---|---|
| PC → board | `its0\n` | MANUAL lamp (operational_mode 1) |
| PC → board | `its1\n` | AUTO lamp (operational_mode 2) |
| PC → board | `its2\n` | EMERGENCY lamp (operational_mode 0) |
| board → PC | `itsN\r\n` every 500 ms | current state; `its2` also while the physical kill is active |

The board ignores `its0`/`its1` while the physical kill is active and goes back to its previous
state when it is released. The board numbers states differently from `operational_mode`
(issue #15: 0 kill, 1 manual, 2 autonomous); `banyu_teleop` maps them
(`banyu_teleop/include/banyu_teleop/esp32_indicator.hpp`) and resends the state every second.

Behaviour in `banyu_teleop` (`teleop_rcin.yaml`):

- SB Up / Mid / Down → AUTO / MANUAL / EMERGENCY lamp, as before (arm, manual, disarm).
- Board reports `its2` while SB is Mid or Up → **physical kill**: teleop publishes kill
  (`/banyu/hardware/kill_switch` = 0, mission stops), disarms the Pixhawk and **holds** the kill.
  Releasing the switch does not re-arm; move SB **Down** to acknowledge, then Mid/Up again.
- Board reports are published on `/banyu/hardware/kill_switch_board` (recorded in the bag).
- No report for `kill_report_timeout_s` (1.5 s) → error log "hardware kill not monitored".
- `use_hardware_kill: false` ignores the reports (land tests with the kill input unwired only).

With `--no-rc`, `esp32_trigger_node` does the same: `'0'/'1'/'2'` commands are sent as
`its2/its0/its1`, and a physical kill is held until a `'0'` command.

> On 2026-11-03 the board reported `its2` permanently and ignored `its0`/`its1`: the physical kill
> input reads as active (switch engaged or ADC line not wired). With that, teleop keeps the vehicle
> killed. Check the switch wiring before a run (`debug/kill_lamp.sh status`).

## 4. RTK GPS

Chain (Task 2 surface GPS leg):

```
base station --RTCM, tcp :2002--> str2str (rtk-rover.service) --> /dev/gps_rtk (u-blox)
/dev/gps_rtk --UBX NAV-PVT--> gps_to_pixhawk.py (run_gps_bridge.sh)
   --GPS_INPUT, udp 127.0.0.1:14553--> MAVProxy (run_mavproxy.sh) --> Pixhawk (GPS1_TYPE 14)
   --> MAVROS /mavros/global_position/raw/fix
```

- **RTK corrections:** the teammate script
  [`fix_rtk_rover.sh`](https://github.com/jausyan/RobotX/blob/main/script/fix_rtk_rover.sh)
  (vendored in `tools/scripts/rtk/`) installs `rtk-rover.service`: `str2str -in
  tcpcli://<base>:2002 -out serial://<u-blox>`, restarted on failure and started on boot. Run it once
  per computer, and again whenever the base IP changes, with `debug/rtk_setup.sh` (installs
  `rtklib` first; prompts for the base IP, default `192.168.0.1:2002`). `rtklib` is installed on the
  Pi; the service is not installed yet because no base station was reachable.
- **GPS bridge:** `tools/scripts/run_gps_bridge.sh` runs `gps_to_pixhawk.py` from the team repo
  `BanyubramantaITS/RTK-GPS-GNSS` (`~/RTK-GPS-GNSS`) with `--pixhawk-port udpout:127.0.0.1:14553`, so
  only MAVProxy opens the Pixhawk serial port. It uses `--no-anti-drift` (the bridge's filter freezes
  the position below 0.35 m/s / 1.6 m, which the AUV is during the whole GPS leg) and re-applies the
  receiver config on each start (`sea` dynamic model, 10° elevation mask, 5 Hz).
  Nothing is sent before the receiver has a fix.
- The u-blox is read by the bridge and written by `str2str` at the same time; that is fine (one
  reader, one writer). Do not start a second reader (e.g. `--check-gps-cfg`) while the bridge runs.
- `gps_to_pixhawk.py --check-gps-cfg` prints the wrong byte for the elevation mask (it always shows
  16°, the low byte of `fixedAltVar`); the dynamic model line is correct.

Check: `debug/rtk_status.sh` (service state, then fix type: 3 = 3D, 5 = RTK float, 6 = RTK fixed),
or `ros2 topic echo /mavros/gpsstatus/gps1/raw --once` with MAVROS running.

## 5. Bring-up order on the robot

```bash
cd ~/ros_ws
./src/BANYU_ROBOTX/tools/scripts/run_mavproxy.sh          # 1. Pixhawk link (outs 14550/14551/14552, in 14553)
./src/BANYU_ROBOTX/tools/scripts/run_gps_bridge.sh        # 2. GPS -> Pixhawk
./src/BANYU_ROBOTX/tools/scripts/launch_mavros.sh         # 3. MAVROS, leave running
./src/BANYU_ROBOTX/tools/scripts/debug/check_devices.sh   # 4. everything green?
./src/BANYU_ROBOTX/tools/scripts/debug/test_hardware.sh   # 5. optional bench test (stack stopped)
# 6. mission stack: tools/scripts/launch_task2.sh --real (on the Task 2 branch; not on this branch yet)
```

`rtk-rover.service` starts by itself on boot once installed.

## 6. Bench test after wiring

One command checks everything that is wired, with a PASS/FAIL line per check:

```bash
./src/BANYU_ROBOTX/tools/scripts/debug/test_hardware.sh            # all tests, interactive
./src/BANYU_ROBOTX/tools/scripts/debug/test_hardware.sh kill       # only the kill switch
./src/BANYU_ROBOTX/tools/scripts/debug/test_hardware.sh --no-prompt  # no questions (skips the physical kill steps)
```

Stop the mission stack first (teleop owns the ESP32, usb_cam the cameras); keep `run_mavproxy.sh`
(and `run_gps_bridge.sh` for the Pixhawk GPS check) running.

| Test | Checks |
|---|---|
| `devices` | the five udev names exist; serial ports not held by another process (MAVProxy may hold the Pixhawk) |
| `kill` | board reports; MANUAL / AUTO / EMERGENCY lamps follow `its0/its1/its2` (you confirm each lamp); you **press** the physical kill → board reports `its2` and ignores AUTO; you **release** it → board returns to AUTO |
| `cameras` | one MJPEG frame from each camera into `/tmp/banyu_hw_test/` (look at them: orientation, focus) |
| `gps` | `rtk-rover` service, then fix type, satellites, accuracy and RTK float/fixed from the receiver (needs sky view; skipped if `run_gps_bridge.sh` owns the receiver) |
| `pixhawk` | heartbeat (mode, armed) through MAVProxy port 14552, the GPS fix the Pixhawk gets from the bridge, and the TX16S SB/SC positions live for 10 s (move SB through all three positions) |

Until the kill switch is wired, `kill` fails at "physical kill released" (the board reads the
unwired input as emergency): expected.

## 7. Debug scripts (`tools/scripts/debug/`)

Stop the mission stack first: a serial port can only have one owner, and a camera only one stream.

| Script | What it does |
|---|---|
| `test_hardware.sh [tests] [--no-prompt]` | Bench test above |
| `check_devices.sh` | udev devices and who holds them, MAVProxy (with the GPS port), MAVROS, GPS bridge, rtk-rover, last kill-switch report |
| `front_camera.sh [--fps N] [--snapshot f.jpg]` | Front camera alone, same usb_cam settings as the mission (or one JPEG, no ROS) |
| `bottom_camera.sh [--fps N] [--snapshot f.jpg]` | Bottom camera alone, same options |
| `camera.sh <front\|bottom> ...` | The common script behind the two above |
| `kill_lamp.sh <manual\|auto\|emergency\|status>` | Set the ESP32 lamp or print its reports (`its2` while setting manual/auto = physical kill active) |
| `rtk_setup.sh` | One-time RTK rover setup (rtklib + teammate script → `rtk-rover.service`) |
| `rtk_status.sh [seconds]` | RTK corrections service and the receiver fix |

View a camera from the laptop on the same `ROS_DOMAIN_ID`: `ros2 run rqt_image_view rqt_image_view`.
Lower level tools: `tools/scripts/esp32_kill_cmd.py`, `tools/scripts/mock_esp32.py` (PTY mock of
the indicator board, `k` toggles the physical kill).
