# Competition Hardware Setup (Raspberry Pi 5)

The AUV computer is a Raspberry Pi 5 (Ubuntu 24.04, Jazzy, workspace `~/ros_ws`). This page lists
every USB device of the competition stack, how each one is wired into the software, the bring-up
order, and the debug scripts.

## 1. Devices

| Device | USB ID | Name (udev) | Used by |
|---|---|---|---|
| Pixhawk 1 (ArduSub) | `1209:5741` | `/dev/pixhawk` | `run_mavproxy.sh` (only process that opens it) |
| ESP32 kill-switch indicator (Espressif USB JTAG/serial) | `303a:1001` | `/dev/esp32_kill` | `banyu_teleop` (or `esp32_trigger_node` with `--no-rc`) |
| u-blox RTK GNSS receiver | `1546:01a9` | `/dev/gps_rtk` | Pixhawk reads it over its own UART (`GPS1_TYPE 1`); USB: `rtk-rover` service (writes RTCM) |
| Front camera "H264 USB Camera" | `32e4:9422` | `/dev/front_camera` | `front_camera` usb_cam node; Task 2 YOLO collision avoidance (planned) |
| Hailo-8 AI accelerator (PCIe) | – | `/dev/hailo0` | YOLO inference (section 8) |
| Bottom camera "2M WebCam" (Horus B) | `2ce3:3894` | `/dev/bottom_camera` | `bottom_camera` usb_cam node (Task 2) |

All configs use these names, so the `ttyACM*` / `video*` order after a replug does not matter.
The rules are in [`tools/udev/99-banyu.rules`](../tools/udev/99-banyu.rules); install them with
[`udev_rules.md`](udev_rules.md). The ESP32 rule matches the generic Espressif ID, so keep only one
ESP32 plugged in.

Check everything at once:

```bash
./src/BANYU_ROBOTX/tools/scripts/check/check_devices.sh
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
  (`/banyu/hardware/kill_switch` = 0, mission stops), disarms the Pixhawk and **holds** the kill
  while the switch is pressed. Teleop keeps sending the SB mode meanwhile (the board ignores it),
  so the report leaving `its2` shows the release. On release the vehicle **returns to the current
  SB position** (Mid/Up re-arms at once, thrusters live; a stopped mission is not resumed). If no
  RC with the SB channel arrived within `rc_timeout_s` (3 s), the kill stays held until SB
  **Down**, then Mid/Up. The Pixhawk may keep streaming the last channels after the transmitter
  is lost, so this check only catches RC that actually goes silent.
- Board reports are published on `/banyu/hardware/kill_switch_board` (recorded in the bag).
- No report for `kill_report_timeout_s` (1.5 s) → error log "hardware kill not monitored".
- `use_hardware_kill: false` ignores the reports (land tests with the kill input unwired only).

With `--no-rc`, `esp32_trigger_node` does the same: `'0'/'1'/'2'` commands are sent as
`its2/its0/its1`, and a physical kill is held until a `'0'` command. It also sets the Pixhawk
(`fcu_mode_sync`): `'1'` → `MANUAL` and armed, `'0'` or a physical kill → `MANUAL` and disarmed,
`'2'` → left to the mission manager. Requests are retried until `/mavros/state` confirms them
(error log after `fcu_sync_timeout_s`, 10 s).

> On 2026-11-03 the board reported `its2` permanently and ignored `its0`/`its1`: the physical kill
> input reads as active (switch engaged or ADC line not wired). With that, teleop keeps the vehicle
> killed. Check the switch wiring before a run (`debug/kill_lamp.sh status`).

## 4. RTK GPS

Chain (Task 2 surface GPS leg):

```
base station --RTCM, tcp :2002--> str2str (rtk-rover.service) --USB--> /dev/gps_rtk (ZED-F9P)
/dev/gps_rtk --USB--> run_gps_bridge.sh (gps_forwarder.py) --GPS_INPUT, udp 14553--> MAVProxy
   --> Pixhawk (GPS1_TYPE 14) --> MAVROS /mavros/global_position/raw/fix, /mavros/gpsstatus/gps1/raw
```

- **Pixhawk GPS params** (checked by `check/check_pixhawk_params.py`): `GPS1_TYPE 1`,
  `SERIAL3_PROTOCOL 5` at `SERIAL3_BAUD 230`, `GPS_AUTO_CONFIG 1` (ArduPilot configures the
  receiver on each boot), `GPS_NAVFILTER 5` (sea dynamic model), `GPS_MIN_ELEV 10`, `GPS1_RATE_MS 200`
  (5 Hz). The Pixhawk reports RTK float/fixed itself (fix type 5/6).
- **RTK corrections:** the teammate script
  [`fix_rtk_rover.sh`](https://github.com/jausyan/RobotX/blob/main/script/fix_rtk_rover.sh)
  (vendored in `tools/scripts/setup/`) installs `rtk-rover.service`: `str2str -in
  tcpcli://<base>:2002 -out serial://<u-blox USB>`, restarted on failure and started on boot. The F9P
  accepts RTCM on any port, so corrections go in over USB while the Pixhawk reads UART1. Run it once
  per computer, and again whenever the base IP changes, with `setup/rtk_setup.sh` (installs
  `rtklib` first; prompts for the base IP, default `192.168.0.1:2002`).
- **Current setup (2026-11-06): `GPS1_TYPE 14`.** The Pixhawk's GPS is `run/run_gps_bridge.sh`, started on its
  own after `run_mavproxy.sh` (like MAVProxy and MAVROS; `launch_task2.sh --real` refuses to start without it).
  It runs `run/gps_forwarder.py`, which replaced the team's `gps_to_pixhawk.py` on 2026-11-06: that one
  reported the NMEA GGA satellite count (capped at 12; NAV-PVT showed 32) and turned on the receiver's
  static hold (30 cm/s, 10 m), which freezes the position on the slow surface leg. The forwarder uses
  NAV-PVT only and turns static hold off.
  The wired `GPS1_TYPE 1` chain and params in this section are the alternative: then do not run the bridge (the
  Pixhawk ignores its `GPS_INPUT` and its receiver setup fights ArduPilot's auto-config) and launch with
  `launch_task2.sh --allow-no-gps-bridge`.
- No satellites (`satellites_visible 0` on the Pixhawk and the receiver) means the antenna: sky
  view, SMA cable. A cold start takes 1–2 min outdoors.

Check: `check/gps_receiver.py` (live receiver view: satellites, signal C/N0, AGC, RTK state), or
`check/rtk_status.sh` (service state, then fix type: 3 = 3D, 5 = RTK float, 6 = RTK fixed),
or `ros2 topic echo /mavros/gpsstatus/gps1/raw --once` with MAVROS running.

## 5. Bring-up order on the robot

Every terminal runs on the UUV domain: `export ROS_DOMAIN_ID=40` (the DDS Router forwards `/system/*` from it to
the GCS). `launch_task2.sh --real` refuses another domain (`--allow-other-domain` for a bench test without the GCS)
and a MAVROS started on a different domain from the stack.

```bash
cd ~/ros_ws
export ROS_DOMAIN_ID=40
./src/BANYU_ROBOTX/tools/scripts/run/tmux_panes.sh            # 0. optional: tmux window, 2x4 panes
./src/BANYU_ROBOTX/tools/scripts/run/run_mavproxy.sh          # 1. Pixhawk link (outs 14550/14551/14552)
./src/BANYU_ROBOTX/tools/scripts/run/launch_mavros.sh         # 2. MAVROS, leave running
./src/BANYU_ROBOTX/tools/scripts/check/check_devices.sh   # 3. everything green?
./src/BANYU_ROBOTX/tools/scripts/check/test_hardware.sh   # 4. optional bench test (stack stopped)
# 5. mission stack: launch_task2.sh --real (on the Task 2 branch; belongs in tools/scripts/run/)
```

`rtk-rover.service` starts by itself on boot once installed. The Pixhawk reads the GPS itself
(section 4); no bridge process is needed.

## 6. Bench test after wiring

One command checks everything that is wired, with a PASS/FAIL line per check:

```bash
./src/BANYU_ROBOTX/tools/scripts/check/test_hardware.sh            # all tests, interactive
./src/BANYU_ROBOTX/tools/scripts/check/test_hardware.sh kill       # only the kill switch
./src/BANYU_ROBOTX/tools/scripts/check/test_hardware.sh --no-prompt  # no questions (skips the physical kill steps)
```

Stop the mission stack first (teleop owns the ESP32, usb_cam the cameras); keep `run_mavproxy.sh`
running.

| Test | Checks |
|---|---|
| `devices` | the five udev names exist; serial ports not held by another process (MAVProxy may hold the Pixhawk) |
| `kill` | board reports; MANUAL / AUTO / EMERGENCY lamps follow `its0/its1/its2` (you confirm each lamp); you **press** the physical kill → board reports `its2` and ignores AUTO; you **release** it → board returns to AUTO |
| `cameras` | one MJPEG frame from each camera into `/tmp/banyu_hw_test/` (look at them: orientation, focus) |
| `gps` | `rtk-rover` service, then fix type, satellites, accuracy and RTK float/fixed from the receiver over USB (needs sky view) |
| `pixhawk` | heartbeat (mode, armed) through MAVProxy port 14552, the GPS fix the Pixhawk reads from the F9P, and the TX16S SB/SC positions live for 10 s (move SB through all three positions) |

Until the kill switch is wired, `kill` fails at "physical kill released" (the board reads the
unwired input as emergency): expected.

## 7. Scripts (`tools/scripts/`)

Grouped by function; the full index, and where the Task 2 branch's scripts go when it is merged, is
[`tools/scripts/README.md`](../tools/scripts/README.md). Every script answers `--help` without doing
anything; `setup/` scripts change the system as soon as they run.
Stop the mission stack before `check/` and `debug/`: a serial port can only have one owner, and a
camera only one stream.

| Script | What it does |
|---|---|
| `run/run_mavproxy.sh` | Pixhawk link: outs 14550 (QGC) / 14551 (MAVROS) / 14552, in 14553 (fallback GPS bridge) |
| `run/launch_mavros.sh` | MAVROS on its own, leave running |
| `run/tmux_panes.sh [-r ROWS] [-c COLS]` | tmux window split into an even grid (default 2 rows x 4 columns), all in the current directory |
| `run/run_gps_bridge.sh` | GPS forwarder (`GPS1_TYPE 14`, the current setup): u-blox USB → Pixhawk `GPS_INPUT`; start on its own after `run_mavproxy.sh` |
| `check/test_hardware.sh [tests] [--no-prompt]` | Bench test (section 6) |
| `check/check_devices.sh` | udev devices and who holds them, MAVProxy, MAVROS, rtk-rover, a stray GPS bridge, last kill-switch report |
| `check/gps_receiver.py [--seconds N]` | Live F9P health over USB without the bridge: fix, RTK, satellites and C/N0 per constellation, antenna/AGC/jamming (read-only polls) |
| `check/rtk_status.sh [seconds]` | RTK corrections service, the Pixhawk's GPS fix and the receiver fix over USB |
| `check/check_pixhawk_params.py`, `check/check_thrust_limit.py` | Pixhawk parameters against `banyu_defaults.parm`, thruster PWM cap |
| `debug/front_camera.sh [--fps N] [--snapshot f.jpg]` | Front camera alone, same usb_cam settings as the mission (or one JPEG, no ROS) |
| `debug/bottom_camera.sh [--fps N] [--snapshot f.jpg]` | Bottom camera alone, same options (`debug/camera.sh` is the common script) |
| `debug/kill_lamp.sh <manual\|auto\|emergency\|status>` | Set the ESP32 lamp or print its reports (`its2` while setting manual/auto = physical kill active) |
| `debug/esp32_kill_cmd.py`, `debug/mock_esp32.py` | Lower level ESP32 tool; PTY mock of the indicator board (`k` toggles the physical kill) |
| `debug/reset_pixhawk_usb.sh` | Re-enumerate a hung Pixhawk USB link |
| `setup/rtk_setup.sh` | One-time RTK rover setup (rtklib + teammate script `setup/fix_rtk_rover.sh` → `rtk-rover.service`) |
| `setup/yolo_setup.sh` | One-time YOLO setup for the Hailo-8 (section 8) |
| `check/yolo_test.sh [--hef f] [--labels f] [--rotation deg] [--save f.jpg]` | YOLO on the Hailo-8 against the live front camera: detections, FPS, annotated frame |
| `move/dive.sh`, `forward.sh`, `backward.sh`, `sway.sh`, `yaw.sh`, `waypoint.sh`, `where.sh` | Movement tests in the water (arms the vehicle; teleop and mission stack stopped): [`movement_tests.md`](movement_tests.md) |

View a camera from the laptop on the same `ROS_DOMAIN_ID`: `ros2 run rqt_image_view rqt_image_view`.

## 8. YOLO on the Hailo-8

The Pi has a **Hailo-8** AI accelerator (26 TOPS, PCIe, `/dev/hailo0`). YOLO runs on it, not on the
Pi CPU. State on 2026-11-03:

| Part | Version / place |
|---|---|
| HailoRT runtime + `hailortcli`, `hailort.service` | 4.24.0 (apt `hailort`) |
| PCIe driver (DKMS) and chip firmware | 4.24.0 (apt `hailort-pcie-driver`); all three must be the same version |
| Python | `~/venv-yolo`: `--system-site-packages` (so `rclpy`, `cv_bridge`, system numpy 1.26 and OpenCV 4.6 work) + `hailort` 4.24.0 wheel, numpy kept < 2 |
| Test models | `~/yolo_models/`: `yolov8s`, `yolov11n`, `yolov11s`, `yolo26n` `.hef` (Hailo Model Zoo v2.18, COCO) |

`~/venv-ardupilot` also has the HailoRT wheel, but with numpy 2, which breaks `cv_bridge`: use
`~/venv-yolo` for anything YOLO + ROS. `setup/yolo_setup.sh` rebuilds all of this (it needs the
runtime/driver debs installed and the wheel in `~/Downloads`).

Measured 2026-11-03 (640x640 input; throughput from `hailortcli run`, latency from
`check/yolo_test.sh` on the live, rotated front camera, ~17-18 FPS end to end = camera limit):

| Model | Hailo-8 throughput | Latency per frame | Output | Works with `yolo_test.sh` |
|---|---|---|---|---|
| YOLOv8s | ~377 FPS | 10.4 ms | on-chip NMS | yes |
| YOLOv11n | ~100 FPS | – | on-chip NMS | yes |
| YOLOv11s | ~49 FPS | 20.9 ms | on-chip NMS | yes |
| YOLO26n | ~93 FPS | – | raw heads (boxes + class scores at strides 8/16/32) | no: needs CPU decoding (NMS-free, so only threshold + top-k) |

All of them are fast enough for a ~20 fps camera. YOLOv11 runs in several contexts on the Hailo-8,
which is why it is slower than YOLOv8s here; pick the architecture for the team model with that in
mind. CPU and USB budget are not a concern for YOLO.

**Team model (in training) → HEF.** The Hailo-8 runs only `.hef` files compiled by the Hailo
Dataflow Compiler (DFC). The DFC runs on an x86_64 Ubuntu machine (or Hailo's Docker image), not on
the Pi:

1. Train with Ultralytics as usual (`best.pt`); keep the input 640x640.
2. Export: `yolo export model=best.pt format=onnx imgsz=640 opset=11`.
3. Compile with the Hailo Model Zoo flow for the same architecture, e.g.
   `hailomz compile yolov8s --ckpt best.onnx --hw-arch hailo8 --calib-path calib/ --classes <N>`.
   Use the Model Zoo / DFC release that matches HailoRT 4.24 (Model Zoo v2.18 and the DFC version in
   its release notes), or the HEF may not load. `calib/` = a few hundred front-camera frames from the
   bags, **rotated the same way as at inference**.
4. Copy the `.hef` and a labels file (one class per line) to `~/yolo_models/` and check it:
   `check/yolo_test.sh --hef ~/yolo_models/team.hef --labels ~/yolo_models/team.txt --save /tmp/y.jpg`.

The input is RGB, letterboxed with grey 114; the HEF output is per class
`[ymin, xmin, ymax, xmax, score]` normalised to the letterboxed input (`check/yolo_test.py` shows
the decoding). The YOLO ROS node does not exist yet; build it on the same pattern (rotate →
letterbox → infer → decode) and give it a `rotation` parameter.
