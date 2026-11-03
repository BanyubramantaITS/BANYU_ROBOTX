# Horus B Camera Guide (2M WebCam)

This document provides setup instructions, driver requirements, hardware constraints, and troubleshooting tips for operating the **Horus B Camera** (`2M WebCam`) on the BANYU ROV platform with ROS 2 Jazzy. The Horus B is the **bottom camera** (Task 2); the front camera is an "H264 USB Camera" (`32e4:9422`, Task 2 YOLO collision avoidance). Both are set up in [`hardware_setup.md`](hardware_setup.md).

`sub_bringup.launch.py` already runs both cameras with `usb_cam`; their settings are in `banyu_bringup/config/hardware_params.yaml` (Horus: `mjpeg2rgb`, 1920x1080, 15 fps). The standalone commands below are for bench testing.

---

## 1. Hardware Specifications & Constraints

- **Device Name:** 2M WebCam (Horus B)
- **USB Vendor / Product ID:** `2ce3:3894`
- **Linux Driver:** `uvcvideo`
- **Supported Resolution:** **1920x1080 only** (the sensor firmware does not support hardware 720p or 480p on the primary capture stream).
- **Supported Formats:**
  - `MJPG` (Motion-JPEG, compressed): 30, 25, 20, 15, 10, 5 FPS.
  - `YUYV` (YUYV 4:2:2, raw uncompressed): 30, 25, 20, 15, 10, 5 FPS.

### Critical Bandwidth Gotcha: Solid Green Screen
- Uncompressed 1080p `YUYV` @ 30 FPS requires **~124 MB/s** (~1 Gbps) of bandwidth.
- Because this camera is connected over a **USB 2.0** bus (maximum practical bandwidth ~35–40 MB/s), the USB controller cannot physically carry raw 1080p frames. The `uvcvideo` kernel driver drops the packets, delivering zeroed buffers (`0x00`).
- In YUV color space, $(Y=0, U=0, V=0)$ converts in RGB to pure bright green `(B=0, G=154, R=0)`.
- **Conclusion:** **You must use `MJPG` (Motion-JPEG)** for 1080p streaming.

---

## 2. ROS 2 Driver Requirement: `usb_cam`

Do **not** use `v4l2_camera` with this camera:
- `v4l2_camera` does not include an onboard MJPEG decompressor and crashes with `cv_bridge::Exception: Unrecognized image encoding []` when requested with `MJPG`.
- Use **`usb_cam`** (`ros-jazzy-usb-cam`), which has built-in hardware/FFmpeg MJPEG decompression via `pixel_format:="mjpeg2rgb"`.

Install `usb_cam` if not already installed:
```bash
sudo apt-get update && sudo apt-get install -y ros-jazzy-usb-cam
```

---

## 3. Running the Camera

### A. Standalone Run
To run the Horus camera standalone and publish directly to `/bottom_camera/image_raw`:

```bash
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:=/dev/bottom_camera \
  -p pixel_format:="mjpeg2rgb" \
  -p image_width:=1920 \
  -p image_height:=1080 \
  -p framerate:=30.0 \
  -r image_raw:=/bottom_camera/image_raw
```

*(The udev symlink works directly with usb_cam 0.8.)*

### B. Reducing CPU Usage
Decompressing 1080p MJPEG at 30 FPS on smaller companion computers (e.g., Raspberry Pi or Jetson) can consume significant CPU. You can drop the capture framerate to **15.0 FPS** (or 10.0 FPS) without losing optical clarity:

```bash
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:=/dev/bottom_camera \
  -p pixel_format:="mjpeg2rgb" \
  -p image_width:=1920 \
  -p image_height:=1080 \
  -p framerate:=15.0 \
  -r image_raw:=/bottom_camera/image_raw
```

---

## 4. Persistent Device Naming (`udev` Rule)

The camera is `/dev/bottom_camera` (rule in `tools/udev/99-banyu.rules`, see `udev_rules.md`).
usb_cam 0.8 opens the symlink directly, so `hardware_params.yaml` uses it as `video_device`.
Standalone run: `tools/scripts/debug/bottom_camera.sh` (or `--snapshot frame.jpg`).

## 5. Verification & Visualization

1. **Check publishing frequency (FPS):**
   ```bash
   ros2 topic hz /bottom_camera/image_raw
   ```

2. **Inspect live stream visually:**
   ```bash
   ros2 run rqt_image_view rqt_image_view /bottom_camera/image_raw
   ```

---

## 6. Troubleshooting

| Symptom | Cause | Solution |
| :--- | :--- | :--- |
| **`Couldn't parse remap rule ... Expected lexeme type (19)`** | Used `--remap video_device:=...` instead of `-p` | `video_device` is a parameter. Use `-p video_device:="..."`. |
| **`Device or resource busy (16)`** | Another app (GNOME Snapshot, Cheese, browser, rqt) is holding `/dev/videoX` | Close any other camera apps or kill the conflicting PID. |
| **Blank / Solid green image in rqt** | `YUYV` raw format chosen at 1080p over USB 2.0 (packets dropped, buffer all `0x00`) | Switch to `pixel_format:="mjpeg2rgb"` using `usb_cam`. |
| **`Device specified is not available ... /dev/../../video2`** | `usb_cam` cannot parse relative symlink paths from `/dev/v4l/by-id/` | usb_cam prefixes the link target with `/dev/`, so `../../videoX` breaks; use the udev name `/dev/bottom_camera` (target `videoX`, works). |
| **Image visible in `rqt` but `ros2 topic list` doesn't show it** | Network IP changed (DHCP renewal or Wi-Fi reconnect) while nodes were running | Add `export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` to `~/.bashrc` and restart the nodes. |
| **`ros2 topic hz` reports not published yet** | Topic remapping mismatch (e.g. checking `/image_raw` when remapped to `/bottom_camera/image_raw`) | Ensure you query the remapped topic name `/bottom_camera/image_raw`. |
