#!/bin/bash
# Run one AUV camera on its own for debugging: the same usb_cam node and hardware_params.yaml
# settings as sub_bringup.launch.py, or a single JPEG grabbed straight from V4L2.
set -e

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
WS_DIR="$(cd "${REPO_DIR}/../.." && pwd)"

print_usage() {
  cat << 'USAGE'
Usage: tools/scripts/debug/camera.sh <front|bottom> [OPTIONS]

  front   /dev/front_camera  "H264 USB Camera" -> /front_camera/image_raw (640x480 MJPEG)
  bottom  /dev/bottom_camera "2M WebCam"        -> /bottom_camera/image_raw (1920x1080 MJPEG)

Options:
  --fps N            Override the framerate from hardware_params.yaml.
  --snapshot FILE    Save one MJPEG frame to FILE with v4l2-ctl (no ROS) and exit.
  --dry-run          Print the command only.
  -h, --help         Show this help.

View from the laptop (same ROS_DOMAIN_ID): ros2 run rqt_image_view rqt_image_view
Check the rate: ros2 topic hz /front_camera/image_raw
The camera must not be streaming elsewhere (stop the mission stack first), or capture fails
with "Device or resource busy".
USAGE
}

CAM="${1:-}"
case "$CAM" in
  front) WIDTH=640; HEIGHT=480; INFO_TOPIC=/camera/camera_info ;;
  bottom) WIDTH=1920; HEIGHT=1080; INFO_TOPIC=/bottom_camera/camera_info ;;
  -h|--help) print_usage; exit 0 ;;
  *) print_usage; exit 1 ;;
esac
shift
FPS=""; SNAPSHOT=""; DRY_RUN=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --fps) FPS="$2"; shift 2 ;;
    --snapshot) SNAPSHOT="$2"; shift 2 ;;
    --dry-run) DRY_RUN=true; shift ;;
    -h|--help) print_usage; exit 0 ;;
    *) echo "Error: unknown option $1" >&2; exit 1 ;;
  esac
done

DEVICE="/dev/${CAM}_camera"
if [ ! -e "$DEVICE" ]; then
  echo "Error: $DEVICE not found (camera plugged in? udev rule tools/udev/99-banyu.rules installed?)" >&2
  [ "$DRY_RUN" = true ] || exit 1
fi
# No fuser check: usb_cam keeps idle handles on every /dev/video*, which do not block capture.
# A camera that is really streaming elsewhere fails with "Device or resource busy".

if [ -n "$SNAPSHOT" ]; then
  CMD=(v4l2-ctl -d "$DEVICE" --set-fmt-video=width=${WIDTH},height=${HEIGHT},pixelformat=MJPG
       --stream-mmap --stream-skip=10 --stream-count=1 --stream-to="$SNAPSHOT")
  echo "${CMD[*]}"
  [ "$DRY_RUN" = true ] && exit 0
  "${CMD[@]}"
  echo "Saved $(readlink -f "$SNAPSHOT") ($(stat -c %s "$SNAPSHOT") bytes)"
  exit 0
fi

[ -f /opt/ros/jazzy/setup.bash ] && source /opt/ros/jazzy/setup.bash
[ -f "${WS_DIR}/install/setup.bash" ] && source "${WS_DIR}/install/setup.bash"
PARAMS="$(ros2 pkg prefix banyu_bringup 2>/dev/null)/share/banyu_bringup/config/hardware_params.yaml"
[ -f "$PARAMS" ] || PARAMS="${REPO_DIR}/banyu_bringup/config/hardware_params.yaml"

# Node name = YAML section, so the device, format and size apply unchanged; remaps as in
# sub_bringup.launch.py (usb_cam does not carry image_raw over to the compressed subtopic).
CMD=(ros2 run usb_cam usb_cam_node_exe --ros-args -r __node:=${CAM}_camera
     --params-file "$PARAMS"
     -r image_raw:=/${CAM}_camera/image_raw
     -r image_raw/compressed:=/${CAM}_camera/image_raw/compressed
     -r camera_info:=${INFO_TOPIC})
[ -n "$FPS" ] && CMD+=(-p framerate:=${FPS})
echo "${CMD[*]}"
[ "$DRY_RUN" = true ] && exit 0
exec "${CMD[@]}"
