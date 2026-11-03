#!/bin/bash
# One-time YOLO setup on the AUV Pi for the Hailo-8 accelerator (idempotent):
#   1. checks HailoRT runtime, PCIe driver and chip firmware are installed and the same version
#   2. creates ~/venv-yolo with --system-site-packages (rclpy, cv_bridge, system numpy/OpenCV)
#      and installs the matching HailoRT Python wheel, keeping numpy < 2 for cv_bridge
#   3. downloads Hailo Model Zoo YOLO HEFs (COCO) into ~/yolo_models for testing
# Test afterwards: tools/scripts/check/yolo_test.sh
set -e

VENV="${BANYU_YOLO_VENV:-$HOME/venv-yolo}"
MODELS="${BANYU_YOLO_MODELS:-$HOME/yolo_models}"
WHEEL_DIR="${BANYU_HAILO_WHEEL_DIR:-$HOME/Downloads}"
ZOO_VERSION="2.18.0"   # Hailo Model Zoo release that pairs with HailoRT 4.24
ZOO_MODELS="yolov8s yolov11n yolov11s yolo26n"

if ! command -v hailortcli > /dev/null; then
  echo "Error: HailoRT is not installed. Install the runtime and PCIe driver (Hailo developer zone," >&2
  echo "  same version for both), e.g.: sudo apt install ./hailort_4.24.0_arm64.deb ./hailort-pcie-driver_4.24.0_all.deb" >&2
  echo "  then reboot so the driver loads the firmware." >&2
  exit 1
fi
RT_VERSION="$(hailortcli --version | grep -oE '[0-9]+\.[0-9]+\.[0-9]+' | head -1)"
DRV_VERSION="$(modinfo -F version hailo_pci 2>/dev/null || echo none)"
FW_VERSION="$(hailortcli fw-control identify 2>/dev/null | grep -oP 'Firmware Version: \K[0-9.]+' || echo none)"
echo "HailoRT ${RT_VERSION}, driver ${DRV_VERSION}, firmware ${FW_VERSION}"
if [ "$RT_VERSION" != "$DRV_VERSION" ] || [ "$RT_VERSION" != "$FW_VERSION" ]; then
  echo "Error: versions differ (or the chip did not answer); reinstall runtime and driver at one version." >&2
  exit 1
fi

if [ ! -x "${VENV}/bin/python3" ]; then
  python3 -m venv --system-site-packages "$VENV"
  "${VENV}/bin/pip" install -q --upgrade pip
fi
if ! "${VENV}/bin/python3" -c "import hailo_platform" 2> /dev/null; then
  WHEEL="$(ls "${WHEEL_DIR}"/hailort-"${RT_VERSION}"-cp312-*aarch64.whl 2> /dev/null | head -1)"
  if [ -z "$WHEEL" ]; then
    echo "Error: no hailort-${RT_VERSION}-cp312-*aarch64.whl in ${WHEEL_DIR} (Hailo developer zone)." >&2
    exit 1
  fi
  echo "numpy<2" > /tmp/banyu-yolo-constraints.txt
  "${VENV}/bin/pip" install -q -c /tmp/banyu-yolo-constraints.txt "$WHEEL"
fi
"${VENV}/bin/python3" -c "import hailo_platform, numpy, cv2; print('venv OK: hailort', hailo_platform.__version__, 'numpy', numpy.__version__, 'cv2', cv2.__version__)"

mkdir -p "$MODELS"
for m in $ZOO_MODELS; do
  [ -s "${MODELS}/${m}.hef" ] && continue
  echo "downloading ${m}.hef (Model Zoo ${ZOO_VERSION}, Hailo-8)"
  curl -sfL -o "${MODELS}/${m}.hef" \
    "https://hailo-model-zoo.s3.eu-west-2.amazonaws.com/ModelZoo/Compiled/v${ZOO_VERSION}/hailo8/${m}.hef"
done
ls -l "$MODELS"
echo "Done. Test: tools/scripts/check/yolo_test.sh --save /tmp/yolo.jpg"
