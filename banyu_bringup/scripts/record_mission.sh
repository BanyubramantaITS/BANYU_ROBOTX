#!/usr/bin/env bash
#
# Record a mission run to a rosbag so it can be inspected after the AUV
# surfaces and the link comes back.
#
# Usage:
#   record_mission.sh [label] [--no-images] [--sim-time]
#
#   label        tag added to the run folder name (e.g. por, task2). Default: run
#   --no-images  skip compressed camera images (much smaller bags)
#   --sim-time   stamp messages with /clock (simulation only)
#
# Environment:
#   BANYU_BAG_DIR       root folder for runs (default: ~/banyu_bags)
#   BANYU_BAG_MIN_FREE  stop recording when free disk drops below this many MB (default: 2048)
#   BANYU_BAG_IMAGES    False to skip images, same as --no-images (default: True)
#   BANYU_BAG_LABEL     overrides the label argument (launch_por.sh sets it to the profile)
#   BANYU_BAG_PARAMS    parameter YAML to save next to the bag (sub_bringup sets it)
#   BANYU_BAG_REPLAY_TRACK  replay track .csv flown in this run, saved next to it (launch_por.sh)
#
# Each run lands in $BANYU_BAG_DIR/<YYYYmmdd_HHMMSS>_<label>/ with:
#   bag/          MCAP bag, split every 60 s so a power cut only loses the last chunk
#   run_info.txt  host, git commit and recorder settings
#   params/       copy of the parameter YAML the run used
#
# Stop with Ctrl-C / SIGINT / SIGTERM; the bag is closed cleanly. If the vehicle lost
# power mid-run, make the bag readable afterwards with:  recover_bag.sh <run>/bag
set -uo pipefail

LABEL="run"
RECORD_IMAGES=1
case "${BANYU_BAG_IMAGES:-True}" in
  False|false|0) RECORD_IMAGES=0 ;;
esac
SIM_TIME_ARGS=()
for arg in "$@"; do
  case "$arg" in
    --no-images) RECORD_IMAGES=0 ;;
    --sim-time) SIM_TIME_ARGS=(--use-sim-time) ;;
    -h|--help) sed -n '2,28p' "$0"; exit 0 ;;
    -*) echo "Unknown option: $arg" >&2; exit 1 ;;
    *) LABEL="$arg" ;;
  esac
done

LABEL="${BANYU_BAG_LABEL:-$LABEL}"
LABEL="${LABEL//[^A-Za-z0-9_.-]/_}"

BAG_ROOT="${BANYU_BAG_DIR:-$HOME/banyu_bags}"
MIN_FREE_MB="${BANYU_BAG_MIN_FREE:-2048}"
RUN_DIR="$BAG_ROOT/$(date +%Y%m%d_%H%M%S)_${LABEL}"

mkdir -p "$RUN_DIR"

free_mb() { df -Pm "$BAG_ROOT" | awk 'NR==2 {print $4}'; }

if [ "$(free_mb)" -lt "$MIN_FREE_MB" ]; then
  echo "[record_mission] only $(free_mb) MB free in $BAG_ROOT (< ${MIN_FREE_MB} MB), not recording" >&2
  exit 1
fi

# Raw images are far too large to keep for a whole dive; the compressed stream is enough
# to review what the detectors saw. Other transports are skipped on every image topic:
# subscribing to compressedDepth makes it try (and fail, logging an error per frame) to
# encode colour/mask images. Unit-test topics never matter on the vehicle.
EXCLUDE='.*/image_raw$|/banyu/detection/debug/[^/]+$|.*/(theora|zstd|compressedDepth|ffmpeg)(/.*)?$|/test.*'
if [ "$RECORD_IMAGES" -eq 0 ]; then
  EXCLUDE="$EXCLUDE|.*/compressed$"
fi

SCRIPT_DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
{
  echo "started:   $(date --iso-8601=seconds)"
  echo "host:      $(hostname)"
  echo "label:     $LABEL"
  echo "images:    $([ "$RECORD_IMAGES" -eq 1 ] && echo compressed || echo none)"
  echo "exclude:   $EXCLUDE"
  echo "git:       $(git -C "$SCRIPT_DIR" describe --always --dirty 2>/dev/null || echo unknown)"
  echo "free_mb:   $(free_mb)"
  echo "params:    ${BANYU_BAG_PARAMS:-none}"
} > "$RUN_DIR/run_info.txt"

if [ -f "${BANYU_BAG_PARAMS:-}" ]; then
  mkdir -p "$RUN_DIR/params"
  cp "$BANYU_BAG_PARAMS" "$RUN_DIR/params/"
fi
if [ -f "${BANYU_BAG_REPLAY_TRACK:-}" ]; then
  mkdir -p "$RUN_DIR/params"
  cp "$BANYU_BAG_REPLAY_TRACK" "$RUN_DIR/params/"
fi

echo "[record_mission] recording to $RUN_DIR"

# --include-hidden-topics picks up action feedback/status (/banyu/actions/*/_action/*).
# A small cache and 60 s splits keep most of the run on disk if power is cut.
ros2 bag record \
  --all-topics \
  --include-hidden-topics \
  --exclude-regex "$EXCLUDE" \
  --storage mcap \
  --storage-preset-profile zstd_fast \
  --max-bag-duration 60 \
  --max-cache-size 20971520 \
  --disable-keyboard-controls \
  --node-name banyu_mission_recorder \
  "${SIM_TIME_ARGS[@]}" \
  --output "$RUN_DIR/bag" &
BAG_PID=$!

stop_recording() {
  # Background jobs of a non-interactive shell ignore SIGINT; the recorder also closes
  # the bag cleanly on SIGTERM.
  kill -TERM "$BAG_PID" 2>/dev/null
  wait "$BAG_PID"
  echo "stopped:   $(date --iso-8601=seconds) ($1)" >> "$RUN_DIR/run_info.txt"
  echo "[record_mission] bag closed: $RUN_DIR ($1)"
  exit 0
}
trap 'stop_recording signal' INT TERM

# Watch disk space so a long run never fills the disk the rest of the stack writes to.
while kill -0 "$BAG_PID" 2>/dev/null; do
  if [ "$(free_mb)" -lt "$MIN_FREE_MB" ]; then
    echo "[record_mission] free disk below ${MIN_FREE_MB} MB, stopping recording" >&2
    stop_recording low_disk
  fi
  sleep 5 &
  wait $!
done

wait "$BAG_PID"
echo "stopped:   $(date --iso-8601=seconds) (recorder exited $?)" >> "$RUN_DIR/run_info.txt"
