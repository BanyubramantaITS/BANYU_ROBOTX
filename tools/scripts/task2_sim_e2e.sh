#!/bin/bash
# SIMULATION ONLY: one end-to-end Task 2 run in the sim, driven by tools/sos_integration/mock_gcs.py.
#   tools/scripts/task2_sim_e2e.sh --out <dir> [--tier core|advanced|disruptive] [--timeout <s>] [--no-gate]
#                                  [--gcs-arg <arg>]... [--launch-arg <arg>]... [--gui]
# Starts the sim headless (up to 3 tries: Gazebo sometimes never finishes loading), calls prepare_autonomy,
# sends the course and run-start from the mock GCS, records rings with their Gazebo ground truth, writes
# <dir>/{progress.txt,sim.log,mock_gcs.jsonl,rings.jsonl} and stops everything it started.
# (no set -u: the ROS setup scripts use unset variables)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
WS_DIR="$(cd "${REPO_DIR}/../.." && pwd)"
OUT=""; MODE="--headless"; TIER="core"; TIMEOUT=900; GATE=(--gps-gate); GCS_ARGS=(); LAUNCH_ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --out) OUT="$2"; shift 2 ;;
    --tier) TIER="$2"; shift 2 ;;
    --timeout) TIMEOUT="$2"; shift 2 ;;
    --no-gate) GATE=(); shift ;;
    --gui) MODE="--gui"; shift ;;
    --gcs-arg) GCS_ARGS+=("$2"); shift 2 ;;
    --launch-arg) LAUNCH_ARGS+=("$2"); shift 2 ;;
    -h|--help) sed -n 2,8p "$0"; exit 0 ;;
    *) echo "unknown argument $1" >&2; exit 1 ;;
  esac
done
[ -n "$OUT" ] || { echo "--out is required" >&2; exit 1; }
mkdir -p "$OUT"
source /opt/ros/jazzy/setup.bash && source "${WS_DIR}/install/setup.bash"
export ROS_DOMAIN_ID=40
echo "start $(date +%T) tier=$TIER" > "$OUT/progress.txt"
UP=0
for ATTEMPT in 1 2 3; do
  "${SCRIPT_DIR}/launch_task2.sh" --sim "$MODE" --no-clean --tier "$TIER" "${GATE[@]}" "${LAUNCH_ARGS[@]}" \
    > "$OUT/sim.log" 2>&1 &
  for _ in $(seq 1 30); do
    sleep 6
    if timeout 5 ros2 topic echo --once /banyu/vehicle/water_depth 2>/dev/null | grep -q 'valid: true'; then UP=1; break; fi
  done
  [ "$UP" = 1 ] && break
  echo "attempt $ATTEMPT: sim did not come up" >> "$OUT/progress.txt"
  "${SCRIPT_DIR}/task2_sim_cleanup.sh"; sleep 10
done
echo "up=$UP" >> "$OUT/progress.txt"
if [ "$UP" = 1 ]; then
  sleep 45   # EKF and datum settle
  python3 "${REPO_DIR}/tools/sos_integration/ring_recorder.py" "$OUT/rings.jsonl" "$TIMEOUT" > /dev/null 2>&1 &
  ros2 service call /banyu/mission/prepare_autonomy std_srvs/srv/Trigger '{}' > "$OUT/prepare.out" 2>&1
  echo "prepared $(date +%T)" >> "$OUT/progress.txt"
  sleep 8
  timeout "$TIMEOUT" python3 "${REPO_DIR}/tools/sos_integration/mock_gcs.py" --send run-start --serve \
    --timeout "$TIMEOUT" --log "$OUT/mock_gcs.jsonl" "${GCS_ARGS[@]}" > "$OUT/mock_gcs.out" 2>&1 &
  GCS_PID=$!
  # Core ends shortly after the survey report, Advanced and Disruptive after the resource request, once the ring
  # recorder has the final state.
  if [ "$TIER" = "core" ]; then DONE_KIND=uuv_pipeline_survey; else DONE_KIND=uuv_resource_delivery; fi
  for _ in $(seq 1 "$TIMEOUT"); do
    kill -0 "$GCS_PID" 2>/dev/null || break
    if grep -q "$DONE_KIND" "$OUT/mock_gcs.jsonl" 2>/dev/null; then sleep 20; kill "$GCS_PID" 2>/dev/null; break; fi
    sleep 1
  done
  wait "$GCS_PID"
  # Gazebo ground truth of the pipeline lights at the end of the run.
  timeout 5 gz topic -e -n1 -t /robotx/pipeline/status > "$OUT/pipeline_status.txt" 2>/dev/null
  echo "mock_gcs exit=$? $(date +%T)" >> "$OUT/progress.txt"
fi
"${SCRIPT_DIR}/task2_sim_cleanup.sh"
echo "ALLDONE $(date +%T)" >> "$OUT/progress.txt"
