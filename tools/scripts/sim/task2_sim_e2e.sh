#!/bin/bash
# SIMULATION ONLY: one end-to-end Task 2 run in the sim, driven by tools/sos_integration/mock_gcs.py.
#   tools/scripts/sim/task2_sim_e2e.sh --out <dir> [--tier core|advanced|disruptive] [--timeout <s>] [--no-gate]
#                                  [--gcs-arg <arg>]... [--launch-arg <arg>]... [--gui]
# Starts the sim headless and MAVROS (run/launch_mavros.sh --sim) (up to 3 tries: Gazebo sometimes never finishes loading), calls prepare_autonomy,
# sends the course and run-start from the mock GCS, records rings with their Gazebo ground truth, writes
# <dir>/{progress.txt,sim.log,mavros.log,mock_gcs.jsonl,rings.jsonl} and stops everything it started.
# (no set -u: the ROS setup scripts use unset variables)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
WS_DIR="$(cd "${REPO_DIR}/../.." && pwd)"
OUT=""; MODE="--headless"; TIER="core"; TIMEOUT=1200; GATE=(--gps-gate); GCS_ARGS=(); LAUNCH_ARGS=()
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
# The mock GCS course boundary is centred on the sim's GPS home.
if [[ " ${LAUNCH_ARGS[*]} " == *" marina_bay "* ]]; then GCS_ARGS+=(--home-lat 1.2816 --home-lon 103.8636); fi
# A run whose start never takes (the sim wedged after arming) is relaunched, at most twice.
for RUN in 1 2 3; do
  NOSTART=0
  UP=0
  for ATTEMPT in 1 2 3; do
    "${SCRIPT_DIR}/../run/launch_task2.sh" --sim "$MODE" --no-clean --tier "$TIER" "${GATE[@]}" "${LAUNCH_ARGS[@]}" \
      > "$OUT/sim.log" 2>&1 &
    # No launch file starts MAVROS; the cleanup stops it with the rest of the domain-40 processes.
    "${SCRIPT_DIR}/../run/launch_mavros.sh" --sim > "$OUT/mavros.log" 2>&1 &
    for _ in $(seq 1 30); do
      sleep 6
      if timeout 5 ros2 topic echo --once /banyu/vehicle/water_depth 2>/dev/null | grep -q 'valid: true'; then UP=1; break; fi
    done
    if [ "$UP" = 1 ]; then
      # EKF and datum settle: wait for the EKF to take the GPS (about 70 s of sim time after boot, 200 s of wall time when the
      # sim runs at 0.34x), then 15 s. A burst of ArduPilot controller resets there wedges the sim (the vehicle never holds
      # and every run start is NOT_READY): relaunch instead.
      # MAVROS prints the FCU status text, in its own log.
      for _ in $(seq 1 60); do grep -q "is using GPS" "$OUT/mavros.log" && break; sleep 5; done
      sleep 15
      C1=$(timeout 5 ros2 topic echo --once /clock 2>/dev/null | grep -m1 'sec:' | tr -dc 0-9); sleep 4
      C2=$(timeout 5 ros2 topic echo --once /clock 2>/dev/null | grep -m1 'sec:' | tr -dc 0-9)
      RESETS=$(grep -c "controller has reset" "$OUT/sim.log")
      if [ "${RESETS:-0}" -le 1 ] && [ -n "$C1" ] && [ -n "$C2" ] && [ "$C2" -gt "$C1" ]; then
        timeout 60 ros2 service call /banyu/mission/prepare_autonomy std_srvs/srv/Trigger '{}' > "$OUT/prepare.out" 2>&1
        echo "prepared $(date +%T)" >> "$OUT/progress.txt"
        # The run start needs the mission's autonomous_ready (up to 60 s) and vehicle execution holding position (the EKF
        # takes the GPS about 70 s after boot; up to 120 s). A sim that never holds (the controller-reset burst when the EKF
        # takes the GPS) is relaunched.
        for _ in $(seq 1 30); do
          timeout 3 ros2 topic echo --once /banyu/mission/run_lifecycle 2>/dev/null | grep -q "autonomous_ready: true" && break
          sleep 2
        done
        HELD=0
        for _ in $(seq 1 40); do
          if timeout 3 ros2 topic echo --once /banyu/execution/readiness 2>/dev/null | grep -q "holding: true"; then
            HELD=1; break
          fi
          sleep 3
        done
        [ "$HELD" = 1 ] && break
        RESETS=$(grep -c "controller has reset" "$OUT/sim.log")
        echo "attempt $ATTEMPT: execution never held position (controller resets ${RESETS})" >> "$OUT/progress.txt"
      else
        echo "attempt $ATTEMPT: sim wedged (controller resets ${RESETS}, clock ${C1} -> ${C2})" >> "$OUT/progress.txt"
      fi
      UP=0
    else
      echo "attempt $ATTEMPT: sim did not come up" >> "$OUT/progress.txt"
    fi
    "${SCRIPT_DIR}/task2_sim_cleanup.sh"; sleep 10
  done
  echo "up=$UP" >> "$OUT/progress.txt"
  if [ "$UP" = 1 ]; then
    python3 "${REPO_DIR}/tools/sos_integration/ring_recorder.py" "$OUT/rings.jsonl" "$TIMEOUT" > /dev/null 2>&1 &
    # Hydrophone check: without --hydrophone nothing may publish pinger observations.
    ros2 topic info /banyu/pinger/observation > "$OUT/pinger_topic.txt" 2>&1
    echo "pinger $(grep -h 'Publisher count' "$OUT/pinger_topic.txt" || echo 'topic absent')" >> "$OUT/progress.txt"
    echo "ready $(date +%T)" >> "$OUT/progress.txt"
    sleep 3
    timeout "$TIMEOUT" python3 "${REPO_DIR}/tools/sos_integration/mock_gcs.py" --send run-start --serve --start-retry-s 30 \
      --timeout "$TIMEOUT" --log "$OUT/mock_gcs.jsonl" "${GCS_ARGS[@]}" > "$OUT/mock_gcs.out" 2>&1 &
    GCS_PID=$!
    # Core ends shortly after the survey report, Advanced and Disruptive after the resource request, once the ring
    # recorder has the final state.
    if [ "$TIER" = "core" ]; then DONE_KIND=uuv_pipeline_survey; else DONE_KIND=uuv_resource_delivery; fi
    # A best-effort report (survey not completed) is a Core survey with no resource delivery: end 40 s after the survey.
    SURVEY_AT=0
    for i in $(seq 1 "$TIMEOUT"); do
      kill -0 "$GCS_PID" 2>/dev/null || break
      if grep -q "$DONE_KIND" "$OUT/mock_gcs.jsonl" 2>/dev/null; then sleep 20; kill "$GCS_PID" 2>/dev/null; break; fi
      if [ "$SURVEY_AT" = 0 ] && grep -q uuv_pipeline_survey "$OUT/mock_gcs.jsonl" 2>/dev/null; then SURVEY_AT=$i; fi
      if [ "$SURVEY_AT" != 0 ] && [ $((i - SURVEY_AT)) -ge 40 ]; then kill "$GCS_PID" 2>/dev/null; break; fi
      # The UUV heartbeat shows the Task 2 tasking once the run has started; none after 150 s (the start retries all
      # NOT_READY) is a wedged sim: relaunch the whole run.
      if [ "$i" = 150 ] && ! grep -q '"current_task": 3' "$OUT/mock_gcs.jsonl" 2>/dev/null; then
        NOSTART=1; kill "$GCS_PID" 2>/dev/null; break
      fi
      sleep 1
    done
    wait "$GCS_PID"
    # Gazebo ground truth of the pipeline lights at the end of the run.
    timeout 5 gz topic -e -n1 -t /robotx/pipeline/status > "$OUT/pipeline_status.txt" 2>/dev/null
    echo "mock_gcs exit=$? $(date +%T)" >> "$OUT/progress.txt"
  fi
  [ "$NOSTART" = 1 ] || break
  echo "run $RUN: the run never started (sim wedged), relaunching" >> "$OUT/progress.txt"
  mkdir -p "$OUT/wedged_run$RUN"
  mv "$OUT/sim.log" "$OUT/mavros.log" "$OUT/mock_gcs.jsonl" "$OUT/mock_gcs.out" "$OUT/rings.jsonl" "$OUT/wedged_run$RUN/" 2>/dev/null
  "${SCRIPT_DIR}/task2_sim_cleanup.sh"; sleep 10
done
"${SCRIPT_DIR}/task2_sim_cleanup.sh"
echo "ALLDONE $(date +%T)" >> "$OUT/progress.txt"
