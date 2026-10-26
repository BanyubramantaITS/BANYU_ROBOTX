#!/bin/bash
# ==============================================================================
# launch_task2.sh — Unified Launcher for RobotX Task 2 (mode:=task2)
# ==============================================================================
# Supports both:
#   1. Gazebo Harmonic SITL Simulation (main_launch.launch.py mode:=task2)
#   2. Real AUV Physical Hardware Deployment (sub_bringup.launch.py mode:=task2)
#
# Only per-run operator choices are flags; everything else keeps the launch-file
# defaults. Modeled on launch_por.sh.
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
WS_DIR="$(cd "${REPO_DIR}/../.." && pwd)"   # <ws>/src/BANYU_ROBOTX -> <ws>

CURRENT_BRANCH="$(git -C "${REPO_DIR}" rev-parse --abbrev-ref HEAD 2>/dev/null || echo "unknown")"

# --- Defaults (identical to the launch files) ---
TARGET="real"            # "real" or "sim" (same default as launch_por.sh)
HEADLESS=false
CLEAN_SIM=true
TIER="advanced"
GPS_GATE=false
GPS_LOSS_DEPTH="0.1"
GPS_REACQUIRE_DELAY="10.0"
USE_GPS=true
DRY_RUN=false
TIER_SET=false
GPS_GATE_OPTS_SET=false
EXTRA_ARGS=()

print_usage() {
  cat << 'EOF'
Usage: ./tools/scripts/launch_task2.sh [TARGET] [OPTIONS] [ros2 launch args...]

Unified launcher for BANYU AUV RobotX Task 2 (mode:=task2).

Target Selection:
  --sim, --gazebo       Launch in Gazebo Harmonic simulation with SITL & MAVROS
                        (banyu_bringup/main_launch.launch.py mode:=task2).
  --real, --hardware    (Default) Launch on the real AUV
                        (banyu_bringup/sub_bringup.launch.py mode:=task2).

Simulation Options (--sim only):
  --headless            Gazebo server without GUI and MAVProxy without console.
  --gui                 (Default) Gazebo GUI and MAVProxy.
  --no-clean            Skip pre-flight cleanup of stale SITL/ardusub/mavproxy processes.
  --tier <t>            Task 2 tier: core | advanced | disruptive (default: advanced).
  --gps-gate            Simulate GPS loss/reacquire around depth (gps_gate:=True).
  --no-gps-gate         (Default) Do not gate GPS (gps_gate:=False).
  --gps-loss-depth <m>  Depth at which GPS is lost (default: 0.1; gps_loss_depth_m).
  --gps-reacquire-delay <s>
                        Seconds after surfacing before GPS returns (default: 10.0;
                        gps_reacquire_delay_s).
                        The gps-gate options are rejected with --real, and so is --tier
                        (sub_bringup.launch.py has no such arguments).

Both targets:
  --use-gps             (Default) GPS available (use_gps:=True); Task 2 needs the surface
                        GPS leg (NavigateSurfaceGps).
  --no-gps              use_gps:=False.
  --dry-run             Print the resolved launch command without launching or cleaning.
  -h, --help            Show this help message.

Any other argument is passed through to ros2 launch (e.g. task2_params_file:=/path.yaml).

Examples:
  ./tools/scripts/launch_task2.sh --sim
  ./tools/scripts/launch_task2.sh --sim --headless --tier disruptive --gps-gate --gps-loss-depth 0.2
  ./tools/scripts/launch_task2.sh --real --dry-run
EOF
}

# --- Parse arguments ---
while [[ $# -gt 0 ]]; do
  case "$1" in
    --sim|--gazebo) TARGET="sim"; shift ;;
    --real|--hardware) TARGET="real"; shift ;;
    --headless) HEADLESS=true; shift ;;
    --gui) HEADLESS=false; shift ;;
    --no-clean) CLEAN_SIM=false; shift ;;
    --tier)
      [ $# -ge 2 ] || { echo "Error: --tier needs a value (core|advanced|disruptive)." >&2; exit 1; }
      TIER="$2"; TIER_SET=true; shift 2 ;;
    --gps-gate) GPS_GATE=true; GPS_GATE_OPTS_SET=true; shift ;;
    --no-gps-gate) GPS_GATE=false; shift ;;
    --gps-loss-depth)
      [ $# -ge 2 ] || { echo "Error: --gps-loss-depth needs a value in metres." >&2; exit 1; }
      GPS_LOSS_DEPTH="$2"; GPS_GATE_OPTS_SET=true; shift 2 ;;
    --gps-reacquire-delay)
      [ $# -ge 2 ] || { echo "Error: --gps-reacquire-delay needs a value in seconds." >&2; exit 1; }
      GPS_REACQUIRE_DELAY="$2"; GPS_GATE_OPTS_SET=true; shift 2 ;;
    --use-gps) USE_GPS=true; shift ;;
    --no-gps) USE_GPS=false; shift ;;
    --dry-run) DRY_RUN=true; shift ;;
    -h|--help) print_usage; exit 0 ;;
    *) EXTRA_ARGS+=("$1"); shift ;;
  esac
done

# --- Validation ---
case "$TIER" in
  core|advanced|disruptive) ;;
  *) echo "Error: --tier must be core, advanced or disruptive (got '$TIER')." >&2; exit 1 ;;
esac
NUM_RE='^[0-9]+([.][0-9]+)?$'
if ! [[ "$GPS_LOSS_DEPTH" =~ $NUM_RE ]]; then
  echo "Error: --gps-loss-depth must be a non-negative number (got '$GPS_LOSS_DEPTH')." >&2; exit 1
fi
if ! [[ "$GPS_REACQUIRE_DELAY" =~ $NUM_RE ]]; then
  echo "Error: --gps-reacquire-delay must be a non-negative number (got '$GPS_REACQUIRE_DELAY')." >&2; exit 1
fi
if [ "$TARGET" = "real" ]; then
  if [ "$GPS_GATE" = true ] || [ "$GPS_GATE_OPTS_SET" = true ]; then
    echo "Error: --gps-gate / --gps-loss-depth / --gps-reacquire-delay are simulation only" >&2
    echo "  (the GPS gate is a sim shim); they are refused with --real. Use --sim." >&2
    exit 1
  fi
  if [ "$TIER_SET" = true ]; then
    echo "Error: --tier is simulation only; sub_bringup.launch.py has no tier argument." >&2
    echo "  Use --sim, or set the tier in the Task 2 params file (task2_params_file:=...)." >&2
    exit 1
  fi
fi

# --- Ensure ROS 2 workspace is sourced ---
if [ -z "$ROS_DISTRO" ] || ! command -v ros2 &> /dev/null; then
  if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    echo "Sourcing ROS 2 Jazzy environment..."
    source /opt/ros/jazzy/setup.bash
  fi
fi
if [ -f "${WS_DIR}/install/setup.bash" ]; then
  source "${WS_DIR}/install/setup.bash"
fi

TF() { [ "$1" = true ] && echo "True" || echo "False"; }

echo "=========================================================="
echo " BANYU AUV — Task 2 Mission Launcher"
echo "=========================================================="
echo "Active Git Branch : $CURRENT_BRANCH"
echo "Target Platform   : $([ "$TARGET" = "sim" ] && echo "Gazebo Simulation (SITL)" || echo "Real AUV Physical Hardware")"
echo "use_gps           : $(TF $USE_GPS)"
if [ "$TARGET" = "sim" ]; then
  echo "Gazebo GUI        : $([ "$HEADLESS" = true ] && echo "HEADLESS (server only)" || echo "ENABLED")"
  echo "Tier              : $TIER"
  echo "GPS gate          : $(TF $GPS_GATE) (loss depth ${GPS_LOSS_DEPTH} m, reacquire delay ${GPS_REACQUIRE_DELAY} s)"
fi
echo "----------------------------------------------------------"

ENV_PREFIX=()
if [ "$TARGET" = "sim" ]; then
  if [ "$CLEAN_SIM" = true ] && [ "$DRY_RUN" = false ]; then
    echo "Pre-flight: Cleaning orphaned ArduSub / MAVProxy / MAVROS processes..."
    killall -9 ardusub mavproxy mavros_node 2>/dev/null || true
    pkill -9 -f "sim_vehicle.py" 2>/dev/null || true
    sleep 0.5
  fi

  # Known issues: SITL's terminal spawner must use bash, and a surrounding tmux session
  # breaks the sim_vehicle.py console/map windows.
  export SITL_RITW_TERMINAL=bash
  unset TMUX
  ENV_PREFIX+=(SITL_RITW_TERMINAL=bash)

  # Hybrid-GPU laptops: render Gazebo on the NVIDIA GPU, only when one is present.
  if command -v nvidia-smi &> /dev/null && nvidia-smi -L &> /dev/null; then
    export __NV_PRIME_RENDER_OFFLOAD=1
    export __GLX_VENDOR_LIBRARY_NAME=nvidia
    ENV_PREFIX+=(__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia)
    echo "NVIDIA GPU        : detected, PRIME render offload enabled"
  else
    echo "NVIDIA GPU        : not detected, default renderer"
  fi

  LAUNCH_CMD=(
    ros2 launch banyu_bringup main_launch.launch.py
    mode:=task2
    tier:="$TIER"
    headless:="$(TF $HEADLESS)"
    use_gps:="$(TF $USE_GPS)"
    gps_gate:="$(TF $GPS_GATE)"
    gps_loss_depth_m:="$GPS_LOSS_DEPTH"
    gps_reacquire_delay_s:="$GPS_REACQUIRE_DELAY"
  )
  [ ${#EXTRA_ARGS[@]} -gt 0 ] && LAUNCH_CMD+=("${EXTRA_ARGS[@]}")

  echo "Simulation Operator Instructions (Task 2):"
  echo "  1. Wait for Gazebo Harmonic and MAVROS to connect (pose at 20-50 Hz)."
  echo "  2. Verify link: ros2 topic echo /mavros/state --once"
  echo "  3. Prepare autonomy: ros2 service call /banyu/mission/prepare_autonomy std_srvs/srv/Trigger \"{}\""
  echo "  4. The run does NOT start from /banyu/mission/start. It starts only when the GCS sends"
  echo "     CMD_RUN_START on /system/mission/command."
  echo "  5. UUV side runs on ROS_DOMAIN_ID 40 (the GCS side uses its own domain)."
  echo "  6. Teardown: killall -9 ardusub mavproxy mavros_node"
else
  if [ -e "/dev/pixhawk" ] || [ -e "/dev/ttyACM0" ]; then
    echo "Pixhawk Port      : OK ($([ -e /dev/pixhawk ] && echo "/dev/pixhawk" || echo "/dev/ttyACM0"))"
  else
    echo "Warning: Neither /dev/pixhawk nor /dev/ttyACM0 detected." >&2
  fi
  if pgrep -f "mavproxy.py" > /dev/null; then
    echo "MAVProxy Process  : RUNNING (127.0.0.1:14551)"
  else
    echo "Warning: MAVProxy does not appear to be running."
    echo "  Run first: ./src/BANYU_ROBOTX/tools/scripts/run_mavproxy.sh"
  fi
  echo "----------------------------------------------------------"

  LAUNCH_CMD=(
    ros2 launch banyu_bringup sub_bringup.launch.py
    mode:=task2
    use_gps:="$(TF $USE_GPS)"
  )
  [ ${#EXTRA_ARGS[@]} -gt 0 ] && LAUNCH_CMD+=("${EXTRA_ARGS[@]}")

  echo "Operator Instructions (Task 2, real AUV):"
  echo "  1. UUV side runs on ROS_DOMAIN_ID 40 (export ROS_DOMAIN_ID=40 in this shell and in"
  echo "     any shell you use to call services); the GCS side uses its own domain."
  echo "  2. Prepare autonomy: ros2 service call /banyu/mission/prepare_autonomy std_srvs/srv/Trigger \"{}\""
  echo "  3. The run does NOT start from /banyu/mission/start. It starts only when the GCS sends"
  echo "     CMD_RUN_START on /system/mission/command."
  echo "  Software Abort: ros2 service call /banyu/mission/abort std_srvs/srv/Trigger \"{}\""
fi

echo "=========================================================="
echo "Command to execute:"
echo "${ENV_PREFIX[*]:+${ENV_PREFIX[*]} }${LAUNCH_CMD[*]}"
echo "=========================================================="

if [ "$DRY_RUN" = true ]; then
  echo "Dry run requested. Exiting without launching."
  exit 0
fi

exec "${LAUNCH_CMD[@]}"
