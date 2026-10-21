#!/bin/bash
# ==============================================================================
# launch_por.sh — Unified Launcher for Proof of Readiness (POR) Autonomous Runs
# ==============================================================================
# Supports both:
#   1. Gazebo Harmonic SITL Simulation (main_launch.launch.py)
#   2. Real AUV Physical Hardware Deployment (sub_bringup.launch.py)
#
# Automatically detects hardware, cleans stale sim processes, validates ports,
# manages RC vs No-RC telemetry modes, and loads any strategy profile across all
# unified feat/por-* capabilities.
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
WS_DIR="$(cd "${REPO_DIR}/.." && pwd)"
CONFIG_DIR="${REPO_DIR}/banyu_bringup/config"

# --- Detect active git branch ---
CURRENT_BRANCH="$(git -C "${REPO_DIR}" rev-parse --abbrev-ref HEAD 2>/dev/null || echo "unknown")"

# --- Target Mode ---
TARGET="real"  # "real" or "sim"
HEADLESS=false
CLEAN_SIM=true

# --- Defaults ---
MODE="por"
USE_RC=true
ENABLE_CAMERAS=true
DRY_RUN=false
PROFILE=""
MOCK_AUTO=false
EXTRA_ARGS=()

print_usage() {
  cat << 'EOF'
Usage: ./tools/scripts/launch_por.sh [TARGET] [OPTIONS]

Unified launcher for BANYU AUV Proof-of-Readiness (POR) missions.
Supports Gazebo Harmonic simulation and physical vehicle deployment.

Target Selection:
  --sim, --gazebo       Launch in Gazebo Harmonic simulation with SITL & MAVROS
                        (uses banyu_bringup/main_launch.launch.py).
  --real, --hardware    (Default) Launch on real AUV hardware
                        (uses banyu_bringup/sub_bringup.launch.py).

Simulation Options (when using --sim):
  --headless            Run Gazebo server without GUI and MAVProxy without console.
  --gui                 (Default for sim) Run Gazebo Harmonic GUI and MAVProxy.
  --no-clean            Skip pre-flight cleanup of stale SITL/ardusub/mavproxy processes.

Physical Hardware Options (when using --real):
  --rc                  (Default) Enable TX16S RC transmitter teleop and safety switches.
  --no-rc               Disable teleop; enable bypass_rc and esp32_trigger_node (use when
                        RC battery is dead or running headless).
  --no-cam              Disable USB camera drivers to save CPU/bandwidth on timed runs.
  --mock-auto           Enable land-test mock auto mode in teleop (spin thrusters on land).

Strategy Profile Presets (--profile <name>):
  dead-thruster         : por_params_dead_thruster_2_3_no_pos.yaml (RC diagonal compensation)
  dead-thruster-norc    : por_params_dead_thruster_2_3_no_rc_no_pos.yaml (No-RC bypass)
  dead-thruster-pixhawk : por_params_dead_thruster_2_3_no_rc_pixhawk_only.yaml
  fast, fast-strafe     : por_params_timed_fast.yaml (Strafe box, ~55s)
  fastest               : por_params_timed_fastest.yaml (Diamond reverse, ~33.3s)
  turn, fastest-turn    : por_params_timed_fastest_turn.yaml (Diamond 180° turn, ~39.1s)
  slow                  : por_params_timed_slow.yaml (Gentle pool testing, 0.12 m/s legs)
  safe                  : por_params_timed_safe.yaml (Conservative timed strafe)
  vision                : por_params_vision_assisted.yaml (Diamond with camera gate return)
  hybrid                : por_params_hybrid.yaml (Timed outbound + camera return)
  default               : por_params.yaml (Standard baseline profile)

General Options:
  --dry-run             Print the resolved launch command and check devices without launching.
  -h, --help            Show this help message.

Examples:
  # Launch Gazebo simulation with GUI and fastest diamond strategy:
  ./tools/scripts/launch_por.sh --sim --profile fastest

  # Launch Gazebo simulation headless for CI / rapid verification:
  ./tools/scripts/launch_por.sh --sim --headless --profile safe

  # Launch real AUV with TX16S transmitter (default dead-thruster / branch profile):
  ./tools/scripts/launch_por.sh --real

  # Launch real AUV headless without transmitter:
  ./tools/scripts/launch_por.sh --real --no-rc --profile dead-thruster-norc
EOF
}

# --- Parse arguments ---
while [[ $# -gt 0 ]]; do
  case "$1" in
    --sim|--gazebo)
      TARGET="sim"
      shift
      ;;
    --real|--hardware)
      TARGET="real"
      shift
      ;;
    --headless)
      HEADLESS=true
      shift
      ;;
    --gui)
      HEADLESS=false
      shift
      ;;
    --no-clean)
      CLEAN_SIM=false
      shift
      ;;
    --rc)
      USE_RC=true
      shift
      ;;
    --no-rc)
      USE_RC=false
      shift
      ;;
    --profile)
      PROFILE="$2"
      shift 2
      ;;
    --no-cam)
      ENABLE_CAMERAS=false
      shift
      ;;
    --mock-auto)
      MOCK_AUTO=true
      shift
      ;;
    --dry-run)
      DRY_RUN=true
      shift
      ;;
    -h|--help)
      print_usage
      exit 0
      ;;
    *)
      EXTRA_ARGS+=("$1")
      shift
      ;;
  esac
done

# --- Ensure ROS 2 workspace is sourced ---
if [ -z "$ROS_DISTRO" ] || ! command -v ros2 &> /dev/null; then
  if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    echo "Sourcing ROS 2 Jazzy environment..."
    source /opt/ros/jazzy/setup.bash
  fi
fi

if [ -f "${WS_DIR}/install/setup.bash" ]; then
  source "${WS_DIR}/install/setup.bash"
elif [ -f "${REPO_DIR}/../../install/setup.bash" ]; then
  source "${REPO_DIR}/../../install/setup.bash"
fi

# --- Resolve profile if not explicitly provided ---
if [ -z "$PROFILE" ]; then
  if [ "$TARGET" = "sim" ]; then
    # Simulation defaults to standard por_params.yaml or fast timed
    PROFILE="por_params.yaml"
  else
    case "$CURRENT_BRANCH" in
      *dead-thruster*)
        if [ "$USE_RC" = true ]; then
          PROFILE="por_params_dead_thruster_2_3_no_pos.yaml"
        else
          PROFILE="por_params_dead_thruster_2_3_no_rc_no_pos.yaml"
        fi
        ;;
      *fast-strategies*)
        PROFILE="por_params_timed_fast.yaml"
        ;;
      *timed-strategy*)
        PROFILE="por_params.yaml"
        ;;
      *vision-assisted*)
        PROFILE="por_params_vision_assisted.yaml"
        ;;
      *)
        if [ "$USE_RC" = true ]; then
          PROFILE="por_params_dead_thruster_2_3_no_pos.yaml"
        else
          PROFILE="por_params_dead_thruster_2_3_no_rc_no_pos.yaml"
        fi
        ;;
    esac
  fi
else
  # Resolve aliases
  case "$PROFILE" in
    dead-thruster|dead-thruster-rc) PROFILE="por_params_dead_thruster_2_3_no_pos.yaml" ;;
    dead-thruster-norc|dead-thruster-no-rc) PROFILE="por_params_dead_thruster_2_3_no_rc_no_pos.yaml" ;;
    dead-thruster-pixhawk) PROFILE="por_params_dead_thruster_2_3_no_rc_pixhawk_only.yaml" ;;
    fast|fast-strafe) PROFILE="por_params_timed_fast.yaml" ;;
    fastest|fastest-diamond) PROFILE="por_params_timed_fastest.yaml" ;;
    turn|fastest-turn) PROFILE="por_params_timed_fastest_turn.yaml" ;;
    slow) PROFILE="por_params_timed_slow.yaml" ;;
    safe) PROFILE="por_params_timed_safe.yaml" ;;
    vision|vision-assisted) PROFILE="por_params_vision_assisted.yaml" ;;
    hybrid) PROFILE="por_params_hybrid.yaml" ;;
    default) PROFILE="por_params.yaml" ;;
  esac
fi

# Ensure full path to profile
if [[ "$PROFILE" != /* ]]; then
  PROFILE_PATH="${CONFIG_DIR}/${PROFILE}"
else
  PROFILE_PATH="${PROFILE}"
fi

if [ ! -f "$PROFILE_PATH" ]; then
  echo "Error: Profile file not found: $PROFILE_PATH" >&2
  exit 1
fi

echo "=========================================================="
echo " BANYU AUV — Unified POR Mission Launcher"
echo "=========================================================="
echo "Active Git Branch : $CURRENT_BRANCH"
echo "Target Platform   : $([ "$TARGET" = "sim" ] && echo "Gazebo Simulation (SITL)" || echo "Real AUV Physical Hardware")"
echo "Selected Profile  : $(basename "$PROFILE_PATH")"
if [ "$TARGET" = "sim" ]; then
  echo "Gazebo GUI        : $([ "$HEADLESS" = true ] && echo "HEADLESS (server only)" || echo "ENABLED")"
else
  echo "Transmitter Mode  : $([ "$USE_RC" = true ] && echo "RC (TX16S enabled)" || echo "No-RC (Headless / Bypass)")"
  echo "Cameras Enabled   : $ENABLE_CAMERAS"
  echo "Land Test Mock    : $MOCK_AUTO"
fi
echo "----------------------------------------------------------"

if [ "$TARGET" = "sim" ]; then
  # Simulation pre-flight checks and cleanup
  if [ "$CLEAN_SIM" = true ] && [ "$DRY_RUN" = false ]; then
    echo "Pre-flight: Cleaning orphaned ArduSub / MAVProxy / MAVROS processes..."
    killall -9 ardusub mavproxy mavros_node 2>/dev/null || true
    pkill -9 -f "sim_vehicle.py" 2>/dev/null || true
    sleep 0.5
  fi

  LAUNCH_CMD=(
    ros2 launch banyu_bringup main_launch.launch.py
    mode:=por
    por_params_file:="$PROFILE_PATH"
    headless:="$([ "$HEADLESS" = true ] && echo "True" || echo "False")"
  )

  if [ ${#EXTRA_ARGS[@]} -gt 0 ]; then
    LAUNCH_CMD+=("${EXTRA_ARGS[@]}")
  fi

  echo "Simulation Operator Instructions:"
  echo "  1. Wait for Gazebo Harmonic and MAVROS to connect (pose at 20-50 Hz)."
  echo "  2. Verify link: ros2 topic echo /mavros/state --once"
  echo "  3. Start mission: ros2 service call /banyu/mission/start std_srvs/srv/Trigger \"{}\""
  echo "  4. Abort mission: ros2 service call /banyu/mission/abort std_srvs/srv/Trigger \"{}\""
  echo "  5. Teardown: killall -9 ardusub mavproxy mavros_node"

else
  # Real Hardware Checks
  if [ -e "/dev/pixhawk" ] || [ -e "/dev/ttyACM0" ]; then
    echo "Pixhawk Port      : OK ($([ -e /dev/pixhawk ] && echo "/dev/pixhawk" || echo "/dev/ttyACM0"))"
  else
    echo "Warning: Neither /dev/pixhawk nor /dev/ttyACM0 detected." >&2
  fi

  if [ -e "/dev/ttyUSB0" ]; then
    echo "ESP32 Relay Port  : OK (/dev/ttyUSB0)"
  else
    echo "Warning: ESP32 port /dev/ttyUSB0 not found." >&2
  fi

  if pgrep -f "mavproxy.py" > /dev/null; then
    echo "MAVProxy Process  : RUNNING (127.0.0.1:14551)"
  else
    echo "Warning: MAVProxy does not appear to be running."
    echo "  Run first: ./tools/scripts/run_mavproxy.sh"
  fi

  echo "----------------------------------------------------------"

  LAUNCH_CMD=(
    ros2 launch banyu_bringup sub_bringup.launch.py
    mode:="$MODE"
    por_params_file:="$PROFILE_PATH"
    enable_cameras:="$ENABLE_CAMERAS"
    enable_mock_auto:="$MOCK_AUTO"
  )

  if [ "$USE_RC" = true ]; then
    LAUNCH_CMD+=(
      enable_teleop:=True
      enable_esp32_trigger:=False
    )
    echo "Operator Instructions (TX16S Active):"
    echo "  1. Switch SF UP (Disarmed) before entering water."
    echo "  2. Switch SB UP (Autonomous, ESP32 relay ON, LED GREEN)."
    echo "  3. In water: Switch SF DOWN (Armed)."
    echo "  4. Start mission: ros2 service call /banyu/mission/start std_srvs/srv/Trigger \"{}\""
    echo "  Takeover: Switch SB MID (Manual). Emergency: Switch SB DOWN (Kill)."
  else
    LAUNCH_CMD+=(
      enable_teleop:=False
      enable_esp32_trigger:=True
    )
    echo "Operator Instructions (No-RC Mode):"
    echo "  1. ESP32 relay is handled by esp32_trigger_node (/dev/ttyUSB0)."
    echo "  2. Power ON relay: ros2 topic pub --once /banyu/trigger/cmd std_msgs/msg/String \"{data: '0'}\""
    echo "  3. In water: start mission: ros2 service call /banyu/mission/start std_srvs/srv/Trigger \"{}\""
    echo "  Emergency Kill: ros2 topic pub --once /banyu/trigger/cmd std_msgs/msg/String \"{data: '2'}\""
    echo "  Software Abort: ros2 service call /banyu/mission/abort std_srvs/srv/Trigger \"{}\""
  fi

  if [ ${#EXTRA_ARGS[@]} -gt 0 ]; then
    LAUNCH_CMD+=("${EXTRA_ARGS[@]}")
  fi
fi

echo "=========================================================="
echo "Command to execute:"
echo "${LAUNCH_CMD[*]}"
echo "=========================================================="

if [ "$DRY_RUN" = true ]; then
  echo "Dry run requested. Exiting without launching."
  exit 0
fi

# Execute launch
exec "${LAUNCH_CMD[@]}"
