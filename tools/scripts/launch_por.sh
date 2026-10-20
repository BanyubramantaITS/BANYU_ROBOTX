#!/bin/bash
# ==============================================================================
# launch_por.sh — Unified Launcher for Proof of Readiness (POR) Autonomous Runs
# ==============================================================================
# Detects active branch/strategy, validates hardware links, configures RC vs
# headless modes, and launches sub_bringup with the correct parameters.
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
CONFIG_DIR="${REPO_DIR}/banyu_bringup/config"

# --- Detect active git branch ---
CURRENT_BRANCH="$(git -C "${REPO_DIR}" rev-parse --abbrev-ref HEAD 2>/dev/null || echo "unknown")"

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
Usage: ./tools/scripts/launch_por.sh [OPTIONS]

Unified launcher for BANYU AUV Proof-of-Readiness (POR) missions.
Automatically selects the optimal YAML profile based on your active git branch.

Options:
  --rc              (Default) Enable TX16S RC transmitter teleop and safety switches.
  --no-rc           Disable teleop; enable bypass_rc and esp32_trigger_node (use when
                    RC battery is dead or running headless).
  --profile <file>  Specify a custom YAML config file (name or full path).
                    Presets:
                      dead-thruster (RC)     : por_params_dead_thruster_2_3_no_pos.yaml
                      dead-thruster (No RC)  : por_params_dead_thruster_2_3_no_rc_no_pos.yaml
                      fast-strafe            : por_params_timed_fast.yaml
                      fastest-diamond        : por_params_timed_fastest.yaml
                      fastest-turn           : por_params_timed_fastest_turn.yaml
                      slow                   : por_params_timed_slow.yaml
                      vision-assisted        : por_params_vision_assisted.yaml
                      hybrid                 : por_params_hybrid.yaml
  --no-cam          Disable USB camera drivers to save CPU/bandwidth on pure timed runs.
  --mock-auto       Enable land-test mock auto mode in teleop (spin thrusters on land).
  --dry-run         Print the resolved launch command and check devices without launching.
  -h, --help        Show this help message.

Branch Presets:
  feat/por-dead-thruster   -> por_params_dead_thruster_2_3_no_pos.yaml (or no_rc variant)
  feat/por-fast-strategies -> por_params_timed_fast.yaml (strafe-box)
  feat/por-timed-strategy  -> por_params.yaml (or por_params_timed_slow.yaml)
  feat/por-vision-assisted -> por_params_vision_assisted.yaml
EOF
}

# --- Parse arguments ---
while [[ $# -gt 0 ]]; do
  case "$1" in
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

# --- Resolve profile if not explicitly provided ---
if [ -z "$PROFILE" ]; then
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
        PROFILE="por_params.yaml"
      else
        PROFILE="por_params_dead_thruster_2_3_no_rc_no_pos.yaml"
      fi
      ;;
  esac
else
  # Resolve aliases
  case "$PROFILE" in
    dead-thruster-rc) PROFILE="por_params_dead_thruster_2_3_no_pos.yaml" ;;
    dead-thruster-norc|dead-thruster-no-rc) PROFILE="por_params_dead_thruster_2_3_no_rc_no_pos.yaml" ;;
    fast|fast-strafe) PROFILE="por_params_timed_fast.yaml" ;;
    fastest|fastest-diamond) PROFILE="por_params_timed_fastest.yaml" ;;
    turn|fastest-turn) PROFILE="por_params_timed_fastest_turn.yaml" ;;
    slow) PROFILE="por_params_timed_slow.yaml" ;;
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

# --- Hardware Checks ---
echo "=========================================================="
echo " BANYU AUV — POR Mission Launcher"
echo "=========================================================="
echo "Active Git Branch : $CURRENT_BRANCH"
echo "Selected Profile  : $(basename "$PROFILE_PATH")"
echo "Transmitter Mode  : $([ "$USE_RC" = true ] && echo "RC (TX16S enabled)" || echo "No-RC (Headless / Bypass)")"
echo "Cameras Enabled   : $ENABLE_CAMERAS"
echo "Land Test Mock    : $MOCK_AUTO"
echo "----------------------------------------------------------"

# Device check
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

# Check if MAVProxy is running
if pgrep -f "mavproxy.py" > /dev/null; then
  echo "MAVProxy Process  : RUNNING (127.0.0.1:14551)"
else
  echo "Warning: MAVProxy does not appear to be running."
  echo "  Run first: ./tools/scripts/run_mavproxy.sh"
fi

echo "----------------------------------------------------------"

# --- Formulate ROS 2 launch arguments ---
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
