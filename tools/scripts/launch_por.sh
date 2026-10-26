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
WS_DIR="$(cd "${REPO_DIR}/../.." && pwd)"   # <ws>/src/BANYU_ROBOTX -> <ws>
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
ALLOW_CHANNEL_OVERLAP=false
ALLOW_UNCAPPED_THRUST=false
RECORD=true
RECORD_IMAGES=true
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
  --no-record           Do not record the run. By default every real run is recorded to a
                        rosbag under $BANYU_BAG_DIR (~/banyu_bags) for review after the dive.
  --no-record-images    Record the run without compressed camera images (much smaller bag).

Strategy Profile Presets (--profile <name>), sim gate-back times (ALT_HOLD, no GPS):
  default               : por_params.yaml                    (safe strafe box, ~37 s)
  safe                  : por_params_timed_safe.yaml         (safe strafe box, ~37 s)
  fast, fast-strafe     : por_params_timed_fast.yaml         (fast strafe box, ~32 s)
  fastest               : por_params_timed_fastest.yaml      (diamond, back up through gate, ~25 s)
  turn, fastest-turn    : por_params_timed_fastest_turn.yaml (diamond, 180 deg turn, ~28 s)
  vision                : por_params_vision_assisted.yaml    (diamond + camera corrections, ~36 s)
  hybrid                : por_params_hybrid.yaml             (strafe box + camera return, ~44 s)
  slow                  : por_params_timed_slow.yaml         (gentle first pool test, ~71 s)
  Thrusters 2+3 dead: generate a profile from pool measurements with
    tools/scripts/dead_thruster_por.py (docs/por/tuning.md section 6) and pass its path.
  Any other file under banyu_bringup/config/ by relative path, e.g.
    --profile blind/por_params_timed_safe.yaml   (pure timer, no sensors, no camera)
  Profiles under sim_dead_thruster/ and sim_faults/ are simulation only (refused with --real).
  --real requires --profile; --no-rc requires a profile with bypass_rc: true.
  Guide: docs/por/README.md

General Options:
  --dry-run             Print the resolved launch command and check devices without launching.
  --allow-channel-overlap
                        Launch even if a profile rc_override channel is also a teleop switch
                        channel (SB/SC). Unsafe: the switch then reads the override.
  --allow-uncapped-thrust
                        Launch on the robot even if the Pixhawk MOT_PWM_MIN/MAX thruster cap
                        (banyu_defaults.parm) cannot be confirmed. Unsafe: the ESC power
                        board cannot supply full thruster current.
  -h, --help            Show this help message.

Examples:
  # Launch Gazebo simulation with GUI and fastest diamond strategy:
  ./tools/scripts/launch_por.sh --sim --profile fastest

  # Launch Gazebo simulation headless for CI / rapid verification:
  ./tools/scripts/launch_por.sh --sim --headless --profile safe

  # Launch real AUV with TX16S transmitter and the safe strafe box:
  ./tools/scripts/launch_por.sh --real --profile safe

  # Launch real AUV headless without transmitter:
  ./tools/scripts/launch_por.sh --real --no-rc --profile por_params_dead_thruster_2_3_no_rc_no_pos.yaml
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
    --no-record)
      RECORD=false
      shift
      ;;
    --no-record-images)
      RECORD_IMAGES=false
      shift
      ;;
    --dry-run)
      DRY_RUN=true
      shift
      ;;
    --allow-channel-overlap)
      ALLOW_CHANNEL_OVERLAP=true
      shift
      ;;
    --allow-uncapped-thrust)
      ALLOW_UNCAPPED_THRUST=true
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
fi

# --- Resolve profile ---
if [ -z "$PROFILE" ]; then
  if [ "$TARGET" = "sim" ]; then
    PROFILE="por_params.yaml"
  else
    # No silent default on the robot: each profile assumes a vehicle state (healthy or
    # dead thrusters, RC or not) and an RC channel map.
    echo "Error: --real needs --profile <name>. Run with --help for the list." >&2
    exit 1
  fi
else
  case "$PROFILE" in
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

profile_bypass_rc() {
  grep -Eq '^[[:space:]]*bypass_rc:[[:space:]]*true' "$PROFILE_PATH"
}

if [ "$TARGET" = "real" ]; then
  case "$PROFILE_PATH" in
    */sim_dead_thruster/*|*/sim_faults/*)
      echo "Error: $(basename "$(dirname "$PROFILE_PATH")")/ profiles are tuned for the simulator" \
        "(sim speeds, sim trim) and are refused on the real AUV." >&2
      exit 1
      ;;
  esac

  if [ "$USE_RC" = false ] && ! profile_bypass_rc; then
    echo "Error: --no-rc needs a profile with banyu_vehicle_status bypass_rc: true" >&2
    echo "  (e.g. one generated with tools/scripts/dead_thruster_por.py --no-rc)." >&2
    echo "  Without it vehicle status never reports AUTONOMOUS and the mission cannot start." >&2
    exit 1
  fi
  if [ "$USE_RC" = true ] && profile_bypass_rc; then
    echo "Warning: profile sets bypass_rc: true but teleop is on. Stick takeover is disabled;" >&2
    echo "  SB still kills (fresh kill_switch), but a stale switch no longer does." >&2
  fi

  if [ "$USE_RC" = true ]; then
    # /mavros/rc/in reports RC override values, so a teleop switch on an override channel
    # reads the autonomy sticks instead of the switch (neutral 1500 = SB Mid = MANUAL).
    TELEOP_YAML="${REPO_DIR}/banyu_teleop/config/teleop_rcin.yaml"
    OVERLAP="$(python3 - "$PROFILE_PATH" "$TELEOP_YAML" << 'PYCHECK'
import sys
import yaml
profile = yaml.safe_load(open(sys.argv[1]))
teleop = yaml.safe_load(open(sys.argv[2]))["banyu_teleop"]["ros__parameters"]
rc = profile.get("vehicle_execution_node", {}).get("ros__parameters", {}).get("rc_override", {})
switches = {"SB (kill/arm)": teleop.get("kill_channel", 5) + 1,
            "SC (flight mode)": teleop.get("mode_channel", 7) + 1}
for name, ch in switches.items():
    for key in ("channel_forward", "channel_lateral", "channel_throttle", "channel_yaw"):
        if rc.get(key) == ch:
            print(f"{name} is on CH{ch}, the same channel as rc_override.{key}")
PYCHECK
)"
    if [ -n "$OVERLAP" ]; then
      echo "----------------------------------------------------------" >&2
      echo "DANGER: RC channel overlap between this profile and banyu_teleop:" >&2
      echo "$OVERLAP" | sed 's/^/  /' >&2
      echo "  The mission's RC override is read back as the switch: neutral = SB Mid (MANUAL)," >&2
      echo "  a strong stick = SB Down (KILL + disarm). See docs/por/README.md (RC channels)." >&2
      if [ "$ALLOW_CHANNEL_OVERLAP" = false ]; then
        echo "  Refusing to launch. Fix RCMAP / teleop_rcin.yaml, or pass --allow-channel-overlap." >&2
        exit 1
      fi
      echo "  --allow-channel-overlap given: launching anyway." >&2
      echo "----------------------------------------------------------" >&2
    fi
  fi
fi

echo "=========================================================="
echo " BANYU AUV — Unified POR Mission Launcher"
echo "=========================================================="
echo "Active Git Branch : $CURRENT_BRANCH"
echo "Target Platform   : $([ "$TARGET" = "sim" ] && echo "Gazebo Simulation (SITL)" || echo "Real AUV Physical Hardware")"
echo "Selected Profile  : ${PROFILE_PATH#"${CONFIG_DIR}/"}"
if [ "$TARGET" = "sim" ]; then
  echo "Gazebo GUI        : $([ "$HEADLESS" = true ] && echo "HEADLESS (server only)" || echo "ENABLED")"
else
  echo "Transmitter Mode  : $([ "$USE_RC" = true ] && echo "RC (TX16S enabled)" || echo "No-RC (Headless / Bypass)")"
  echo "Cameras Enabled   : $ENABLE_CAMERAS"
  echo "Land Test Mock    : $MOCK_AUTO"
  if [ "$RECORD" = true ]; then
    BAG_ROOT="${BANYU_BAG_DIR:-$HOME/banyu_bags}"
    mkdir -p "$BAG_ROOT"
    BAG_FREE_MB="$(df -Pm "$BAG_ROOT" | awk 'NR==2 {print $4}')"
    echo "Mission Recording : ON -> ${BAG_ROOT} ($((BAG_FREE_MB / 1024)) GB free," \
      "images $([ "$RECORD_IMAGES" = true ] && echo "compressed" || echo "off"))"
  else
    echo "Mission Recording : OFF"
  fi
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

  # The real AUV has no GPS; simulate the same unless the caller asks for use_gps:=True.
  if [[ " ${EXTRA_ARGS[*]} " != *" use_gps:="* ]]; then
    LAUNCH_CMD+=(use_gps:=False)
  fi

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
    echo "  Run first: ./src/BANYU_ROBOTX/tools/scripts/run_mavproxy.sh"
  fi

  # The ESC power board cannot supply full thruster current: the Pixhawk must carry the
  # MOT_PWM_MIN/MAX cap from banyu_defaults.parm. Read over MAVProxy's 14552 output, which
  # is free until nogps_odom_beacon starts.
  if THRUST_CHECK="$(python3 -I "${SCRIPT_DIR}/check_thrust_limit.py" 2>&1)"; then
    echo "Thruster Cap      : ${THRUST_CHECK#check_thrust_limit: }"
  else
    echo "----------------------------------------------------------" >&2
    echo "DANGER: thruster output cap not confirmed on the Pixhawk:" >&2
    echo "$THRUST_CHECK" | sed 's/^/  /' >&2
    if [ "$DRY_RUN" = true ]; then
      echo "  (dry run: a real launch would refuse here)" >&2
    elif [ "$ALLOW_UNCAPPED_THRUST" = false ]; then
      echo "  Refusing to launch. Load banyu_defaults.parm in QGC, or pass --allow-uncapped-thrust." >&2
      exit 1
    else
      echo "  --allow-uncapped-thrust given: launching anyway." >&2
    fi
    echo "----------------------------------------------------------" >&2
  fi

  echo "----------------------------------------------------------"

  LAUNCH_CMD=(
    ros2 launch banyu_bringup sub_bringup.launch.py
    mode:="$MODE"
    por_params_file:="$PROFILE_PATH"
    enable_cameras:="$([ "$ENABLE_CAMERAS" = true ] && echo "True" || echo "False")"
    enable_mock_auto:="$([ "$MOCK_AUTO" = true ] && echo "True" || echo "False")"
  )

  if [ "$USE_RC" = true ]; then
    LAUNCH_CMD+=(
      enable_teleop:=True
      enable_esp32_trigger:=False
    )
    echo "Operator Instructions (TX16S Active):"
    echo "  1. Switch SB DOWN (Disarmed / Kill) before entering water."
    echo "  2. In water: Switch SB UP (Autonomous mode, ESP32 relay ON, arms Pixhawk)."
    echo "  3. Start mission: ros2 service call /banyu/mission/start std_srvs/srv/Trigger \"{}\""
    echo "  Takeover: Switch SB MID (Manual mode, auto-armed). Emergency: Switch SB DOWN (Kill & Disarm)."
  else
    LAUNCH_CMD+=(
      enable_teleop:=False
      enable_esp32_trigger:=True
    )
    echo "Operator Instructions (No-RC Mode):"
    echo "  1. ESP32 relay is handled by esp32_trigger_node (/dev/ttyUSB0)."
    echo "  2. Power ON relay (autonomous): ros2 topic pub --once /banyu/trigger/cmd std_msgs/msg/String \"{data: '2'}\""
    echo "  3. In water: start mission: ros2 service call /banyu/mission/start std_srvs/srv/Trigger \"{}\""
    echo "  Emergency Kill (relay off, mission stops, Pixhawk disarms):"
    echo "    ros2 topic pub --once /banyu/trigger/cmd std_msgs/msg/String \"{data: '0'}\""
    echo "  Software Abort: ros2 service call /banyu/mission/abort std_srvs/srv/Trigger \"{}\""
  fi

  # The link drops underwater, so the bag is the only record of the dive.
  LAUNCH_CMD+=(
    record:="$([ "$RECORD" = true ] && echo "True" || echo "False")"
    record_images:="$([ "$RECORD_IMAGES" = true ] && echo "True" || echo "False")"
  )
  if [ "$RECORD" = true ]; then
    # Name the run folder after the profile; record_mission.sh reads this from the environment.
    PROFILE_NAME="$(basename "$PROFILE_PATH" .yaml)"
    export BANYU_BAG_LABEL="por_${PROFILE_NAME#por_params_}"
    if [ "$BAG_FREE_MB" -lt "${BANYU_BAG_MIN_FREE:-2048}" ]; then
      echo "----------------------------------------------------------" >&2
      echo "Warning: only ${BAG_FREE_MB} MB free in ${BAG_ROOT}; the recorder needs" \
        "${BANYU_BAG_MIN_FREE:-2048} MB and will NOT record this run." >&2
      echo "  Free space (copy off old runs from ${BAG_ROOT}) or pass --no-record." >&2
      echo "----------------------------------------------------------" >&2
    fi
    echo "After the run: copy ${BAG_ROOT}/<run>/ off the vehicle; if it lost power first, run"
    echo "  ros2 run banyu_bringup recover_bag.sh ${BAG_ROOT}/<run>/bag"
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
