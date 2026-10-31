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

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
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
RECORD=""         # default: on for --real, off for --sim
RECORD_IMAGES=""  # default: on for --real, off for --sim
REPLAY_TRACK=""
EXTRA_ARGS=()

print_usage() {
  cat << 'EOF'
Usage: ./tools/scripts/launch_por.sh [TARGET] [OPTIONS]

Unified launcher for BANYU AUV Proof-of-Readiness (POR) missions.
Supports Gazebo Harmonic simulation and physical vehicle deployment.

Target Selection:
  --sim, --gazebo       Launch in Gazebo Harmonic simulation with SITL
                        (uses banyu_bringup/main_launch.launch.py).
  --real, --hardware    (Default) Launch on real AUV hardware
                        (uses banyu_bringup/sub_bringup.launch.py).

Simulation Options (when using --sim):
  --headless            Run Gazebo server without GUI and MAVProxy without console.
  --gui                 (Default for sim) Run Gazebo Harmonic GUI and MAVProxy.
  --no-clean            Skip pre-flight cleanup of stale SITL/ardusub/mavproxy processes.
  --record              Record the sim run (sim time, ground truth /sim/ground_truth/odom) to
                        $BANYU_BAG_DIR (~/banyu_bags); off by default in sim. Add
                        --record-images to include camera images (slows the sim).

Physical Hardware Options (when using --real):
  --rc                  (Default) Enable TX16S RC transmitter teleop and safety switches.
  --no-rc               Disable teleop; enable bypass_rc and esp32_trigger_node (use when
                        RC battery is dead or running headless).
  --no-cam              Disable USB camera drivers to save CPU/bandwidth on timed runs.
  --mock-auto           Enable land-test mock auto mode in teleop (spin thrusters on land).
  --no-record           Do not record the run. By default every real run is recorded to a
                        rosbag under $BANYU_BAG_DIR (~/banyu_bags) for review after the dive.
  --no-record-images    Record the run without compressed camera images (much smaller bag).

MAVROS is not started by this script: run ./launch_mavros.sh [--sim] in its own terminal
and keep it running across profile changes. --real refuses to launch without it.

Strategy Profile Presets (--profile <name>), sim gate-back times (ALT_HOLD, no GPS):
  default               : por_params.yaml                    (safe strafe box, ~37 s)
  safe                  : por_params_timed_safe.yaml         (safe strafe box, ~37 s)
  fast, fast-strafe     : por_params_timed_fast.yaml         (fast strafe box, ~32 s)
  fastest               : por_params_timed_fastest.yaml      (diamond, back up through gate, ~25 s)
  turn, fastest-turn    : por_params_timed_fastest_turn.yaml (diamond, 180 deg turn, ~28 s)
  vision                : por_params_vision_assisted.yaml    (diamond + camera corrections, ~36 s)
  hybrid                : por_params_hybrid.yaml             (strafe box + camera return, ~44 s)
  slow                  : por_params_timed_slow.yaml         (gentle first pool test, ~71 s)
  replay, teach         : por_params_replay.yaml             (fly a recorded manual drive, see below)
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
  --replay-track NAME   Track for replay profiles: a file name in $BANYU_REPLAY_DIR
                        (~/banyu_replays, .csv optional) or a path. Default: the newest track.
  -h, --help            Show this help message.

Teach & replay:
  Drive the course by hand (QGC joystick or TX16S, ALT_HOLD) during any recorded launch,
  cut the drive into a track, then fly it with a replay profile:
    python3 tools/scripts/por_teach_extract.py ~/banyu_bags/<run> --name <track>
    ./tools/scripts/launch_por.sh --real --profile replay --replay-track <track>

Examples:
  # Launch Gazebo simulation with GUI and fastest diamond strategy, MAVROS in a second terminal:
  ./tools/scripts/launch_por.sh --sim --profile fastest
  ./tools/scripts/launch_mavros.sh --sim

  # Real AUV (three terminals; only the last one is restarted to change profile):
  ./run_mavproxy.sh
  ./launch_mavros.sh
  ./launch_por.sh --real --no-rc --profile por_params_timed_safe_no_rc.yaml

  # Launch Gazebo simulation headless for CI / rapid verification:
  ./tools/scripts/launch_por.sh --sim --headless --profile safe

  # Launch real AUV with TX16S transmitter and the safe strafe box:
  ./tools/scripts/launch_por.sh --real --profile safe

  # Launch real AUV headless without transmitter:
  ./tools/scripts/launch_por.sh --real --no-rc --profile por_params_timed_safe_no_rc.yaml
  (every robot profile has a *_no_rc.yaml twin, also under blind/)

  # Replay the newest recorded manual drive without a transmitter:
  ./tools/scripts/launch_por.sh --real --no-rc --profile por_params_replay_no_rc.yaml
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
    --replay-track)
      REPLAY_TRACK="$2"
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
    --record)
      RECORD=true
      shift
      ;;
    --no-record)
      RECORD=false
      shift
      ;;
    --record-images)
      RECORD_IMAGES=true
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

# Every robot run is recorded (no link underwater); sim runs only when asked.
if [ -z "$RECORD" ]; then
  RECORD=$([ "$TARGET" = "real" ] && echo true || echo false)
fi
if [ -z "$RECORD_IMAGES" ]; then
  RECORD_IMAGES=$([ "$TARGET" = "real" ] && echo true || echo false)
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

mavros_count() { pgrep -x mavros_node | wc -l; }

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
    replay|teach) PROFILE="por_params_replay.yaml" ;;
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

# Replay profiles fly a teach track; pin it to an absolute path so the banner and the
# mission (which would otherwise resolve "latest" itself) use the same file.
REPLAY_TRACK_PATH=""
if grep -Eq '^[[:space:]]*use_replay:[[:space:]]*true' "$PROFILE_PATH"; then
  REPLAY_DIR="${BANYU_REPLAY_DIR:-$HOME/banyu_replays}"
  case "${REPLAY_TRACK:-latest}" in
    latest) REPLAY_TRACK_PATH="$(ls -t "$REPLAY_DIR"/*.csv 2>/dev/null | head -n 1 || true)" ;;
    */*) REPLAY_TRACK_PATH="$(readlink -f "$REPLAY_TRACK" || true)" ;;
    *.csv) REPLAY_TRACK_PATH="$REPLAY_DIR/$REPLAY_TRACK" ;;
    *) REPLAY_TRACK_PATH="$REPLAY_DIR/$REPLAY_TRACK.csv" ;;
  esac
  if [ -z "$REPLAY_TRACK_PATH" ] || [ ! -f "$REPLAY_TRACK_PATH" ]; then
    echo "Error: replay track '${REPLAY_TRACK:-latest}' not found in $REPLAY_DIR." >&2
    echo "  Make one with tools/scripts/por_teach_extract.py <bag run folder>." >&2
    [ "$DRY_RUN" = true ] || exit 1
    REPLAY_TRACK_PATH="${REPLAY_TRACK:-latest}"
  fi
elif [ -n "$REPLAY_TRACK" ]; then
  echo "Warning: --replay-track is ignored: the profile does not set use_replay: true." >&2
fi

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
if [ -n "$REPLAY_TRACK_PATH" ]; then
  if [ -f "$REPLAY_TRACK_PATH" ]; then
    echo "Replay Track      : ${REPLAY_TRACK_PATH} ($(awk -F, '!/^#/ && $1 ~ /^[0-9.]+$/ {
      t = $1; if ($6 > d) d = $6 } END {printf "%.1f s, max depth %.2f m", t, d}' "$REPLAY_TRACK_PATH"))"
  else
    echo "Replay Track      : MISSING (${REPLAY_TRACK_PATH})"
  fi
fi
if [ "$TARGET" = "sim" ]; then
  echo "Gazebo GUI        : $([ "$HEADLESS" = true ] && echo "HEADLESS (server only)" || echo "ENABLED")"
else
  echo "Transmitter Mode  : $([ "$USE_RC" = true ] && echo "RC (TX16S enabled)" || echo "No-RC (Headless / Bypass)")"
  echo "Cameras Enabled   : $ENABLE_CAMERAS"
  echo "Land Test Mock    : $MOCK_AUTO"
fi
if [ "$RECORD" = true ]; then
  BAG_ROOT="${BANYU_BAG_DIR:-$HOME/banyu_bags}"
  mkdir -p "$BAG_ROOT"
  BAG_FREE_MB="$(df -Pm "$BAG_ROOT" | awk 'NR==2 {print $4}')"
  echo "Mission Recording : ON -> ${BAG_ROOT} ($((BAG_FREE_MB / 1024)) GB free," \
    "images $([ "$RECORD_IMAGES" = true ] && echo "compressed" || echo "off"))"
  # Name the run folder after the profile; record_mission.sh reads these from the environment.
  PROFILE_NAME="$(basename "$PROFILE_PATH" .yaml)"
  export BANYU_BAG_LABEL="por_${PROFILE_NAME#por_params_}$([ "$TARGET" = "sim" ] && echo "_sim")"
  if [ -f "$REPLAY_TRACK_PATH" ]; then
    export BANYU_BAG_REPLAY_TRACK="$REPLAY_TRACK_PATH"
  fi
else
  echo "Mission Recording : OFF"
fi
echo "----------------------------------------------------------"

if [ "$TARGET" = "sim" ]; then
  # Simulation pre-flight checks and cleanup
  if [ "$CLEAN_SIM" = true ] && [ "$DRY_RUN" = false ]; then
    # mavros_node is left alone: it runs from launch_mavros.sh in its own terminal.
    echo "Pre-flight: Cleaning orphaned ArduSub / MAVProxy processes..."
    killall -9 ardusub mavproxy 2>/dev/null || true
    pkill -9 -f "sim_vehicle.py" 2>/dev/null || true
    sleep 0.5
  fi

  LAUNCH_CMD=(
    ros2 launch banyu_bringup main_launch.launch.py
    mode:=por
    por_params_file:="$PROFILE_PATH"
    headless:="$([ "$HEADLESS" = true ] && echo "True" || echo "False")"
  )

  if [ -n "$REPLAY_TRACK_PATH" ]; then
    LAUNCH_CMD+=(replay_track:="$REPLAY_TRACK_PATH")
  fi

  LAUNCH_CMD+=(
    record:="$([ "$RECORD" = true ] && echo "True" || echo "False")"
    record_images:="$([ "$RECORD_IMAGES" = true ] && echo "True" || echo "False")"
  )

  # The real AUV has no GPS; simulate the same unless the caller asks for use_gps:=True.
  if [[ " ${EXTRA_ARGS[*]} " != *" use_gps:="* ]]; then
    LAUNCH_CMD+=(use_gps:=False)
  fi

  if [ ${#EXTRA_ARGS[@]} -gt 0 ]; then
    LAUNCH_CMD+=("${EXTRA_ARGS[@]}")
  fi

  echo "Simulation Operator Instructions:"
  if [ "$(mavros_count)" -eq 0 ]; then
    echo "  0. Start MAVROS in another terminal: ./launch_mavros.sh --sim"
  else
    echo "  0. MAVROS already running ($(mavros_count) mavros_node); restart it if this sim restarts."
  fi
  echo "  1. Wait for Gazebo Harmonic and MAVROS to connect (pose at 20-50 Hz)."
  echo "  2. Verify link: ros2 topic echo /mavros/state --once"
  echo "  3. Start mission: ros2 service call /banyu/mission/start std_srvs/srv/Trigger \"{}\""
  echo "  4. Abort mission: ros2 service call /banyu/mission/abort std_srvs/srv/Trigger \"{}\""
  echo "  5. Teardown: killall -9 ardusub mavproxy; Ctrl+C in the launch_mavros.sh terminal"
  if [ "$RECORD" = true ]; then
    echo "  Recording to ${BAG_ROOT}/<date>_${BANYU_BAG_LABEL}/ (stop with Ctrl+C to close the bag)."
    echo "  Compare a replay with its teach run: tools/scripts/por_path_compare.py <teach run> <replay run>"
  fi

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

  # MAVROS runs on its own (launch_mavros.sh); the stack never starts it.
  MAVROS_RUNNING="$(mavros_count)"
  if [ "$MAVROS_RUNNING" -eq 1 ]; then
    echo "MAVROS            : RUNNING"
  elif [ "$MAVROS_RUNNING" -eq 0 ]; then
    echo "Error: MAVROS is not running. Start it in another terminal first: ./launch_mavros.sh" >&2
    [ "$DRY_RUN" = true ] || exit 1
  else
    echo "Warning: ${MAVROS_RUNNING} mavros_node processes are running (left over from old" \
      "launches?). Stop them all (pkill -x mavros_node) and start one with ./launch_mavros.sh." >&2
  fi

  echo "----------------------------------------------------------"

  LAUNCH_CMD=(
    ros2 launch banyu_bringup sub_bringup.launch.py
    mode:="$MODE"
    por_params_file:="$PROFILE_PATH"
    enable_cameras:="$([ "$ENABLE_CAMERAS" = true ] && echo "True" || echo "False")"
    enable_mock_auto:="$([ "$MOCK_AUTO" = true ] && echo "True" || echo "False")"
  )

  if [ -n "$REPLAY_TRACK_PATH" ]; then
    LAUNCH_CMD+=(replay_track:="$REPLAY_TRACK_PATH")
    echo "Replay: disable the QGC joystick before the start; its sticks would override the replay."
  fi

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
