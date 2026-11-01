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
LIGHT_SEED=""
SPAWN_SEED=""
SPAWN_XYZ=""
SENSORS="full"
PIPE_DEPTH=""
SCENARIO_ARGS=()
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
  --tier <t>            Task 2 tier: core | advanced | disruptive (sim default: advanced).
  --light-seed <n>      Pick the damaged pipeline node and flash colours (seeds 2, 1, 5, 9 = node 1, 2, 3, 4;
                        default: node 2, tin RED, circle BLUE).
  --spawn-seed <n>      Spawn the UUV at a random place 10-20 m from the world origin (the GPS home, midway between the two
                        buoys) with a random heading, picked from this seed, so it has to navigate to the start buoy by GPS.
  --spawn <x> <y> <yaw> Spawn the UUV at x east, y north (m from the origin) with heading yaw (rad).
  --pipe-depth <m>      Put the pipeline's light nodes this deep (1 to 13.4 m; default: the world's 13.4 m).
  --pipe-move <dx> <dy> <dyaw>
                        Scenario: move the pipe against the buoys: rotate about its start end by dyaw deg, shift dx east,
                        dy north (m).
  --visibility <m>      Scenario: water visibility in metres (turbidity).
  --fcu-faults <list>   Scenario: FCU sensor faults, comma separated: compass=<deg> (heading off), gps_offset=<e>:<n> (RTK
                        float, m), gps_drop=<t>:<d> (no fix for d s, t s after start).
  --fisheye             Scenario: distort the bottom camera like a fisheye (the detector undistorts it).
  --light-fault <n:kind>
                        Scenario: functional node n shows a build-guide fault: thermistor (RED blink 500 ms), mag (RED blink
                        250 ms) or hot (LEDs off).
  --buoy-error <x> <y>  Scenario: the USV reports both buoys x m east and y m north of their real place.
  --current <e> <n> [<gust>]
                        Scenario: water current east and north (m/s), plus a gust along it for 5 s of every 30 s.
  --gps-gate            Simulate GPS loss/reacquire around depth (gps_gate:=True).
  --no-gps-gate         (Default) Do not gate GPS (gps_gate:=False).
  --gps-loss-depth <m>  Depth at which GPS is lost (default: 0.1; gps_loss_depth_m).
  --gps-reacquire-delay <s>
                        Seconds after surfacing before GPS returns (default: 10.0;
                        gps_reacquire_delay_s).
                        The gps-gate options are rejected with --real.

Both targets:
  --tier <t>            (see above) On --real the default is core, like sub_bringup.launch.py.
  --sensors <s>         full (default: Bar30 depth, ALT_HOLD under water) | minimal (bottom camera, Pixhawk
                        IMU/compass and RTK GPS at the surface only: STABILIZE under water, heave from the camera,
                        surface proven by a GPS fix; no depth sensor is used for any decision).
  --use-gps             (Default) GPS available (use_gps:=True); Task 2 needs the surface
                        GPS leg (NavigateSurfaceGps).
  --no-gps              use_gps:=False.
  --dry-run             Print the resolved launch command without launching or cleaning.
  -h, --help            Show this help message.

Any other argument is passed through to ros2 launch (e.g. task2_params_file:=/path.yaml).

Examples:
  ./tools/scripts/launch_task2.sh --sim
  ./tools/scripts/launch_task2.sh --sim --headless --tier disruptive --gps-gate --gps-loss-depth 0.2
  ./tools/scripts/launch_task2.sh --sim --sensors minimal --pipe-depth 8
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
    --light-seed)
      [ $# -ge 2 ] || { echo "Error: --light-seed needs a value." >&2; exit 1; }
      LIGHT_SEED="$2"; GPS_GATE_OPTS_SET=true; shift 2 ;;
    --spawn-seed)
      [ $# -ge 2 ] || { echo "Error: --spawn-seed needs a value." >&2; exit 1; }
      SPAWN_SEED="$2"; GPS_GATE_OPTS_SET=true; shift 2 ;;
    --spawn)
      [ $# -ge 4 ] || { echo "Error: --spawn needs x y yaw." >&2; exit 1; }
      SPAWN_XYZ="$2 $3 $4"; GPS_GATE_OPTS_SET=true; shift 4 ;;
    --pipe-depth)
      [ $# -ge 2 ] || { echo "Error: --pipe-depth needs a value in metres." >&2; exit 1; }
      PIPE_DEPTH="$2"; GPS_GATE_OPTS_SET=true; shift 2 ;;
    --pipe-move)
      [ $# -ge 4 ] || { echo "Error: --pipe-move needs dx dy dyaw." >&2; exit 1; }
      SCENARIO_ARGS+=(pipe_dx:="$2" pipe_dy:="$3" pipe_dyaw_deg:="$4"); GPS_GATE_OPTS_SET=true; shift 4 ;;
    --fcu-faults)
      [ $# -ge 2 ] || { echo "Error: --fcu-faults needs a list." >&2; exit 1; }
      SCENARIO_ARGS+=(fcu_faults:="$2"); GPS_GATE_OPTS_SET=true; shift 2 ;;
    --fisheye) SCENARIO_ARGS+=(camera_model:=fisheye); GPS_GATE_OPTS_SET=true; shift ;;
    --light-fault)
      [ $# -ge 2 ] || { echo "Error: --light-fault needs node:kind." >&2; exit 1; }
      SCENARIO_ARGS+=(light_fault:="$2"); GPS_GATE_OPTS_SET=true; shift 2 ;;
    --visibility)
      [ $# -ge 2 ] || { echo "Error: --visibility needs metres." >&2; exit 1; }
      SCENARIO_ARGS+=(visibility_m:="$2"); GPS_GATE_OPTS_SET=true; shift 2 ;;
    --buoy-error)
      [ $# -ge 3 ] || { echo "Error: --buoy-error needs x y." >&2; exit 1; }
      SCENARIO_ARGS+=(buoy_error_x:="$2" buoy_error_y:="$3"); GPS_GATE_OPTS_SET=true; shift 3 ;;
    --current)
      [ $# -ge 3 ] || { echo "Error: --current needs east north [gust]." >&2; exit 1; }
      SCENARIO_ARGS+=(current_east:="$2" current_north:="$3"); GPS_GATE_OPTS_SET=true; shift 3
      if [[ $# -ge 1 && "$1" =~ ^[0-9.]+$ ]]; then SCENARIO_ARGS+=(current_gust:="$1"); shift; fi ;;
    --sensors)
      [ $# -ge 2 ] || { echo "Error: --sensors needs a value (full|minimal)." >&2; exit 1; }
      SENSORS="$2"; shift 2 ;;
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
case "$SENSORS" in
  full|minimal) ;;
  *) echo "Error: --sensors must be full or minimal (got '$SENSORS')." >&2; exit 1 ;;
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
    echo "Error: --gps-gate / --gps-loss-depth / --gps-reacquire-delay / --light-seed / --spawn* / --pipe-* / scenario options are simulation only" >&2
    echo "  (the GPS gate is a sim shim); they are refused with --real. Use --sim." >&2
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
echo "Tier              : $([ "$TARGET" = real ] && [ "$TIER_SET" != true ] && echo core || echo "$TIER")"
echo "Sensors           : $SENSORS$([ "$SENSORS" = minimal ] && echo " (camera + IMU/compass + surface GPS; STABILIZE under water, no depth sensor)")"
if [ "$TARGET" = "sim" ]; then
  [ -n "$PIPE_DEPTH" ] && echo "Pipe depth        : light nodes at ${PIPE_DEPTH} m"
  [ ${#SCENARIO_ARGS[@]} -gt 0 ] && echo "Scenario          : ${SCENARIO_ARGS[*]}"
  echo "Gazebo GUI        : $([ "$HEADLESS" = true ] && echo "HEADLESS (server only)" || echo "ENABLED")"
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
    sensors:="$SENSORS"
    headless:="$(TF $HEADLESS)"
    use_gps:="$(TF $USE_GPS)"
    gps_gate:="$(TF $GPS_GATE)"
    gps_loss_depth_m:="$GPS_LOSS_DEPTH"
    gps_reacquire_delay_s:="$GPS_REACQUIRE_DELAY"
  )
  [ -n "$LIGHT_SEED" ] && LAUNCH_CMD+=(light_seed:="$LIGHT_SEED")
  [ -n "$PIPE_DEPTH" ] && LAUNCH_CMD+=(pipe_depth:="$PIPE_DEPTH")
  [ ${#SCENARIO_ARGS[@]} -gt 0 ] && LAUNCH_CMD+=("${SCENARIO_ARGS[@]}")
  if [ -n "$SPAWN_SEED" ]; then
    # Random spawn: 10-20 m from the origin, random bearing and heading, from this seed (the buoys are 3.8 m either side).
    SPAWN_XYZ="$(python3 -c "
import math, random, sys
r = random.Random(int(sys.argv[1]))
d, b = r.uniform(10.0, 20.0), r.uniform(-math.pi, math.pi)
print('%.2f %.2f %.3f' % (d * math.cos(b), d * math.sin(b), r.uniform(-math.pi, math.pi)))" "$SPAWN_SEED")"
  fi
  if [ -n "$SPAWN_XYZ" ]; then
    read -r SX SY SYAW <<< "$SPAWN_XYZ"
    LAUNCH_CMD+=(spawn_x:="$SX" spawn_y:="$SY" spawn_yaw:="$SYAW")
    echo "Spawn             : x ${SX} m east, y ${SY} m north, heading ${SYAW} rad"
  fi
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
    tier:="$([ "$TIER_SET" = true ] && echo "$TIER" || echo core)"
    sensors:="$SENSORS"
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
