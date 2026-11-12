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

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
WS_DIR="$(cd "${REPO_DIR}/../.." && pwd)"   # <ws>/src/BANYU_ROBOTX -> <ws>

CURRENT_BRANCH="$(git -C "${REPO_DIR}" rev-parse --abbrev-ref HEAD 2>/dev/null || echo "unknown")"

# --- Defaults (identical to the launch files) ---
TARGET="real"            # "real" or "sim" (same default as launch_por.sh)
HEADLESS=false
CLEAN_SIM=true
TIER="advanced"
GPS_GATE=false
GPS_LOSS_DEPTH="0.2"
GPS_REACQUIRE_DELAY="10.0"
USE_GPS=true
DRY_RUN=false
TIER_SET=false
GPS_GATE_OPTS_SET=false
LIGHT_SEED=""
SPAWN_SEED=""
SPAWN_XYZ=""
SENSORS="full"
GAMBLE_RED="2,3"
USE_HYDROPHONE=false
ALLOW_CHANNEL_OVERLAP=false
ALLOW_NO_GPS_BRIDGE=false
ALLOW_OTHER_DOMAIN=false
UUV_DOMAIN_ID=40          # RobotX SoS: the DDS Router forwards /system/* from this domain to the GCS (10)
NO_RC=false
DIVE_POINT="auto"
DIVE_POINT_SET=false
PIPE_DEPTH=""
TEST_MISSION=false
WAYPOINT_MISSION=false
AUTO_START=true
DEBUG=false
RECORD=""                # empty = the target's default: on for --real, off for --sim
RECORD_IMAGES=""
SCENARIO_ARGS=()
EXTRA_ARGS=()

print_usage() {
  cat << 'EOF'
Usage: ./tools/scripts/run/launch_task2.sh [TARGET] [OPTIONS] [ros2 launch args...]

Unified launcher for BANYU AUV RobotX Task 2 (mode:=task2).

Target Selection:
  --sim, --gazebo       Launch in Gazebo Harmonic simulation with SITL
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
  --site <name>         marina_bay: GPS home and course coordinates at Marina Bay, Singapore (default: Long Beach).
  --fisheye             Scenario: distort the bottom camera like a fisheye (the detector undistorts it).
  --ir-led              Scenario: the bottom camera's LED ring on (a spot light along the camera).
  --camera-width <px>   Bottom camera width (4:3; default 320x240; the real HORUS is 1920x1080).
  --light-fault <n:kind>
                        Scenario: functional node n shows a build-guide fault: thermistor (RED blink 500 ms), mag (RED blink
                        250 ms) or hot (LEDs off).
  --buoy-error <x> <y>  Scenario: the USV reports both buoys x m east and y m north of their real place.
  --current <e> <n> [<gust>]
                        Scenario: water current east and north (m/s), plus a gust along it for 5 s of every 30 s.
  --gps-gate            Simulate GPS loss/reacquire around depth (gps_gate:=True).
  --no-gps-gate         (Default) Do not gate GPS (gps_gate:=False).
  --gps-loss-depth <m>  Depth at which GPS is lost (default: 0.2; gps_loss_depth_m).
  --gps-reacquire-delay <s>
                        Seconds after surfacing before GPS returns (default: 10.0;
                        gps_reacquire_delay_s).
                        The gps-gate options are rejected with --real.

Both targets:
  --waypoint-mission, -w
                        Fly the Task 2 WAYPOINT test mission (task2_waypoint_mission.xml, waypoint_mission:=True).
                        Tests GPS waypoint navigation on the surface (midpoint strategy between USV buoys) and
                        inter-vehicle communication to UAV/GCS (PipelineSurvey + ResourceDelivery).
                        Auto-starts on flicking SB Up (also accepts GCS CMD_RUN_START).
                        No diving or underwater legs; stays safely on the surface.
  --test-mission        Fly the Task 2 TEST mission (task2_test_mission.xml, test_mission:=True) instead of the survey,
                        with the real midpoint strategy: after SB Up, the run start and the USV's two buoys (PipelineBuoys)
                        the AUV goes midway between the buoys by GPS (keeping clear of both), turns to face the end buoy,
                        dives to mission.test.depth_m (1 m), flies a timed forward leg (toward the end buoy) and a timed
                        backward leg holding that depth, surfaces and sends the Core survey report (all GREEN, no pipe).
                        With only the start buoy it dives inside it, like the real run. Tuning:
                        banyu_bringup/config/task2_test_mission.yaml. Start it with
                        ./tools/scripts/run/task2_test_waypoint.py <start LAT LON> --end <end LAT LON>.
                        Implies --dive-point midpoint; not with --sensors gamble.
  --no-auto-start       With --waypoint-mission: SB Up does not start the run (auto_start:=False). Start it like a real
                        run: SB Up, prepare_autonomy, then RoboCommand's Run Start (GCS CMD_RUN_START), so the reports
                        go to RoboCommand under its run. Example:
                          ./tools/scripts/run/launch_task2.sh --real -w --no-auto-start
  --tier <t>            (see above) On --real the default is core, like sub_bringup.launch.py.
  --sensors <s>         full (default: Bar30 depth, ALT_HOLD under water, dive to the fixed inspection depth)
                        | baro (Bar30 + ALT_HOLD, but the dive stops by camera at the pipe, pipe-width standoff,
                        dive point 1.5 m inside the start buoy, ring order dead-reckoned along the buoy line)
                        | gamble (Pixhawk, Bar30 and surface GPS: go midway between the buoys, short timed dive and
                        bobbing, surface, report the start buoy with a RANDOM light order; no camera, no pipe, no repair).
                        Every profile uses the Bar30 for depth; a Bar30 fault makes the depth invalid for the run.
                        Dives at the midpoint unless --dive-point buoy is given (--dive-point auto).
  --no-rc               --real: no transmitter. Teleop off, esp32_trigger_node on, bypass_rc:=True: the mode is
                        set in software instead of SB (MANUAL until the first command):
                          ros2 topic pub --once /banyu/trigger/cmd std_msgs/msg/String "{data: '2'}"   (= SB Up)
                          ... "{data: '1'}" = SB Mid (Pixhawk MANUAL, armed), "{data: '0'}" = SB Down
                          (Pixhawk MANUAL, disarmed); '2' leaves the Pixhawk mode to the mission.
                        Without RC there is no stick takeover: stop with '0', the abort service or the
                        physical kill switch.
  --allow-no-gps-bridge  --real: launch even though run_gps_bridge.sh (the GPS forwarder) is not running. Only for a
                        GPS wired to the Pixhawk (GPS1_TYPE 1); with GPS1_TYPE 14 the Pixhawk then has no GPS.
  --allow-other-domain  --real: launch with ROS_DOMAIN_ID other than 40 (bench test without the GCS). The DDS Router
                        only forwards /system/* from domain 40: the GCS then gets no heartbeat, status or survey.
  --allow-channel-overlap
                        --real: launch even if an rc_override channel of task2_params.yaml (or the sensors
                        overlay) is also a banyu_teleop switch channel (SB/SC). Unsafe: the switch then reads the
                        override.
  --dive-point <p>      auto (default: midpoint)
                        | buoy (dive inside the start buoy, at least mission.buoy_standoff_m from it, back up along
                        the pipe to its start, then survey forward)
                        | midpoint (navigate midway between the start and end buoys, dive facing the end buoy, back up
                        along the pipe to its start, then survey forward; needs both buoys from the USV).
  --hydrophone          Opt in to the hydrophone (use_hydrophone:=True): pinger_localization, the sim
                        HydrophoneArray plugin and pinger homing in SearchPipeline. Default OFF: the
                        competition UUV has no hydrophone, and no decision reads pinger data.
  --gamble-red <list>   --sensors gamble: where the RED goes, 1 = node closest to the active buoy, 4 = farthest.
                        "2,3" (default) = 2 or 3 at random; "3" = always 3; "1,2,3,4" = any.
  --use-gps             (Default) GPS available (use_gps:=True); Task 2 needs the surface
                        GPS leg (NavigateSurfaceGps).
  --no-gps              use_gps:=False.
  --record              --sim: record the run (sim time) to $BANYU_BAG_DIR (~/banyu_bags); off by default in sim.
  --record-images       --sim: also record compressed camera images (slows the sim).
  --no-record           --real: do not record. By default every real run is recorded (no link under water).
  --no-record-images    --real: record without compressed camera images (much smaller bag).
  --dry-run             Print the resolved launch command without launching or cleaning.
  -h, --help            Show this help message.

MAVROS and the GPS forwarder are not started by this script. Robot order, each in its own terminal:
run_mavproxy.sh -> run_gps_bridge.sh -> launch_mavros.sh -> launch_task2.sh --real
(--sim: this script first, then launch_mavros.sh --sim; the sim has its own GPS).

Any other argument is passed through to ros2 launch (e.g. task2_params_file:=/path.yaml).

Examples:
  ./tools/scripts/run/launch_task2.sh --sim
  ./tools/scripts/run/launch_task2.sh --sim --headless --tier disruptive --gps-gate --gps-loss-depth 0.2
  ./tools/scripts/run/launch_task2.sh --sim --sensors baro --pipe-depth 8
  ./tools/scripts/run/launch_task2.sh --sim --sensors baro --pipe-depth 3 --hydrophone
  ./tools/scripts/run/launch_task2.sh --real --dry-run
  ./tools/scripts/run/launch_task2.sh --real --test-mission
  ./tools/scripts/run/launch_task2.sh --sim --test-mission --record
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
    --site)
      [ $# -ge 2 ] || { echo "Error: --site needs a name." >&2; exit 1; }
      SCENARIO_ARGS+=(site:="$2"); GPS_GATE_OPTS_SET=true; shift 2 ;;
    --ir-led) SCENARIO_ARGS+=(ir_led:=on); GPS_GATE_OPTS_SET=true; shift ;;
    --camera-width)
      [ $# -ge 2 ] || { echo "Error: --camera-width needs pixels." >&2; exit 1; }
      SCENARIO_ARGS+=(camera_width:="$2"); GPS_GATE_OPTS_SET=true; shift 2 ;;
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
    --gamble-red)
      [ $# -ge 2 ] || { echo "Error: --gamble-red needs positions, e.g. 2,3 or 3." >&2; exit 1; }
      GAMBLE_RED="$2"; shift 2 ;;
    --sensors)
      [ $# -ge 2 ] || { echo "Error: --sensors needs a value (full|baro|gamble)." >&2; exit 1; }
      SENSORS="$2"; shift 2 ;;
    --hydrophone) USE_HYDROPHONE=true; shift ;;
    --dive-point)
      [ $# -ge 2 ] || { echo "Error: --dive-point needs a value (buoy|midpoint)." >&2; exit 1; }
      DIVE_POINT="$2"; DIVE_POINT_SET=true; shift 2 ;;
    --allow-channel-overlap) ALLOW_CHANNEL_OVERLAP=true; shift ;;
    --allow-no-gps-bridge) ALLOW_NO_GPS_BRIDGE=true; shift ;;
    --allow-other-domain) ALLOW_OTHER_DOMAIN=true; shift ;;
    --no-rc) NO_RC=true; shift ;;
    --test-mission) TEST_MISSION=true; shift ;;
    --waypoint-mission|-w) WAYPOINT_MISSION=true; shift ;;
    --no-auto-start) AUTO_START=false; shift ;;
    --debug) DEBUG=true; shift ;;
    --record) RECORD=true; shift ;;
    --no-record) RECORD=false; shift ;;
    --record-images) RECORD_IMAGES=true; shift ;;
    --no-record-images) RECORD_IMAGES=false; shift ;;
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
  full|baro|gamble) ;;
  *) echo "Error: --sensors must be full, baro or gamble (got '$SENSORS')." >&2; exit 1 ;;
esac
case "$DIVE_POINT" in
  auto|buoy|midpoint) ;;
  *) echo "Error: --dive-point must be auto, buoy or midpoint (got '$DIVE_POINT')." >&2; exit 1 ;;
esac
if [ "$TEST_MISSION" = true ] && [ "$WAYPOINT_MISSION" = true ]; then
  echo "Error: --test-mission cannot be combined with --waypoint-mission." >&2; exit 1
fi
if [ "$WAYPOINT_MISSION" = true ] && [ "$SENSORS" = gamble ]; then
  echo "Error: --waypoint-mission cannot be combined with --sensors gamble (both pick the mission tree)." >&2; exit 1
fi
if [ "$AUTO_START" = false ] && [ "$WAYPOINT_MISSION" != true ]; then
  echo "Error: --no-auto-start needs --waypoint-mission (no other mission starts itself on SB Up)." >&2; exit 1
fi
if [ "$TEST_MISSION" = true ] && [ "$SENSORS" = gamble ]; then
  echo "Error: --test-mission cannot be combined with --sensors gamble (both pick the mission tree)." >&2; exit 1
fi
# The test mission and waypoint mission fly the real midpoint strategy.
if [ "$WAYPOINT_MISSION" = true ]; then
  if [ "$DIVE_POINT_SET" = true ] && [ "$DIVE_POINT" != midpoint ]; then
    echo "Error: --waypoint-mission always navigates midway between the buoys (--dive-point midpoint)." >&2; exit 1
  fi
  DIVE_POINT="midpoint"
fi
if [ "$TEST_MISSION" = true ]; then
  if [ "$DIVE_POINT_SET" = true ] && [ "$DIVE_POINT" != midpoint ]; then
    echo "Error: --test-mission always dives midway between the buoys (--dive-point midpoint)." >&2; exit 1
  fi
  DIVE_POINT="midpoint"
fi
# auto, as in the launch files: midpoint (--dive-point buoy dives inside the start buoy instead).
[ "$DIVE_POINT" = auto ] && DIVE_POINT="midpoint"
# Recording defaults as in the launch files: every real run, sim runs only when asked.
if [ -z "$RECORD" ]; then
  [ "$TARGET" = real ] && RECORD=true || RECORD=false
fi
if [ -z "$RECORD_IMAGES" ]; then
  [ "$TARGET" = real ] && RECORD_IMAGES=true || RECORD_IMAGES=false
fi
if ! [[ "$GAMBLE_RED" =~ ^[1-4](,[1-4])*$ ]]; then
  echo "Error: --gamble-red must be positions 1-4, e.g. 2,3 or 3 (got '$GAMBLE_RED')." >&2; exit 1
fi
NUM_RE='^[0-9]+([.][0-9]+)?$'
if ! [[ "$GPS_LOSS_DEPTH" =~ $NUM_RE ]]; then
  echo "Error: --gps-loss-depth must be a non-negative number (got '$GPS_LOSS_DEPTH')." >&2; exit 1
fi
if ! [[ "$GPS_REACQUIRE_DELAY" =~ $NUM_RE ]]; then
  echo "Error: --gps-reacquire-delay must be a non-negative number (got '$GPS_REACQUIRE_DELAY')." >&2; exit 1
fi
if [ "$NO_RC" = true ] && [ "$TARGET" = sim ]; then
  echo "Error: --no-rc is for --real (the sim has no transmitter; its deadman stands in for it)." >&2; exit 1
fi
if [ "$TARGET" = "real" ]; then
  if [ "$GPS_GATE" = true ] || [ "$GPS_GATE_OPTS_SET" = true ]; then
    echo "Error: --gps-gate / --gps-loss-depth / --gps-reacquire-delay / --light-seed / --spawn* / --pipe-* / scenario options are simulation only" >&2
    echo "  (the GPS gate is a sim shim); they are refused with --real. Use --sim." >&2
    exit 1
  fi
  # /mavros/rc/in reports RC override values, so a teleop switch on an override channel reads the autonomy sticks
  # instead of the switch (neutral 1500 = SB Mid = MANUAL), as in launch_por.sh.
  OVERLAP="$(python3 - "${REPO_DIR}/banyu_bringup/config/task2_params.yaml" \
    "${REPO_DIR}/banyu_bringup/config/task2_sensors_${SENSORS}.yaml" \
    "${REPO_DIR}/banyu_teleop/config/teleop_rcin.yaml" << 'PYCHECK'
import sys
import yaml
rc = {}
for path in sys.argv[1:3]:
    doc = yaml.safe_load(open(path)) or {}
    rc.update(doc.get("vehicle_execution_node", {}).get("ros__parameters", {}).get("rc_override", {}) or {})
teleop = yaml.safe_load(open(sys.argv[3]))["banyu_teleop"]["ros__parameters"]
switches = {"SB (kill/arm)": teleop.get("kill_channel", 5) + 1,
            "SC (flight mode)": teleop.get("mode_channel", 7) + 1}
for name, ch in switches.items():
    for key in ("channel_forward", "channel_lateral", "channel_throttle", "channel_yaw"):
        if rc.get(key) == ch:
            print(f"{name} is on CH{ch}, the same channel as rc_override.{key}")
PYCHECK
)"
  if [ -n "$OVERLAP" ]; then
    echo "DANGER: RC channel overlap between task2_params.yaml and banyu_teleop:" >&2
    echo "$OVERLAP" | sed 's/^/  /' >&2
    if [ "$ALLOW_CHANNEL_OVERLAP" = false ]; then
      echo "  Refusing to launch. Fix RCMAP / teleop_rcin.yaml, or pass --allow-channel-overlap." >&2
      exit 1
    fi
    echo "  --allow-channel-overlap given: launching anyway." >&2
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

mavros_count() { pgrep -x mavros_node | wc -l; }

TF() { [ "$1" = true ] && echo "True" || echo "False"; }

echo "=========================================================="
echo " BANYU AUV — Task 2 Mission Launcher"
echo "=========================================================="
echo "Active Git Branch : $CURRENT_BRANCH"
echo "Target Platform   : $([ "$TARGET" = "sim" ] && echo "Gazebo Simulation (SITL)" || echo "Real AUV Physical Hardware")"
echo "use_gps           : $(TF $USE_GPS)"
echo "Tier              : $([ "$TARGET" = real ] && [ "$TIER_SET" != true ] && echo core || echo "$TIER")"
echo "Mission           : $([ "$WAYPOINT_MISSION" = true ] && echo "WAYPOINT (midpoint GPS transit, align heading, send reports to UAV/GCS; task2_waypoint_mission.yaml)" || ([ "$TEST_MISSION" = true ] && echo "TEST (midpoint by GPS, face the end buoy, 1 m dive, timed forward/backward legs, surface, Core report; task2_test_mission.yaml)" || echo "survey (banyu_mission.xml$([ "$SENSORS" = gamble ] && echo ", gamble tree"))"))"
[ "$WAYPOINT_MISSION" = true ] && echo "Run start         : $([ "$AUTO_START" = true ] && echo "SB Up starts the run itself (run 1; RoboCommand's Run Start is rejected; --no-auto-start)" || echo "prepare_autonomy + GCS CMD_RUN_START (RoboCommand's run); SB Up does not start it")"
[ "$TARGET" = real ] && echo "RC / mode switch   : $([ "$NO_RC" = true ] && echo "NO RC: mode from /banyu/trigger/cmd ('2' auto, '1' manual, '0' kill); MANUAL until set" || echo "TX16S, SB (teleop)")"
echo "Detector debug    : $([ "$DEBUG" = true ] && echo "ON (windows where there is a display; topics /banyu/pipeline/debug/*)" || echo "off (--debug)")"
echo "Sensors           : $SENSORS$([ "$SENSORS" = baro ] && echo " (Bar30 + ALT_HOLD; dive stops by camera at the pipe, order dead-reckoned)")$([ "$SENSORS" = gamble ] && echo " (GPS to the dive point, timed dive and bobbing, Bar30 surfacing, GUESSED light order, RED at position ${GAMBLE_RED}; no survey is done)")"
if [ "$WAYPOINT_MISSION" = true ]; then
  echo "Dive point        : midpoint (navigates midway between the buoys on surface; no dive)"
elif [ "$TEST_MISSION" = true ]; then
  echo "Dive point        : midpoint (midway between the buoys, facing the end buoy; start buoy only: inside it)"
elif [ "$SENSORS" = gamble ]; then
  echo "Dive point        : $([ "$DIVE_POINT" = midpoint ] && echo "midpoint (midway between the buoys; start buoy only: inside it)" || echo "buoy (inside the start buoy, standoff from it)"); timed dive in place, no pipe"
else
  echo "Dive point        : $([ "$DIVE_POINT" = midpoint ] && echo "midpoint (midway between the buoys)" || echo "buoy (inside the start buoy, standoff from it)"); both back up to the pipe's start, then survey forward"
fi
echo "Surface transit   : keeps clear of both buoys (mission.buoy_keep_out_m)"
echo "Task 4 zones      : RoboCommand keep-out zones / moving object are avoided on the surface (docs/task4.md; tools/scripts/run/task4_command.py)"
echo "Hydrophone        : $([ "$USE_HYDROPHONE" = true ] && echo "ON (opt-in; pinger data used by SearchPipeline)" || echo "OFF (none fitted; start buoy from the USV PipelineBuoys only)")"
if [ "$TARGET" = real ] && [ "$USE_HYDROPHONE" = true ]; then
  echo "Warning: --hydrophone on --real: the competition UUV has no hydrophone; nothing will publish pings." >&2
fi
if [ "$TARGET" = "sim" ]; then
  [ -n "$PIPE_DEPTH" ] && echo "Pipe depth        : light nodes at ${PIPE_DEPTH} m"
  [ ${#SCENARIO_ARGS[@]} -gt 0 ] && echo "Scenario          : ${SCENARIO_ARGS[*]}"
  echo "Gazebo GUI        : $([ "$HEADLESS" = true ] && echo "HEADLESS (server only)" || echo "ENABLED")"
  echo "GPS gate          : $(TF $GPS_GATE) (loss depth ${GPS_LOSS_DEPTH} m, reacquire delay ${GPS_REACQUIRE_DELAY} s)"
fi
echo "----------------------------------------------------------"

# Name the run folder and save the overlay profiles the run loads next to the bag (record_mission.sh reads these).
EFFECTIVE_TIER="$([ "$TARGET" = real ] && [ "$TIER_SET" != true ] && echo core || echo "$TIER")"
CONFIG_DIR="${REPO_DIR}/banyu_bringup/config"
export BANYU_BAG_LABEL="task2_$([ "$WAYPOINT_MISSION" = true ] && echo waypoint || ([ "$TEST_MISSION" = true ] && echo test || echo "$EFFECTIVE_TIER"))_${SENSORS}$([ "$TARGET" = sim ] && echo _sim)"
BAG_OVERLAYS=("${CONFIG_DIR}/task2_tier_${EFFECTIVE_TIER}.yaml" "${CONFIG_DIR}/task2_sensors_${SENSORS}.yaml")
[ "$TARGET" = real ] && BAG_OVERLAYS+=("${CONFIG_DIR}/hardware_params.yaml")
[ "$DIVE_POINT" = midpoint ] && BAG_OVERLAYS+=("${CONFIG_DIR}/task2_dive_midpoint.yaml")
[ "$USE_HYDROPHONE" = true ] && BAG_OVERLAYS+=("${CONFIG_DIR}/task2_hydrophone_on.yaml")
[ "$TEST_MISSION" = true ] && BAG_OVERLAYS+=("${CONFIG_DIR}/task2_test_mission.yaml")
[ "$WAYPOINT_MISSION" = true ] && BAG_OVERLAYS+=("${CONFIG_DIR}/task2_waypoint_mission.yaml")
[ "$AUTO_START" = false ] && BAG_OVERLAYS+=("${CONFIG_DIR}/task2_no_auto_start.yaml")
export BANYU_BAG_EXTRA_PARAMS="$(IFS=:; echo "${BAG_OVERLAYS[*]}")"
if [ "$RECORD" = true ]; then
  BAG_ROOT="${BANYU_BAG_DIR:-$HOME/banyu_bags}"
  mkdir -p "$BAG_ROOT"
  BAG_FREE_MB="$(df -Pm "$BAG_ROOT" | awk 'NR==2 {print $4}')"
  echo "Mission Recording : ON -> ${BAG_ROOT}/<date>_${BANYU_BAG_LABEL}/ ($((BAG_FREE_MB / 1024)) GB free," \
    "images $([ "$RECORD_IMAGES" = true ] && echo compressed || echo none))"
  if [ "$BAG_FREE_MB" -lt "${BANYU_BAG_MIN_FREE:-2048}" ]; then
    echo "Warning: only ${BAG_FREE_MB} MB free in ${BAG_ROOT}; the recorder needs" \
      "${BANYU_BAG_MIN_FREE:-2048} MB and will NOT record this run." >&2
  fi
else
  echo "Mission Recording : OFF$([ "$TARGET" = real ] && echo " (--no-record: nothing of the dive is kept!)")"
fi

ENV_PREFIX=()
if [ "$TARGET" = "sim" ]; then
  if [ "$CLEAN_SIM" = true ] && [ "$DRY_RUN" = false ]; then
    # mavros_node is left alone: it runs from launch_mavros.sh in its own terminal.
    echo "Pre-flight: Cleaning orphaned ArduSub / MAVProxy processes..."
    killall -9 ardusub mavproxy 2>/dev/null || true
    pkill -9 -f "sim_vehicle.py" 2>/dev/null || true
    sleep 0.5
  fi

  # Known issues: SITL's terminal spawner must use bash, and a surrounding tmux session
  # breaks the sim_vehicle.py console/map windows. Bind gz-transport to loopback so a
  # laptop network change cannot stall /clock via multicast errors.
  export SITL_RITW_TERMINAL=bash
  export GZ_IP="${GZ_IP:-127.0.0.1}"
  unset TMUX
  ENV_PREFIX+=(SITL_RITW_TERMINAL=bash GZ_IP="$GZ_IP")

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
    use_hydrophone:="$(TF $USE_HYDROPHONE)"
    dive_point:="$DIVE_POINT"
    test_mission:="$(TF $TEST_MISSION)"
    waypoint_mission:="$(TF $WAYPOINT_MISSION)"
    auto_start:="$(TF $AUTO_START)"
    debug:="$(TF $DEBUG)"
    debug_topics:="$(TF $DEBUG)"
    gamble_red:="$GAMBLE_RED"
    record:="$(TF $RECORD)"
    record_images:="$(TF $RECORD_IMAGES)"
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
  if [ "$(mavros_count)" -eq 0 ]; then
    echo "  0. Start MAVROS in another terminal: ./tools/scripts/run/launch_mavros.sh --sim"
  else
    echo "  0. MAVROS already running ($(mavros_count) mavros_node); restart it if this sim restarts."
  fi
  echo "  1. Wait for Gazebo Harmonic and MAVROS to connect (pose at 20-50 Hz)."
  echo "  2. Verify link: ros2 topic echo /mavros/state --once"
  if [ "$WAYPOINT_MISSION" = true ]; then
  echo "  3. WAYPOINT test mission: course, prepare_autonomy, CMD_RUN_START in one command (the sim provider sends the"
  echo "     world's two buoys like the USV, so give no LAT LON; the sim has no SB switch):"
  echo "       ROS_DOMAIN_ID=40 ./tools/scripts/run/task2_test_waypoint.py --course-center 33.810313 -118.393866"
  echo "  4. It goes midway between the buoys on the surface, aligns heading, and sends PipelineSurvey + ResourceDelivery (to UAV)."
  elif [ "$TEST_MISSION" = true ]; then
  echo "  3. TEST mission: course, prepare_autonomy, CMD_RUN_START in one command (the sim provider sends the"
  echo "     world's two buoys like the USV, so give no LAT LON; the sim has no SB switch):"
  echo "       ROS_DOMAIN_ID=40 ./tools/scripts/run/task2_test_waypoint.py --course-center 33.810313 -118.393866"
  echo "  4. It goes midway between the buoys, faces the end buoy, dives to mission.test.depth_m, flies forward"
  echo "     then backward, surfaces and reports; the tool prints the PipelineSurvey."
  else
  echo "  3. Prepare autonomy: ros2 service call /banyu/mission/prepare_autonomy std_srvs/srv/Trigger \"{}\""
  echo "  4. The run does NOT start from /banyu/mission/start. It starts only when the GCS sends"
  echo "     CMD_RUN_START on /system/mission/command."
  echo "     After the start the UUV stands by on the surface until the USV's PipelineBuoys gives the active"
  echo "     buoy (no fallback to the YAML coordinates); abort to stop."
  fi
  echo "     Hydrophone $([ "$USE_HYDROPHONE" = true ] && echo ON || echo "OFF: /banyu/pinger/observation has no publisher")."
  [ "$DEBUG" = true ] && echo "     Detector view: windows, or ros2 run rqt_image_view rqt_image_view /banyu/pipeline/debug/annotated"
  echo "  5. UUV side runs on ROS_DOMAIN_ID 40 (the GCS side uses its own domain)."
  echo "  6. Teardown: killall -9 ardusub mavproxy; Ctrl+C in the launch_mavros.sh terminal"
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
    echo "  Run first: ./src/BANYU_ROBOTX/tools/scripts/run/run_mavproxy.sh"
  fi

  # Device names come from tools/udev/99-banyu.rules (docs/udev_rules.md).
  for DEV in esp32_kill:"ESP32 Kill Switch" gps_rtk:"RTK GPS (F9P)    " bottom_camera:"Bottom Camera    "; do
    NAME="${DEV%%:*}"; LABEL="${DEV#*:}"
    if [ -e "/dev/${NAME}" ]; then
      echo "${LABEL} : OK (/dev/${NAME} -> $(readlink -f "/dev/${NAME}"))"
    else
      echo "Warning: /dev/${NAME} not found (udev rule installed?)." >&2
    fi
  done
  # The GPS forwarder runs on its own (run_gps_bridge.sh), like MAVProxy and MAVROS: with GPS1_TYPE 14
  # the Pixhawk's only GPS is its GPS_INPUT.
  if pgrep -f "python[0-9.]* [^ ]*gps_forwarder[.]py" > /dev/null; then
    echo "GPS Forwarder     : RUNNING (run_gps_bridge.sh)"
  elif [ "$ALLOW_NO_GPS_BRIDGE" = true ]; then
    echo "GPS Forwarder     : not running (--allow-no-gps-bridge: GPS must be wired to the Pixhawk, GPS1_TYPE 1)"
  else
    echo "Error: the GPS forwarder is not running. Start it in another terminal first:" \
      "./tools/scripts/run/run_gps_bridge.sh (or --allow-no-gps-bridge for GPS1_TYPE 1)" >&2
    [ "$DRY_RUN" = true ] || exit 1
  fi
  echo "GPS fix           : check with ./tools/scripts/check/rtk_status.sh before the start"

  # Heartbeat, MissionStatus and the survey reach the GCS only from the UUV domain; MAVROS on another domain than
  # the stack leaves the heartbeat without /mavros/state (STATE_KILLED).
  if [ "${ROS_DOMAIN_ID:-0}" = "$UUV_DOMAIN_ID" ]; then
    echo "ROS_DOMAIN_ID     : ${UUV_DOMAIN_ID} (UUV; /system/* goes to the GCS through the DDS Router)"
  elif [ "$ALLOW_OTHER_DOMAIN" = true ]; then
    echo "ROS_DOMAIN_ID     : ${ROS_DOMAIN_ID:-0 (unset)} (--allow-other-domain: NOTHING reaches the GCS)"
  else
    echo "Error: ROS_DOMAIN_ID is ${ROS_DOMAIN_ID:-unset}, not ${UUV_DOMAIN_ID}: the GCS would get no heartbeat," \
      "status or survey. export ROS_DOMAIN_ID=${UUV_DOMAIN_ID} here and in the MAVROS terminal" \
      "(or --allow-other-domain for a bench test)." >&2
    [ "$DRY_RUN" = true ] || exit 1
  fi
  for PID in $(pgrep -x mavros_node); do
    MAVROS_DOMAIN="$(tr '\0' '\n' < "/proc/${PID}/environ" 2>/dev/null | sed -n 's/^ROS_DOMAIN_ID=//p')"
    if [ "${MAVROS_DOMAIN:-0}" != "${ROS_DOMAIN_ID:-0}" ]; then
      echo "Error: mavros_node (pid ${PID}) runs on ROS_DOMAIN_ID ${MAVROS_DOMAIN:-0}, this stack on" \
        "${ROS_DOMAIN_ID:-0}: the mission would see no vehicle. Restart launch_mavros.sh with the same domain." >&2
      [ "$DRY_RUN" = true ] || exit 1
    fi
  done

  # MAVROS runs on its own (launch_mavros.sh); the stack never starts it.
  MAVROS_RUNNING="$(mavros_count)"
  if [ "$MAVROS_RUNNING" -eq 1 ]; then
    echo "MAVROS            : RUNNING"
  elif [ "$MAVROS_RUNNING" -eq 0 ]; then
    echo "Error: MAVROS is not running. Start it in another terminal first: ./tools/scripts/run/launch_mavros.sh" >&2
    [ "$DRY_RUN" = true ] || exit 1
  else
    echo "Warning: ${MAVROS_RUNNING} mavros_node processes are running (left over from old" \
      "launches?). Stop them all (pkill -x mavros_node) and start one with launch_mavros.sh." >&2
  fi
  echo "----------------------------------------------------------"

  LAUNCH_CMD=(
    ros2 launch banyu_bringup sub_bringup.launch.py
    mode:=task2
    tier:="$([ "$TIER_SET" = true ] && echo "$TIER" || echo core)"
    sensors:="$SENSORS"
    use_hydrophone:="$(TF $USE_HYDROPHONE)"
    dive_point:="$DIVE_POINT"
    test_mission:="$(TF $TEST_MISSION)"
    waypoint_mission:="$(TF $WAYPOINT_MISSION)"
    auto_start:="$(TF $AUTO_START)"
    debug:="$(TF $DEBUG)"
    debug_topics:="$(TF $DEBUG)"
    gamble_red:="$GAMBLE_RED"
    use_gps:="$(TF $USE_GPS)"
    record:="$(TF $RECORD)"
    record_images:="$(TF $RECORD_IMAGES)"
    enable_teleop:="$([ "$NO_RC" = true ] && echo False || echo True)"
    enable_esp32_trigger:="$(TF $NO_RC)"
    bypass_rc:="$(TF $NO_RC)"
  )
  [ ${#EXTRA_ARGS[@]} -gt 0 ] && LAUNCH_CMD+=("${EXTRA_ARGS[@]}")

  echo "Operator Instructions (Task 2, real AUV):"
  if [ "$NO_RC" = true ]; then
  echo "  NO RC: wherever these steps say SB, use (ROS_DOMAIN_ID=40):"
  echo "    SB Up   : ros2 topic pub --once /banyu/trigger/cmd std_msgs/msg/String \"{data: '2'}\""
  echo "    SB Mid  : ... \"{data: '1'}\" (Pixhawk MANUAL, armed)    SB Down: ... \"{data: '0'}\" (MANUAL, disarmed)"
  echo "    No stick takeover without RC. Physical kill switch: send '0' to acknowledge before '2' again."
  fi
  echo "  1. UUV side runs on ROS_DOMAIN_ID 40 (export ROS_DOMAIN_ID=40 in this shell and in"
  echo "     any shell you use to call services); the GCS side uses its own domain."
  if [ "$WAYPOINT_MISSION" = true ]; then
  echo "  2. WAYPOINT test mission:"
  if [ "$AUTO_START" = false ]; then
  echo "     - No auto-start: flick SB Up (AUTO), then ros2 service call /banyu/mission/prepare_autonomy std_srvs/srv/Trigger \"{}\""
  echo "       and wait for READY; the run starts on RoboCommand's Run Start (GCS CMD_RUN_START). Buoys without the USV:"
  echo "       ROS_DOMAIN_ID=40 ./tools/scripts/run/task2_test_waypoint.py <start LAT> <LON> --end <end LAT> <LON> --no-start --no-course"
  fi
  echo "     - $([ "$AUTO_START" = false ] && echo "After the start" || echo "Flick SB Up (AUTO): starts automatically,") waits for USV buoys, transits midway"
  echo "       between buoys on the surface, aligns heading, and sends PipelineSurvey + ResourceDelivery (to UAV)."
  echo "     - Or send buoys/start via script: ROS_DOMAIN_ID=40 ./tools/scripts/run/task2_test_waypoint.py <start LAT> <LON> --end <end LAT> <LON>"
  echo "     - SB Mid/Down at any time takes over (MANUAL/KILL)."
  elif [ "$TEST_MISSION" = true ]; then
  echo "  2. TEST mission: flick SB Up (AUTO). Then send the buoys; this also publishes the course, calls"
  echo "     prepare_autonomy and sends CMD_RUN_START (as the GCS would), and the buoys go out as the USV's"
  echo "     PipelineBuoys (start = active):"
  echo "       ROS_DOMAIN_ID=40 ./tools/scripts/run/task2_test_waypoint.py <start LAT> <LON> --end <end LAT> <LON>"
  echo "     (real GCS/USV on the link? add --no-course / --no-start, or leave out the buoys)"
  echo "  3. The AUV goes midway between the buoys by GPS, faces the end buoy, dives to mission.test.depth_m,"
  echo "     flies the timed forward and backward legs, surfaces and sends the Core PipelineSurvey; the tool"
  echo "     prints it when the link is back."
  echo "     SB Mid/Down at any time takes over (MANUAL/KILL)."
  else
  echo "  2. Prepare autonomy: ros2 service call /banyu/mission/prepare_autonomy std_srvs/srv/Trigger \"{}\""
  echo "  3. The run does NOT start from /banyu/mission/start. It starts only when the GCS sends"
  echo "     CMD_RUN_START on /system/mission/command."
  echo "  4. After the start the UUV stands by on the surface until the USV's PipelineBuoys gives the active"
  echo "     buoy (no fallback to the YAML coordinates). Not coming? Abort:"
  echo "     There is no hydrophone: the active buoy comes only from PipelineBuoys (GREEN indicator)."
  fi
  [ "$DEBUG" = true ] && echo "  Detector view: ROS_DOMAIN_ID=40 ros2 run rqt_image_view rqt_image_view /banyu/pipeline/debug/annotated"
  echo "  After the run: the bag is in ${BANYU_BAG_DIR:-$HOME/banyu_bags}/<date>_${BANYU_BAG_LABEL}/ (params/ = the profiles)."
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
