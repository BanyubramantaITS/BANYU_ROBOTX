#!/bin/bash
# ==============================================================================
# launch_mavros.sh — start MAVROS on its own (no launch file starts it)
# ==============================================================================
# Real AUV, three terminals:
#   1. ./run_mavproxy.sh
#   2. ./launch_mavros.sh                 (leave running)
#   3. ./launch_por.sh --real --profile <profile>   (Ctrl+C and relaunch per profile)
# Simulation: ./launch_por.sh --sim ..., then ./launch_mavros.sh --sim
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
WS_DIR="$(cd "${REPO_DIR}/../.." && pwd)"
MAVROS_CONFIG="${REPO_DIR}/banyu_bringup/config/mavros_config.yaml"

TARGET="real"
FCU_URL="udp://127.0.0.1:14551@"
DRY_RUN=false

print_usage() {
  cat << 'EOF'
Usage: ./tools/scripts/launch_mavros.sh [--real | --sim] [OPTIONS]

Starts MAVROS (mavros_node with banyu_bringup/config/mavros_config.yaml) in this terminal.
The launch files no longer start MAVROS, so start it once and keep it running while the
mission stack is relaunched (launch_por.sh) for each profile.

Target:
  --real, --hardware  (Default) Wall time, FCU via run_mavproxy.sh on 127.0.0.1:14551.
  --sim, --gazebo     Sim time (Gazebo /clock), FCU via the SITL MAVProxy on 127.0.0.1:14551.
                      Restart MAVROS when the simulation is restarted (the clock resets).

Options:
  --fcu-url URL       FCU connection (default udp://127.0.0.1:14551@), e.g.
                      /dev/ttyACM0:921600 for a direct USB link without MAVProxy.
  --dry-run           Print the command without starting MAVROS.
  -h, --help          Show this help message.

Stop with Ctrl+C. Leftover instances: pgrep -af mavros_node / pkill -x mavros_node
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --real|--hardware) TARGET="real"; shift ;;
    --sim|--gazebo) TARGET="sim"; shift ;;
    --fcu-url) FCU_URL="$2"; shift 2 ;;
    --dry-run) DRY_RUN=true; shift ;;
    -h|--help) print_usage; exit 0 ;;
    *) echo "Error: unknown option $1 (see --help)" >&2; exit 1 ;;
  esac
done

if [ -z "$ROS_DISTRO" ] || ! command -v ros2 &> /dev/null; then
  if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
  fi
fi
if [ -f "${WS_DIR}/install/setup.bash" ]; then
  source "${WS_DIR}/install/setup.bash"
fi

MAVROS_PREFIX="$(ros2 pkg prefix mavros 2>/dev/null || true)"
if [ -z "$MAVROS_PREFIX" ]; then
  echo "Error: the mavros package is not installed (apt install ros-jazzy-mavros)." >&2
  exit 1
fi
USE_SIM_TIME=$([ "$TARGET" = "sim" ] && echo true || echo false)

echo "=========================================================="
echo " BANYU AUV — MAVROS (keep this terminal open)"
echo "=========================================================="
echo "Target            : $([ "$TARGET" = "sim" ] && echo "Simulation (sim time)" || echo "Real AUV (wall time)")"
echo "FCU URL           : $FCU_URL"
echo "ROS_DOMAIN_ID     : ${ROS_DOMAIN_ID:-0 (unset)}"
if [[ "$FCU_URL" == udp://* ]]; then
  if pgrep -f "mavproxy" > /dev/null; then
    echo "MAVProxy          : RUNNING"
  elif [ "$TARGET" = "real" ]; then
    echo "Warning: MAVProxy is not running. Start it first: ./run_mavproxy.sh" >&2
  else
    echo "MAVProxy          : not running yet (MAVROS waits for the SITL)"
  fi
fi
# Two instances would fight over the 14551 link and double every topic.
RUNNING="$(pgrep -x mavros_node | wc -l)"
if [ "$RUNNING" -gt 0 ]; then
  echo "Error: MAVROS is already running (${RUNNING} mavros_node). Stop it first:" \
    "pkill -x mavros_node" >&2
  [ "$DRY_RUN" = true ] || exit 1
fi

# Run mavros_node directly: a launch Node action injects "-r __node:=mavros", which crashes
# MAVROS (mavlink/mavros#1977), and `ros2 run` would orphan it on Ctrl+C.
MAVROS_CMD=(
  "${MAVROS_PREFIX}/lib/mavros/mavros_node"
  --ros-args
  --params-file "$MAVROS_CONFIG"
  -p "fcu_url:=${FCU_URL}"
  -p "use_sim_time:=${USE_SIM_TIME}"
  --log-level mavros.param:=error
  --log-level mavros.rallypoint:=error
  --log-level mavros.geofence:=error
)

echo "Next: ./launch_por.sh $([ "$TARGET" = "sim" ] && echo "(sim already running)" || echo "--real [--no-rc] --profile <profile>")"
echo "=========================================================="
echo "Command to execute:"
echo "${MAVROS_CMD[*]}"
echo "=========================================================="

if [ "$DRY_RUN" = true ]; then
  echo "Dry run requested. Exiting without launching."
  exit 0
fi

exec "${MAVROS_CMD[@]}"
