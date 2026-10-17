#!/usr/bin/env bash
#
# Launch the BANYU ROV simulation through banyu_bringup's main_launch.
#
# Usage:
#   ./launch_sim.sh por      # proof-of-readiness sim (no DVL)
#   ./launch_sim.sh task2    # Task 2 sim (DVL)
#   ./launch_sim.sh          # defaults to 'por'
#
# Any extra arguments after the mode are passed straight through to
# `ros2 launch` (e.g. ./launch_sim.sh task2 use_sim_time:=False).
set -euo pipefail

MODE="${1:-por}"
case "$MODE" in
  por|task2) ;;
  *)
    echo "Unknown mode: $MODE" >&2
    echo "Usage: $0 [por|task2] [extra ros2 launch args...]" >&2
    exit 1
    ;;
esac

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"

cd "$WS_ROOT"

ROS_DISTRO_TO_USE="${ROS_DISTRO:-jazzy}"
bash -c "source /opt/ros/${ROS_DISTRO_TO_USE}/setup.bash && source install/setup.bash && \
  ros2 launch banyu_bringup main_launch.launch.py mode:=$MODE ${*:2}"

