#!/usr/bin/env bash
# ==============================================================================
# RobotX 2026 Underwater Magnetically Activated Light Simulation Launcher
# Launches Gazebo Sim with 4 light nodes and the Python simulation controller.
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export SDF_PATH="${SCRIPT_DIR}/models:${SDF_PATH}"
export GZ_SIM_RESOURCE_PATH="${SCRIPT_DIR}/models:${GZ_SIM_RESOURCE_PATH}"

echo "========================================================================"
echo "  Starting RobotX 2026 4-Node Light Simulation (banyu_simulation)"
echo "  World: pipeline_lights.sdf (4 Nodes along Pipeline)"
echo "========================================================================"

# Check for headless flag
HEADLESS=false
if [[ "$1" == "--headless" ]] || [[ "$1" == "-s" ]]; then
    HEADLESS=true
fi

# Function to clean up background processes on exit
cleanup() {
    echo ""
    echo "[Launcher] Terminating simulation processes..."
    if [[ -n "${CTRL_PID}" ]]; then
        kill "${CTRL_PID}" 2>/dev/null || true
    fi
    if [[ -n "${GZ_PID}" ]]; then
        kill "${GZ_PID}" 2>/dev/null || true
    fi
    echo "[Launcher] Done."
}
trap cleanup EXIT INT TERM

# 1. Start Gazebo Sim
if [ "$HEADLESS" = true ]; then
    echo "[1/2] Launching Gazebo Sim in HEADLESS mode..."
    gz sim -s -r "${SCRIPT_DIR}/worlds/pipeline_lights.sdf" &
    GZ_PID=$!
else
    echo "[1/2] Launching Gazebo Sim GUI..."
    gz sim -r "${SCRIPT_DIR}/worlds/pipeline_lights.sdf" &
    GZ_PID=$!
fi

sleep 3

# 2. Start Controller
echo "[2/2] Launching RobotX Simulation Controller..."
python3 "${SCRIPT_DIR}/scripts/robotx_light_controller.py" pipeline_lights &
CTRL_PID=$!

echo ""
echo "========================================================================"
echo "  Simulation Active (RobotX 2026 Task 2: Infrastructure Survey & Repair)!"
echo "  - Stream Live Status:   python3 scripts/test_pipeline_lights.py --monitor"
echo "  - Switch Tier:          python3 scripts/test_pipeline_lights.py --tier DISRUPTIVE"
echo "  - Set Damaged Node:     python3 scripts/test_pipeline_lights.py --damaged-node 2"
echo "  - Configure Sequence:   python3 scripts/test_pipeline_lights.py --first-color RED --second-color BLUE"
echo "  - Proximity Tap (<2s):  python3 scripts/test_pipeline_lights.py --magnet-tap"
echo "  - Magnetic Repair:      python3 scripts/test_pipeline_lights.py --repair"
echo "  - Reset Pipeline:       python3 scripts/test_pipeline_lights.py --reset"
echo "========================================================================"
echo ""

# Keep launcher process alive
wait "${GZ_PID}"
