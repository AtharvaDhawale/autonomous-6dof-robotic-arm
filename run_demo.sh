#!/usr/bin/env bash
# ==============================================================================
# Unified Two-Stage Pick-and-Place Demonstration Launcher
# 1. Launches Gazebo Sim + MoveIt 2 + RViz in the background.
# 2. Polls ROS 2 action graph until trajectory controllers are online.
# 3. Executes the automated pick-and-place telemetry state machine.
# ==============================================================================

set -e

# Source ROS 2 and workspace
source /opt/ros/jazzy/setup.bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/install/setup.bash"

echo "=============================================================================="
echo " STAGE 1: LAUNCHING GAZEBO PHYSICS & MOVEIT 2 MOTION PLANNING"
echo "=============================================================================="

# Launch moveit_gazebo in background process group
ros2 launch custom_arm_moveit_config moveit_gazebo.launch.py &
SIM_PID=$!

cleanup() {
    echo ""
    echo "[INFO] Shutting down simulation and demo nodes..."
    trap - SIGINT SIGTERM EXIT
    if kill -0 "$SIM_PID" 2>/dev/null; then
        kill -SIGINT "$SIM_PID" 2>/dev/null || true
        wait "$SIM_PID" 2>/dev/null || true
    fi
    pkill -f "moveit_gazebo" 2>/dev/null || true
    pkill -f "gz sim" 2>/dev/null || true
    pkill -f "robot_state_publisher" 2>/dev/null || true
    echo "[INFO] Clean shutdown complete."
    exit 0
}

trap cleanup SIGINT SIGTERM EXIT

echo "[INFO] Simulation launched with PID $SIM_PID. Waiting for controllers..."

# Stage 2: Poll for controller action server readiness
TIMEOUT=35
ELAPSED=0
CONTROLLER_READY=false

while [ $ELAPSED -lt $TIMEOUT ]; do
    if ! kill -0 "$SIM_PID" 2>/dev/null; then
        echo "[ERROR] Simulation process terminated unexpectedly."
        exit 1
    fi

    if ros2 action list 2>/dev/null | grep -q "/arm_controller/follow_joint_trajectory"; then
        CONTROLLER_READY=true
        break
    fi

    sleep 1
    ELAPSED=$((ELAPSED + 1))
    echo "  [$(printf '%02d' $ELAPSED)/$TIMEOUT s] Waiting for /arm_controller/follow_joint_trajectory..."
done

if [ "$CONTROLLER_READY" = false ]; then
    echo "[WARN] Controller action server wait timed out ($TIMEOUT s)."
    echo "[INFO] Proceeding to telemetry sequence (fallback mode available)..."
else
    echo "[SUCCESS] Gazebo trajectory controllers verified active!"
fi

sleep 2

echo "=============================================================================="
echo " STAGE 2: STARTING AUTOMATED PICK-AND-PLACE TELEMETRY SEQUENCE"
echo "=============================================================================="

ros2 run custom_arm_kinematics pick_and_place_telemetry

echo ""
echo "[SUCCESS] Pick-and-place sequence finished. Press Ctrl+C to close simulation."
wait "$SIM_PID"
