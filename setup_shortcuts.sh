#!/usr/bin/env bash
# ==============================================================================
# Robotic Arm Workspace Shell Shortcuts Setup
# Appends clean, idempotent aliases to ~/.bashrc for seamless global execution.
# ==============================================================================

BASHRC="$HOME/.bashrc"
MARKER_START="# >>> Robotic Arm WS Shortcuts >>>"
MARKER_END="# <<< Robotic Arm WS Shortcuts <<<"

# Remove existing block if present to ensure clean updates
if grep -q "$MARKER_START" "$BASHRC" 2>/dev/null; then
    echo "[INFO] Existing Robotic Arm WS shortcuts found in $BASHRC. Updating..."
    sed -i "/$MARKER_START/,/$MARKER_END/d" "$BASHRC"
else
    echo "[INFO] Installing Robotic Arm WS shortcuts to $BASHRC..."
fi

cat << 'ALIASES' >> "$BASHRC"
# >>> Robotic Arm WS Shortcuts >>>
source /opt/ros/jazzy/setup.bash 2>/dev/null
export ROBOTIC_ARM_WS="$HOME/robotic_arm_ws"

alias arm_build="cd \$ROBOTIC_ARM_WS && colcon build --symlink-install && source install/setup.bash"
alias arm_rviz="source /opt/ros/jazzy/setup.bash && source \$ROBOTIC_ARM_WS/install/setup.bash 2>/dev/null && ros2 launch custom_arm_description display.launch.py"
alias arm_moveit="source /opt/ros/jazzy/setup.bash && source \$ROBOTIC_ARM_WS/install/setup.bash 2>/dev/null && ros2 launch custom_arm_moveit_config demo.launch.py"
alias arm_sim="source /opt/ros/jazzy/setup.bash && source \$ROBOTIC_ARM_WS/install/setup.bash 2>/dev/null && ros2 launch custom_arm_moveit_config moveit_gazebo.launch.py"
alias arm_telemetry="source /opt/ros/jazzy/setup.bash && source \$ROBOTIC_ARM_WS/install/setup.bash 2>/dev/null && ros2 run custom_arm_kinematics pick_and_place_telemetry"
alias arm_ik_test="source /opt/ros/jazzy/setup.bash && source \$ROBOTIC_ARM_WS/install/setup.bash 2>/dev/null && ros2 run custom_arm_kinematics ik_coordinate_solver"
# <<< Robotic Arm WS Shortcuts <<<
ALIASES

echo "[SUCCESS] Shell shortcuts configured successfully in $BASHRC:"
echo "  - arm_build     : Rebuilds workspace with colcon and sources install space"
echo "  - arm_rviz      : Launches RViz2 with interactive joint sliders"
echo "  - arm_moveit    : Launches MoveIt 2 demo motion planning"
echo "  - arm_sim       : Launches Gazebo physics + MoveIt 2 unified simulation"
echo "  - arm_telemetry : Runs 7-stage automated pick-and-place with live telemetry"
echo "  - arm_ik_test   : Runs 4-waypoint IK solver and TF2 validation sequence"
echo ""
echo "Run 'source ~/.bashrc' or open a new terminal to activate."
