# Autonomous 6-DOF Robotic Arm & Parallel Gripper Pipeline

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy%20Jalisco-3498DB?style=for-the-badge&logo=ros)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-FF6F00?style=for-the-badge&logo=gazebo)](https://gazebosim.org/)
[![MoveIt 2](https://img.shields.io/badge/MoveIt%202-Motion%20Planning-5271FF?style=for-the-badge)](https://moveit.picknik.ai/)
[![Python](https://img.shields.io/badge/Python-3.12-3776AB?style=for-the-badge&logo=python)](https://python.org)
[![C++](https://img.shields.io/badge/C++-17-00599C?style=for-the-badge&logo=c%2B%2B)](https://isocpp.org)
[![License](https://img.shields.io/badge/License-Apache%202.0-green.svg?style=for-the-badge)](LICENSE)

An end-to-end, modular ROS 2 robotic arm manipulation framework featuring closed-form analytical kinematics, MoveIt 2 motion planning, Gazebo Sim physics simulation with `ros2_control`, live TF2 coordinate telemetry, and automated pick-and-place task execution.

---

## Demonstration

![Autonomous Pick and Place Telemetry](media/demo.gif)

---

## System Architecture

The workspace is organized into five decoupled, production-grade ROS 2 packages adhering to the standard ROS 2 robotics stack layout:

```text
robotic_arm_ws/
├── src/
│   └── custom_arm/
│       ├── custom_arm_description/    # Kinematic chain, Xacro primitives, visual meshes & RViz
│       ├── custom_arm_moveit_config/  # SRDF, OMPL planners, kinematics solvers, trajectory bindings
│       ├── custom_arm_gazebo/         # SDF worlds, physics parameters, spawning & Gazebo clock bridge
│       ├── custom_arm_control/        # ros2_control controller manager & trajectory controller configs
│       └── custom_arm_kinematics/     # MoveIt IK client, TF2 telemetry listener & pick-and-place state machine
├── media/                             # Telemetry recordings and demonstration GIFs
├── run_demo.sh                        # One-command dual-stage orchestrator
├── setup_shortcuts.sh                 # Global bash alias installer
├── record_demo.sh                     # Automated X11 recording and 2-pass GIF converter
└── README.md
```

### Package Manifest Breakdown

| Package | Build Type | Core Responsibilities & Interfaces |
| :--- | :--- | :--- |
| **`custom_arm_description`** | `ament_cmake` | Parametric URDF/Xacro model, positive-definite inertia tensors, joint dynamics, `gz_ros2_control` hardware interface tags, and RViz inspection pipeline. |
| **`custom_arm_moveit_config`**| `ament_cmake` | Semantic description (`robot.srdf`), KDL kinematics plugin, OMPL planners (`RRTConnect`, `RRTstar`, `PRM`), Pilz Cartesian limits, and MoveIt execution controllers. |
| **`custom_arm_control`**      | `ament_cmake` | `controller_manager` configuration (100 Hz), `JointStateBroadcaster`, and `JointTrajectoryController` for both the 6-DOF arm and parallel gripper. |
| **`custom_arm_gazebo`**       | `ament_cmake` | SDF 1.8 physics environment (`pick_and_place.world`), ODE solver settings, pedestal table, orange graspable target cube, and `ros_gz_bridge`. |
| **`custom_arm_kinematics`**   | `ament_python` | Analytical geometric IK solver, `/compute_ik` service client, MoveGroup action client, TF2 coordinate validator, and 7-phase state machine. |

---

## Kinematic Chain & Link Specifications

| Link | Type | Dimensions / Geometry | Mass | Parent Joint | Joint Type | Motion Range | Effort / Velocity Limits |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `world` | Ground Frame | Fixed anchor frame | — | — | — | — | — |
| `base_link` | Cylinder | $r=0.08\,\text{m}, l=0.05\,\text{m}$ | $2.0\,\text{kg}$ | `world_to_base` | Fixed | — | — |
| `link_1` | Cylinder | $r=0.05\,\text{m}, l=0.10\,\text{m}$ | $1.2\,\text{kg}$ | `joint_1` (Pan) | Revolute ($Z$) | $[-3.14, +3.14]\,\text{rad}$ | $50.0\,\text{Nm} \;\vert\; 1.5\,\text{rad/s}$ |
| `link_2` | Box | $0.05 \times 0.06 \times 0.30\,\text{m}$ | $1.5\,\text{kg}$ | `joint_2` (Lift) | Revolute ($Y$) | $[-1.57, +1.57]\,\text{rad}$ | $50.0\,\text{Nm} \;\vert\; 1.5\,\text{rad/s}$ |
| `link_3` | Box | $0.04 \times 0.05 \times 0.25\,\text{m}$ | $1.0\,\text{kg}$ | `joint_3` (Elbow) | Revolute ($Y$) | $[-2.00, +2.00]\,\text{rad}$ | $40.0\,\text{Nm} \;\vert\; 1.5\,\text{rad/s}$ |
| `link_4` | Cylinder | $r=0.035\,\text{m}, l=0.06\,\text{m}$ | $0.6\,\text{kg}$ | `joint_4` (Roll) | Revolute ($Z$) | $[-3.14, +3.14]\,\text{rad}$ | $20.0\,\text{Nm} \;\vert\; 2.0\,\text{rad/s}$ |
| `link_5` | Cylinder | $r=0.03\,\text{m}, l=0.04\,\text{m}$ | $0.4\,\text{kg}$ | `joint_5` (Pitch) | Revolute ($Y$) | $[-1.57, +1.57]\,\text{rad}$ | $20.0\,\text{Nm} \;\vert\; 2.0\,\text{rad/s}$ |
| `palm_base`| Box | $0.08 \times 0.06 \times 0.04\,\text{m}$ | $0.3\,\text{kg}$ | `joint_6` (Yaw) | Revolute ($Z$) | $[-3.14, +3.14]\,\text{rad}$ | $15.0\,\text{Nm} \;\vert\; 2.5\,\text{rad/s}$ |
| `finger_left` | Box | $0.015 \times 0.02 \times 0.06\,\text{m}$| $0.05\,\text{kg}$| `left_finger_joint` | Prismatic ($X$) | $[0.00, 0.025]\,\text{m}$ | $20.0\,\text{N} \;\vert\; 0.05\,\text{m/s}$ |
| `finger_right`| Box | $0.015 \times 0.02 \times 0.06\,\text{m}$| $0.05\,\text{kg}$| `right_finger_joint`| Prismatic ($-X$) | $[0.00, 0.025]\,\text{m}$ | Mimic (`multiplier=1.0`) |
| `tcp` | Virtual Frame | Tool Center Point offset | — | `tcp_joint` | Fixed ($Z=+0.07$) | — | — |

---

## Live Kinematics Telemetry & State Machine

The automated pick-and-place demo executes a 7-phase state machine with real-time Cartesian and joint telemetry:

```text
┌──────────────────────────────────────────────────────────────────────┐
│ PHASE: Phase 2: APPROACH_CUBE                                        │
├──────────────────────────────────────────────────────────────────────┤
│ TCP Target Position [m]: X=+0.300 | Y=+0.000 | Z=+0.520              │
│ TCP Target Euler  [deg]: R=  +0.0° | P= +90.0° | Y=  +0.0°            │
│ Joint Angles [deg]     : J1=  +0.0° | J2= +75.2° | J3= -94.0°      │
│                          J4=  +0.0° | J5=+108.8° | J6=  +0.0°      │
│ Gripper State          : [Open  ] (Stroke:  0.025 m)                 │
│ Live TF delta error    : < 0.001 m (Kinematically Verified)          │
└──────────────────────────────────────────────────────────────────────┘
```

### State Machine Lifecycle
1. **`HOME_POSE`**: Arm moves to upright ready stance ($Z=0.45\,\text{m}$), gripper opens ($0.025\,\text{m}$).
2. **`APPROACH_CUBE`**: Approaches pre-grasp pose directly above the target object ($X=0.30\,\text{m}, Y=0.0\,\text{m}, Z=0.52\,\text{m}$).
3. **`DESCEND_GRASP`**: Descends end-effector to object level ($Z=0.44\,\text{m}$), closes gripper around cube ($\text{stroke}=0.005\,\text{m}$).
4. **`ELEVATE_OBJECT`**: Vertical Cartesian lift ($+0.12\,\text{m}$ to $Z=0.56\,\text{m}$) securing the object.
5. **`TRANSFER_TO_BASKET`**: Smooth arc trajectory transfer to right place zone ($X=0.15\,\text{m}, Y=-0.28\,\text{m}, Z=0.56\,\text{m}$).
6. **`PLACE_AND_RELEASE`**: Lowers to pedestal surface ($Z=0.44\,\text{m}$) and opens gripper fingers.
7. **`RETRACT_HOME`**: Vertical retreat and return to safe standby stance.

---

## Quick Start Guide

### 1. Prerequisites
Ensure ROS 2 Jazzy and Gazebo Harmonic are installed:
```bash
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-moveit
```

### 2. Build Workspace
```bash
cd ~/robotic_arm_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 3. Install Global Shell Shortcuts
Run the shortcut configuration utility to enable one-word commands from any shell:
```bash
bash setup_shortcuts.sh
source ~/.bashrc
```

Available shortcuts:
- `arm_build`: Rebuilds workspace with `colcon` and sources install directory.
- `arm_rviz`: Launches RViz2 with interactive joint sliders for URDF inspection.
- `arm_moveit`: Launches MoveIt 2 motion planning standalone demo.
- `arm_sim`: Launches Gazebo physics + MoveIt 2 + RViz in unified simulation.
- `arm_telemetry`: Runs the automated 7-stage pick-and-place sequence with live telemetry.
- `arm_ik_test`: Runs the 4-waypoint inverse kinematics solver and TF2 validation.

---

## Running the Complete Simulation & Demo

### Option A: One-Command Dual-Stage Orchestrator (Recommended)
Automatically initializes the Gazebo physics environment, validates controller health, and triggers the pick-and-place telemetry pipeline:
```bash
~/robotic_arm_ws/run_demo.sh
```

### Option B: Manual Side-by-Side Execution

#### Terminal 1 — Physics Simulation & Motion Planner
```bash
arm_sim
# Equivalent to: ros2 launch custom_arm_moveit_config moveit_gazebo.launch.py
```

#### Terminal 2 — Automated Pick-and-Place Telemetry Sequence
```bash
arm_telemetry
# Equivalent to: ros2 run custom_arm_kinematics pick_and_place_telemetry
```

---

## Screen Recording & GIF Export

To record live RViz / Gazebo demonstrations and produce optimized, lightweight GIFs for documentation:
```bash
~/robotic_arm_ws/record_demo.sh
```
The script captures the X11 display at 30 FPS and executes a two-pass palette optimization (`fps=15,scale=960:-1:flags=lanczos`), outputting the result directly to `media/demo.gif`.

---

## License

This project is licensed under the Apache 2.0 License. See [LICENSE](LICENSE) for details.

## Author & Maintainer

- **Atharva**
- Built with ROS 2 Jazzy Jalisco, Gazebo Harmonic, and MoveIt 2.
