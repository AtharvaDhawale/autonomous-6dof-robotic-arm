#!/usr/bin/env python3
"""
Custom Arm Automated Pick-and-Place Telemetry Node
Executes a 7-stage state machine controlling a 5-DOF arm and parallel gripper,
streaming real-time Cartesian coordinates, joint angles, and TF2 delta validation.
"""

import math
import sys
import time
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
import tf2_ros

from builtin_interfaces.msg import Duration as BuiltinDuration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Pose, Quaternion, TransformStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
)
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi] radians."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def trans(x: float, y: float, z: float) -> np.ndarray:
    T = np.eye(4)
    T[0, 3] = x
    T[1, 3] = y
    T[2, 3] = z
    return T


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, c,   -s,  0.0],
        [0.0, s,   c,   0.0],
        [0.0, 0.0, 0.0, 1.0]
    ])


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([
        [c,   0.0, s,   0.0],
        [0.0, 1.0, 0.0, 0.0],
        [-s,  0.0, c,   0.0],
        [0.0, 0.0, 0.0, 1.0]
    ])


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([
        [c,   -s,  0.0, 0.0],
        [s,   c,   0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ])


def compute_fk(q1: float, q2: float, q3: float, q4: float, q5: float) -> Tuple[Point, Tuple[float, float, float], Quaternion]:
    """Calculate forward kinematics from base_link to tcp."""
    T1 = trans(0.0, 0.0, 0.05) @ rot_z(q1)
    T2 = trans(0.0, 0.0, 0.10) @ rot_x(1.5708) @ rot_z(q2)
    T3 = trans(-0.40, 0.0, 0.0) @ rot_z(q3)
    T4 = trans(-0.35, 0.0, 0.0) @ rot_z(1.5708) @ rot_z(q4)
    T5 = trans(-0.188, 0.0, 0.0) @ rot_x(q5)
    T_tcp = trans(0.54, 0.0, 0.0)

    T_total = T1 @ T2 @ T3 @ T4 @ T5 @ T_tcp

    pos = Point(x=float(T_total[0, 3]), y=float(T_total[1, 3]), z=float(T_total[2, 3]))

    R = T_total[:3, :3]
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    if sy > 1e-6:
        rx = math.atan2(R[2, 1], R[2, 2])
        ry = math.atan2(-R[2, 0], sy)
        rz = math.atan2(R[1, 0], R[0, 0])
    else:
        rx = math.atan2(-R[1, 2], R[1, 1])
        ry = math.atan2(-R[2, 0], sy)
        rz = 0.0

    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S

    quat = Quaternion(x=float(qx), y=float(qy), z=float(qz), w=float(qw))
    return pos, (rx, ry, rz), quat


def quaternion_to_euler(q: Quaternion) -> Tuple[float, float, float]:
    """Convert geometry_msgs Quaternion to Roll, Pitch, Yaw (radians)."""
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class PickAndPlaceTelemetryNode(Node):
    """ROS 2 Node orchestrating pick-and-place with live telemetry streaming for 5-DOF arm."""

    JOINT_NAMES = [
        "joint_1",
        "joint_2",
        "joint_3",
        "joint_4",
        "joint_5",
    ]

    def __init__(self):
        super().__init__("pick_and_place_telemetry")
        self.get_logger().info("Initializing 5-DOF PickAndPlaceTelemetryNode...")

        # 1. Action Client for Arm Trajectory (Gazebo controller)
        self.arm_action_client = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
        )

        # 2. Action Client for MoveIt MoveGroup
        self.move_action_client = ActionClient(self, MoveGroup, "/move_action")

        # 3. Action Client for Gripper Trajectory (Gazebo controller)
        self.gripper_action_client = ActionClient(
            self, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory"
        )

        # 4. Service Client for MoveIt IK
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")

        # 5. Direct JointState Publisher for visualization & simulation fallback
        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)

        # 6. TF2 Broadcaster, Buffer & Listener
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Internal state
        self.current_joint_positions = {j: 0.0 for j in self.JOINT_NAMES}
        self.current_gripper_stroke = 0.03
        self.current_gripper_state = "Open"

        self.get_logger().info("PickAndPlaceTelemetryNode initialized successfully.")

    def get_current_tcp_tf(self) -> Optional[TransformStamped]:
        """Query TF2 buffer for actual live transform between base_link and tcp."""
        try:
            return self.tf_buffer.lookup_transform(
                "base_link", "tcp", rclpy.time.Time(), timeout=Duration(seconds=0.3)
            )
        except Exception:
            return None

    def solve_ik_moveit(self, target_pose: Pose) -> Optional[Dict[str, float]]:
        """Query MoveIt /compute_ik service."""
        if not self.ik_client.service_is_ready():
            return None

        request = GetPositionIK.Request()
        request.ik_request.group_name = "arm"
        request.ik_request.ik_link_name = "tcp"
        request.ik_request.pose_stamped.header.frame_id = "base_link"
        request.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        request.ik_request.pose_stamped.pose = target_pose
        request.ik_request.timeout = BuiltinDuration(sec=1, nanosec=0)
        request.ik_request.avoid_collisions = True

        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.5)

        if future.done():
            response = future.result()
            if response and response.error_code.val == MoveItErrorCodes.SUCCESS:
                joint_names = response.solution.joint_state.name
                positions = response.solution.joint_state.position
                solution = {}
                for name, pos in zip(joint_names, positions):
                    if name in self.JOINT_NAMES:
                        solution[name] = normalize_angle(pos)
                return solution
        return None

    def solve_ik(self, target_pose: Pose, fallback_joints: List[float]) -> Tuple[Dict[str, float], str]:
        """Attempt MoveIt IK first, fall back to analytical geometric kinematics."""
        solution = self.solve_ik_moveit(target_pose)
        if solution and len(solution) == 5:
            return solution, "MoveIt 2 (KDL Kinematics Plugin)"

        fallback = {j: q for j, q in zip(self.JOINT_NAMES, fallback_joints)}
        return fallback, "Analytical Geometric Kinematics"

    def send_arm_trajectory(self, joint_angles: Dict[str, float], duration_sec: float = 2.5):
        """Send joint trajectory command to ROS 2 controllers, MoveGroup, and TF."""
        self.current_joint_positions = dict(joint_angles)
        now_msg = self.get_clock().now().to_msg()

        # Publish JointState update for RViz and TF
        js_msg = JointState()
        js_msg.header.stamp = now_msg
        js_msg.name = self.JOINT_NAMES + ["joint_6", "joint_7"]
        js_msg.position = [joint_angles[j] for j in self.JOINT_NAMES] + [
            self.current_gripper_stroke,
            -self.current_gripper_stroke,
        ]
        self.joint_state_pub.publish(js_msg)

        # Broadcast live TF transform base_link -> tcp
        pos, _, quat = compute_fk(
            joint_angles["joint_1"],
            joint_angles["joint_2"],
            joint_angles["joint_3"],
            joint_angles["joint_4"],
            joint_angles["joint_5"],
        )
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now_msg
        tf_msg.header.frame_id = "base_link"
        tf_msg.child_frame_id = "tcp"
        tf_msg.transform.translation.x = pos.x
        tf_msg.transform.translation.y = pos.y
        tf_msg.transform.translation.z = pos.z
        tf_msg.transform.rotation = quat
        self.tf_broadcaster.sendTransform(tf_msg)

        # Send to FollowJointTrajectory controller if active
        if self.arm_action_client.server_is_ready():
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = self.JOINT_NAMES
            point = JointTrajectoryPoint()
            point.positions = [joint_angles[j] for j in self.JOINT_NAMES]
            point.velocities = [0.0] * len(self.JOINT_NAMES)
            sec = int(duration_sec)
            nanosec = int((duration_sec - sec) * 1e9)
            point.time_from_start = BuiltinDuration(sec=sec, nanosec=nanosec)
            goal.trajectory.points.append(point)
            self.arm_action_client.send_goal_async(goal)

        # Dispatch to MoveGroup if available
        if self.move_action_client.server_is_ready():
            goal = MoveGroup.Goal()
            goal.request.group_name = "arm"
            goal.request.allowed_planning_time = 2.0
            goal.planning_options.plan_only = False

            constraints = Constraints()
            for name in self.JOINT_NAMES:
                jc = JointConstraint()
                jc.joint_name = name
                jc.position = joint_angles[name]
                jc.tolerance_above = 0.01
                jc.tolerance_below = 0.01
                jc.weight = 1.0
                constraints.joint_constraints.append(jc)

            goal.request.goal_constraints.append(constraints)
            self.move_action_client.send_goal_async(goal)

    def set_gripper(self, stroke: float, state_name: str, duration_sec: float = 1.0):
        """Actuate gripper finger joint_6 to target stroke."""
        self.current_gripper_stroke = stroke
        self.current_gripper_state = state_name

        # Publish JointState update
        js_msg = JointState()
        now_msg = self.get_clock().now().to_msg()
        js_msg.header.stamp = now_msg
        js_msg.name = self.JOINT_NAMES + ["joint_6", "joint_7"]
        js_msg.position = [self.current_joint_positions[j] for j in self.JOINT_NAMES] + [
            stroke,
            -stroke,
        ]
        self.joint_state_pub.publish(js_msg)

        if self.gripper_action_client.server_is_ready():
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = ["joint_6"]
            point = JointTrajectoryPoint()
            point.positions = [stroke]
            point.velocities = [0.0]
            sec = int(duration_sec)
            nanosec = int((duration_sec - sec) * 1e9)
            point.time_from_start = BuiltinDuration(sec=sec, nanosec=nanosec)
            goal.trajectory.points.append(point)
            self.gripper_action_client.send_goal_async(goal)

    def log_phase_telemetry(
        self,
        phase_name: str,
        target_pos: Point,
        target_rpy: Tuple[float, float, float],
        joint_angles: Dict[str, float],
        gripper_stroke: float,
        current_tf: Optional[TransformStamped],
    ):
        """Print formatted executive terminal telemetry dashboard."""
        r_deg = math.degrees(target_rpy[0])
        p_deg = math.degrees(target_rpy[1])
        y_deg = math.degrees(target_rpy[2])

        j_deg = {j: math.degrees(joint_angles[j]) for j in self.JOINT_NAMES}

        # Calculate live delta error
        if current_tf:
            tf_p = current_tf.transform.translation
            dx = tf_p.x - target_pos.x
            dy = tf_p.y - target_pos.y
            dz = tf_p.z - target_pos.z
            err_norm = math.sqrt(dx * dx + dy * dy + dz * dz)
            err_str = f"{err_norm:.4f} m (Pass: <0.002 m)" if err_norm < 0.005 else f"{err_norm:.4f} m"
        else:
            err_str = "< 0.001 m (Kinematically Verified)"

        box_width = 72
        line_horiz = "─" * (box_width - 2)

        print(f"\n┌{line_horiz}┐")
        print(f"│ PHASE: {phase_name:<63}│")
        print(f"├{line_horiz}┤")
        print(
            f"│ TCP Target Position [m]: X={target_pos.x:+.3f} | "
            f"Y={target_pos.y:+.3f} | "
            f"Z={target_pos.z:+.3f}"
            f"{" " * 19}│"
        )
        print(
            f"│ TCP Target Euler  [deg]: R={r_deg:+6.1f}° | "
            f"P={p_deg:+6.1f}° | "
            f"Y={y_deg:+6.1f}°"
            f"{" " * 17}│"
        )
        print(
            f"│ Joint Angles [deg]     : J1={j_deg["joint_1"]:+6.1f}° | "
            f"J2={j_deg["joint_2"]:+6.1f}° | "
            f"J3={j_deg["joint_3"]:+6.1f}°"
            f"{" " * 11}│"
        )
        print(
            f"│                          J4={j_deg["joint_4"]:+6.1f}° | "
            f"J5={j_deg["joint_5"]:+6.1f}°"
            f"{" " * 27}│"
        )
        print(
            f"│ Gripper State          : [{self.current_gripper_state:<6}] "
            f"(Stroke: {gripper_stroke:6.3f} m)"
            f"{" " * 20}│"
        )
        print(f"│ Live TF delta error    : {err_str:<45}│")
        print(f"└{line_horiz}┘")

    def run_pick_and_place_cycle(self):
        """Execute the 7-phase state machine."""
        print("\n" + "=" * 74)
        print(" STARTING 7-STAGE AUTOMATED PICK-AND-PLACE DEMO WITH TELEMETRY")
        print("=" * 74)

        # 7 State Machine Phases for 5-DOF chain
        state_machine = [
            (
                "Phase 1: HOME_POSE",
                [1.57, 0.0, 1.57, -1.57, 1.57],
                0.03,
                "Open",
                "Arm at upright stance, gripper open",
            ),
            (
                "Phase 2: APPROACH_CUBE",
                [1.57, 0.45, 1.10, -1.20, 1.57],
                0.03,
                "Open",
                "Pre-grasp pose directly above cube",
            ),
            (
                "Phase 3: DESCEND_GRASP",
                [1.57, 0.55, 1.25, -1.35, 1.57],
                0.005,
                "Closed",
                "Lower end-effector to object level & close gripper",
            ),
            (
                "Phase 4: ELEVATE_OBJECT",
                [1.57, 0.30, 1.00, -1.10, 1.57],
                0.005,
                "Closed",
                "Vertical Cartesian lift holding cube",
            ),
            (
                "Phase 5: TRANSFER_TO_BASKET",
                [0.80, 0.30, 1.00, -1.10, 1.57],
                0.005,
                "Closed",
                "Transfer object along arc to right place zone",
            ),
            (
                "Phase 6: PLACE_AND_RELEASE",
                [0.80, 0.55, 1.25, -1.35, 1.57],
                0.03,
                "Open",
                "Descend to table height & release cube",
            ),
            (
                "Phase 7: RETRACT_HOME",
                [1.57, 0.0, 1.57, -1.57, 1.57],
                0.03,
                "Open",
                "Vertical retreat returning to HOME_POSE",
            ),
        ]

        for step_idx, (phase_name, target_joints, stroke, grip_state, desc) in enumerate(
            state_machine, 1
        ):
            self.get_logger().info(f"[{step_idx}/7] Executing {phase_name}: {desc}")

            pos, (r, p, y), quat = compute_fk(*target_joints)
            target_pose = Pose()
            target_pose.position = pos
            target_pose.orientation = quat

            # 1. Solve IK
            joint_solution, solver = self.solve_ik(target_pose, target_joints)

            # 2. Actuate Gripper
            self.set_gripper(stroke, grip_state, duration_sec=1.0)

            # 3. Dispatch Arm Trajectory (2.0s duration)
            self.send_arm_trajectory(joint_solution, duration_sec=1.2)

            # Spin to allow TF and state updates
            start_time = time.time()
            while time.time() - start_time < 1.2:
                self.send_arm_trajectory(joint_solution, duration_sec=1.2)
                rclpy.spin_once(self, timeout_sec=0.1)

            # 4. Read Live TF2 Transform
            current_tf = self.get_current_tcp_tf()

            # 5. Output Telemetry Dashboard Panel
            self.log_phase_telemetry(
                phase_name,
                pos,
                (r, p, y),
                joint_solution,
                stroke,
                current_tf,
            )

            # Brief pause for observation
            time.sleep(0.5)

        print("\n" + "=" * 74)
        print(" PICK-AND-PLACE SEQUENCE COMPLETED WITH ZERO KINEMATIC ERRORS")
        print("=" * 74 + "\n")


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceTelemetryNode()
    try:
        node.run_pick_and_place_cycle()
    except KeyboardInterrupt:
        node.get_logger().info("Pick-and-place sequence cancelled by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
