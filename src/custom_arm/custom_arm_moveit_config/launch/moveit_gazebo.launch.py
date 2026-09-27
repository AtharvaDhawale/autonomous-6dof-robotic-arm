import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_pkg_share = get_package_share_directory("custom_arm_moveit_config")
    gazebo_pkg_share = get_package_share_directory("custom_arm_gazebo")

    default_rviz_config = os.path.join(moveit_pkg_share, "config", "moveit.rviz")

    # Launch arguments ensuring use_sim_time: True propagates across all simulation components
    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true"
    )

    rviz_config_arg = DeclareLaunchArgument(
        name="rviz_config",
        default_value=default_rviz_config,
        description="Path to RViz configuration file"
    )

    # Build MoveIt configurations with simulation time enabled
    moveit_config = (
        MoveItConfigsBuilder("custom_arm", package_name="custom_arm_moveit_config")
        .robot_description()
        .robot_description_semantic()
        .robot_description_kinematics()
        .joint_limits()
        .trajectory_execution()
        .planning_pipelines(pipelines=["ompl"])
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )

    # 1. Include Gazebo simulation launch (physics, world, robot spawn, controllers, clock bridge, robot_state_publisher)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg_share, "launch", "gazebo.launch.py")
        ),
        launch_arguments={"use_sim_time": LaunchConfiguration("use_sim_time")}.items()
    )

    # 2. Move Group Node (connected to Gazebo ros2_control FollowJointTrajectory action servers)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ]
    )

    # 3. RViz2 Node pre-loaded with MotionPlanning display
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        rviz_config_arg,
        gazebo_launch,
        move_group_node,
        rviz_node
    ])
