import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # MoveIt configuration builder
    moveit_config = (
        MoveItConfigsBuilder('custom_arm', package_name='custom_arm_moveit_config')
        .robot_description()
        .robot_description_semantic()
        .robot_description_kinematics()
        .joint_limits()
        .trajectory_execution()
        .planning_pipelines(pipelines=['ompl'])
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )

    moveit_pkg_share = get_package_share_directory('custom_arm_moveit_config')
    default_rviz_config = os.path.join(moveit_pkg_share, 'config', 'moveit.rviz')

    # Launch arguments
    rviz_config_arg = DeclareLaunchArgument(
        name='rviz_config',
        default_value=default_rviz_config,
        description='Path to RViz configuration file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # 1. Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # 2. Virtual Joint State Broadcaster for standalone offline planning
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'source_list': ['move_group/fake_controller_joint_states']
            }
        ]
    )

    # 3. Move Group Node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # 4. RViz2 Node pre-loaded with MotionPlanning display
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        rviz_config_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        move_group_node,
        rviz_node
    ])
