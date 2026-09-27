import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gazebo_pkg_share = FindPackageShare('custom_arm_gazebo')
    desc_pkg_share = FindPackageShare('custom_arm_description')

    default_world = PathJoinSubstitution([gazebo_pkg_share, 'worlds', 'pick_and_place.world'])
    default_urdf = PathJoinSubstitution([desc_pkg_share, 'urdf', 'robot.urdf.xacro'])

    world_arg = DeclareLaunchArgument(
        name='world',
        default_value=default_world,
        description='Full path to SDF world file'
    )

    urdf_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_urdf,
        description='Full path to robot xacro file'
    )

    # 1. Start Gazebo Sim
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': ['-r ', LaunchConfiguration('world')]
        }.items()
    )

    # 2. Robot State Publisher (wrapped in ParameterValue for ROS 2 Jazzy)
    robot_description_content = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', LaunchConfiguration('model')]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }]
    )

    # 3. Spawn Robot Model into Gazebo on Pedestal Table (Z = 0.40m)
    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-name', 'custom_arm',
            '-topic', '/robot_description',
            '-z', '0.40'
        ]
    )

    # 4. Clock & Communication Bridge
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ]
    )

    # 5. Controller Spawners
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='arm_controller_spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='gripper_controller_spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        urdf_arg,
        gazebo_sim,
        robot_state_publisher_node,
        spawn_robot_node,
        clock_bridge,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner
    ])
