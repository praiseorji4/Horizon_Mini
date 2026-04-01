import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_description = get_package_share_directory('mini_description')
    pkg_bringup = get_package_share_directory('mini_bringup')
    pkg_lidar = get_package_share_directory('oradar_lidar')

    # ── LiDAR (includes msms200_scan.launch.py) ──────────────────────────
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_lidar, 'launch', 'ms200_scan.launch.py')
        ])
    )

    # ── Robot description ─────────────────────────────────────────────────
    robot_description_config = Command([
        'xacro ',
        os.path.join(pkg_description, 'urdf', 'body', 'mini_robot.urdf.xacro'),
        ' use_gazebo:=false'
    ])

    # ── Nodes ─────────────────────────────────────────────────────────────
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': False
        }]
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description_config},
            os.path.join(pkg_bringup, 'config', 'mini_controllers.yaml'),
            {'use_sim_time': False}
        ],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        parameters=[{'use_sim_time': False}]
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        parameters=[{'use_sim_time': False}]
    )

    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        parameters=[{
            'use_sim_time': False,
            'frame_id': 'base_footprint'
        }],
        remappings=[
            ('/cmd_vel_in', '/cmd_vel'),
            ('/cmd_vel_out', '/diff_drive_controller/cmd_vel'),
        ]
    )

    return LaunchDescription([
        lidar_launch,                        # <-- LiDAR included here
        node_robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        twist_stamper,
    ])