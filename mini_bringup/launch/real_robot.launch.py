import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_description = get_package_share_directory('mini_description')
    pkg_bringup = get_package_share_directory('mini_bringup')

    # ── LiDAR ─────────────────────────────────────────────────────────────
    # Defined directly (not via ms200_scan.launch.py) to set the correct
    # scan_topic (/scan) and frame_id (lidar_link matching the URDF).
    # The ms200 launch's static TF is omitted — robot_state_publisher
    # already publishes base_link → lidar_link from the URDF.
    lidar_node = Node(
        package='oradar_lidar',
        executable='oradar_scan',
        name='MS200',
        output='screen',
        parameters=[
            {'device_model': 'MS200'},
            {'frame_id': 'lidar_link'},
            {'scan_topic': '/scan'},
            {'port_name': '/dev/ttyACM1'},
            {'baudrate': 230400},
            {'angle_min': 0.0},
            {'angle_max': 360.0},
            {'range_min': 0.05},
            {'range_max': 20.0},
            {'clockwise': False},
            {'motor_speed': 10},
        ]
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
        lidar_node,
        node_robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        twist_stamper,
    ])