import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    mini_bringup_dir = get_package_share_directory('mini_bringup')
    mini_description_dir = get_package_share_directory('mini_description')

    slam_params_file = os.path.join(mini_bringup_dir, 'config', 'mapper_params_online_async.yaml')
    nav2_params_file = os.path.join(mini_bringup_dir, 'config', 'nav2_params.yaml')
    rviz_config_file = os.path.join(mini_description_dir, 'rviz', 'display.rviz')

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': 'false'
        }.items()
    )

    # Waits for odom to be visible on the network before Nav2 starts.
    # Blocks until /odom appears, then exits — Nav2 launches after.
    wait_for_odom = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'until ros2 topic info /diff_drive_controller/odom --verbose 2>/dev/null '
            '| grep -q "Publisher count: [1-9]"; '
            'do echo "[wait] odom not ready yet..."; sleep 1; done; '
            'echo "[wait] odom is live."'
        ],
        output='screen'
    )

    nav2 = TimerAction(
        period=15.0,   # Pi bringup + controller spawner + network discovery
        actions=[
            LogInfo(msg='[LifecycleLaunch] Nav2 starting — Pi TFs should be live.'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'false',
                    'params_file': nav2_params_file
                }.items()
            )
        ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        wait_for_odom,    # informational — shows you when Pi odom arrives
        slam_toolbox,
        nav2,             # 15s delay — covers Pi boot + controller spawning
        rviz,
    ])