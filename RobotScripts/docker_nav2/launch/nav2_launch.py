#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    # Package paths
    yahboomcar_nav_dir = get_package_share_directory('yahboomcar_nav')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    # Launch file paths
    laser_launch = os.path.join(yahboomcar_nav_dir, 'launch', 'laser_bringup_launch.py')
    slam_launch = os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
    nav2_launch = os.path.join(yahboomcar_nav_dir, 'launch', 'navigation_dwa_launch.py')

    # Launch actions
    laser_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(laser_launch)
    )

    slam_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': os.path.join(
                yahboomcar_nav_dir,
                'params',
                'mapper_params_online_async.yaml'
            )
        }.items()
    )

    nav2_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            'params_file': os.path.join(
                yahboomcar_nav_dir, 'params', 'dwa_nav_params.yaml'
            ),
            'use_sim_time': 'false'
        }.items()
    )

    return LaunchDescription([
        # 1. laser_bringup_launch.py
        laser_action,

        # 2. SLAM toolbox
        TimerAction(
            period=10.0,             # delay in seconds
            actions=[slam_action]
        ),

        # 3. Nav2 launch
        TimerAction(
            period=20.0,
            actions=[nav2_action]
        ),

        # 4. Navigation Node launch
        TimerAction(
            period=40.0,
            actions=[
                ExecuteProcess(
                    cmd=['python3', '/root/yahboomcar_ros2_ws/yahboomcar_ws/src/yahboomcar_nav/launch/NavigationNode.py'],
                    output='screen'
                )
            ]
        )
    ])
