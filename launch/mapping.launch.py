#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('articubot_one'), 'config')

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='mapping_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', 'turtlebot3_lds_2d.lua'
        ],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odom_fused')
        ]
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_node',
        output='screen',
        parameters=[
            {"use_sim_time": True},
            {"resolution": 0.05},
            {"publish_period_sec": 1.0}
        ]
    )

    return LaunchDescription([
        TimerAction(period=10.0, actions=[cartographer_node]),
        TimerAction(period=11.0, actions=[occupancy_grid_node])
    ])
