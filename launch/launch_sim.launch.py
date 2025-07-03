#!/usr/bin/env python3
import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'articubot_one'
    pkg_share    = get_package_share_directory(package_name)

    # Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'rsp.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'use_ros2_control': 'true'
        }.items()
    )

    # Gazebo Launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            )
        ]),
        launch_arguments={
            'extra_gazebo_args':
                '--ros-args --params-file ' + os.path.join(pkg_share, 'config', 'gazebo_params.yaml'),
            'world': os.path.join(pkg_share, 'worlds', 'cones.world')
        }.items()
    )

    # Spawn the robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen'
    )

    # Controller spawners
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    # Joystick
    joystick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'joystick.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # IMU Covariance Override
    imu_cov_override_node = Node(
        package=package_name,
        executable='imu_covariance_override_node.py',
        name='imu_covariance_override',
        output='screen'
    )

    # === REPLACED CUSTOM EKF WITH robot_localization EKF ===
    robot_localization_ekf = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[os.path.join(pkg_share, 'config', 'ekf.yaml')],
                remappings=[
                    ('/odometry/filtered', '/odom_fused')
                ]
            )
        ]
    )

    # SLAM Toolbox
    slam_toolbox_node = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')],
                remappings=[('/odom', '/odom_fused')]
            )
        ]
    )

    # SLAM Evaluator – Execute Python script directly
    slam_evaluator = TimerAction(
        period=5.0,
        actions=[
            Node(
                package=package_name,
                executable='slam_evaluator.py',
                name='slam_evaluator',
                output='screen',
                emulate_tty=True,
                parameters=[{
                    'robot_name': 'my_bot',
                    'est_topic': '/pose',
                    'output_dir': os.path.expanduser('~/slam_eval'),
                    'world_frame': 'odom',
                    'base_frame': 'base_link'
                }]
            )
        ]
    )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        joystick_launch,
        imu_cov_override_node,
        robot_localization_ekf,
        slam_toolbox_node,
        slam_evaluator,
    ])
