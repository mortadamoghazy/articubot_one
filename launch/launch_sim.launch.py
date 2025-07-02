import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'articubot_one'

    # Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'use_ros2_control': 'true'
        }.items()
    )

    # Gazebo Launch
    gazebo_params_file = os.path.join(
        get_package_share_directory(package_name), 'config', 'gazebo_params.yaml'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file,
            'world': os.path.join(get_package_share_directory(package_name), 'worlds', 'cones.world')
        }.items()
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen'
    )

    # Spawners for controllers
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

    # Joystick teleop (optional)
    joystick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch', 'joystick.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # IMU Covariance Override Node
    imu_covariance_override_node = Node(
        package=package_name,
        executable='imu_covariance_override_node.py',
        name='imu_covariance_override',
        output='screen'
    )

    # EKF Fusion Node
    ekf_fusion_node = Node(
        package=package_name,
        executable='ekf_fusion_node.py',
        name='ekf_fusion',
        output='screen'
    )

    # SLAM Toolbox Node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory(package_name),
            'config', 'mapper_params_online_async.yaml')],
        remappings=[
            ('/odom', '/odom_fused')  # Use EKF fused odometry
        ]
    )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        joystick_launch,
        imu_covariance_override_node,
        ekf_fusion_node,
        slam_toolbox_node  # ✅ SLAM Toolbox added here
    ])
