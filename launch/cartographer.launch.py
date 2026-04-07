"""
Cartographer SLAM launch for Sendbooster AGV

Jetson에서 bringup 실행 후, PC에서 이 launch로 SLAM.
(robot_state_publisher는 bringup에서 실행하므로 여기선 제외)

Usage:
  ros2 launch sendbooster_agv_bringup cartographer.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('sendbooster_agv_bringup')
    config_dir = os.path.join(bringup_dir, 'config')

    resolution = LaunchConfiguration('resolution')
    publish_period_sec = LaunchConfiguration('publish_period_sec')

    # Cartographer node: uses front LiDAR scan
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', 'cartographer_1lidar.lua',
        ],
        remappings=[
            ('scan', 'scan_raw_front'),  # 전방 LiDAR만 사용 (뒤에서 조종하는 사람 방지)
            ('odom', 'odometry/filtered'),
        ],
    )

    # Occupancy grid publisher
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-resolution', resolution,
            '-publish_period_sec', publish_period_sec,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('resolution', default_value='0.05'),
        DeclareLaunchArgument('publish_period_sec', default_value='1.0'),

        cartographer_node,
        occupancy_grid_node,
    ])
