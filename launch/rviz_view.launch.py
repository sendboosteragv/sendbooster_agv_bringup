"""
RViz2 viewer for remote monitoring (PC side).

Jetson에서 bringup + cartographer 실행 후, PC에서 이 launch만 실행.

Usage:
  ros2 launch sendbooster_agv_bringup rviz_view.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('sendbooster_agv_bringup')
    rviz_config = os.path.join(bringup_dir, 'rviz', 'slam_view.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        rviz_node,
    ])
