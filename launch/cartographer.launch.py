"""
Cartographer SLAM launch for Sendbooster AGV (Real Robot)

Usage:
  ros2 launch sendbooster_agv_bringup cartographer.launch.py                   # 2 LiDAR (default)
  ros2 launch sendbooster_agv_bringup cartographer.launch.py num_lidars:=1     # 1 LiDAR (front only)
  ros2 launch sendbooster_agv_bringup cartographer.launch.py use_rviz:=false   # without Rviz
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('sendbooster_agv_bringup')
    config_dir = os.path.join(bringup_dir, 'config')

    num_lidars = LaunchConfiguration('num_lidars')
    use_rviz = LaunchConfiguration('use_rviz')
    resolution = LaunchConfiguration('resolution')
    publish_period_sec = LaunchConfiguration('publish_period_sec')

    # Select lua config based on num_lidars
    lua_1lidar = 'cartographer_1lidar.lua'
    lua_2lidar = 'cartographer_2lidar.lua'

    # Cartographer node: 1 LiDAR (front only)
    cartographer_1lidar = Node(
        condition=IfCondition(PythonExpression(["'", num_lidars, "' == '1'"])),
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', lua_1lidar,
        ],
        remappings=[
            ('scan', 'scan'),
        ],
    )

    # Cartographer node: 2 LiDAR (front + back)
    cartographer_2lidar = Node(
        condition=IfCondition(PythonExpression(["'", num_lidars, "' != '1'"])),
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', lua_2lidar,
        ],
        remappings=[
            ('scan', 'scan'),
            ('scan_2', 'scan2'),
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

    # Rviz
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('num_lidars', default_value='2',
                              description='Number of LiDARs (1 or 2)'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('resolution', default_value='0.05'),
        DeclareLaunchArgument('publish_period_sec', default_value='1.0'),

        cartographer_1lidar,
        cartographer_2lidar,
        occupancy_grid_node,
        rviz_node,
    ])
