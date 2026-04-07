"""
Stress Test launch for Sendbooster AGV

Harsh simulation environment for Jetson Orin Nano validation:
  - 300m x 200m factory maze (vs 10m x 8m warehouse)
  - Higher sensor noise (LiDAR 3x, IMU 3-5x)
  - Topic delay relay (simulates hardware serial/polling latency)
  - Lower sensor update rates (LiDAR 7Hz, IMU 30Hz)
  - Asymmetric wheel friction (drift simulation)

Split execution:
  - gazebo:=true  nav:=false  → Gazebo container
  - gazebo:=false nav:=true   → Nav container (CPU/mem limited)

Usage:
  # All-in-one:
  ros2 launch sendbooster_agv_bringup stress_test.launch.py

  # Gazebo only:
  ros2 launch sendbooster_agv_bringup stress_test.launch.py gazebo:=true nav:=false

  # Nav only (with delays):
  ros2 launch sendbooster_agv_bringup stress_test.launch.py gazebo:=false nav:=true \\
    map:=/path/to/factory_maze_map.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('sendbooster_agv_bringup')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    ekf_config = os.path.join(bringup_dir, 'config', 'ekf.yaml')
    nav2_config = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    default_world = os.path.join(bringup_dir, 'worlds', 'factory_maze.world')
    default_map = os.path.join(bringup_dir, 'map', 'map.yaml')
    stress_urdf = os.path.join(
        bringup_dir, 'urdf', 'sendbooster_agv_gazebo_stress.urdf')

    with open(stress_urdf, 'r') as f:
        robot_description = f.read()

    # Launch configurations
    use_gazebo = LaunchConfiguration('gazebo')
    use_nav = LaunchConfiguration('nav')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    map_file = LaunchConfiguration('map')

    # Delay parameters
    odom_delay = LaunchConfiguration('odom_delay_ms')
    imu_delay = LaunchConfiguration('imu_delay_ms')
    scan_delay = LaunchConfiguration('scan_delay_ms')

    declare_args = [
        DeclareLaunchArgument('gazebo', default_value='true'),
        DeclareLaunchArgument('nav', default_value='true'),
        DeclareLaunchArgument('headless', default_value='true',
                              description='Headless by default for stress test'),
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('map', default_value=default_map),
        # Hardware delay simulation (ms)
        DeclareLaunchArgument('odom_delay_ms', default_value='30.0',
                              description='Motor driver RS485 round-trip delay'),
        DeclareLaunchArgument('imu_delay_ms', default_value='15.0',
                              description='IMU serial polling delay'),
        DeclareLaunchArgument('scan_delay_ms', default_value='50.0',
                              description='LiDAR scan buffering delay'),
    ]

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value='/usr/share/gazebo-11/models'
    )

    # ================================================================
    # GAZEBO COMPONENTS
    # ================================================================

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
        ),
        condition=IfCondition(use_gazebo),
        launch_arguments={
            'world': world,
            'gui': PythonExpression([
                "'false' if '", headless, "' == 'true' else 'true'"]),
            'verbose': 'false',
        }.items(),
    )

    robot_state_publisher = Node(
        condition=IfCondition(use_gazebo),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    spawn_entity = Node(
        condition=IfCondition(use_gazebo),
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'sendbooster_agv',
            '-x', '0.0', '-y', '0.0', '-z', '0.1',
        ],
    )

    # ================================================================
    # NAV STACK COMPONENTS (with delay relay)
    # ================================================================

    # Topic Delay Relay: simulates hardware communication latency
    # Input:  /odom, /imu/data, /scan_raw_front, /scan_raw_back
    # Output: /odom_delayed, /imu/data_delayed, /scan_raw_front_delayed, /scan_raw_back_delayed
    topic_delay_relay = Node(
        condition=IfCondition(use_nav),
        package='sendbooster_agv_bringup',
        executable='topic_delay_relay.py',
        name='topic_delay_relay',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'odom_delay_ms': odom_delay,
            'imu_delay_ms': imu_delay,
            'scan_delay_ms': scan_delay,
        }],
    )

    # Scan Processor: subscribes to DELAYED scan topics
    scan_processor = Node(
        condition=IfCondition(use_nav),
        package='sendbooster_agv_bringup',
        executable='scan_processor',
        name='scan_processor',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'num_lidars': 2,
            'destination_frame': 'base_link',
            'max_scan_age': 1.0,  # More lenient due to added delay
            'front_angle_min': -1.5708,
            'front_angle_max': 1.5708,
            'front_invert': True,
            'back_angle_min': -1.5708,
            'back_angle_max': 1.5708,
            'back_invert': False,
        }],
        remappings=[
            ('scan_raw_front', 'scan_raw_front_delayed'),
            ('scan_raw_back', 'scan_raw_back_delayed'),
        ],
    )

    # EKF: subscribes to DELAYED odom/imu topics
    ekf_node = Node(
        condition=IfCondition(use_nav),
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': True}],
        remappings=[
            ('odom', 'odom_delayed'),
            ('imu/data', 'imu/data_delayed'),
        ],
    )

    # Nav2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        condition=IfCondition(use_nav),
        launch_arguments={
            'map': map_file,
            'params_file': nav2_config,
            'autostart': 'True',
            'use_composition': 'True',
            'use_sim_time': 'True',
        }.items(),
    )

    return LaunchDescription(
        declare_args + [
            set_gazebo_model_path,
            # Gazebo
            gazebo_launch,
            robot_state_publisher,
            spawn_entity,
            # Nav (with delay relay)
            topic_delay_relay,
            scan_processor,
            ekf_node,
            nav2_launch,
        ]
    )
