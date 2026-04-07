"""
Launch file for Sendbooster AGV (Real Robot)

Launches: URDF TF + Motor Driver (with IMU fusion) + AHRS IMU + LiDAR + Scan Processor + (optional) Nav2

Localization modes (when nav:=true):
  - localization:=odom  → odom-only (static map→odom, rolling costmap, no AMCL) — Jetson-optimized
  - localization:=amcl  → AMCL + static map (requires map file)

Usage:
  # Sensors only (no Nav2):
  ros2 launch sendbooster_agv_bringup bringup.launch.py

  # With Nav2 odom-only (recommended for Jetson):
  ros2 launch sendbooster_agv_bringup bringup.launch.py nav:=true localization:=odom

  # With Nav2 AMCL:
  ros2 launch sendbooster_agv_bringup bringup.launch.py nav:=true localization:=amcl map:=/path/to/map.yaml

  # 1 LiDAR mode:
  ros2 launch sendbooster_agv_bringup bringup.launch.py num_lidars:=1

  # Disable Foxglove bridge:
  ros2 launch sendbooster_agv_bringup bringup.launch.py foxglove:=false
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
    ekf_config = os.path.join(bringup_dir, 'config', 'ekf.yaml')
    cyclonedds_config = os.path.join(bringup_dir, 'config', 'cyclonedds.xml')
    nav2_odom_config = os.path.join(bringup_dir, 'config', 'nav2_params_odom.yaml')
    nav2_amcl_config = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    keepout_mask = os.path.join(bringup_dir, 'map', 'keepout.yaml')

    num_lidars = LaunchConfiguration('num_lidars')
    lidar_front_port = LaunchConfiguration('lidar_front_port')
    lidar_back_port = LaunchConfiguration('lidar_back_port')
    use_nav = LaunchConfiguration('nav')
    localization = LaunchConfiguration('localization')
    map_file = LaunchConfiguration('map')
    use_foxglove = LaunchConfiguration('foxglove')

    # Conditions
    nav_and_odom = PythonExpression([
        "'", use_nav, "' == 'true' and '", localization, "' == 'odom'"])
    nav_and_amcl = PythonExpression([
        "'", use_nav, "' == 'true' and '", localization, "' == 'amcl'"])

    # URDF
    urdf_file = os.path.join(bringup_dir, 'urdf', 'sendbooster_agv.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # ================================================================
    # HARDWARE NODES (always launched)
    # ================================================================

    # ── Robot State Publisher (URDF → static TF) ──
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # ── Motor Driver ──
    motor_driver_node = Node(
        package='sendbooster_agv_bringup',
        executable='motor_driver_node',
        name='motor_driver_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/motor_driver',
            'baudrate': 19200,
            'mdui_id': 184,
            'mdt_id': 183,
            'motor_id': 1,
            'gear_ratio': 10,
            'poles': 10,
            'wheel_radius': 0.0965,
            'wheel_separation': 0.37,
            'encoder_resolution': 65536,
            'max_rpm': 100,
            'control_rate': 10.0,
            'cmd_vel_timeout': 1.0,
            'watchdog_timeout': 1.0,
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_footprint',
            'publish_tf': False,  # EKF가 odom→base_footprint TF 발행
            'use_encoder_odom': True,  # 엔코더 기반 오도메트리 (command-based보다 정확)
            'use_imu': False,     # EKF가 IMU 퓨전 담당
        }]
    )

    # ── AHRS IMU ──
    ahrs_node = Node(
        package='stella_ahrs',
        executable='stella_ahrs_node',
        name='stella_ahrs_node',
        output='screen',
        parameters=[{
            'port': '/dev/imu',
            'baud_rate': 115200,
            'publish_tf': False,
            'frame_id': 'imu_link',
            'parent_frame_id': 'base_link',
        }]
    )

    # ── EKF Fusion (wheel odom + IMU → /odometry/filtered) ──
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
    )

    # ── Front LiDAR (always launched) ──
    lidar_front_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_front',
        output='screen',
        parameters=[{
            'serial_port': lidar_front_port,
            'serial_baudrate': 115200,
            'frame_id': 'base_scan',
            'angle_compensate': True,
            'scan_mode': 'Sensitivity',
        }],
        remappings=[
            ('scan', 'scan_raw_front'),
        ],
    )

    # ── Back LiDAR (only when num_lidars=2) ──
    lidar_back_node = Node(
        condition=IfCondition(PythonExpression(["'", num_lidars, "' != '1'"])),
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_back',
        output='screen',
        parameters=[{
            'serial_port': lidar_back_port,
            'serial_baudrate': 115200,
            'frame_id': 'base_scan_back',
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }],
        remappings=[
            ('scan', 'scan_raw_back'),
        ],
    )

    # ── Scan Processor: filter + merge in one C++ node ──
    scan_processor_node = Node(
        package='sendbooster_agv_bringup',
        executable='scan_processor',
        name='scan_processor',
        output='screen',
        parameters=[{
            'num_lidars': num_lidars,
            'destination_frame': 'base_link',
            'max_scan_age': 0.3,
            'front_angle_min': -1.5708,
            'front_angle_max': 1.5708,
            'front_invert': True,
            'back_angle_min': -1.5708,
            'back_angle_max': 1.5708,
            'back_invert': False,
        }],
    )

    # ── Foxglove Bridge (WebSocket-based remote monitoring, replaces rviz2 over DDS) ──
    foxglove_bridge = Node(
        condition=IfCondition(use_foxglove),
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'send_buffer_limit': 10000000,      # 10MB — 대역폭 제한
            'topic_whitelist': ['.*'],           # 전체 토픽 허용 (필요시 제한)
            'max_qos_depth': 1,                  # 최신 메시지만 (메모리 절약)
            'num_threads': 1,                    # Jetson: 스레드 최소화
            'use_sim_time': False,
        }],
    )

    # ── DDS config (optional, disabled for compatibility) ──
    # To restrict DDS to localhost, uncomment and set CYCLONEDDS_URI in ~/.bashrc
    # set_dds_config = SetEnvironmentVariable(
    #     name='CYCLONEDDS_URI',
    #     value='file://' + cyclonedds_config
    # )

    # ================================================================
    # NAV2 — ODOM MODE (no AMCL, rolling costmap, Jetson-optimized)
    # ================================================================

    # Static TF: map → odom (identity, since no AMCL)
    static_map_odom_tf = Node(
        condition=IfCondition(nav_and_odom),
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    controller_server_odom = Node(
        condition=IfCondition(nav_and_odom),
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[nav2_odom_config],
    )

    planner_server_odom = Node(
        condition=IfCondition(nav_and_odom),
        package='nav2_planner',
        executable='planner_server',
        output='screen',
        parameters=[nav2_odom_config],
    )

    behavior_server_odom = Node(
        condition=IfCondition(nav_and_odom),
        package='nav2_behaviors',
        executable='behavior_server',
        output='screen',
        parameters=[nav2_odom_config],
    )

    bt_navigator_odom = Node(
        condition=IfCondition(nav_and_odom),
        package='nav2_bt_navigator',
        executable='bt_navigator',
        output='screen',
        parameters=[nav2_odom_config],
    )

    lifecycle_manager_odom = Node(
        condition=IfCondition(nav_and_odom),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
            ],
            'bond_timeout': 30.0,
            'attempt_respawn_reconnection': True,
            'bond_respawn_max_duration': 30.0,
        }],
    )

    # ================================================================
    # KEEPOUT ZONE FILTER (AMCL mode only)
    # ================================================================

    filter_mask_server = Node(
        condition=IfCondition(nav_and_amcl),
        package='nav2_map_server',
        executable='map_server',
        name='filter_mask_server',
        output='screen',
        parameters=[nav2_amcl_config, {
            'use_sim_time': False,
            'yaml_filename': keepout_mask,
            'topic_name': 'filter_mask',
        }],
    )

    costmap_filter_info_server = Node(
        condition=IfCondition(nav_and_amcl),
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='screen',
        parameters=[nav2_amcl_config, {'use_sim_time': False}],
    )

    lifecycle_manager_filters = Node(
        condition=IfCondition(nav_and_amcl),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_filters',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': [
                'filter_mask_server',
                'costmap_filter_info_server',
            ],
            'bond_timeout': 30.0,
        }],
    )

    # ================================================================
    # NAV2 — AMCL MODE (map_server + AMCL + full Nav2)
    # ================================================================

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        condition=IfCondition(nav_and_amcl),
        launch_arguments={
            'map': map_file,
            'params_file': nav2_amcl_config,
            'autostart': 'True',
            'use_composition': 'True',
            'use_sim_time': 'False',
        }.items(),
    )

    return LaunchDescription([
        # set_dds_config,
        # Launch arguments
        DeclareLaunchArgument('num_lidars', default_value='2',
                              description='Number of LiDARs (1 or 2)'),
        DeclareLaunchArgument('lidar_front_port', default_value='/dev/rplidar_back',
                              description='Front LiDAR serial port (hw rplidar_back is physically at front)'),
        DeclareLaunchArgument('lidar_back_port', default_value='/dev/rplidar_front',
                              description='Back LiDAR serial port (hw rplidar_front is physically at back)'),
        DeclareLaunchArgument('nav', default_value='false',
                              description='Launch Nav2 navigation stack'),
        DeclareLaunchArgument('localization', default_value='odom',
                              description='Localization mode: odom (rolling costmap) or amcl (static map)'),
        DeclareLaunchArgument('map', default_value='',
                              description='Nav2 map yaml file (amcl mode only)'),
        DeclareLaunchArgument('foxglove', default_value='true',
                              description='Launch Foxglove bridge for remote monitoring'),
        # Monitoring
        foxglove_bridge,
        # Hardware nodes
        robot_state_publisher_node,
        motor_driver_node,
        ahrs_node,
        ekf_node,
        lidar_front_node,
        lidar_back_node,
        scan_processor_node,
        # Nav2 — odom mode
        static_map_odom_tf,
        controller_server_odom,
        planner_server_odom,
        behavior_server_odom,
        bt_navigator_odom,
        lifecycle_manager_odom,
        # Nav2 — amcl mode
        nav2_amcl_launch,
        # Keepout zone filter
        filter_mask_server,
        costmap_filter_info_server,
        lifecycle_manager_filters,
    ])
