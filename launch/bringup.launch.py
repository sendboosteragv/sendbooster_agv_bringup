"""
Launch file for Sendbooster AGV (Real Robot)

Launches: URDF TF + Motor Driver + AHRS IMU + EKF + LiDAR + Scan Merger

Usage:
  ros2 launch sendbooster_agv_bringup bringup.launch.py                   # 2 LiDAR (default)
  ros2 launch sendbooster_agv_bringup bringup.launch.py num_lidars:=1     # 1 LiDAR (front only)
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
    ekf_config = os.path.join(bringup_dir, 'config', 'ekf.yaml')

    num_lidars = LaunchConfiguration('num_lidars')
    lidar_front_port = LaunchConfiguration('lidar_front_port')
    lidar_back_port = LaunchConfiguration('lidar_back_port')

    # URDF
    sim_dir = get_package_share_directory('sendbooster_agv_simulation')
    urdf_file = os.path.join(sim_dir, 'urdf', 'sendbooster_agv.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

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
            'encoder_resolution': 6535,
            'max_rpm': 100,
            'control_rate': 50.0,
            'publish_rate': 50.0,
            'cmd_vel_timeout': 0.5,
            'watchdog_timeout': 1.0,
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_link',
            'publish_tf': False,
        }]
    )

    # ── AHRS IMU ──
    ahrs_node = Node(
        package='stella_ahrs',
        executable='stella_ahrs_node',
        name='stella_ahrs_node',
        output='screen',
        parameters=[{
            'port': '/dev/ttyUSB0',
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
            ('scan', 'scan'),
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
            'scan_mode': 'Sensitivity',
        }],
        remappings=[
            ('scan', 'scan2'),
        ],
    )

    # ── Scan Merger: 2 LiDAR → /scan_merged ──
    scan_merger_node = Node(
        condition=IfCondition(PythonExpression(["'", num_lidars, "' != '1'"])),
        package='sendbooster_agv_bringup',
        executable='laser_scan_merger',
        name='laser_scan_merger',
        output='screen',
        parameters=[{
            'destination_frame': 'base_link',
            'scan_destination_topic': '/scan_merged',
        }],
    )

    # ── Scan Relay: 1 LiDAR → /scan을 /scan_merged로 릴레이 ──
    scan_relay_node = Node(
        condition=IfCondition(PythonExpression(["'", num_lidars, "' == '1'"])),
        package='topic_tools',
        executable='relay',
        name='scan_relay',
        output='screen',
        parameters=[{
            'input_topic': '/scan',
            'output_topic': '/scan_merged',
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('num_lidars', default_value='2',
                              description='Number of LiDARs (1 or 2)'),
        DeclareLaunchArgument('lidar_front_port', default_value='/dev/rplidar_front',
                              description='Front LiDAR serial port'),
        DeclareLaunchArgument('lidar_back_port', default_value='/dev/rplidar_back',
                              description='Back LiDAR serial port'),

        robot_state_publisher_node,
        motor_driver_node,
        ahrs_node,
        ekf_node,
        lidar_front_node,
        lidar_back_node,
        scan_merger_node,
        scan_relay_node,
    ])
