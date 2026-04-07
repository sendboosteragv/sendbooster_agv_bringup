"""
Odom-only navigation test — no AMCL, no map_server.
Uses static map→odom transform and rolling-window global costmap.
For isolating controller/planner issues from localization.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('sendbooster_agv_bringup')
    ekf_config = os.path.join(bringup_dir, 'config', 'ekf.yaml')
    nav2_config = os.path.join(bringup_dir, 'config', 'nav2_params_odom.yaml')
    fastdds_xml = os.path.join(bringup_dir, 'config', 'fastdds_udp.xml')

    set_fastdds = SetEnvironmentVariable(
        name='FASTRTPS_DEFAULT_PROFILES_FILE', value=fastdds_xml)

    # Static TF: map → odom (identity)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}],
    )

    scan_processor = Node(
        package='sendbooster_agv_bringup',
        executable='scan_processor',
        name='scan_processor',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'num_lidars': 2,
            'destination_frame': 'base_link',
            'max_scan_age': 0.5,
            'front_angle_min': -1.5708,
            'front_angle_max': 1.5708,
            'front_invert': True,
            'back_angle_min': -1.5708,
            'back_angle_max': 1.5708,
            'back_invert': False,
        }],
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': True}],
    )

    # Nav2 nodes individually (no lifecycle manager race condition)
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[nav2_config, {'use_sim_time': True}],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        output='screen',
        parameters=[nav2_config, {'use_sim_time': True}],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        output='screen',
        parameters=[nav2_config, {'use_sim_time': True}],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        output='screen',
        parameters=[nav2_config, {'use_sim_time': True}],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
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

    return LaunchDescription([
        set_fastdds,
        static_tf,
        scan_processor,
        ekf_node,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager,
    ])
