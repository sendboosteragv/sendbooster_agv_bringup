"""
MPPI Controller simulation launch for Sendbooster AGV

Same as simulation.launch.py but uses MPPI controller (nav2_params_mppi.yaml).
Defaults to AMCL mode with my_map for testing.

Usage:
  # AMCL mode with my_map (default):
  ros2 launch sendbooster_agv_bringup simulation_mppi.launch.py

  # Odom-only mode:
  ros2 launch sendbooster_agv_bringup simulation_mppi.launch.py localization:=odom

  # Custom map:
  ros2 launch sendbooster_agv_bringup simulation_mppi.launch.py map:=/path/to/map.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('sendbooster_agv_bringup')

    try:
        gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    except Exception:
        gazebo_ros_dir = ''

    ekf_config = os.path.join(bringup_dir, 'config', 'ekf.yaml')
    default_nav2_config = os.path.join(bringup_dir, 'config', 'nav2_params_mppi.yaml')
    default_world = os.path.join(bringup_dir, 'worlds', 'map_world.world')
    default_map = os.path.join(bringup_dir, 'map', 'my_map.yaml')
    keepout_mask = os.path.join(bringup_dir, 'map', 'keepout_mask.yaml')
    gazebo_urdf = os.path.join(bringup_dir, 'urdf', 'sendbooster_agv_gazebo.urdf')

    robot_description = ''
    if os.path.exists(gazebo_urdf):
        with open(gazebo_urdf, 'r') as f:
            robot_description = f.read()

    # Launch configurations
    use_gazebo = LaunchConfiguration('gazebo')
    use_nav = LaunchConfiguration('nav')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    map_file = LaunchConfiguration('map')
    localization = LaunchConfiguration('localization')
    params_file = LaunchConfiguration('params_file')

    # Conditions
    nav_and_odom = PythonExpression([
        "'", use_nav, "' == 'true' and '", localization, "' == 'odom'"])
    nav_and_amcl = PythonExpression([
        "'", use_nav, "' == 'true' and '", localization, "' == 'amcl'"])

    declare_args = [
        DeclareLaunchArgument('gazebo', default_value='true',
                              description='Launch Gazebo simulator'),
        DeclareLaunchArgument('nav', default_value='true',
                              description='Launch Nav2 stack'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run Gazebo headless'),
        DeclareLaunchArgument('world', default_value=default_world,
                              description='Gazebo world file'),
        DeclareLaunchArgument('map', default_value=default_map,
                              description='Nav2 map yaml file (amcl mode only)'),
        DeclareLaunchArgument('localization', default_value='amcl',
                              description='Localization mode: odom or amcl'),
        DeclareLaunchArgument('params_file', default_value=default_nav2_config,
                              description='Nav2 params YAML file'),
    ]

    models_dir = os.path.join(bringup_dir, 'models')
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=models_dir + ':/usr/share/gazebo-11/models'
    )

    # ================================================================
    # GAZEBO COMPONENTS
    # ================================================================

    gazebo_launch_file = os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py') if gazebo_ros_dir else ''
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        condition=IfCondition(use_gazebo),
        launch_arguments={
            'world': world,
            'gui': PythonExpression([
                "'false' if '", headless, "' == 'true' else 'true'"]),
            'verbose': 'false',
        }.items(),
    ) if gazebo_ros_dir else SetEnvironmentVariable(name='_GAZEBO_SKIP', value='1')

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
            '-x', '0.0', '-y', '-1.0', '-z', '0.1',
            '-Y', '-1.5708',
        ],
    )

    # ================================================================
    # NAV STACK — COMMON
    # ================================================================

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
        condition=IfCondition(use_nav),
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': True}],
    )

    # ================================================================
    # NAV STACK — ODOM MODE
    # ================================================================

    static_map_odom_tf = Node(
        condition=IfCondition(nav_and_odom),
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}],
    )

    controller_server = Node(
        condition=IfCondition(nav_and_odom),
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}],
    )

    planner_server = Node(
        condition=IfCondition(nav_and_odom),
        package='nav2_planner',
        executable='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}],
    )

    behavior_server = Node(
        condition=IfCondition(nav_and_odom),
        package='nav2_behaviors',
        executable='behavior_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}],
    )

    bt_navigator = Node(
        condition=IfCondition(nav_and_odom),
        package='nav2_bt_navigator',
        executable='bt_navigator',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}],
    )

    lifecycle_manager_odom = Node(
        condition=IfCondition(nav_and_odom),
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

    # ================================================================
    # NAV STACK — AMCL MODE
    # ================================================================

    map_server = Node(
        condition=IfCondition(nav_and_amcl),
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': True, 'yaml_filename': map_file}],
    )

    amcl_node = Node(
        condition=IfCondition(nav_and_amcl),
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}],
    )

    lifecycle_manager_loc = Node(
        condition=IfCondition(nav_and_amcl),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
            'bond_timeout': 30.0,
        }],
    )

    controller_server_amcl = Node(
        condition=IfCondition(nav_and_amcl),
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}],
    )

    planner_server_amcl = Node(
        condition=IfCondition(nav_and_amcl),
        package='nav2_planner',
        executable='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}],
    )

    behavior_server_amcl = Node(
        condition=IfCondition(nav_and_amcl),
        package='nav2_behaviors',
        executable='behavior_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}],
    )

    bt_navigator_amcl = Node(
        condition=IfCondition(nav_and_amcl),
        package='nav2_bt_navigator',
        executable='bt_navigator',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}],
    )

    lifecycle_manager_amcl = Node(
        condition=IfCondition(nav_and_amcl),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': [
                'filter_mask_server',
                'costmap_filter_info_server',
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
    # KEEPOUT ZONE FILTER
    # ================================================================

    filter_mask_server = Node(
        condition=IfCondition(nav_and_amcl),
        package='nav2_map_server',
        executable='map_server',
        name='filter_mask_server',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': True,
            'yaml_filename': keepout_mask,
        }],
    )

    costmap_filter_info_server = Node(
        condition=IfCondition(nav_and_amcl),
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}],
    )

    # ================================================================
    # RVIZ (only when not headless)
    # ================================================================

    rviz_config = os.path.join(bringup_dir, 'rviz', 'nav_sim.rviz')
    rviz_node = Node(
        condition=UnlessCondition(headless),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription(
        declare_args + [
            set_gazebo_model_path,
            # Gazebo
            gazebo_launch,
            robot_state_publisher,
            spawn_entity,
            # Nav — common
            scan_processor,
            ekf_node,
            # Nav — odom mode
            static_map_odom_tf,
            controller_server,
            planner_server,
            behavior_server,
            bt_navigator,
            lifecycle_manager_odom,
            # Nav — amcl mode
            map_server,
            amcl_node,
            lifecycle_manager_loc,
            filter_mask_server,
            costmap_filter_info_server,
            controller_server_amcl,
            planner_server_amcl,
            behavior_server_amcl,
            bt_navigator_amcl,
            lifecycle_manager_amcl,
            # Visualization
            rviz_node,
        ]
    )
