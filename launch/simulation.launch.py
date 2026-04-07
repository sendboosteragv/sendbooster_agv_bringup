"""
Simulation launch for Sendbooster AGV

Launches Gazebo world + robot + Nav2 stack (scan_processor, EKF, Nav2).
Designed for split execution with Docker:
  - gazebo:=true  nav:=false  → Gazebo container (no resource limits)
  - gazebo:=false nav:=true   → Nav container (Jetson Orin Nano limits)
  - gazebo:=true  nav:=true   → All-in-one (for local testing)

Localization modes:
  - localization:=odom  → odom-only (static map→odom, rolling costmap, no AMCL)
  - localization:=amcl  → AMCL + static map (requires map file)

Usage:
  # All-in-one odom-only (recommended for Jetson):
  ros2 launch sendbooster_agv_bringup simulation.launch.py localization:=odom

  # All-in-one with AMCL:
  ros2 launch sendbooster_agv_bringup simulation.launch.py localization:=amcl map:=/path/to/map.yaml

  # Distributed — PC (Gazebo only, set CYCLONEDDS_URI before):
  export CYCLONEDDS_URI='<CycloneDDS><Domain><General><AllowMulticast>true</AllowMulticast></General></Domain></CycloneDDS>'
  export ROS_DOMAIN_ID=30
  ros2 launch sendbooster_agv_bringup simulation.launch.py gazebo:=true nav:=false

  # Distributed — Jetson (Nav2 only):
  export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>l4tbr0</NetworkInterfaceAddress><AllowMulticast>true</AllowMulticast></General></Domain></CycloneDDS>'
  export ROS_DOMAIN_ID=30
  ros2 launch sendbooster_agv_bringup simulation.launch.py gazebo:=false nav:=true localization:=amcl
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

    # gazebo_ros is optional (not installed on Jetson)
    try:
        gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    except Exception:
        gazebo_ros_dir = ''

    ekf_config = os.path.join(bringup_dir, 'config', 'ekf.yaml')
    nav2_odom_config = os.path.join(bringup_dir, 'config', 'nav2_params_odom.yaml')
    nav2_amcl_config = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    default_world = os.path.join(bringup_dir, 'worlds', 'map_world.world')
    default_map = os.path.join(bringup_dir, 'map', 'map.yaml')
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

    # Conditions
    is_odom_mode = PythonExpression(["'", localization, "' == 'odom'"])
    is_amcl_mode = PythonExpression(["'", localization, "' == 'amcl'"])
    nav_and_odom = PythonExpression([
        "'", use_nav, "' == 'true' and '", localization, "' == 'odom'"])
    nav_and_amcl = PythonExpression([
        "'", use_nav, "' == 'true' and '", localization, "' == 'amcl'"])

    # ── Declare Arguments ──
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
        DeclareLaunchArgument('localization', default_value='odom',
                              description='Localization mode: odom (rolling costmap) or amcl (static map)'),
    ]

    models_dir = os.path.join(bringup_dir, 'models')
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=models_dir + ':/usr/share/gazebo-11/models'
    )

    # DDS config: for distributed sim, set env vars before launching:
    #   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    #   export ROS_DOMAIN_ID=30
    # All-in-one mode uses system default RMW (no override needed).

    # ================================================================
    # GAZEBO COMPONENTS (when gazebo:=true)
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
    # NAV STACK — COMMON (when nav:=true)
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
    # NAV STACK — ODOM MODE (no AMCL, rolling costmap)
    # Launches Nav2 nodes individually to avoid map_server/AMCL deps
    # ================================================================

    # Static TF: map → odom (identity, since no AMCL)
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
        parameters=[nav2_odom_config, {'use_sim_time': True}],
    )

    planner_server = Node(
        condition=IfCondition(nav_and_odom),
        package='nav2_planner',
        executable='planner_server',
        output='screen',
        parameters=[nav2_odom_config, {'use_sim_time': True}],
    )

    behavior_server = Node(
        condition=IfCondition(nav_and_odom),
        package='nav2_behaviors',
        executable='behavior_server',
        output='screen',
        parameters=[nav2_odom_config, {'use_sim_time': True}],
    )

    bt_navigator = Node(
        condition=IfCondition(nav_and_odom),
        package='nav2_bt_navigator',
        executable='bt_navigator',
        output='screen',
        parameters=[nav2_odom_config, {'use_sim_time': True}],
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
    # NAV STACK — AMCL MODE (individual nodes, no nav2_bringup)
    # map_server + AMCL for localization, same Nav2 core as odom mode
    # ================================================================

    map_server = Node(
        condition=IfCondition(nav_and_amcl),
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[nav2_amcl_config, {'use_sim_time': True, 'yaml_filename': map_file}],
    )

    amcl_node = Node(
        condition=IfCondition(nav_and_amcl),
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_amcl_config, {'use_sim_time': True}],
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
        parameters=[nav2_amcl_config, {'use_sim_time': True}],
    )

    planner_server_amcl = Node(
        condition=IfCondition(nav_and_amcl),
        package='nav2_planner',
        executable='planner_server',
        output='screen',
        parameters=[nav2_amcl_config, {'use_sim_time': True}],
    )

    behavior_server_amcl = Node(
        condition=IfCondition(nav_and_amcl),
        package='nav2_behaviors',
        executable='behavior_server',
        output='screen',
        parameters=[nav2_amcl_config, {'use_sim_time': True}],
    )

    bt_navigator_amcl = Node(
        condition=IfCondition(nav_and_amcl),
        package='nav2_bt_navigator',
        executable='bt_navigator',
        output='screen',
        parameters=[nav2_amcl_config, {'use_sim_time': True}],
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

    return LaunchDescription(
        declare_args + [
            set_gazebo_model_path,
            # Gazebo components
            gazebo_launch,
            robot_state_publisher,
            spawn_entity,
            # Nav stack — common
            scan_processor,
            ekf_node,
            # Nav stack — odom mode
            static_map_odom_tf,
            controller_server,
            planner_server,
            behavior_server,
            bt_navigator,
            lifecycle_manager_odom,
            # Nav stack — amcl mode
            map_server,
            amcl_node,
            lifecycle_manager_loc,
            controller_server_amcl,
            planner_server_amcl,
            behavior_server_amcl,
            bt_navigator_amcl,
            lifecycle_manager_amcl,
        ]
    )
