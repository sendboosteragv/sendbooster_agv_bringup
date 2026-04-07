import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Paths
    bringup_dir = get_package_share_directory('sendbooster_agv_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Default map path (from sendbooster_agv_bringup/map/)
    default_map_path = os.path.join(bringup_dir, 'map', 'map.yaml')
    # Launch arguments
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to map yaml file'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'config', 'nav2_params.yaml'),
        description='Full path to Nav2 parameters file'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='False',
        description='Whether to start rviz2 (disable on Jetson)'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Automatically start Nav2 lifecycle nodes'
    )

    use_composition_arg = DeclareLaunchArgument(
        'use_composition',
        default_value='True',
        description='Use composed nodes (single process, saves RAM)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation clock (True for Gazebo)'
    )

    # Include nav2_bringup launch
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': LaunchConfiguration('autostart'),
            'use_composition': LaunchConfiguration('use_composition'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    # Optional rviz2
    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')],
        output='screen',
    )

    return LaunchDescription([
        map_arg,
        params_file_arg,
        use_rviz_arg,
        autostart_arg,
        use_composition_arg,
        use_sim_time_arg,
        nav2_bringup_launch,
        rviz_node,
    ])
