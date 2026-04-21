from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    """
    RViz simulation launch file for path_manager
    Automatically sets: real=false, rviz_simulation=true, enable_visualization=true

    Usage:
        ros2 launch path_manager rviz_path_manager.launch.py scenario:=scenario_sam_defense
    """

    pkg_share = FindPackageShare('path_manager')
    path_manager_launch = PathJoinSubstitution([pkg_share, 'launch', 'path_manager.launch.py'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario',
            default_value='scenario_basic',
            description='Scenario configuration file (e.g., scenario_basic, scenario_sam_defense, scenario_complex)'
        ),
        DeclareLaunchArgument(
            'map_config',
            default_value='map_threat_zones',
            description='Map configuration file (default: map_threat_zones)'
        ),
        DeclareLaunchArgument(
            'drone_id',
            default_value='1',
            description='Target drone ID to run (0-5)'
        ),
        DeclareLaunchArgument(
            'record_bag',
            default_value='false',
            description='Enable rosbag recording for trajectory topics'
        ),
        DeclareLaunchArgument(
            'disable_file_logging',
            default_value='false',
            description='Disable file logging (logs will only appear in console)'
        ),
        # Include base path_manager launch with RViz defaults
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(path_manager_launch),
            launch_arguments={
                'real': 'false',
                'rviz_simulation': 'true',
                'enable_visualization': 'true',
                'scenario': LaunchConfiguration('scenario'),
                'map_config': LaunchConfiguration('map_config'),
                'drone_id': LaunchConfiguration('drone_id'),
                'record_bag': LaunchConfiguration('record_bag'),
                'disable_file_logging': LaunchConfiguration('disable_file_logging'),
            }.items()
        ),
    ])
