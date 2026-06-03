from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    """
    RViz simulation launch file for path_manager
    Automatically sets: rviz_simulation=true, enable_visualization=true

    Usage:
        ros2 launch path_manager rviz_path_manager.launch.py scenario:=scenario_basic
    """

    pkg_share = FindPackageShare('path_manager')
    path_manager_launch = PathJoinSubstitution([pkg_share, 'launch', 'path_manager.launch.py'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario',
            default_value='scenario_basic',
            description='Scenario configuration file (e.g., scenario_basic, scenario_complex)'
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
        DeclareLaunchArgument(
            'world',
            default_value='',
            description='Map name used to synthesize the ESDF cache path. Leave '
                        'empty to fall back to optimizer_params.yaml manager/world.'
        ),
        # Include base path_manager launch with RViz defaults
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(path_manager_launch),
            launch_arguments={
                'rviz_simulation': 'true',
                'enable_visualization': 'true',
                'scenario': LaunchConfiguration('scenario'),
                'drone_id': LaunchConfiguration('drone_id'),
                'record_bag': LaunchConfiguration('record_bag'),
                'disable_file_logging': LaunchConfiguration('disable_file_logging'),
                'world': LaunchConfiguration('world'),
            }.items()
        ),

        # TEMP follower: samples /planning/trajectory and publishes
        # /dynamics/sim_state + /dynamics/sim_path + drone_0_base TF so RViz can
        # show the moving agent. Replace with mmp_dynamics_sim when ready.
        Node(
            package='mmp_dummy_follower',
            executable='dummy_follower_node',
            name='dummy_follower_node',
            output='screen',
        ),
    ])
