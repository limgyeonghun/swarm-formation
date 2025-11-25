from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    scenario = LaunchConfiguration('scenario', default='default')

    # Reference path_manager package for config files
    pkg_path_manager = FindPackageShare('path_manager')

    # Path to obstacles.yaml in path_manager
    obstacles_param_file = PathJoinSubstitution([
        pkg_path_manager,
        'config',
        'obstacles.yaml'
    ])

    # Path to drone_hardware.yaml in path_manager
    drones_param_file = PathJoinSubstitution([
        pkg_path_manager,
        'config',
        'drone_hardware.yaml'
    ])

    # Path to scenario file (for initial positions)
    scenario_param_file = PathJoinSubstitution([
        pkg_path_manager,
        'config',
        ['scenario_', scenario, '.yaml']
    ])

    # Path to optimizer_params.yaml in path_manager
    optimizer_param_file = PathJoinSubstitution([
        pkg_path_manager,
        'config',
        'optimizer_params.yaml'
    ])

    # Path to map.yaml in path_manager (for road boundary parameters)
    map_param_file = PathJoinSubstitution([
        pkg_path_manager,
        'config',
        'map.yaml'
    ])

    # RViz config (still from path_visualization)
    pkg_path_visualization = FindPackageShare('path_visualization')
    rviz_config_file = PathJoinSubstitution([
        pkg_path_visualization,
        'config',
        'rviz_config.rviz'
    ])

    # Path visualization node, passing config files
    path_visualization = Node(
        package='path_visualization',
        executable='path_visualization_node',
        name='path_visualization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            obstacles_param_file,
            drones_param_file,
            scenario_param_file,  # Add scenario file for initial positions
            optimizer_param_file,
            map_param_file
        ]
    )

    # RViz2 node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'scenario',
            default_value='default',
            description='Scenario name for initial drone positions'
        ),
        path_visualization,
        rviz2_node
    ])