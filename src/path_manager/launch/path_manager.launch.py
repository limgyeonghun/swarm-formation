from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    visualize = LaunchConfiguration('visualize', default='false')

    pkg_path_manager = FindPackageShare('path_manager')

    obstacles_param_file = PathJoinSubstitution([
        pkg_path_manager,
        'config',
        'obstacles.yaml'
    ])

    optimizer_params_file = PathJoinSubstitution([
    pkg_path_manager,
    'config',
    'optimizer_params.yaml'
    ])

    path_manager = Node(
        package='path_manager',
        executable='path_manager_node',
        name='path_manager',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            obstacles_param_file,
            optimizer_params_file
        ]
    )

    # visualization launch file
    visualization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('path_visualization'),
                'launch',
                'path_visualization.launch.py'
            ])
        ]),
        condition=IfCondition(visualize)
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'visualize',
            default_value='false',
            description='Enable visualization with path_visualization and RViz if true'
        ),
        path_manager,
        visualization_launch
    ])