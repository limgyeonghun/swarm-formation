from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    pkg_path_visualization = FindPackageShare('path_visualization')

    obstacles_param_file = PathJoinSubstitution([
        pkg_path_visualization,
        'config',
        'obstacles.yaml'
    ])

    # RViz config (path_visualization/config/rviz_config.rviz)
    rviz_config_file = PathJoinSubstitution([
        pkg_path_visualization,
        'config',
        'rviz_config.rviz'
    ])

    path_visualization = Node(
        package='path_visualization',
        executable='path_visualization_node',
        name='path_visualization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            obstacles_param_file
        ]
    )

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
        path_visualization,
        rviz2_node
    ])