from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Declare launch arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'rviz_simulation',
            default_value='false',
            description='Enable RViz visualization (default: false)'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'drone_id',
            default_value='1',
            description='Target drone ID to run (default: 1)'
        )
    )

    # Include path_manager.launch.py (control nodes only)
    path_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('path_manager'),
                'launch',
                'path_manager.launch.py',
            ])
        ]),
        launch_arguments={
            'drone_id': LaunchConfiguration('drone_id'),
            'real': 'false',
            'rviz_simulation': LaunchConfiguration('rviz_simulation'),
        }.items()
    )

    return LaunchDescription(declared_arguments + [path_manager_launch])
