from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'system_id',
            default_value='1',
            description='MAVLink system ID for this vehicle'
        ),

        Node(
            package='jfi_bridge',
            executable='jfi_bridge_node',
            name='jfi_bridge_node',
            output='screen',
            parameters=[{
                'system_id': LaunchConfiguration('system_id'),
            }],
            remappings=[
                # Add any topic remappings here if needed
            ]
        ),
    ])
