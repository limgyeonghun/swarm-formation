from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 런치 인자 선언
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 패키지 경로
    pkg_path_visualization = FindPackageShare('path_visualization')

    # 파라미터 파일 경로
    obstacles_param_file = PathJoinSubstitution([
        pkg_path_visualization,
        'config',
        'obstacles.yaml'
    ])

    # RViz 설정 파일 경로
    rviz_config_file = PathJoinSubstitution([
        pkg_path_visualization,
        'config',
        'rviz_config.rviz'
    ])

    # path_visualization 정의
    path_visualization = Node(
        package='path_visualization',
        executable='path_visualization',
        name='path_visualization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            obstacles_param_file
        ]
    )

    # RViz2 노드 정의
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