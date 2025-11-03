from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    ExecuteProcess,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_setup(context, *args, **kwargs):
    # Configuration
    px4_src_path = LaunchConfiguration('px4_src_path').perform(context)
    world_name = LaunchConfiguration('world_name').perform(context)
    num_rovers = int(LaunchConfiguration('num_rovers').perform(context))
    rviz_simulation = LaunchConfiguration('rviz_simulation').perform(context)
    drone_id = LaunchConfiguration('drone_id').perform(context)
    rover_ids_str = LaunchConfiguration('rover_ids').perform(context)

    # Parse rover IDs (comma-separated string like "1,4,5,9")
    if rover_ids_str:
        rover_ids = [int(x.strip()) for x in rover_ids_str.split(',')]
    else:
        # Default: use sequential IDs starting from 0
        rover_ids = list(range(num_rovers))

    print(f"Using rover IDs: {rover_ids}")
    print(f"Number of rovers: {len(rover_ids)}")

    # MicroXRCEAgent for PX4-ROS2 communication
    xrce_agent_process = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen',
    )

    # Rover initial positions based on your command
    rover_poses = [
        "-234.56,115.33,12.67,0,0,-2.98",  # Rover 0 (instance 0)
        "-232.57,115.57,12.7,0,0,-2.99",   # Rover 1 (instance 1)
        "-230.59,115.82,12.64,0,0,-3",     # Rover 2 (instance 2)
        "-228.6,116.06,12.68,0,0,-3",      # Rover 3 (instance 3)
    ]

    nodes_to_start = [
        xrce_agent_process,
    ]

    # Create PX4 instances for each rover
    for idx, rover_id in enumerate(rover_ids):
        # Use idx for position in rover_poses array
        # Use rover_id for PX4 instance ID
        pose_idx = idx if idx < len(rover_poses) else 0

        if idx == 0:
            # First rover launches Gazebo with world
            env_dict = {
                'GZ_SIM_RESOURCE_PATH': f'{px4_src_path}/Tools/simulation/gz/models:{px4_src_path}/Tools/simulation/gz/worlds',
                'GZ_IP': '127.0.0.1',
                'PX4_GZ_WORLD': world_name,
                'PX4_GZ_WORLDS': f'{px4_src_path}/Tools/simulation/gz/worlds',
                'PX4_GZ_MODELS': f'{px4_src_path}/Tools/simulation/gz/models',
                'PX4_SYS_AUTOSTART': '4012',
                'PX4_GZ_MODEL_POSE': rover_poses[pose_idx],
                'PX4_SIM_MODEL': 'gz_rover_ackermann',
            }
        else:
            # Subsequent rovers connect to existing Gazebo (standalone mode)
            env_dict = {
                'GZ_SIM_RESOURCE_PATH': f'{px4_src_path}/Tools/simulation/gz/models:{px4_src_path}/Tools/simulation/gz/worlds',
                'GZ_IP': '127.0.0.1',
                'PX4_GZ_STANDALONE': '1',
                'PX4_GZ_WORLD': world_name,
                'PX4_GZ_WORLDS': f'{px4_src_path}/Tools/simulation/gz/worlds',
                'PX4_GZ_MODELS': f'{px4_src_path}/Tools/simulation/gz/models',
                'PX4_SYS_AUTOSTART': '4012',
                'PX4_GZ_MODEL_POSE': rover_poses[pose_idx],
                'PX4_SIM_MODEL': 'gz_rover_ackermann',
            }

        px4_process = ExecuteProcess(
            cmd=[
                f'{px4_src_path}/build/px4_sitl_default/bin/px4',
                '-i', str(rover_id),  # Use custom rover ID here
            ],
            additional_env=env_dict,
            output='screen',
        )

        if idx == 0:
            # Launch first rover immediately
            nodes_to_start.append(px4_process)
        else:
            # Delay subsequent rovers by 3 seconds each
            delay = 3.0 * idx
            nodes_to_start.append(
                TimerAction(
                    period=delay,
                    actions=[px4_process]
                )
            )

    # Include the path_manager.launch.py after all rovers are started
    # Wait for all rovers to be ready (3 seconds per rover + 5 seconds buffer)
    path_manager_delay = (len(rover_ids) * 3.0) + 5.0

    path_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('path_manager'),
                'launch',
                'path_manager.launch.py',
            ])
        ]),
        launch_arguments={
            'drone_id': drone_id,
            'real': 'false',
            'rviz_simulation': rviz_simulation,
        }.items()
    )

    nodes_to_start.append(
        TimerAction(
            period=path_manager_delay,
            actions=[path_manager_launch]
        )
    )

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'px4_src_path',
            default_value='/home/suv/ws/PX4/main/PX4-Autopilot',
            description='PX4 source code path (default: /home/suv/ws/PX4/main/PX4-Autopilot)'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'world_name',
            default_value='c-track',
            description='Gazebo world name (without .sdf extension)'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'num_rovers',
            default_value='4',
            description='Number of rovers to spawn (default: 4)'
        )
    )

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

    declared_arguments.append(
        DeclareLaunchArgument(
            'rover_ids',
            default_value='',
            description='Comma-separated rover IDs to spawn (e.g., "1,4,5,9"). If empty, uses sequential IDs 0,1,2,3...'
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
