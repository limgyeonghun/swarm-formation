from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import yaml
import os

def load_yaml_file(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

def create_drone_nodes(context, *args, **kwargs):
    visualize = LaunchConfiguration('visualize')

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

    drones_param_file = PathJoinSubstitution([
        pkg_path_manager,
        'config',
        'drones.yaml'
    ])

    map_param_file = PathJoinSubstitution([
        pkg_path_manager,
        'config',
        'map.yaml'
    ])

    drones_param_file_path = context.perform_substitution(drones_param_file)

    drones_params = load_yaml_file(drones_param_file_path)

    if '/**' not in drones_params:
        raise RuntimeError("drones.yaml does not contain '/**' namespace")

    drone_config = drones_params['/**']['ros__parameters']

    num_drones = drone_config.get('num_drones', 1)
    if not isinstance(num_drones, int) or num_drones < 1:
        raise RuntimeError(f"Invalid num_drones in drones.yaml: {num_drones}")

    print(f"Launching {num_drones} drones from drones.yaml")

    replan_fsm_nodes = []
    rover_control_nodes = []
    for i in range(num_drones):
        drone_key = f'drone_{i}'
        if drone_key not in drone_config:
            raise RuntimeError(f"Drone configuration for {drone_key} not found in drones.yaml")

        drone_params = drone_config[drone_key]

        params = {
            'visualize': visualize,
            'drone_id': drone_params['drone_id'],
            'start_point_x': float(drone_params['start_point_x']),
            'start_point_y': float(drone_params['start_point_y']),
            'start_point_z': float(drone_params['start_point_z']),
            'end_point_x': float(drone_params['end_point_x']),
            'end_point_y': float(drone_params['end_point_y']),
            'end_point_z': float(drone_params['end_point_z']),
        }

        print(f"Creating drone {drone_params['drone_id']}: start=({params['start_point_x']}, {params['start_point_y']}, {params['start_point_z']}), end=({params['end_point_x']}, {params['end_point_y']}, {params['end_point_z']})")

        replan_fsm_node = Node(
            package='path_manager',
            executable='path_manager_node',
            name=f'replan_fsm_drone_{i}',
            output='screen',
            parameters=[
                params,
                obstacles_param_file,
                optimizer_params_file,
                drones_param_file,
                map_param_file
            ],
        )
        replan_fsm_nodes.append(replan_fsm_node)

        rover_control_node = Node(
            package='rover_control',
            executable='rover_control_node',
            name=f'RoverControl_drone_{i}',
            output='screen',
            parameters=[
                {'rover_id': drone_params['drone_id']},
                {'visualize': visualize},
                {'start_point_x': drone_params['start_point_x']},
                {'start_point_y': drone_params['start_point_y']},
                {'start_point_z': drone_params['start_point_z']},
            ]
        )
        rover_control_nodes.append(rover_control_node)

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

    # Wrap replan_fsm_nodes and visualization_launch in a TimerAction to delay by 5 seconds
    delayed_nodes = TimerAction(
        period=0.0,
        actions=replan_fsm_nodes + [visualization_launch]
    )

    # Return rover_control_nodes immediately, and the delayed nodes
    return rover_control_nodes + [delayed_nodes]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'visualize',
            default_value='false',
            description='Enable visualization with path_visualization and RViz if true'
        ),
        OpaqueFunction(function=create_drone_nodes)
    ])