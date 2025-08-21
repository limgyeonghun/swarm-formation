from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import yaml
import os

def load_yaml_file(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

def create_drone_nodes(context, *args, **kwargs):
    rviz_sim_str = context.perform_substitution(LaunchConfiguration('rviz_simulation'))
    rviz_sim = (rviz_sim_str.lower() == 'true')

    real_str = context.perform_substitution(LaunchConfiguration('real'))
    real_mode = (real_str.lower() == 'true')

    pkg_share = FindPackageShare('path_manager')
    obstacles_file  = PathJoinSubstitution([pkg_share, 'config', 'obstacles.yaml'])
    optimizer_file  = PathJoinSubstitution([pkg_share, 'config', 'optimizer_params.yaml'])
    drones_file     = PathJoinSubstitution([pkg_share, 'config', 'drones.yaml'])
    map_file        = PathJoinSubstitution([pkg_share, 'config', 'map.yaml'])

    drones_params = load_yaml_file(context.perform_substitution(drones_file))
    drone_cfg = drones_params['/**']['ros__parameters']
    num_drones = drone_cfg.get('num_drones', 1)

    fsm_params = drone_cfg.get('fsm', {})
    n_seconds_ahead = float(fsm_params.get('n_seconds_ahead', 0.0))

    target_idle_timeout_sec = 0.25
    arrival_distance_threshold = 0.25

    replan_nodes = []
    traj_nodes   = []
    rover_nodes  = []

    for i in range(num_drones):
        cfg = drone_cfg[f'drone_{i}']
        did = cfg['drone_id']

        params = {
            'rviz_simulation': rviz_sim,    # bool
            'drone_id':        did,
            'start_point_x':   float(cfg['start_point_x']),
            'start_point_y':   float(cfg['start_point_y']),
            'start_point_z':   float(cfg['start_point_z']),
            'end_point_x':     float(cfg['end_point_x']),
            'end_point_y':     float(cfg['end_point_y']),
            'end_point_z':     float(cfg['end_point_z']),
        }

        remaps = []
        if not real_mode:
            id_str = str(did+1)
            remaps = [
                (
                    f'V{id_str}/planning/broadcast_traj_send',
                    '/planning/broadcast_traj_recv'
                ),
                (
                    f'V{id_str}/j_fi/broadcast_traj_recv',
                    '/planning/broadcast_traj_recv'
                ),
            ]

        replan_nodes.append(
            Node(
                package='path_manager',
                executable='path_manager_node',
                name=f'replan_fsm_drone_{i}',
                output='screen',
                parameters=[
                    params,
                    obstacles_file,
                    optimizer_file,
                    drones_file,
                    map_file,
                ],
                remappings=remaps,
            )
        )

        traj_nodes.append(
            Node(
                package='path_manager',
                executable='traj_server',
                name=f'TrajServer_drone_{i}',
                output='screen',
                parameters=[{
                    'drone_id': did,
                    'rviz_simulation': rviz_sim,
                    'fsm/n_seconds_ahead': n_seconds_ahead,
                }],
            )
        )

        rover_nodes.append(
            Node(
                package='rover_control',
                executable='rover_control_node',
                name=f'RoverControl_drone_{i}',
                output='screen',
                parameters=[
                    {'rover_id':        did},
                    {'rviz_simulation': rviz_sim},  # bool
                    {'start_point_x':   cfg['start_point_x']},
                    {'start_point_y':   cfg['start_point_y']},
                    {'start_point_z':   cfg['start_point_z']},
                    {'target_idle_timeout_sec':   target_idle_timeout_sec},
                    {'arrival_distance_threshold':   arrival_distance_threshold},
                ],
            )
        )

    visualization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('path_visualization'),
                'launch',
                'path_visualization.launch.py',
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('rviz_simulation'))
    )

    delayed = TimerAction(
        period = 0.0,
        actions = traj_nodes + replan_nodes + [visualization],
    )

    return rover_nodes + [delayed]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_simulation',
            default_value='false',
            description='Enable RViz visualization (bool)'
        ),
        DeclareLaunchArgument(
            'real',
            default_value='false',
            description='Enable real-topic remapping'
        ),
        OpaqueFunction(function=create_drone_nodes),
    ])
