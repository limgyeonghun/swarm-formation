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
import glob

def load_yaml_file(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

def find_serial_port():
    usb_ports = glob.glob('/dev/ttyUSB*')
    if usb_ports:
        return usb_ports[0]
    acm_ports = glob.glob('/dev/ttyACM*')
    if acm_ports:
        return acm_ports[0]
    return '/dev/ttyUSB0'

def create_drone_nodes(context, *args, **kwargs):
    rviz_sim_str = context.perform_substitution(LaunchConfiguration('rviz_simulation'))
    rviz_sim = (rviz_sim_str.lower() == 'true')

    real_str = context.perform_substitution(LaunchConfiguration('real'))
    real_mode = (real_str.lower() == 'true')

    # Target drone ID
    drone_id_str = context.perform_substitution(LaunchConfiguration('drone_id'))
    target_drone_id = int(drone_id_str)
    print(f"Target drone ID: {target_drone_id}")

    # JFI params
    jfi_port_arg = context.perform_substitution(LaunchConfiguration('jfi_port'))
    jfi_baud_rate_str = context.perform_substitution(LaunchConfiguration('jfi_baud_rate'))
    jfi_baud_rate = int(jfi_baud_rate_str)

    if jfi_port_arg == 'auto':
        jfi_port = find_serial_port()
        print(f"Auto-detected JFI Port: {jfi_port}")
    else:
        jfi_port = jfi_port_arg
        print(f"Manual JFI Port: {jfi_port}")
    print(f"JFI Baud Rate: {jfi_baud_rate}")

    # Config paths
    pkg_share = FindPackageShare('path_manager')
    obstacles_file  = PathJoinSubstitution([pkg_share, 'config', 'obstacles.yaml'])
    optimizer_file  = PathJoinSubstitution([pkg_share, 'config', 'optimizer_params.yaml'])
    drones_file     = PathJoinSubstitution([pkg_share, 'config', 'drones.yaml'])
    map_file        = PathJoinSubstitution([pkg_share, 'config', 'map.yaml'])

    # Load drones.yaml
    drones_params = load_yaml_file(context.perform_substitution(drones_file))
    drone_cfg = drones_params['/**']['ros__parameters']
    num_drones = drone_cfg.get('num_drones', 1)

    fsm_params = drone_cfg.get('fsm', {})
    n_seconds_ahead = float(fsm_params.get('n_seconds_ahead', 0.0))

    target_idle_timeout_sec = 0.25
    arrival_distance_threshold = 0.75

    replan_nodes = []
    traj_nodes   = []
    rover_nodes  = []
    jfi_nodes    = []

    # Drones to run
    if num_drones == 1:
        drones_to_run = [target_drone_id]
        print(f"Single drone mode: running drone {target_drone_id}")
    else:
        drones_to_run = list(range(num_drones))
        print(f"Multi-drone mode: running drones {drones_to_run}")

    # Create nodes per drone
    for drone_id in drones_to_run:
        target_cfg = None
        target_index = 0

        for i in range(6):  # drone_0 .. drone_5
            drone_key = f'drone_{i}'
            if drone_key in drone_cfg:
                if drone_cfg[drone_key]['drone_id'] == drone_id:
                    target_cfg = drone_cfg[drone_key]
                    target_index = i
                    print(f"Found {drone_key} with drone_id={drone_id}")
                    break

        if target_cfg is None:
            print(f"Error: drone_id {drone_id} not found in drones.yaml")
            continue

        cfg = target_cfg
        did = drone_id
        i = target_index

        params = {
            'rviz_simulation': rviz_sim,
            'drone_id':        did,
            'start_point_x':   float(cfg['start_point_x']),
            'start_point_y':   float(cfg['start_point_y']),
            'start_point_z':   float(cfg['start_point_z']),
        }

        remaps = []
        if not real_mode:
            id_str = str(did + 1)
            remaps = [
                (f'V{id_str}/planning/broadcast_traj_send', '/planning/broadcast_traj_recv'),
                (f'V{id_str}/j_fi/broadcast_traj_recv', '/planning/broadcast_traj_recv'),
            ]

        replan_nodes.append(
            Node(
                package='path_manager',
                executable='path_manager_node',
                name=f'replan_fsm_drone_{i}',
                output='screen',
                parameters=[params, obstacles_file, optimizer_file, drones_file, map_file],
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
                    {'rover_id': did},
                    {'rviz_simulation': rviz_sim},
                    {'start_point_x': cfg['start_point_x']},
                    {'start_point_y': cfg['start_point_y']},
                    {'start_point_z': cfg['start_point_z']},
                    {'target_idle_timeout_sec': target_idle_timeout_sec},
                    {'arrival_distance_threshold': arrival_distance_threshold},
                ],
            )
        )

        if real_mode:
            system_id = did + 1
            jfi_nodes.append(
                Node(
                    package='jfi_comm',
                    executable='serial_comm_node',
                    name=f'jfi_comm_drone_{i}',
                    output='screen',
                    parameters=[
                        {'port_name': jfi_port},
                        {'baud_rate': jfi_baud_rate},
                        {'system_id': system_id},
                        {'component_id': 1},
                    ]
                )
            )
            print(f"JFI node added for drone {did} with system_id {system_id}")
        else:
            print("JFI node skipped (not in real mode)")

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

    # Formation commander - no remapping needed as it only publishes formation commands
    formation_commander = Node(
        package='path_manager',
        executable='formation_commander',
        name='formation_commander',
        output='screen',
    )

    immediate_actions = [visualization] + rover_nodes + jfi_nodes

    traj_nodes_delayed = TimerAction(
        period=0.0,
        actions=traj_nodes,
    )

    replan_nodes_delayed = TimerAction(
        period=0.0,
        actions=replan_nodes,
    )

    formation_commander_delayed = TimerAction(
        period=0.0,
        actions=[formation_commander],
    )

    return immediate_actions + [traj_nodes_delayed, replan_nodes_delayed, formation_commander_delayed]

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
        DeclareLaunchArgument(
            'drone_id',
            default_value='1',
            description='Target drone ID to run (0-5)'
        ),
        DeclareLaunchArgument(
            'jfi_port',
            default_value='auto',
            description='JFI serial port device path (use "auto" for auto-detection)'
        ),
        DeclareLaunchArgument(
            'jfi_baud_rate',
            default_value='115200',
            description='JFI serial port baud rate'
        ),
        OpaqueFunction(function=create_drone_nodes),
    ])
