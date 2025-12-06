from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
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
import os
from datetime import datetime

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

    record_bag_str = context.perform_substitution(LaunchConfiguration('record_bag'))
    record_bag = (record_bag_str.lower() == 'true')

    # NOTE: real_mode and rviz_sim are independent:
    # - real_mode=true: Use JFI serial communication
    # - rviz_sim=true: Use trajectory-based position (no real PX4)
    # - real_mode=true + rviz_sim=true: Jetson environment (serial + simulated position)

    # Target drone ID
    drone_id_str = context.perform_substitution(LaunchConfiguration('drone_id'))
    target_drone_id = int(drone_id_str)
    print(f"Target drone ID: {target_drone_id}")

    # Scenario selection
    scenario = context.perform_substitution(LaunchConfiguration('scenario'))
    print(f"Selected scenario: {scenario}")

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
    map_file = PathJoinSubstitution([pkg_share, 'config', 'map.yaml'])

    # Load base drone hardware configuration
    drones_file = PathJoinSubstitution([pkg_share, 'config', 'drone_hardware.yaml'])
    drones_params = load_yaml_file(context.perform_substitution(drones_file))
    drone_cfg = drones_params['/**']['ros__parameters']
    num_drones = drone_cfg.get('num_drones', 1)
    print(f"Loaded drone hardware config: drone_hardware.yaml (num_drones={num_drones})")

    # Load scenario-specific configuration (initial positions + mission)
    scenario_filename = f'scenario_{scenario}.yaml'
    scenario_file = PathJoinSubstitution([pkg_share, 'config', scenario_filename])
    scenario_params = load_yaml_file(context.perform_substitution(scenario_file))
    scenario_cfg = scenario_params['/**']['ros__parameters']
    print(f"Loaded scenario config: {scenario_filename}")

    # Add drone initial positions from scenario to hardware config
    for i in range(num_drones):
        drone_key = f'drone_{i}'
        if drone_key in scenario_cfg and drone_key in drone_cfg:
            # Add start positions from scenario to drone config
            drone_cfg[drone_key]['start_point_x'] = scenario_cfg[drone_key]['start_point_x']
            drone_cfg[drone_key]['start_point_y'] = scenario_cfg[drone_key]['start_point_y']
            drone_cfg[drone_key]['start_point_z'] = scenario_cfg[drone_key]['start_point_z']
            print(f"  Added {drone_key} start position from scenario: ({scenario_cfg[drone_key]['start_point_x']}, {scenario_cfg[drone_key]['start_point_y']}, {scenario_cfg[drone_key]['start_point_z']})")

    fsm_params = drone_cfg.get('fsm', {})
    n_seconds_ahead = float(fsm_params.get('n_seconds_ahead', 0.0))

    target_idle_timeout_sec = 0.25
    arrival_distance_threshold = 0.75

    replan_nodes = []
    traj_nodes   = []
    rover_nodes  = []
    jfi_nodes    = []

    # Drones to run - real mode runs single drone, simulation runs all
    if real_mode:
        # Real mode: run only the specified drone
        drones_to_run = [target_drone_id]
        print(f"Real mode: running only drone {target_drone_id}")
    else:
        # Simulation mode: run all drones from config
        drones_to_run = []
        for i in range(6):  # Check drone_0 to drone_5
            drone_key = f'drone_{i}'
            if drone_key in drone_cfg:
                drones_to_run.append(drone_cfg[drone_key]['index'])
        print(f"Simulation mode: running drones {drones_to_run}")

    # Create nodes per drone
    for drone_index in drones_to_run:
        target_cfg = None
        target_key_index = 0

        for i in range(6):  # drone_0 .. drone_5
            drone_key = f'drone_{i}'
            if drone_key in drone_cfg:
                if drone_cfg[drone_key]['index'] == drone_index:
                    target_cfg = drone_cfg[drone_key]
                    target_key_index = i
                    print(f"Found {drone_key} with index={drone_index}")
                    break

        if target_cfg is None:
            print(f"Error: index {drone_index} not found in drones.yaml")
            continue

        cfg = target_cfg
        idx = drone_index
        i = target_key_index

        # Get mavlink_id from config, or default to index + 1
        mavlink_id = cfg.get('mavlink_id', idx + 1)

        params = {
            'rviz_simulation': rviz_sim,
            'drone_id':        idx,
            'mavlink_id':      mavlink_id,
            'start_point_x':   float(cfg.get('start_point_x', 0.0)),
            'start_point_y':   float(cfg.get('start_point_y', 0.0)),
            'start_point_z':   float(cfg.get('start_point_z', 0.0)),
        }

        # Add ALL drones' initial positions for Hungarian algorithm (deadlock prevention)
        for j in range(num_drones):
            drone_key_j = f'drone_{j}'
            if drone_key_j in drone_cfg:
                params[f'{drone_key_j}.start_point_x'] = float(drone_cfg[drone_key_j]['start_point_x'])
                params[f'{drone_key_j}.start_point_y'] = float(drone_cfg[drone_key_j]['start_point_y'])
                params[f'{drone_key_j}.start_point_z'] = float(drone_cfg[drone_key_j]['start_point_z'])

        remaps = []
        id_str = str(idx + 1)
        if not real_mode:
            # Simulation mode: Remap FSM's /V{id}/formation_command to Commander's /formation_command
            # Also remap trajectory topics to shared /planning/broadcast_traj_recv
            remaps = [
                (f'/V{id_str}/formation_command', '/formation_command'),
                (f'V{id_str}/planning/broadcast_traj_send', '/planning/broadcast_traj_recv'),
                (f'V{id_str}/j_fi/broadcast_traj_recv', '/planning/broadcast_traj_recv'),
            ]
        else:
            # Real mode: No remapping needed
            # - Drone 0: Commander publishes /formation_command -> JFI0 subscribes /formation_command -> Serial
            # - Drone 1,2,3: JFI publishes /V{id}/formation_command -> FSM subscribes /V{id}/formation_command
            # For Drone 0 FSM: Need to remap /V1/formation_command -> /formation_command (to receive from Commander directly)
            if idx == 0:
                remaps = [
                    (f'/V{id_str}/formation_command', '/formation_command'),
                ]
            else:
                remaps = []

        # No additional remapping needed - formation_targets now uses topic_prefix directly
        all_remaps = remaps

        replan_nodes.append(
            Node(
                package='path_manager',
                executable='path_manager_node',
                name=f'replan_fsm_drone_{i}',
                output='screen',
                parameters=[params, obstacles_file, optimizer_file, drones_file, map_file],
                remappings=all_remaps,
            )
        )

        traj_nodes.append(
            Node(
                package='path_manager',
                executable='traj_server',
                name=f'TrajServer_drone_{i}',
                output='screen',
                parameters=[params,optimizer_file],
            )
        )

        rover_nodes.append(
            Node(
                package='rover_control',
                executable='rover_control_node',
                name=f'RoverControl_drone_{i}',
                output='screen',
                parameters=[
                    {'index': idx},
                    {'mavlink_id': mavlink_id},
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
            # Use namespace to isolate jfi_comm topics per drone
            jfi_namespace = f'drone_{idx}'

            # JFI serial communication node
            jfi_nodes.append(
                Node(
                    package='jfi_comm',
                    executable='serial_comm_node',
                    name=f'jfi_comm_drone_{i}',
                    namespace=jfi_namespace,
                    output='screen',
                    parameters=[
                        {'port_name': jfi_port},
                        {'baud_rate': jfi_baud_rate},
                        {'system_id': mavlink_id},
                        {'component_id': 1},
                    ]
                )
            )
            # JFI bridge node (converts between ROS messages and SwarmComm)
            # Bridge needs remapping to connect namespace jfi_comm with global /V{id} topics
            jfi_nodes.append(
                Node(
                    package='jfi_bridge',
                    executable='jfi_bridge_node',
                    name=f'jfi_bridge_drone_{i}',
                    output='screen',
                    parameters=[
                        {'system_id': mavlink_id},
                    ],
                    remappings=[
                        ('jfi_comm/in/packet', f'/{jfi_namespace}/jfi_comm/in/packet'),
                        ('jfi_comm/out/packet', f'/{jfi_namespace}/jfi_comm/out/packet'),
                    ]
                )
            )
            print(f"JFI nodes added for drone index={idx} with mavlink_id={mavlink_id}, namespace={jfi_namespace}")
        else:
            print("JFI nodes skipped (not in real mode)")

    # Build parameters for path_visualization with scenario start points
    viz_params = [
        drones_file,  # Base drone hardware
        obstacles_file,
        optimizer_file,
        map_file,
    ]
    # Add start_point overrides from scenario
    start_point_params = {}
    for i in range(num_drones):
        drone_key = f'drone_{i}'
        if drone_key in scenario_cfg:
            start_point_params[f'{drone_key}.start_point_x'] = float(scenario_cfg[drone_key]['start_point_x'])
            start_point_params[f'{drone_key}.start_point_y'] = float(scenario_cfg[drone_key]['start_point_y'])
            start_point_params[f'{drone_key}.start_point_z'] = float(scenario_cfg[drone_key]['start_point_z'])

    visualization_node = Node(
        package='path_visualization',
        executable='path_visualization_node',
        name=f'path_visualization_{target_drone_id}',
        output='screen',
        parameters=viz_params + [start_point_params],
        condition=IfCondition(LaunchConfiguration('enable_visualization'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('path_visualization'),
            'config',
            'rviz_config.rviz'
        ])],
        condition=IfCondition(LaunchConfiguration('rviz_simulation'))
    )

    formation_remaps = []
    if not real_mode:
        for vid in range(1, num_drones + 1):
            formation_remaps += [
                (f'/V{vid}/planning/broadcast_traj_send', '/planning/broadcast_traj_recv'),
                (f'/V{vid}/j_fi/broadcast_traj_recv',     '/planning/broadcast_traj_recv'),
            ]

    formation_commander = Node(
        package='path_manager',
        executable='formation_commander',
        name='formation_commander',
        output='screen',
        parameters=[
            drones_file,      # Drone hardware config (mavlink_id, system settings)
            {'scenario': scenario}  # Pass scenario name - formation_commander will load YAML directly
        ],
        remappings=formation_remaps,
    )

    immediate_actions = [visualization_node, rviz_node] + rover_nodes + jfi_nodes

    traj_nodes_delayed = TimerAction(
        period=0.0,
        actions=traj_nodes,
    )

    replan_nodes_delayed = TimerAction(
        period=0.0,
        actions=replan_nodes,
    )

    formation_commander_delayed = TimerAction(
        period=20.0,
        actions=[formation_commander],
    )

    # ROSbag recording (optional)
    rosbag_actions = []
    if record_bag:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_dir = './logs'
        bag_name = f"drone{target_drone_id}_trajectory_{timestamp}"
        bag_path = os.path.join(bag_dir, bag_name)

        # Create directory if it doesn't exist
        os.makedirs(bag_dir, exist_ok=True)

        # Target position topic for this drone
        target_position_topic = f'/agent{target_drone_id}/target_position'

        # Formation debugging topics (this rover only - namespace isolated!)
        vid = target_drone_id + 1
        formation_cmd_topic = f'/V{vid}/formation_command'  # From serial (jfi_bridge)
        formation_target_topic = f'/V{vid}/formation_target'  # Internal loopback (topic_prefix)

        print(f"ROSbag recording enabled: {bag_path}")
        print(f"Recording: /opt_trajectory, {target_position_topic}, formation debug topics")
        rosbag_process = ExecuteProcess(
            cmd=['ros2', 'bag', 'record',
                 '-o', bag_path,
                 target_position_topic,
                 formation_cmd_topic,
                 formation_target_topic],
            output='screen',
            shell=False
        )
        rosbag_actions = [rosbag_process]
    else:
        print("ROSbag recording disabled")

    # Formation commander runs only on drone 0 (in real mode) or in simulation mode
    if real_mode and target_drone_id != 0:
        # Real mode and not drone 0: don't run formation_commander
        print(f"Drone {target_drone_id}: formation_commander will NOT run (only drone 0 runs it in real mode)")
        return immediate_actions + [traj_nodes_delayed, replan_nodes_delayed] + rosbag_actions
    else:
        # Simulation mode or drone 0: run formation_commander
        print(f"Formation_commander will run (simulation mode or drone 0)")
        return immediate_actions + [traj_nodes_delayed, replan_nodes_delayed, formation_commander_delayed] + rosbag_actions

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
            'scenario',
            default_value='test',
            description='Scenario name (default, straight, test)'
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
        DeclareLaunchArgument(
            'record_bag',
            default_value='false',
            description='Enable rosbag recording for trajectory topics'
        ),
        DeclareLaunchArgument(
            'enable_visualization',
            default_value='false',
            description='Enable path visualization node'
        ),
        OpaqueFunction(function=create_drone_nodes),
    ])
