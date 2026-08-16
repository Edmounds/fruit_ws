import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ========================================================================
    # 1. Global configuration
    # ========================================================================
    serial_config_path = (
        '/home/rc4/fruit_ws/src/auto_serial_bridge/config/protocol.yaml'
    )
    with open(serial_config_path, encoding='utf-8') as config_file:
        serial_config = yaml.safe_load(config_file)
    serial_parameters = dict(
        serial_config['serial_controller']['ros__parameters']
    )

    # ========================================================================
    # 2. Node list
    # ========================================================================
    nodes_to_launch = [
        {
            'package': 'auto_serial_bridge',
            'executable': 'serial_node',
            'name': 'serial_controller',
            'parameters': [serial_parameters],
        },
    ]

    # ========================================================================
    # 3. Launch arguments
    # ========================================================================
    use_sim_time = LaunchConfiguration('use_sim_time')
    port = LaunchConfiguration('port')
    log_level = LaunchConfiguration('log_level')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock if true'
    )
    declare_port = DeclareLaunchArgument(
        'port', default_value=str(serial_parameters['port']),
        description='Serial device path'
    )
    declare_log_level = DeclareLaunchArgument(
        'log_level', default_value='info'
    )
    serial_parameters['port'] = port

    # ========================================================================
    # 4. Generate actions
    # ========================================================================
    launch_actions = [
        declare_use_sim_time,
        declare_port,
        declare_log_level,
    ]
    for node_config in nodes_to_launch:
        parameters = [{'use_sim_time': use_sim_time}]
        parameters.extend(node_config.get('parameters', []))
        launch_actions.append(
            Node(
                package=node_config['package'],
                executable=node_config['executable'],
                name=node_config['name'],
                output='screen',
                emulate_tty=True,
                parameters=parameters,
                arguments=['--ros-args', '--log-level', log_level],
                remappings=node_config.get('remappings', []),
            )
        )

    return LaunchDescription(launch_actions)
