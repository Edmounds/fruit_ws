import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # ========================================================================
    # 1. 全局配置区
    # ========================================================================
    common_config = (
        '/home/rc4/fruit_ws/src/fruit_perception/config/'
        'fruit_perception.yaml'
    )
    with open(common_config, encoding='utf-8') as config_file:
        camera_config = yaml.safe_load(config_file)
    qr_launch = '/home/rc4/fruit_ws/src/fruit_qr_planner/launch/fruit_qr_planner.launch.py'
    perception_parameters = camera_config[
        'fruit_perception_node'
    ]['ros__parameters']

    # ========================================================================
    # 2. 节点清单
    # ========================================================================
    nodes_to_launch = []
    if perception_parameters.get('enable_camera', True):
        nodes_to_launch.append({
            'package': 'realsense2_camera',
            'plugin': 'realsense2_camera::RealSenseNodeFactory',
            'namespace': '/',
            'name': 'camera',
            'parameters': [
                camera_config['camera']['ros__parameters']
            ],
        })
    perception_node_parameters = dict(perception_parameters)
    perception_node_parameters['zone'] = LaunchConfiguration('zone')
    perception_node_parameters['ripeness_model'] = LaunchConfiguration('ripeness_model')
    perception_node_parameters['performance_hint'] = LaunchConfiguration('performance_hint')
    nodes_to_launch.append({
        'package': 'fruit_perception',
        'plugin': 'fruit_perception::FruitPerceptionNode',
        'name': 'fruit_perception_node',
        'parameters': [perception_node_parameters],
    })

    # ========================================================================
    # 3. 声明 Launch 参数
    # ========================================================================
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true',
    )
    declare_ripeness_model = DeclareLaunchArgument(
        'ripeness_model',
        default_value='edge',
        choices=['edge', 'yolo'],
        description='Startup choice of ripeness classifier',
    )
    declare_zone = DeclareLaunchArgument(
        'zone',
        default_value=perception_parameters.get('zone', 'A'),
        choices=['A', 'B', 'C'],
        description='Initial perception zone; runtime changes use the zone ROS parameter',
    )
    declare_performance_hint = DeclareLaunchArgument(
        'performance_hint',
        default_value='latency',
        choices=['latency', 'throughput'],
        description='OpenVINO GPU detector performance hint',
    )
    declare_perception_log_level = DeclareLaunchArgument(
        'perception_log_level',
        default_value='warn',
        choices=['debug', 'info', 'warn', 'error', 'fatal'],
        description='Log level for fruit_perception_node',
    )

    # ========================================================================
    # 4. 自动生成动作列表
    # ========================================================================
    composable_nodes = []
    for node_cfg in nodes_to_launch:
        final_parameters = [{'use_sim_time': use_sim_time}]
        if 'parameters' in node_cfg:
            final_parameters.extend(node_cfg['parameters'])

        composable_nodes.append(
            ComposableNode(
                package=node_cfg['package'],
                plugin=node_cfg['plugin'],
                namespace=node_cfg.get('namespace', ''),
                name=node_cfg['name'],
                parameters=final_parameters,
                remappings=node_cfg.get('remappings', []),
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        )

    container = ComposableNodeContainer(
        name='fruit_perception_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        output='screen',
        emulate_tty=True,
        arguments=[
            '--ros-args', '--log-level', 'warn', '--log-level',
            ['fruit_perception_node:=', LaunchConfiguration('perception_log_level')],
            '--log-level',
            'camera:=info' if os.environ.get('FRUIT_LOGGING_DEBUG') == '1' else 'camera:=warn',
        ],
    )

    qr_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(qr_launch),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_ripeness_model,
        declare_zone,
        declare_performance_hint,
        declare_perception_log_level,
        container,
        qr_planner,
    ])
