import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # ========================================================================
    # 1. 全局配置区
    # ========================================================================
    my_package_name = 'fruit_perception'
    my_pkg_share = get_package_share_directory(my_package_name)
    common_config = os.path.join(my_pkg_share, 'config', 'fruit_perception.yaml')
    with open(common_config, encoding='utf-8') as config_file:
        camera_config = yaml.safe_load(config_file)
    bridge_config = os.path.join(
        get_package_share_directory('fruit_hardware_bridge'),
        'config',
        'fruit_hardware_bridge.yaml',
    )
    serial_config_path = (
        '/home/rc4/fruit_ws/src/auto_serial_bridge/config/protocol.yaml'
    )
    with open(serial_config_path, encoding='utf-8') as config_file:
        serial_config = yaml.safe_load(config_file)
    serial_parameters = dict(
        serial_config['serial_controller']['ros__parameters']
    )
    perception_parameters = camera_config[
        'fruit_perception_node'
    ]['ros__parameters']
    left_camera_parameters = dict(
        camera_config['left_camera']['ros__parameters']
    )
    right_camera_parameters = dict(
        camera_config['right_camera']['ros__parameters']
    )
    left_camera_parameters['video_device'] = os.path.realpath(
        left_camera_parameters['video_device']
    )
    right_camera_parameters['video_device'] = os.path.realpath(
        right_camera_parameters['video_device']
    )

    # ========================================================================
    # 2. 节点清单
    # ========================================================================
    nodes_to_launch = []
    if perception_parameters['enable_left_camera']:
        nodes_to_launch.append({
            'package': 'usb_cam',
            'plugin': 'usb_cam::UsbCamNode',
            'name': 'left_camera',
            'parameters': [left_camera_parameters],
            'remappings': [
                ('image_raw', '/left_camera/image_raw'),
                ('camera_info', '/left_camera/camera_info'),
            ],
        })
    if perception_parameters['enable_right_camera']:
        nodes_to_launch.append({
            'package': 'usb_cam',
            'plugin': 'usb_cam::UsbCamNode',
            'name': 'right_camera',
            'parameters': [right_camera_parameters],
            'remappings': [
                ('image_raw', '/right_camera/image_raw'),
                ('camera_info', '/right_camera/camera_info'),
            ],
        })
    if perception_parameters['enable_d435i']:
        nodes_to_launch.append({
            'package': 'realsense2_camera',
            'plugin': 'realsense2_camera::RealSenseNodeFactory',
            'namespace': 'camera',
            'name': 'camera',
            'parameters': [
                camera_config['d435i_camera']['ros__parameters']
            ],
        })
    nodes_to_launch.append({
        'package': 'fruit_perception',
        'plugin': 'fruit_perception::FruitPerceptionNode',
        'name': 'fruit_perception_node',
        'parameters': [common_config],
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
    )

    fruit_result_bridge = Node(
        package='fruit_hardware_bridge',
        executable='fruit_result_bridge',
        name='fruit_result_bridge',
        output='screen',
        emulate_tty=True,
        parameters=[bridge_config, {'use_sim_time': use_sim_time}],
    )
    serial_bridge = Node(
        package='auto_serial_bridge',
        executable='serial_node',
        name='serial_controller',
        output='screen',
        emulate_tty=True,
        parameters=[serial_parameters, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        declare_use_sim_time,
        container,
        fruit_result_bridge,
        serial_bridge,
    ])
