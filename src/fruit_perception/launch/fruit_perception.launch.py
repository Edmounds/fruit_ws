import subprocess

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def _set_v4l2_control(device, control, value):
    subprocess.run(
        [
            'v4l2-ctl',
            '--device', device,
            '--set-ctrl', f'{control}={value}',
        ],
        check=True,
    )


def _apply_manual_camera_controls(camera_parameters, v4l2_controls):
    device = camera_parameters['video_device']
    _set_v4l2_control(
        device,
        'backlight_compensation',
        v4l2_controls['backlight_compensation'],
    )
    _set_v4l2_control(
        device,
        'white_balance_automatic',
        int(camera_parameters['auto_white_balance']),
    )
    if not camera_parameters['auto_white_balance']:
        _set_v4l2_control(
            device,
            'white_balance_temperature',
            camera_parameters['white_balance'],
        )

    auto_exposure = camera_parameters['autoexposure']
    _set_v4l2_control(device, 'auto_exposure', 3 if auto_exposure else 1)
    if not auto_exposure:
        _set_v4l2_control(
            device,
            'exposure_time_absolute',
            camera_parameters['exposure'],
        )


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
    qr_config = (
        '/home/rc4/fruit_ws/src/fruit_qr_planner/config/'
        'fruit_qr_planner.yaml'
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
    usb_camera_controls = camera_config[
        'usb_camera_controls'
    ]['ros__parameters']
    left_camera_parameters = dict(
        camera_config['left_camera']['ros__parameters']
    )
    left_camera_controls = dict(
        backlight_compensation=usb_camera_controls[
            'left_backlight_compensation'
        ]
    )
    right_camera_parameters = dict(
        camera_config['right_camera']['ros__parameters']
    )
    right_camera_controls = dict(
        backlight_compensation=usb_camera_controls[
            'right_backlight_compensation'
        ]
    )
    # ========================================================================
    # 2. 节点清单
    # ========================================================================
    nodes_to_launch = []
    camera_parameters_to_configure = []
    if perception_parameters['enable_left_camera']:
        nodes_to_launch.append({
            'package': 'usb_cam',
            'plugin': 'usb_cam::UsbCamNode',
            'name': 'left_camera',
            'parameters': [left_camera_parameters],
            'remappings': [
                ('image_raw', '/left_camera/image_raw'),
                ('camera_info', '/left_camera/camera_info'),
                ('/set_capture', '/left_camera/set_capture'),
            ],
        })
        camera_parameters_to_configure.append(
            (left_camera_parameters, left_camera_controls)
        )
    if perception_parameters['enable_right_camera']:
        nodes_to_launch.append({
            'package': 'usb_cam',
            'plugin': 'usb_cam::UsbCamNode',
            'name': 'right_camera',
            'parameters': [right_camera_parameters],
            'remappings': [
                ('image_raw', '/right_camera/image_raw'),
                ('camera_info', '/right_camera/camera_info'),
                ('/set_capture', '/right_camera/set_capture'),
            ],
        })
        camera_parameters_to_configure.append(
            (right_camera_parameters, right_camera_controls)
        )
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
    def apply_manual_camera_controls(_context):
        for camera_parameters, v4l2_controls in camera_parameters_to_configure:
            _apply_manual_camera_controls(camera_parameters, v4l2_controls)
        return []

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

    camera_mode_manager = Node(
        package='fruit_perception',
        executable='camera_mode_manager',
        name='camera_mode_manager',
        output='screen',
        emulate_tty=True,
        parameters=[common_config, {'use_sim_time': use_sim_time}],
    )
    qr_planner = Node(
        package='fruit_qr_planner',
        executable='fruit_qr_planner_node',
        name='fruit_qr_planner',
        output='screen',
        emulate_tty=True,
        parameters=[qr_config, {'use_sim_time': use_sim_time}],
    )
    serial_bridge = Node(
        package='auto_serial_bridge',
        executable='serial_node',
        name='serial_controller',
        output='screen',
        emulate_tty=True,
        parameters=[serial_parameters, {'use_sim_time': use_sim_time}],
        arguments=[
            '--ros-args', '--log-level', 'serial_controller:=debug',
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        OpaqueFunction(function=apply_manual_camera_controls),
        container,
        camera_mode_manager,
        qr_planner,
        serial_bridge,
    ])
