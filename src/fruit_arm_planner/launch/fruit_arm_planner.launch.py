from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ========================================================================
    # 1. 全局配置区 (遵循项目规范：直接指定 src 下的配置文件绝对路径)
    # ========================================================================
    my_package_name = 'fruit_arm_planner'
    common_config = '/home/rc4/fruit_ws/src/fruit_arm_planner/config/arm_presets.yaml'

    # ========================================================================
    # 2. 节点清单
    # ========================================================================
    nodes_to_launch = [
        {
            'package': my_package_name,
            'executable': 'arm_planner_node',
            'name': 'arm_planner_node',
            'parameters': [common_config],
        },
    ]

    # ========================================================================
    # 3. 声明 Launch 参数
    # ========================================================================
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true',
    )
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level',
    )

    # ========================================================================
    # 4. 自动生成动作列表
    # ========================================================================
    launch_actions = [
        declare_use_sim_time,
        declare_log_level,
    ]

    for node_cfg in nodes_to_launch:
        final_parameters = [{'use_sim_time': use_sim_time}]
        if 'parameters' in node_cfg:
            final_parameters.extend(node_cfg['parameters'])

        node = Node(
            package=node_cfg['package'],
            executable=node_cfg['executable'],
            name=node_cfg['name'],
            output='screen',
            emulate_tty=True,
            parameters=final_parameters,
            arguments=[
                '--ros-args',
                '--log-level', log_level,
                '--log-level', 'rcl:=info',
                '--log-level', 'rclpy:=info',
                '--log-level', 'rcutils:=info',
                '--log-level', 'rmw_cyclonedds_cpp:=info',
            ],
            remappings=node_cfg.get('remappings', []),
        )
        launch_actions.append(node)

    return LaunchDescription(launch_actions)
