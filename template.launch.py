import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # ========================================================================
    # 1. 全局配置区 
    # ========================================================================
    
    # 你的功能包名字 
    my_package_name = 'conqu_bringup'
    
    # 获取 share 目录 
    my_pkg_share = get_package_share_directory(my_package_name)
    
    # 全局通用的配置文件 (比如机器人底盘参数，多个节点都要用)
    common_config = os.path.join(my_pkg_share, 'config', 'robot_params.yaml')

    # ========================================================================
    # 2. 节点清单  
    # 只需要在这里添加字典，就能自动启动新节点
    # ========================================================================
    nodes_to_launch = [
        # --- 节点 1: 
        {
            'package':    'conqu_visual_slam',
            'executable': 'lidar_processor_node',
            'name':       'lidar_processor_node',
            'parameters': [common_config, {'lidar_frequency': 10.0}] # 支持加载文件 + 额外参数
        },
        # --- 节点 2: 
        {
            'package':    'conqu_visual_slam',
            'executable': 'visual_slam_node',
            'name':       'visual_slam_node',
            'parameters': [common_config] # 只加载通用配置
        },
    ]

    # ========================================================================
    # 3. 声明 Launch 参数 
    # ========================================================================
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    # ========================================================================
    # 4. 自动生成动作列表
    # ========================================================================
    launch_actions = []
    
    # 添加参数声明
    launch_actions.append(declare_use_sim_time)

    # 循环遍历清单，创建 Node 对象
    for node_cfg in nodes_to_launch:
        # 自动注入 use_sim_time 参数，这对仿真非常重要
        final_parameters = [{'use_sim_time': use_sim_time}]
        
        # 如果配置里有 parameters 字段，合并进去
        if 'parameters' in node_cfg:
            final_parameters.extend(node_cfg['parameters'])

        node = Node(
            package=node_cfg['package'],
            executable=node_cfg['executable'],
            name=node_cfg['name'],
            output='screen',
            emulate_tty=True, # 保持日志颜色
            parameters=final_parameters,
            # 如果需要重映射的话，也可以在字典里加字段支持
            remappings=node_cfg.get('remappings', []) 
        )
        launch_actions.append(node)
        
    return LaunchDescription(launch_actions)