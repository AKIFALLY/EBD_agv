#!/usr/bin/env python3
"""
WCS 系统完整启动 Launch 文件

同时启动：
1. PLC 代理节点 (plc_proxy_node)
2. ECS 设备控制节点 (ecs_node)
3. Room Task Build 节点 (room_task_build_node)

使用方式：
    ros2 launch alan_room_task_build wcs_system.launch.py

参数：
    log_level:=info    # 日志级别 (debug, info, warn, error)
    use_ecs:=true      # 是否启动 ECS
    use_plc:=true      # 是否启动 PLC 代理
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """生成 Launch 描述"""

    # 声明 Launch 参数
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='日志级别 (debug, info, warn, error)'
    )

    use_ecs_arg = DeclareLaunchArgument(
        'use_ecs',
        default_value='true',
        description='是否启动 ECS 节点'
    )

    use_plc_arg = DeclareLaunchArgument(
        'use_plc',
        default_value='true',
        description='是否启动 PLC 代理节点'
    )

    # 获取参数值
    log_level = LaunchConfiguration('log_level')
    use_ecs = LaunchConfiguration('use_ecs')
    use_plc = LaunchConfiguration('use_plc')

    # 1. PLC 代理节点
    plc_proxy_node = Node(
        package='plc_proxy',
        executable='plc_service',
        name='plc_service',
        namespace='agvc',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition(use_plc),
        respawn=True,  # 崩溃后自动重启
        respawn_delay=2.0,  # 重启延迟 2 秒
    )

    # 2. ECS 设备控制节点
    ecs_node = Node(
        package='ecs',
        executable='ecs_core',
        name='ecs_core',
        namespace='agvc',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition(use_ecs),
        respawn=True,
        respawn_delay=2.0,
    )

    # 3. Room Task Build 节点
    room_task_build_node = Node(
        package='alan_room_task_build',
        executable='room_task_build_node',
        name='room_task_build_node',
        namespace='agvc',  # 使用 agvc 命名空间，连接到 /agvc/read_continuous_data
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        respawn=True,
        respawn_delay=2.0,
    )

    # 启动信息
    launch_info = LogInfo(
        msg=[
            '\n',
            '=' * 60, '\n',
            '🚀 WCS 系统启动中...\n',
            '=' * 60, '\n',
            '启用的节点：\n',
            '  - PLC 代理: ', use_plc, '\n',
            '  - ECS 设备控制: ', use_ecs, '\n',
            '  - Room Task Build: true\n',
            '日志级别: ', log_level, '\n',
            '=' * 60, '\n',
        ]
    )

    # 返回 LaunchDescription
    return LaunchDescription([
        # 声明参数
        log_level_arg,
        use_ecs_arg,
        use_plc_arg,

        # 启动信息
        launch_info,

        # 启动节点
        plc_proxy_node,
        ecs_node,
        room_task_build_node,
    ])
