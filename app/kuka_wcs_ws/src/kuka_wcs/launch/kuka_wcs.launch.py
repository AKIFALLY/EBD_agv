#!/usr/bin/env python3
"""
KUKA WCS Launch File
管理 KUKA WCS 节点的启动
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 资料库连接参数
    db_url = 'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'

    return LaunchDescription([
        # 显示启动信息
        LogInfo(msg='Starting KUKA WCS (Warehouse Control System) node...'),

        # 定义资料库 URL 参数（可以被覆盖）
        DeclareLaunchArgument(
            'db_url',
            default_value=db_url,
            description='Database connection URL'
        ),

        # KUKA WCS 节点
        Node(
            package='kuka_wcs',
            executable='kuka_wcs_node',
            name='kuka_wcs_node',
            namespace='agvc',
            output='screen',
            parameters=[
                {'db_url': LaunchConfiguration('db_url')},
            ],
            respawn=True,                      # 自动重启
            respawn_delay=2.0                  # 重启延迟（秒）
        ),

        LogInfo(msg='KUKA WCS node launched successfully!')
    ])
