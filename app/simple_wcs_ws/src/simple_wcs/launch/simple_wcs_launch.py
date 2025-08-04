#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # 宣告啟動參數
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level'
        ),
        
        # Simple WCS 引擎節點
        Node(
            package='simple_wcs',
            executable='simple_wcs_node',
            name='simple_wcs_engine',
            output='screen',
            parameters=[
                {'log_level': LaunchConfiguration('log_level')}
            ],
            remappings=[
                # 重新映射 topic 名稱
                ('/simple_wcs/task_decisions', '/wcs/task_decisions'),
                ('/simple_wcs/system_status', '/wcs/system_status'),
            ]
        ),
    ])