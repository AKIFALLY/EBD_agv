"""
KUKA WCS 測試 Launch 文件
用於測試和開發的簡化啟動配置
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """生成測試 Launch 描述"""
    
    # Launch 參數
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='DEBUG',
        description='Log level for testing'
    )
    
    # KUKA WCS 測試節點
    kuka_wcs_test_node = Node(
        package='kuka_wcs',
        executable='kuka_wcs_node',
        name='kuka_wcs_test_node',
        namespace='test',
        parameters=[
            {
                'api_base_url': 'http://192.168.11.206:10870',
                'api_username': 'admin',
                'api_password': 'Admin',
                'query_cycle_time': 10.0,
                'decision_cycle_time': 15.0,
                'enable_auto_task_assignment': False,  # 測試時關閉自動分配
                'max_pending_tasks': 10,
                'min_robot_battery_level': 10,
                'log_level': LaunchConfiguration('log_level')
            }
        ],
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
    return LaunchDescription([
        log_level_arg,
        kuka_wcs_test_node,
    ])
