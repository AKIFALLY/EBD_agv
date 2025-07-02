"""
KUKA WCS Launch 文件
啟動 KUKA WCS 系統的所有必要節點
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """生成 Launch 描述"""
    
    # 獲取 package 路徑
    pkg_share = FindPackageShare('kuka_wcs')
    
    # 配置文件路徑
    config_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'kuka_wcs_config.yaml'
    ])
    
    # Launch 參數
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to the KUKA WCS configuration file'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='INFO',
        description='Log level for the nodes'
    )
    
    enable_auto_assignment_arg = DeclareLaunchArgument(
        'enable_auto_assignment',
        default_value='true',
        description='Enable automatic task assignment'
    )
    
    # KUKA WCS 主節點
    kuka_wcs_node = Node(
        package='kuka_wcs',
        executable='kuka_wcs_node',
        name='kuka_wcs_node',
        namespace='kuka_wcs',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'enable_auto_task_assignment': LaunchConfiguration('enable_auto_assignment'),
                'log_level': LaunchConfiguration('log_level')
            }
        ],
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
    # 啟動信息
    start_info = LogInfo(
        msg=[
            '🚀 啟動 KUKA WCS 系統...\n',
            '📁 配置文件: ', LaunchConfiguration('config_file'), '\n',
            '📊 日誌級別: ', LaunchConfiguration('log_level'), '\n',
            '🤖 自動任務分配: ', LaunchConfiguration('enable_auto_assignment')
        ]
    )
    
    return LaunchDescription([
        # Launch 參數
        config_file_arg,
        log_level_arg,
        enable_auto_assignment_arg,
        
        # 啟動信息
        start_info,
        
        # 節點
        kuka_wcs_node,
    ])
