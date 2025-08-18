#!/usr/bin/env python3
"""
RCS (Robot Control System) Launch File
管理 RCS 核心節點的啟動
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 資料庫連接參數
    db_url = 'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'
    
    return LaunchDescription([
        # 顯示啟動資訊
        LogInfo(msg='Starting RCS (Robot Control System) node...'),
        
        # 定義資料庫 URL 參數（可以被覆蓋）
        DeclareLaunchArgument(
            'db_url',
            default_value=db_url,
            description='Database connection URL'
        ),
        
        # RCS 核心節點
        Node(
            package='rcs',
            executable='rcs_core',
            name='rcs_core',
            namespace='agvc',
            output='screen',
            parameters=[
                {'db_url': LaunchConfiguration('db_url')},
            ],
            respawn=True,                      # 自動重啟
            respawn_delay=2.0                  # 重啟延遲（秒）
        ),
        
        LogInfo(msg='RCS node launched successfully!')
    ])