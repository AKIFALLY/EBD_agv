#!/usr/bin/env python3
"""
Transfer Box Task Build Launch 文件

啟動通用傳送箱任務建立節點，執行以下功能：
1. 遍歷所有傳送箱，監控 Rack carrier_bitmap 並寫入 PLC
2. 統一監控 PLC DM3010-3011 (work_id)
3. 遍歷所有傳送箱，讀取 PLC 回饋在席值並更新 Rack
4. 自動清理已完成的 Task

支援的傳送箱：
- 入口傳送箱 (Location 20001, Work IDs: 2000102, 2002102)
- 出口傳送箱 (Location 20002, Work IDs: 2000201, 2001201)

使用方式：
    ros2 launch transfer_box_task_build transfer_box_task_build.launch.py

參數：
    log_level:=info    # 日誌級別 (debug, info, warn, error)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """生成 Launch 描述"""

    # 聲明 Launch 參數
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='日誌級別 (debug, info, warn, error)'
    )

    # 獲取參數值
    log_level = LaunchConfiguration('log_level')

    # Transfer Box Task Build 節點
    transfer_box_task_build_node = Node(
        package='transfer_box_task_build',
        executable='transfer_box_task_build_node',
        name='transfer_box_task_build_node',
        namespace='agvc',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        respawn=True,
        respawn_delay=2.0,
    )

    # 啟動信息
    launch_info = LogInfo(
        msg=[
            '\n',
            '=' * 60, '\n',
            '🚀 通用傳送箱任務建立系統啟動中...\n',
            '=' * 60, '\n',
            '功能：\n',
            '  1. 遍歷所有傳送箱，監控 Rack carrier_bitmap 並寫入 PLC\n',
            '  2. 統一監控 PLC DM3010-3011 (work_id)\n',
            '  3. 遍歷所有傳送箱，讀取 PLC 回饋在席值並更新 Rack\n',
            '  4. 自動清理已完成的 Task\n',
            '支援傳送箱：\n',
            '  - 入口傳送箱 (Location 20001)\n',
            '  - 出口傳送箱 (Location 20002)\n',
            '日誌級別: ', log_level, '\n',
            '=' * 60, '\n',
        ]
    )

    return LaunchDescription([
        log_level_arg,
        launch_info,
        transfer_box_task_build_node,
    ])
