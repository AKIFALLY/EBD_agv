#!/usr/bin/env python3

"""
System Core Launch File
啟動 RosAGV 核心系統的四個關鍵節點：
- agvc_database_node: 資料庫代理服務 (優先啟動)
- rcs_core: RCS 車隊控制核心
- wcs_base_node: WCS 基礎節點
- task_condition_query_node: 任務條件查詢節點

使用方法:
ros2 launch web_api_launch system_core_launch.py

這個啟動檔案確保四個核心系統節點按正確順序啟動，
並具備自動重啟功能以提高系統穩定性。
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, LogInfo, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    """產生launch描述"""
    
    # Launch arguments
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level for all nodes (debug, info, warn, error, fatal)'
    )
    
    # 參數配置 - 啟用更詳細的日誌以便監控
    log_level = LaunchConfiguration('log_level')
    
    # 注意：wcs_ws 工作空間需要手動載入，因為它不在標準 AGVC 載入序列中
    # 請確保在啟動前已經載入 wcs_ws: source /app/wcs_ws/install/setup.bash
    
    # 1. 資料庫節點 - 優先啟動 (其他節點依賴資料庫服務)
    agvc_database_node = Node(
        package='db_proxy',
        executable='agvc_database_node',
        name='agvc_database_node',
        namespace='system',  # 使用 system 命名空間避免衝突
        output='screen',
        parameters=[{
            'use_sim_time': False
        }],
        respawn=True,  # 自動重啟
        respawn_delay=5.0,  # 重啟延遲5秒
    )
    
    # 日誌訊息：資料庫啟動
    db_startup_msg = LogInfo(
        msg="🗄️ 正在啟動 AGVC Database Node (system core)..."
    )
    
    # 2. RCS 核心節點 - 3秒後啟動 (等待資料庫就緒)
    rcs_core_node = TimerAction(
        period=3.0,  # 延遲3秒啟動，確保資料庫服務就緒
        actions=[
            LogInfo(msg="🚗 正在啟動 RCS Core Node..."),
            Node(
                package='rcs',
                executable='rcs_core',
                name='rcs_core',
                namespace='system',
                output='screen',
                parameters=[{
                    'use_sim_time': False
                }],
                respawn=True,
                respawn_delay=5.0,
            )
        ]
    )
    
    # 3. WCS 基礎節點 - 5秒後啟動 (等待資料庫和RCS就緒)
    wcs_base_node = TimerAction(
        period=5.0,  # 延遲5秒啟動，確保前面服務都就緒
        actions=[
            LogInfo(msg="🏭 正在啟動 WCS Base Node..."),
            Node(
                package='wcs_base',
                executable='wcs_base_node',
                name='wcs_base_node',
                namespace='system',
                output='screen',
                parameters=[{
                    'use_sim_time': False,
                    # WCS 專用參數
                    'db_url_agvc': 'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'
                }],
                respawn=True,
                respawn_delay=5.0,
            )
        ]
    )
    
    # 4. 任務條件查詢節點 - 7秒後啟動 (等待 WCS 基礎服務就緒)
    task_condition_query_node = TimerAction(
        period=7.0,  # 延遲7秒啟動，確保 WCS 基礎節點已就緒
        actions=[
            LogInfo(msg="🔍 正在啟動 Task Condition Query Node..."),
            Node(
                package='wcs_base',
                executable='task_condition_query_node',
                name='task_condition_query_node',
                namespace='system',
                output='screen',
                parameters=[{
                    'use_sim_time': False,
                    # 任務條件查詢專用參數
                    'db_url_agvc': 'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'
                }],
                respawn=True,
                respawn_delay=5.0,
            )
        ]
    )
    
    # 系統啟動完成訊息和診斷指令
    system_ready_msg = TimerAction(
        period=10.0,  # 延遲到10秒，確保所有節點都已啟動
        actions=[
            LogInfo(msg="✅ RosAGV 核心系統啟動序列完成！"),
            LogInfo(msg="📊 活動節點 (system 命名空間):"),
            LogInfo(msg="   • /system/agvc_database_node - 資料庫代理服務"),
            LogInfo(msg="   • /system/rcs_core - RCS 車隊控制核心"),
            LogInfo(msg="   • /system/wcs_base_node - WCS 基礎決策引擎"),
            LogInfo(msg="   • /system/task_condition_query_node - 任務條件查詢節點"),
            LogInfo(msg=""),
            LogInfo(msg="🔧 系統診斷指令:"),
            LogInfo(msg="   ros2 node list | grep system"),
            LogInfo(msg="   ros2 topic list | grep system"),
            LogInfo(msg="   ros2 service call /system/sql_query db_proxy_interfaces/srv/SqlQuery \"sql: 'SELECT 1'\""),
            LogInfo(msg=""),
            LogInfo(msg="🚨 如有問題請使用: r agvc-check, r quick-diag")
        ]
    )
    
    return LaunchDescription([
        # Launch arguments
        log_level_arg,
        
        # Launch sequence with dependency management
        db_startup_msg,
        agvc_database_node,           # T+0s: 資料庫節點先啟動
        rcs_core_node,                # T+3s: RCS核心節點
        wcs_base_node,                # T+5s: WCS基礎節點
        task_condition_query_node,    # T+7s: 任務條件查詢節點
        system_ready_msg,             # T+10s: 啟動完成訊息
    ])