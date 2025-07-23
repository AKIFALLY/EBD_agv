"""
任務條件查詢節點啟動檔案
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """生成啟動描述"""
    
    # 宣告啟動參數
    db_url_arg = DeclareLaunchArgument(
        'db_url_agvc',
        default_value='postgresql+psycopg2://agvc:password@192.168.100.254/agvc',
        description='AGVC 資料庫連接字串'
    )
    
    auto_interval_arg = DeclareLaunchArgument(
        'auto_execution_interval',
        default_value='300.0',
        description='自動執行間隔時間（秒）'
    )
    
    enable_auto_arg = DeclareLaunchArgument(
        'enable_auto_execution',
        default_value='true',
        description='是否啟用自動執行'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='日誌等級 (debug, info, warn, error)'
    )
    
    # 任務條件查詢節點
    task_condition_query_node = Node(
        package='wcs_base',
        executable='task_condition_query_node',
        name='task_condition_query_node',
        parameters=[{
            'db_url_agvc': LaunchConfiguration('db_url_agvc'),
            'auto_execution_interval': LaunchConfiguration('auto_execution_interval'),
            'enable_auto_execution': LaunchConfiguration('enable_auto_execution'),
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        db_url_arg,
        auto_interval_arg,
        enable_auto_arg,
        log_level_arg,
        task_condition_query_node,
    ])
