"""
AI WCS 系統啟動文件
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def launch_setup(context, *args, **kwargs):
    """啟動設定函數"""
    
    # 獲取啟動參數
    enable_logging = LaunchConfiguration('enable_logging')
    log_level = LaunchConfiguration('log_level')
    
    # AI WCS 統一主節點 - 包含所有組件
    ai_wcs_node = Node(
        package='ai_wcs',
        executable='ai_wcs_node',
        name='ai_wcs_unified_node',
        output='screen',
        parameters=[{
            'decision_cycle_interval': 10.0,
            'max_concurrent_tasks': 50,
            'enable_statistics_logging': True
        }],
        arguments=['--ros-args', '--log-level', log_level]
    )
    
    return [ai_wcs_node]


def generate_launch_description():
    """生成啟動描述"""
    
    # 聲明啟動參數
    enable_logging_arg = DeclareLaunchArgument(
        'enable_logging',
        default_value='true',
        description='Enable detailed logging'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level (debug, info, warn, error)'
    )
    
    return LaunchDescription([
        enable_logging_arg,
        log_level_arg,
        OpaqueFunction(function=launch_setup)
    ])