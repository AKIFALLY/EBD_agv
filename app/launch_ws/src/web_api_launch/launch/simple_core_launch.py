#!/usr/bin/env python3

"""
Simple Core Launch File (Development Version)
啟動 RosAGV 核心系統的四個節點 - 開發簡化版本

功能：
- 同時啟動四個核心節點，無延遲
- 適合開發和測試環境
- 輸出集中到螢幕便於調試

使用方法:
ros2 launch web_api_launch simple_core_launch.py

與 system_core_launch.py 的差異：
- 無啟動延遲，四個節點同時啟動
- 無自動重啟功能
- 更簡潔的日誌輸出
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo


def generate_launch_description():
    """產生簡化版launch描述"""
    
    return LaunchDescription([
        LogInfo(msg="🚀 啟動 RosAGV 核心系統 (開發版本)..."),
        
        # 同時啟動四個核心節點
        Node(
            package='db_proxy',
            executable='agvc_database_node',
            name='agvc_database_node',
            output='screen',
        ),
        
        Node(
            package='rcs',
            executable='rcs_core',
            name='rcs_core',
            output='screen',
        ),
        
        Node(
            package='wcs_base',
            executable='wcs_base_node',
            name='wcs_base_node',
            output='screen',
        ),
        
        Node(
            package='wcs_base',
            executable='task_condition_query_node',
            name='task_condition_query_node',
            output='screen',
        ),
        
        LogInfo(msg="✅ 所有四個節點已啟動！使用: ros2 node list 檢查狀態"),
    ])