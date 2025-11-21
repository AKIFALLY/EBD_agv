#!/usr/bin/env python3
"""
SensorPart Publisher Launch File

啟動 SensorPart Publisher 節點，支援 namespace 和參數配置。

使用範例：
    # 不帶 namespace
    ros2 launch sensorpart sensorpart_publisher.launch.py

    # 帶 namespace（多 AGV 場景）
    ros2 launch sensorpart sensorpart_publisher.launch.py namespace:=cargo_agv_1

    # 自訂參數
    ros2 launch sensorpart sensorpart_publisher.launch.py \
        namespace:=cargo_agv_1 \
        host:=192.168.2.100 \
        port:=2005 \
        publish_rate:=10.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """生成 Launch 描述"""

    # 宣告 launch 參數
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='ROS namespace for the node (e.g., cargo_agv_1)'
    )

    declare_host = DeclareLaunchArgument(
        'host',
        default_value='192.168.2.111',
        description='SensorPart camera IP address'
    )

    declare_port = DeclareLaunchArgument(
        'port',
        default_value='2005',
        description='SensorPart camera port'
    )

    declare_debounce_seconds = DeclareLaunchArgument(
        'debounce_seconds',
        default_value='1.0',
        description='Debounce time window in seconds (防抖時間窗口)'
    )

    # SensorPart Publisher 節點
    sensorpart_publisher_node = Node(
        package='sensorpart',
        executable='sensorpart_publisher_node',
        name='sensorpart_publisher',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{
            'host': LaunchConfiguration('host'),
            'port': LaunchConfiguration('port'),
            'debounce_seconds': LaunchConfiguration('debounce_seconds'),
        }],
        emulate_tty=True,  # 讓日誌顯示顏色
    )

    return LaunchDescription([
        # 宣告參數
        declare_namespace,
        declare_host,
        declare_port,
        declare_debounce_seconds,

        # 啟動節點
        sensorpart_publisher_node,
    ])
