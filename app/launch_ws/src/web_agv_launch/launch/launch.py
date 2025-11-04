import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    AGV 車載 Web 服務 Launch 檔案
    啟動 AGVUI (AGV 車載監控界面)
    動態命名空間：根據 AGV_ID 環境變數或 launch 參數設定
    """

    # 從環境變數讀取 AGV ID，若無則使用 'agv' 作為預設值
    default_agv_id = os.environ.get('AGV_ID', 'agv')

    return LaunchDescription([
        # 宣告 AGV ID 參數（可從命令列覆蓋）
        DeclareLaunchArgument(
            'agv_id',
            default_value=default_agv_id,
            description='AGV ID for namespace (e.g., loader02, cargo02, unloader02)'
        ),

        # AGVUI - AGV 車載監控界面 (Port 8003)
        Node(
            package='agvui',
            executable='agv_ui_server',
            name='agv_ui_server',
            namespace=LaunchConfiguration('agv_id'),  # 動態命名空間
            output='screen',
        ),
    ])
