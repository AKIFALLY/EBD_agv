from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    AGV 車載 Web 服務 Launch 檔案
    啟動 AGVUI (AGV 車載監控界面)
    """
    
    return LaunchDescription([
        # AGVUI - AGV 車載監控界面 (Port 8003)
        Node(
            package='agvui',
            executable='agv_ui_server',
            name='agv_ui_server',
            namespace='agv',  # 使用 agv 命名空間
            output='screen',
        ),
    ])
