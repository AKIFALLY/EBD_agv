import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

# ✅ 確保 YAML 設定檔存在
param_file = "/app/config/ecs_config.yaml"
agv_command_file = "/app/agv_cmd_service_ws/src/agv_cmd_service/config/agv_cmd_service.yaml"

if not os.path.exists(param_file):
    print(f"⚠️ YAML 設定檔不存在: {param_file}")
else:
    print(f"✅ 設定檔找到: {param_file}")

if not os.path.exists(agv_command_file):
    print(f"⚠️ YAML 設定檔不存在: {agv_command_file}")
else:
    print(f"✅ 設定檔找到: {agv_command_file}")

def generate_launch_description():
    return LaunchDescription([
        #ExecuteProcess(
        #    cmd=["bash", "-c", "source /app/plc_proxy_ws/install/setup.bash && env"],
        #),
        #DeclareLaunchArgument(
        #    'param_file',
        #    default_value=param_file,
        #    description='Path to parameter file'
        #),
#
        #Node(
        #    package='plc_proxy',
        #    executable='plc_service',
        #    name='plc_proxy_service',
        #    parameters=[param_file],  # ✅ 直接傳遞 YAML 檔案
        #),
#
        DeclareLaunchArgument(
            'agv_command_file',
            default_value=agv_command_file,
            description='Path to agv command file'
        ),

       Node(
            package='agv_cmd_service',  # 套件名稱（package name）
            executable='agv_cmd_service_node',  # 可執行檔名稱（executable name）
            name='agv_cmd_service_node',  # ROS 2 節點名稱（node name）
            parameters=[agv_command_file],  # 參數設定（YAML 檔案）
        ),
    ])
