import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


# ✅ 讀取特定 AGV 的設定
def load_yaml_config(yaml_path, agv_id):
    with open(yaml_path, 'r') as f:
        full_config = yaml.safe_load(f)
    if agv_id not in full_config:
        raise KeyError(f"❌ 在 YAML 中找不到 AGV ID: {agv_id}")
    return full_config[agv_id]


def generate_launch_description():
    # 🔧 從環境變數動態讀取 AGV 配置
    agv_id = os.environ.get('AGV_ID', 'loader01')
    ros_namespace = os.environ.get('ROS_NAMESPACE', f'/{agv_id}')
    device_config_file = os.environ.get('DEVICE_CONFIG_FILE', f'/app/config/agv/{agv_id}_config.yaml')

    # 從 AGV ID 提取 room_id
    try:
        room_id = int(agv_id[-2:])  # 取出最後兩位數字
    except (ValueError, IndexError):
        room_id = 1  # 預設值

    # 參數檔路徑
    agv_command_file = "/app/agv_cmd_service_ws/src/agv_cmd_service/config/agv_cmd_service.yaml"

    print(f"🚛 Loader AGV Launch 配置:")
    print(f"  AGV_ID: {agv_id}")
    print(f"  ROS_NAMESPACE: {ros_namespace}")
    print(f"  DEVICE_CONFIG_FILE: {device_config_file}")
    print(f"  ROOM_ID: {room_id}")

    # 確保檔案存在
    if not os.path.exists(agv_command_file):
        print(f"⚠️ agv_command_file 設定檔不存在: {agv_command_file}")
    if not os.path.exists(device_config_file):
        print(f"⚠️ 設備配置檔不存在: {device_config_file}")

    # 讀入 AGV 設定
    # 不再讀取 config，直接用 device_config_file 作為 ROS 2 node 參數檔

    return LaunchDescription([
        DeclareLaunchArgument(
            'agv_command_file',
            default_value=agv_command_file,
            description='Path to agv command file'
        ),

        Node(
            package='plc_proxy',
            executable='plc_service',
            name='plc_service',
            namespace=agv_id,
            parameters=[device_config_file],
        ),

        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy_linux_node',
            namespace=agv_id,
            parameters=[{"dev": "/dev/input/js0"}],
        ),

        Node(
           package='loader_agv',
           executable='loader_agv_node',
           name='agv_core_node',
           namespace=agv_id,
           parameters=[{"room_id": room_id}],
        ),
    ])
