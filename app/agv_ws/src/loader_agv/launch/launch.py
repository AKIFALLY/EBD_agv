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
    agv_id = 'cargo02'
    room_id = num = int(agv_id[-2:])  # 取出 "01" 並轉成整數 1

    # 兩個參數檔路徑
    param_file = "/app/config/ecs_config.yaml"
    agv_command_file = "/app/agv_cmd_service_ws/src/agv_cmd_service/config/agv_cmd_service.yaml"

    # 確保檔案存在
    if not os.path.exists(param_file):
        print(f"⚠️ YAML 設定檔不存在: {param_file}")
    if not os.path.exists(agv_command_file):
        print(f"⚠️ YAML 設定檔不存在: {agv_command_file}")

    # 讀入 AGV01 設定
    config = load_yaml_config(param_file, agv_id)

    return LaunchDescription([
        DeclareLaunchArgument(
            'param_file',
            default_value=param_file,
            description='Path to ecs_config.yaml'
        ),

        DeclareLaunchArgument(
            'agv_command_file',
            default_value=agv_command_file,
            description='Path to agv command file'
        ),

        # ExecuteProcess(
        #    cmd=["bash", "-c", "source /app/plc_proxy_ws/install/setup.bash && env"]
        # ),

        # ✅ plc_service 使用解析後的參數 dict
        Node(
            package='plc_proxy',
            executable='plc_service',
            name='plc_service',
            namespace=agv_id,
            parameters=[param_file],
        ),

        # ✅ agv_cmd_service 仍使用外部 YAML 檔（若該檔格式本身就包含 ros__parameters）
        # Node(
        #    package='agv_cmd_service',
        #    executable='agv_cmd_service_node',
        #    name='agv_cmd_service',
        #    namespace=agv_id,
        #    parameters=[agv_command_file],  # 這裡沒問題
        # ),

        # ✅ agv_base 使用解析後的參數 dict
        

         #在AGVC上執行,暫時放到這邊
         #Node(
         #   package='db_proxy',
         #   executable='agvc_database_node',
         #   name='agvc_database_node',
         #   #namespace=agv_id,
         #   # parameters=[config['agv_base']['ros__parameters']],
         #),

        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy_linux_node',
            namespace=agv_id,
            parameters=[{"dev": "/dev/input/js0"}],  # ✅ 用大括號包成 dict，再放到 list 裡
        ),

        Node(
            package='loader_agv',
            executable='agv_core_node',
            name='agv_core_node',
            namespace=agv_id,
            parameters=[{"room_id": room_id}],
        ),

    ])
