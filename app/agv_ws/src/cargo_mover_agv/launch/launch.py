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
    agv_id = os.environ.get('AGV_ID', 'cargo01')
    ros_namespace = os.environ.get('ROS_NAMESPACE', f'/{agv_id}')
    device_config_file = os.environ.get('DEVICE_CONFIG_FILE', f'/app/config/agv/{agv_id}_config.yaml')

    # 參數檔路徑
    param_file = "/app/config/ecs_config.yaml"
    agv_command_file = "/app/agv_cmd_service_ws/src/agv_cmd_service/config/agv_cmd_service.yaml"

    print(f"🚗 Cargo AGV Launch 配置:")
    print(f"  AGV_ID: {agv_id}")
    print(f"  ROS_NAMESPACE: {ros_namespace}")
    print(f"  DEVICE_CONFIG_FILE: {device_config_file}")

    # 確保檔案存在
    if not os.path.exists(param_file):
        logger.warning(f"⚠️ YAML 設定檔不存在: {param_file}")
    if not os.path.exists(agv_command_file):
        print(f"⚠️ YAML 設定檔不存在: {agv_command_file}")
    if not os.path.exists(device_config_file):
        print(f"⚠️ 設備配置檔不存在: {device_config_file}")

    # 讀入 AGV 設定
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
            parameters=[param_file, device_config_file],
        ),

        # Node(
        #    package='cargo_mover_agv',
        #    executable='test_agv_core_node',
        #    name='test_agv_core_node',
        #    namespace=agv_id,
        #    parameters=[param_file],
        # ),

        # ✅ agv_cmd_service 仍使用外部 YAML 檔（若該檔格式本身就包含 ros__parameters）
        # Node(
        #    package='agv_cmd_service',
        #    executable='agv_cmd_service_node',
        #    name='agv_cmd_service',
        #    namespace=agv_id,
        #    parameters=[agv_command_file],  # 這裡沒問題
        # ),

        # ✅ agv_base 使用解析後的參數 dict
        # Node(
        #    package='cargo_mover_agv',
        #    executable='agv_core_node',
        #    name='agv_core_node',
        #    namespace=agv_id,
        #    # parameters=[config['agv_base']['ros__parameters']],
        # ),


    ])
