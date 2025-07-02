from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 定義 YAML 配置文件的路徑
    param_file = '/app/config/ecs_config.yaml'  # 更新為正確的路徑

    return LaunchDescription([
        # 定義參數文件
        DeclareLaunchArgument(
            'param_file', default_value=param_file, description='Path to parameter file'
        ),
        # 啟動 plc_service 節點，這是與 PLC 通訊的服務節點
        Node(
            package='plc_proxy',
            executable='plc_service',  # 使用 plc_service 節點
            name='plc_service',  # 節點名稱
            namespace='agvc',  # 命名空間
            parameters=[LaunchConfiguration('param_file')],  # 使用 YAML 配置文件中的參數
            output="screen"
        ),
        #
        # 啟動 ecs_core 節點，並傳入配置參數
        Node(
            package='ecs',
            executable='ecs_core',  # 請根據實際情況替換
            name='ecs_core',  # 節點名稱
            namespace='agvc',  # 命名空間
            parameters=[LaunchConfiguration('param_file')],  # 傳遞參數
            output="screen"
        ),

        # 啟動 door_controller_mqtt 節點，並傳入配置參數
        # Node(
        #    package='ecs',
        #    executable='door_controller_node_mqtt',
        #    name='door_controller_node_mqtt',  # 節點名稱
        #    namespace='agvc',  # 命名空間
        #    parameters=[LaunchConfiguration('param_file')],  # 傳遞參數
        #    output="screen"
        # ),
    ])
