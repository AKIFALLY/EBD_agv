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
        Node(
            package='plc_proxy',
            executable='plc_service',  # 使用 plc_service 節點
            name='plc_service',  # 節點名稱
            namespace='agvc',  # 命名空間
            parameters=[LaunchConfiguration('param_file')],  # 使用 YAML 配置文件中的參數
            output="screen"
        ),
        Node(
            package='web_api',
            executable='api_server',
            name='web_api_server',
            namespace='agvc',  # 命名空間
        )
    ])
