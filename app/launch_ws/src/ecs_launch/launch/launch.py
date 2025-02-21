from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 獲取 YAML 檔案的路徑
    param_file = os.path.join(
        '/app/'
        'ecs_config',  # 指定 'config' 資料夾
        'params.yaml'  # YAML 配置文件
    )

    return LaunchDescription([
        Node(
            package='rmw_zenoh_cpp',
            executable='rmw_zenohd',
            name='rmw_zenohd',
        ),
        DeclareLaunchArgument(
            'param_file', default_value=param_file, description='Path to parameter file'
        ),
        Node(
            package='ecs',
            executable='plc_service',
            name='ecs_plc_service',#重命名為ecs使用的plc_servicve
            parameters=[LaunchConfiguration('param_file')]  # 使用 YAML 文件中的參數
        ),
       #Node(
       #    package='web_api',
       #    executable='api_server',
       #    name='web_api_server',
       #)
    ])
