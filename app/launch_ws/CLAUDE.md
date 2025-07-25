# launch_ws CLAUDE.md

## 模組概述
啟動檔案管理工作空間，集中管理RosAGV系統的啟動配置與部署腳本

## 專案結構
```
src/
└── launch_manager/     # 啟動管理核心
    ├── launch/         # ROS 2啟動檔案
    ├── configs/        # 啟動配置檔案
    ├── scripts/        # 部署與管理腳本
    └── templates/      # 配置模板
```

## 核心功能

### 啟動管理
- **系統啟動**: 統一的系統啟動入口
- **服務編排**: 依序啟動相依服務
- **配置管理**: 集中管理啟動參數
- **環境檢測**: 自動檢測並適配部署環境

### 部署支援
- **容器部署**: Docker容器化部署支援
- **環境隔離**: AGV與AGVC環境分離管理
- **健康檢查**: 服務啟動後健康狀態檢查
- **錯誤恢復**: 啟動失敗自動恢復機制

## 開發指令

### 環境設定
```bash
# AGV容器內
source /app/setup.bash && all_source
cd /app/launch_ws

# AGVC容器內
source /app/setup.bash && agvc_source
cd /app/launch_ws
```

### 系統啟動
```bash
# 啟動完整AGV系統
ros2 launch launch_manager agv_system.launch.py

# 啟動AGVC管理系統
ros2 launch launch_manager agvc_system.launch.py

# 啟動特定子系統
ros2 launch launch_manager agv_motion.launch.py
ros2 launch launch_manager web_services.launch.py
```

### 構建與測試
```bash
build_ws launch_ws
ros2 test launch_manager  # 啟動系統測試
```

## 啟動檔案架構

### 系統級啟動檔案
```python
# launch/agv_system.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # 聲明啟動參數
    agv_type_arg = DeclareLaunchArgument(
        'agv_type',
        default_value='cargo_mover',
        description='AGV類型: cargo_mover, loader, unloader'
    )
    
    agv_id_arg = DeclareLaunchArgument(
        'agv_id', 
        default_value='AGV001',
        description='AGV識別碼'
    )
    
    enable_joystick_arg = DeclareLaunchArgument(
        'enable_joystick',
        default_value='true',
        description='是否啟用搖桿控制'
    )
    
    # 基礎系統啟動
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('launch_manager'),
            '/launch/agv_base.launch.py'
        ]),
        launch_arguments={
            'agv_id': LaunchConfiguration('agv_id'),
            'agv_type': LaunchConfiguration('agv_type')
        }.items()
    )
    
    # 感測器系統啟動
    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('launch_manager'),
            '/launch/sensor_system.launch.py'
        ]),
        launch_arguments={
            'agv_id': LaunchConfiguration('agv_id')
        }.items()
    )
    
    # 搖桿控制啟動(條件式)
    joystick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('launch_manager'),
            '/launch/joystick_control.launch.py'
        ]),
        condition=IfCondition(LaunchConfiguration('enable_joystick')),
        launch_arguments={
            'agv_id': LaunchConfiguration('agv_id')
        }.items()
    )
    
    return LaunchDescription([
        agv_type_arg,
        agv_id_arg,
        enable_joystick_arg,
        base_launch,
        sensor_launch,
        joystick_launch
    ])
```

### 子系統啟動檔案
```python
# launch/agv_base.launch.py  
def generate_launch_description():
    # 載入配置參數
    config_file = os.path.join(
        get_package_share_directory('launch_manager'),
        'configs', 'agv_base_config.yaml'
    )
    
    # AGV基礎節點
    agv_base_node = Node(
        package='agv_base',
        executable='agv_node_base',
        name='agv_base',
        parameters=[config_file, {
            'agv_id': LaunchConfiguration('agv_id'),
            'agv_type': LaunchConfiguration('agv_type')
        }],
        output='screen',
        emulate_tty=True
    )
    
    # PLC通訊節點
    plc_proxy_node = Node(
        package='plc_proxy',
        executable='plc_proxy_node',
        name='plc_proxy',
        parameters=[config_file],
        output='screen',
        emulate_tty=True
    )
    
    # Keyence PLC驅動
    keyence_plc_node = Node(
        package='keyence_plc',
        executable='keyence_plc_node',
        name='keyence_plc',
        parameters=[config_file],
        output='screen',
        emulate_tty=True
    )
    
    # 指令服務
    cmd_service_node = Node(
        package='agv_cmd_service',
        executable='agv_cmd_service_node',
        name='agv_cmd_service',
        parameters=[config_file],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('agv_id', default_value='AGV001'),
        DeclareLaunchArgument('agv_type', default_value='cargo_mover'),
        agv_base_node,
        plc_proxy_node,
        keyence_plc_node,
        cmd_service_node
    ])
```

### AGVC系統啟動
```python
# launch/agvc_system.launch.py
def generate_launch_description():
    # AGVC配置檔案
    agvc_config = os.path.join(
        get_package_share_directory('launch_manager'),
        'configs', 'agvc_system_config.yaml'
    )
    
    # 資料庫代理服務
    db_proxy_node = Node(
        package='db_proxy',
        executable='db_proxy_node',
        name='db_proxy',
        parameters=[agvc_config],
        output='screen'
    )
    
    # ECS設備控制系統
    ecs_node = Node(
        package='ecs',
        executable='ecs_node',
        name='ecs',
        parameters=[agvc_config],
        output='screen'
    )
    
    # RCS機器人控制系統
    rcs_node = Node(
        package='rcs',
        executable='rcs_node',
        name='rcs',
        parameters=[agvc_config],
        output='screen'
    )
    
    # WCS倉庫控制系統
    wcs_node = Node(
        package='wcs',
        executable='wcs_node',
        name='wcs',
        parameters=[agvc_config],
        output='screen'
    )
    
    # KUKA Fleet整合
    kuka_fleet_node = Node(
        package='kuka_fleet',
        executable='kuka_fleet_node',
        name='kuka_fleet',
        parameters=[agvc_config],
        output='screen'
    )
    
    # Web API服務
    web_api_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('launch_manager'),
            '/launch/web_services.launch.py'
        ])
    )
    
    return LaunchDescription([
        db_proxy_node,
        ecs_node,
        rcs_node,
        wcs_node,
        kuka_fleet_node,
        web_api_launch
    ])
```

## 配置管理

### 動態配置載入
```python
# configs/config_loader.py
class ConfigLoader:
    def __init__(self):
        self.config_cache = {}
        self.environment = os.getenv('CONTAINER_TYPE', 'unknown')
        
    def load_config(self, config_name: str) -> dict:
        """載入配置檔案"""
        if config_name in self.config_cache:
            return self.config_cache[config_name]
            
        config_path = self.resolve_config_path(config_name)
        
        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
            
        # 環境變數替換
        config = self.substitute_env_vars(config)
        
        # 快取配置
        self.config_cache[config_name] = config
        
        return config
        
    def resolve_config_path(self, config_name: str) -> str:
        """解析配置檔案路徑"""
        base_path = '/app/config'
        
        # 根據環境選擇配置目錄
        if self.environment == 'agv':
            env_path = os.path.join(base_path, 'agv')
        elif self.environment == 'agvc':
            env_path = os.path.join(base_path, 'agvc')
        else:
            env_path = base_path
            
        config_file = f"{config_name}.yaml"
        config_path = os.path.join(env_path, config_file)
        
        if os.path.exists(config_path):
            return config_path
        else:
            # 回退到通用配置
            return os.path.join(base_path, config_file)
```

### 參數模板系統
```yaml
# templates/agv_config_template.yaml
agv_system:
  agv_id: "${AGV_ID}"
  agv_type: "${AGV_TYPE}"
  
  # 運動參數
  motion:
    max_linear_velocity: 2.0
    max_angular_velocity: 1.57
    acceleration_limit: 1.0
    
  # 感測器配置
  sensors:
    lidar:
      enabled: true
      device_path: "/dev/ttyACM0"
      frame_rate: 10
      
    imu:
      enabled: true
      device_path: "/dev/ttyUSB0"
      sample_rate: 100
      
    camera:
      enabled: "${ENABLE_CAMERA:-false}"
      device_id: 0
      resolution: [1280, 720]
      
  # PLC通訊設定
  plc:
    host: "${PLC_HOST}"
    port: 8501
    timeout: 5.0
    
  # 安全設定
  safety:
    emergency_stop_enabled: true
    obstacle_detection_range: 1.0
    safety_zone_radius: 0.5
```

## 部署腳本

### 系統部署腳本
```bash
#!/bin/bash
# scripts/deploy_agv_system.sh

set -e

# 配置參數
AGV_ID=${1:-"AGV001"}
AGV_TYPE=${2:-"cargo_mover"}
ENABLE_JOYSTICK=${3:-"true"}

echo "部署AGV系統: ID=$AGV_ID, Type=$AGV_TYPE"

# 檢查環境
if [ "$CONTAINER_TYPE" != "agv" ]; then
    echo "錯誤: 必須在AGV容器內執行此腳本"
    exit 1
fi

# 載入ROS環境
source /app/setup.bash
all_source

# 檢查依賴服務
echo "檢查依賴服務..."
check_zenoh_status
check_ros_env

# 啟動AGV系統
echo "啟動AGV系統..."
ros2 launch launch_manager agv_system.launch.py \
    agv_id:=$AGV_ID \
    agv_type:=$AGV_TYPE \
    enable_joystick:=$ENABLE_JOYSTICK &

AGV_PID=$!

# 等待服務啟動
echo "等待服務啟動..."
sleep 10

# 健康檢查
echo "執行健康檢查..."
if ros2 service call /agv_base/get_status std_srvs/srv/Trigger; then
    echo "AGV系統啟動成功"
else
    echo "AGV系統啟動失敗"
    kill $AGV_PID
    exit 1
fi

# 保持運行
wait $AGV_PID
```

### 健康檢查腳本
```python
# scripts/health_check.py
import rclpy
import sys
import time
from rclpy.node import Node
from std_srvs.srv import Trigger

class HealthChecker(Node):
    def __init__(self):
        super().__init__('health_checker')
        self.services_to_check = [
            '/agv_base/get_status',
            '/plc_proxy/get_status', 
            '/keyence_plc/get_status'
        ]
        
    def check_all_services(self) -> bool:
        """檢查所有關鍵服務"""
        all_healthy = True
        
        for service_name in self.services_to_check:
            if not self.check_service(service_name):
                all_healthy = False
                
        return all_healthy
        
    def check_service(self, service_name: str) -> bool:
        """檢查單一服務健康狀態"""
        try:
            client = self.create_client(Trigger, service_name)
            
            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f"服務不可用: {service_name}")
                return False
                
            request = Trigger.Request()
            future = client.call_async(request)
            
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() and future.result().success:
                self.get_logger().info(f"服務正常: {service_name}")
                return True
            else:
                self.get_logger().error(f"服務異常: {service_name}")
                return False
                
        except Exception as e:
            self.get_logger().error(f"檢查服務 {service_name} 時發生錯誤: {e}")
            return False

def main():
    rclpy.init()
    
    health_checker = HealthChecker()
    
    try:
        if health_checker.check_all_services():
            print("所有服務健康檢查通過")
            sys.exit(0)
        else:
            print("部分服務健康檢查失敗")
            sys.exit(1)
    finally:
        health_checker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 環境檢測與適配

### 環境檢測器
```python
# scripts/environment_detector.py
class EnvironmentDetector:
    def __init__(self):
        self.container_type = os.getenv('CONTAINER_TYPE')
        self.hardware_info = self.detect_hardware()
        
    def detect_hardware(self) -> dict:
        """檢測硬體配置"""
        hardware = {
            'cpu_cores': os.cpu_count(),
            'memory_gb': self.get_memory_size(),
            'has_gpu': self.check_gpu_availability(),
            'usb_devices': self.list_usb_devices(),
            'network_interfaces': self.list_network_interfaces()
        }
        
        return hardware
        
    def detect_sensors(self) -> dict:
        """檢測可用感測器"""
        sensors = {
            'lidar': self.check_lidar_devices(),
            'camera': self.check_camera_devices(), 
            'imu': self.check_imu_devices(),
            'joystick': self.check_joystick_devices()
        }
        
        return sensors
        
    def generate_optimal_config(self) -> dict:
        """根據檢測結果生成最佳配置"""
        config = {
            'system': {
                'container_type': self.container_type,
                'hardware': self.hardware_info
            },
            'sensors': self.detect_sensors(),
            'performance': self.calculate_performance_settings()
        }
        
        return config
```

## 測試與驗證

### 啟動測試
```bash
# 測試AGV系統啟動
ros2 test launch_manager --package-select agv_system_test

# 測試AGVC系統啟動
ros2 test launch_manager --package-select agvc_system_test

# 整合測試
ros2 run launch_manager integration_test
```

### 效能基準測試
```python
# tests/performance_benchmark.py
class PerformanceBenchmark:
    def __init__(self):
        self.metrics = {}
        
    def benchmark_startup_time(self, launch_file: str) -> float:
        """測量系統啟動時間"""
        start_time = time.time()
        
        # 啟動系統
        launch_process = self.launch_system(launch_file)
        
        # 等待所有服務就緒
        self.wait_for_services_ready()
        
        startup_time = time.time() - start_time
        
        # 清理
        launch_process.terminate()
        
        return startup_time
        
    def benchmark_resource_usage(self, launch_file: str) -> dict:
        """測量資源使用情況"""
        # 啟動系統並監控資源使用
        launch_process = self.launch_system(launch_file)
        
        cpu_usage = self.monitor_cpu_usage(duration=60)
        memory_usage = self.monitor_memory_usage(duration=60)
        
        # 清理
        launch_process.terminate()
        
        return {
            'cpu_usage_avg': cpu_usage['average'],
            'cpu_usage_peak': cpu_usage['peak'],
            'memory_usage_avg': memory_usage['average'],
            'memory_usage_peak': memory_usage['peak']
        }
```

## 故障排除

### 常見問題
1. **啟動失敗**: 檢查依賴服務與配置檔案
2. **服務無法啟動**: 確認容器環境與權限設定
3. **配置載入錯誤**: 驗證配置檔案格式與路徑
4. **健康檢查失敗**: 檢查服務狀態與網路連接

### 診斷工具
```bash
# 檢查啟動狀態
ros2 launch launch_manager system_diagnostics.launch.py

# 查看服務狀態
ros2 service list
ros2 node list

# 檢查配置載入
ros2 run launch_manager config_validator
```

## 最佳實踐

### 啟動順序管理
- **依賴檢查**: 確保依賴服務先啟動
- **健康檢查**: 啟動後驗證服務狀態
- **優雅關閉**: 實施正確的關閉順序
- **錯誤恢復**: 啟動失敗自動重試機制

### 配置管理
- **環境分離**: 不同環境使用不同配置
- **參數驗證**: 啟動前驗證配置參數
- **模板化**: 使用配置模板提高複用性
- **版本控制**: 配置檔案版本化管理

## 重要提醒
- 啟動系統直接影響整體穩定性，變更需謹慎測試
- 配置參數需根據實際硬體環境調整
- 健康檢查機制不可忽略，確保系統可靠性
- 支援AGV與AGVC雙環境，注意環境檢測邏輯