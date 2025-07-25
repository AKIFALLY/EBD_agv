# kuka_fleet_ws CLAUDE.md

## 模組概述
KUKA Fleet 適配器工作空間，提供與 KUKA Fleet Manager API 的整合介面，實現 KUKA AGV 車隊管理、任務派發和狀態監控功能。

## 專案結構
```
src/
└── kuka_fleet_adapter/           # KUKA Fleet 適配器套件
    ├── kuka_fleet_adapter/
    │   ├── __init__.py          # 模組初始化
    │   ├── kuka_fleet_adapter.py # 主要適配器實現
    │   ├── kuka_api_client.py   # KUKA Fleet API 客戶端
    │   ├── api.md              # API 文檔
    │   └── api_callback.md     # API 回調文檔
    ├── test/
    │   └── test_kuka_api_convenience.py # API 便利方法測試
    ├── package.xml             # ROS 2 套件配置
    ├── setup.py               # Python 套件設定
    └── setup.cfg              # 建置配置
```

## 核心功能

### KukaFleetAdapter - KUKA 車隊適配器
負責 ROS 2 節點整合和車隊管理邏輯：

```python
# kuka_fleet_adapter.py 核心類別
class KukaFleetAdapter:
    # AGV 狀態常數
    STATUS_REMOVED = 1    # 離場
    STATUS_OFFLINE = 2    # 離線  
    STATUS_IDLE = 3       # 空閒
    STATUS_RUNNING = 4    # 任務中
    STATUS_CHARGING = 5   # 充電中
    STATUS_UPDATING = 6   # 更新中
    STATUS_ERROR = 7      # 錯誤
    
    # 任務類型
    MISSION_MOVE = "MOVE"       # 移動
    MISSION_RACK_MOVE = "RACK_MOVE"  # 搬運
    
    MAP_LAYOUT_DISTRICT = "test-test1"  # 地圖佈局區域
```

**主要方法**：
- `__init__(node)`: 初始化適配器，整合 ROS 2 節點
- `start_monitoring()`: 啟動週期性狀態監控
- `stop_monitoring()`: 停止狀態監控
- `monitor_robot_and_container()`: 監控機器人與容器狀態
- `select_agv(status, robot_id)`: 根據狀態篩選 AGV
- `move(nodes, robot_id, mission_code)`: 執行移動任務
- `rack_move(nodes, robot_id, mission_code)`: 執行搬運任務
- `workflow(workflow, robot_id, mission_code)`: 執行工作流程任務

### KukaApiClient - KUKA API 客戶端
實現完整的 KUKA Fleet API 通訊：

**認證功能**：
- `login()`: API 登入認證
- `is_token_valid()`: 檢查 Token 有效性

**任務管理**：
- `submit_mission()`: 提交任務
- `job_query()`: 查詢作業
- `mission_cancel()`: 取消任務
- `pause_mission()`: 暫停任務
- `recover_mission()`: 恢復任務

**機器人管理**：
- `robot_query()`: 查詢機器人狀態
- `robot_in()`: 機器人入場
- `robot_out()`: 機器人離場
- `robot_charge()`: 機器人充電

**容器管理**：
- `container_query_all()`: 查詢所有容器
- `container_in()`: 容器入場
- `container_out()`: 容器出場

**便利方法**：
- `get_all_robots()`: 獲取所有機器人狀態
- `get_robot_by_id(robot_id)`: 根據 ID 查詢機器人
- `get_all_containers_in_map()`: 獲取所有在場容器
- `get_running_jobs()`: 獲取運行中的作業

## 開發指令

### 環境設定 (AGVC容器內)
```bash
source /app/setup.bash
agvc_source  # 載入AGVC工作空間
cd /app/kuka_fleet_ws
```

### 建置與安裝
```bash
# 建置工作空間
build_ws kuka_fleet_ws

# 或使用完整建置指令
source /opt/ros/jazzy/setup.bash && source /opt/ws_rmw_zenoh/install/setup.bash && cd /app/kuka_fleet_ws && colcon build
```

### 服務啟動
```bash
# 使用 ROS 2 啟動
ros2 run kuka_fleet_adapter kuka_fleet_adapter

# 使用自定義參數啟動
ros2 run kuka_fleet_adapter kuka_fleet_adapter \
  --ros-args \
  -p api_base_url:="http://192.168.10.3:10870" \
  -p api_username:="admin" \
  -p api_password:="Admin"
```

### 測試
```bash
# 執行 API 便利方法測試
cd /app/kuka_fleet_ws/src/kuka_fleet_adapter/test
python3 test_kuka_api_convenience.py

# 或使用 colcon 測試
colcon test --packages-select kuka_fleet_adapter
colcon test-result --verbose
```

## ROS 2 整合

### 節點參數
```python
# 預設參數配置
'api_base_url': 'http://192.168.10.3:10870'
'api_username': 'admin'
'api_password': 'Admin'
'query_cycle_time': 0.1    # 查詢週期 (秒)
'timer_period': 0.05       # 監控間隔 (秒)
```

### 節點生命週期
```python
def main(args=None):
    rclpy.init(args=args)
    example_node = rclpy.create_node('kuka_adapter_demo_node')
    adapter = KukaFleetAdapter(example_node)
    adapter.start_monitoring()
    
    try:
        rclpy.spin(example_node)
    except KeyboardInterrupt:
        pass
    finally:
        adapter.stop_monitoring()
        example_node.destroy_node()
        rclpy.shutdown()
```

## 任務執行範例

### 移動任務
```python
# 建立適配器
adapter = KukaFleetAdapter(node)

# 執行移動任務
nodes = [1, 2, 3]  # 節點序列
robot_id = 101
mission_code = "MOVE_001"
result = adapter.move(nodes, robot_id, mission_code)
```

### 搬運任務
```python
# 執行搬運任務
nodes = ["test-test1-1", "test-test1-5"]
robot_id = 101
mission_code = "RACK_001"  
result = adapter.rack_move(nodes, robot_id, mission_code)
```

### 工作流程任務
```python
# 執行工作流程
workflow = "WORKFLOW_001"
robot_id = 101
mission_code = "WF_001"
result = adapter.workflow(workflow, robot_id, mission_code)
```

## API 使用範例

### 基本 API 操作
```python
from kuka_fleet_adapter.kuka_api_client import KukaApiClient

# 建立客戶端 (自動登入)
client = KukaApiClient(
    base_url='http://192.168.10.3:10870',
    username='admin',
    password='Admin'
)

# 查詢所有機器人
robots = client.get_all_robots()
print(f"找到 {len(robots.get('data', []))} 台機器人")

# 查詢特定機器人
robot = client.get_robot_by_id('101')

# 查詢所有容器
containers = client.get_all_containers_in_map()

# 查詢運行中作業
jobs = client.get_running_jobs()
```

### 任務管理
```python
# 提交任務
mission = {
    "orgId": "Ching-Tech",
    "requestId": "REQ_001",
    "missionCode": "MISSION_001",
    "missionType": "MOVE",
    "robotIds": [101],
    "missionData": [
        {
            "sequence": 1,
            "position": "test-test1-1",
            "type": "NODE_POINT",
            "passStrategy": "AUTO"
        }
    ]
}
result = client.submit_mission(mission)

# 查詢任務狀態
jobs = client.job_query({"jobCode": "MISSION_001"})

# 取消任務
cancel_result = client.mission_cancel({
    "missionCode": "MISSION_001",
    "cancelMode": "FORCE"
})
```

## 配置管理

### API 配置
```python
# 在 kuka_fleet_adapter.py 中的配置
self.node.declare_parameter('api_base_url', 'http://192.168.10.3:10870')
self.node.declare_parameter('api_username', 'admin')
self.node.declare_parameter('api_password', 'Admin')
self.node.declare_parameter('query_cycle_time', 0.1)
self.node.declare_parameter('timer_period', 0.05)
```

### 機器人配置
```python
# 預設機器人配置
ROBOT_CONFIG = {
    "robotModels": ["KMP 400i diffDrive"],
    "robotType": "LIFT",
    "orgId": "Ching-Tech"
}
```

## 故障排除

### 常見問題

#### API 連線失敗
```bash
# 檢查網路連通性
ping 192.168.10.3

# 測試 API 端點
curl -X POST http://192.168.10.3:10870/api/login \
  -H "Content-Type: application/json" \
  -d '{"username":"admin","password":"Admin"}'
```

#### 節點啟動失敗
```bash
# 檢查套件安裝
ros2 pkg list | grep kuka_fleet_adapter

# 檢查節點狀態
ros2 node list | grep kuka
ros2 node info /kuka_fleet_adapter

# 檢查參數
ros2 param list /kuka_fleet_adapter
```

#### 任務執行失敗
```bash
# 檢查機器人狀態
python3 -c "
from kuka_fleet_adapter.kuka_api_client import KukaApiClient
client = KukaApiClient(username='admin', password='Admin')
if client.token:
    result = client.get_all_robots()
    print(f'機器人狀態: {result}')
"
```

## 測試方法

### 手動測試
```bash
# 測試 API 便利方法
cd /app/kuka_fleet_ws/src/kuka_fleet_adapter/test
python3 test_kuka_api_convenience.py
```

### 模組測試
```bash
# 測試模組載入
python3 -c "
from kuka_fleet_adapter.kuka_fleet_adapter import KukaFleetAdapter
from kuka_fleet_adapter.kuka_api_client import KukaApiClient
print('✅ 模組載入成功')
"
```

### 整合測試
```bash
# 啟動節點測試
ros2 run kuka_fleet_adapter kuka_fleet_adapter &
sleep 5
ros2 node list | grep kuka_fleet_adapter
pkill -f kuka_fleet_adapter
```

## 整合點

### 與其他系統整合
- **rcs_ws**: 使用 KukaFleetAdapter 進行 KUKA 車隊管理
- **wcs_ws**: 透過 KukaFleetAdapter 整合 KUKA 任務派發

### 外部依賴
- **KUKA Fleet Manager**: 必須可達 http://192.168.10.3:10870
- **ROS 2 Jazzy**: 核心 ROS 2 框架
- **Python requests**: HTTP 客戶端庫

## 重要提醒
- 此工作空間需手動啟動，未包含在容器自動啟動腳本中
- 必須確保 KUKA Fleet Manager 可連線
- API 認證使用 Authorization header，不需要 Bearer 前綴
- 所有 ROS 2 相關操作必須在 AGVC 容器內執行
- 地圖區域配置為 "test-test1"，需根據實際環境調整