# KUKA AGV Fleet 整合

## 🎯 KUKA AGV Fleet 系統整合方案

本文檔詳細說明 RosAGV 與 KUKA Fleet Manager 的整合架構。KUKA Fleet Manager 是用於管理 KUKA AGV (KMP 400i) 車隊的系統，提供任務調度、機器人監控和容器管理功能。

## 📋 整合概覽

### KUKA AGV Fleet 系統定位
```
KUKA AGV Fleet 在 RosAGV 生態中的角色
├── 🚗 移動機器人車隊
│   ├── KMP 400i diffDrive AGV
│   ├── Fleet Manager 車隊管理系統
│   └── 容器運輸和搬運任務
├── 🔗 系統整合層
│   ├── kuka_fleet_ws ROS 2 工作空間
│   ├── REST API 通訊介面 (Port 10870)
│   └── 任務狀態回調機制
└── 📦 運輸協作
    ├── AGV 任務調度
    ├── 容器管理和追蹤
    └── 即時狀態監控
```

### 整合架構
```
整合架構圖
RosAGV AGVC 系統
├── KUKA WCS (倉儲控制系統)
├── kuka_fleet_ws (KUKA 整合工作空間)
│   ├── KukaFleetAdapter (ROS 2 適配器)
│   ├── KukaApiClient (API 客戶端)
│   └── 狀態監控定時器
└── Web API (Port 8000)
    └── /interfaces/api/amr/missionStateCallback
        ↕️ HTTP REST API
KUKA Fleet Manager (192.168.10.3:10870)
├── AGV 車隊管理
├── 任務調度系統
└── 容器追蹤管理
    ↕️
KMP 400i AGV 車隊
```

## 🔧 技術架構

### KUKA Fleet Adapter
```python
# KUKA Fleet Adapter 核心架構 (實際實作)
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
    MISSION_MOVE = "MOVE"           # 移動任務
    MISSION_RACK_MOVE = "RACK_MOVE" # 搬運任務

    # 地圖佈局區域
    MAP_LAYOUT_DISTRICT = "test-test1"

    def __init__(self, node: Node):
        self.api_client = KukaApiClient(
            base_url='http://192.168.10.3:10870',
            username='admin',
            password='Admin'
        )
        # 週期性監控機器人和容器狀態
        self.query_cycle_time = 0.1  # 0.1秒查詢一次
        self.timer_period = 0.05     # 0.05秒監控間隔
```

### API 客戶端實作
```python
# KUKA Fleet API 客戶端 (實際實作)
class KukaApiClient:
    def __init__(self, base_url="http://192.168.10.3:10870", username=None, password=None):
        self.base_url = base_url
        self.token = None
        if username and password:
            self.login(username, password)

    def login(self, username: str, password: str):
        """登入獲取 token"""
        payload = {"username": username, "password": password}
        response = self._post_request("/api/login", data=payload)
        if response.get("success"):
            token = response.get("data", {}).get("token")
            self.token = token  # 注意：不使用 Bearer 前綴

    def _get_headers(self, include_auth=True):
        headers = {"Content-Type": "application/json"}
        if include_auth and self.token:
            headers["Authorization"] = self.token  # 直接使用 token，無 Bearer
        return headers
```

## 🌐 REST API 介面

### 認證機制
```http
# 登入取得 token
POST http://192.168.10.3:10870/api/login
Content-Type: application/json

{
    "username": "admin",
    "password": "Admin"
}

# 回應
{
    "success": true,
    "data": {
        "token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9..."
    }
}
```

### 任務管理 API

#### 提交任務
```http
# 提交 AGV 任務
POST /api/amr/submitMission
Content-Type: application/json
Authorization: {token}

{
    "orgId": "Ching-Tech",
    "requestId": "REQ_001",
    "missionCode": "MISSION_001",
    "missionType": "MOVE",
    "robotModels": ["KMP 400i diffDrive"],
    "robotIds": [101],
    "robotType": "LIFT",
    "priority": 1,
    "missionData": [
        {
            "sequence": 1,
            "position": "test-test1-1",
            "type": "NODE_POINT",
            "passStrategy": "AUTO"
        },
        {
            "sequence": 2,
            "position": "test-test1-5",
            "type": "NODE_POINT",
            "passStrategy": "AUTO"
        }
    ]
}
```

#### 查詢任務
```http
# 查詢作業狀態
POST /api/amr/jobQuery
Content-Type: application/json
Authorization: {token}

{
    "jobCode": "MISSION_001",
    "status": 2  // 0=All, 1=Pending, 2=Running, 3=Completed, 4=Failed, 5=Cancelled
}
```

#### 取消任務
```http
# 取消任務
POST /api/amr/missionCancel
Content-Type: application/json
Authorization: {token}

{
    "missionCode": "MISSION_001",
    "cancelMode": "FORCE"
}
```

### 機器人管理 API

#### 查詢機器人狀態
```http
# 查詢所有機器人
POST /api/amr/robotQuery
Content-Type: application/json
Authorization: {token}

{}  // 空物件查詢所有

# 查詢特定機器人
{
    "robotId": "101"
}

# 回應範例
{
    "success": true,
    "data": [
        {
            "robotId": "101",
            "status": 3,  // 3=空閒
            "batteryLevel": 85,
            "position": "test-test1-10",
            "currentMission": null
        }
    ]
}
```

### 容器管理 API

#### 容器入場
```http
# 容器入場
POST /api/amr/containerIn
Content-Type: application/json
Authorization: {token}

{
    "requestId": "uuid-001",
    "containerCode": "container-001",
    "position": "test-test1-33",
    "isNew": true
}
```

#### 查詢容器
```http
# 查詢所有容器（含離場）
POST /api/amr/containerQueryAll
Content-Type: application/json
Authorization: {token}

{
    "containerCode": "container-001"  // 可選，空物件查詢所有
}

# 回應
{
    "success": true,
    "data": [
        {
            "containerCode": "container-001",
            "inMapStatus": 1,  // 1=在場, 0=離場
            "position": "test-test1-33"
        }
    ]
}
```

## 🔄 任務狀態回調機制

### 回調端點實作
KUKA Fleet Manager 會主動回調 RosAGV 系統，報告任務狀態變化：

```python
# Web API 回調接收端點 (routers/kuka.py)
@router.post("/interfaces/api/amr/missionStateCallback")
async def mission_state_callback(data: MissionStateCallbackData):
    """
    接收 KUKA 系統的任務狀態回報

    支援的狀態：
    - MOVE_BEGIN: 開始移動
    - ARRIVED: 到達任務節點
    - UP_CONTAINER: 升箱完成
    - DOWN_CONTAINER: 放下完成
    - ROLLER_RECEIVE: 滾筒上料完成
    - ROLLER_SEND: 滾筒下料完成
    - PICKER_RECEIVE: 料箱取料完成
    - PICKER_SEND: 料箱下料完成
    - FORK_UP: 叉車叉取完成
    - FORK_DOWN: 叉車放下完成
    - COMPLETED: 任務完成
    - CANCELED: 任務取消完成
    """
    # 根據 missionCode 查找對應任務
    task = session.query(Task).filter(Task.mission_code == data.missionCode).first()

    # 更新任務 parameters 欄位
    task.parameters.update({
        "kuka_mission_status": data.missionStatus,
        "kuka_robot_id": data.robotId,
        "kuka_last_update": datetime.now().isoformat()
    })
```

### 回調資料結構
```python
class MissionStateCallbackData(BaseModel):
    missionCode: str           # 任務代碼 (必填)
    missionStatus: str         # 任務狀態 (必填)
    robotId: Optional[str]     # 執行任務的機器人 ID
    containerCode: Optional[str]    # 容器代碼
    currentPosition: Optional[str]  # 當前位置
    message: Optional[str]          # 補充說明
```

## 🚀 實作範例

### 執行移動任務
```python
from kuka_fleet_adapter.kuka_fleet_adapter import KukaFleetAdapter

# 建立適配器
adapter = KukaFleetAdapter(ros_node)

# 執行移動任務
nodes = [1, 5, 10]  # 節點編號，會轉換為 test-test1-1, test-test1-5, test-test1-10
robot_id = 101
mission_code = "MOVE_001"

result = adapter.move(nodes, robot_id, mission_code)
print(f"任務提交結果: {result}")
```

### 執行搬運任務
```python
# 執行架台搬運任務
nodes = ["test-test1-1", "test-test1-20"]  # 起點和終點
robot_id = 101
mission_code = "RACK_MOVE_001"

result = adapter.rack_move(nodes, robot_id, mission_code)
print(f"搬運任務結果: {result}")
```

### 查詢 AGV 狀態
```python
from kuka_fleet_adapter.kuka_api_client import KukaApiClient

# 建立客戶端
client = KukaApiClient(
    base_url='http://192.168.10.3:10870',
    username='admin',
    password='Admin'
)

# 查詢所有機器人
robots = client.get_all_robots()
for robot in robots.get('data', []):
    print(f"Robot {robot['robotId']}: 狀態={robot['status']}, 電量={robot['batteryLevel']}%")

# 查詢空閒機器人
idle_robots = [r for r in robots.get('data', []) if r['status'] == 3]
print(f"找到 {len(idle_robots)} 台空閒機器人")
```

### 容器管理範例
```python
# 容器入場
container_in_data = {
    "requestId": str(uuid.uuid4()),
    "containerCode": "RACK-001",
    "position": "test-test1-33",
    "isNew": True
}
result = client.container_in(container_in_data)

# 查詢容器位置
containers = client.container_query_all({"containerCode": "RACK-001"})
for container in containers.get('data', []):
    print(f"容器 {container['containerCode']} 在 {container['position']}")
```

## 🔧 ROS 2 整合

### 節點配置
```python
# ROS 2 節點參數
self.node.declare_parameter('api_base_url', 'http://192.168.10.3:10870')
self.node.declare_parameter('api_username', 'admin')
self.node.declare_parameter('api_password', 'Admin')
self.node.declare_parameter('query_cycle_time', 0.1)  # 查詢週期
self.node.declare_parameter('timer_period', 0.05)     # 監控間隔
```

### 啟動節點
```bash
# 啟動 KUKA Fleet 適配器
ros2 run kuka_fleet_adapter kuka_fleet_adapter

# 使用參數覆蓋
ros2 run kuka_fleet_adapter kuka_fleet_adapter \
  --ros-args \
  -p api_base_url:="http://192.168.10.3:10870" \
  -p api_username:="admin" \
  -p api_password:="Admin"
```

## 📊 監控和診斷

### 狀態監控
```python
# KukaFleetAdapter 內建監控
def monitor_robot_and_container(self):
    """週期性監控機器人和容器狀態"""
    # 查詢機器人
    robots = self.api_client.robot_query({})
    for robot in robots.get("data", []):
        self.logger.debug(f"Robot {robot['robotId']}: Status={robot['status']}")

    # 查詢容器
    containers = self.api_client.container_query_all({})
    for container in containers.get("data", []):
        self.logger.debug(f"Container {container['containerCode']}: {container['position']}")
```

### 故障排除
```bash
# 檢查 KUKA Fleet Manager 連線
ping 192.168.10.3
curl -X POST http://192.168.10.3:10870/api/login \
  -H "Content-Type: application/json" \
  -d '{"username":"admin","password":"Admin"}'

# 測試回調端點
curl -X POST http://localhost:8000/interfaces/api/amr/missionStateCallback \
  -H "Content-Type: application/json" \
  -d '{"missionCode":"TEST001","missionStatus":"COMPLETED"}'

# 檢查 ROS 2 節點
ros2 node list | grep kuka
ros2 node info /kuka_fleet_adapter
```

## 🔧 配置管理

### 環境配置
```yaml
# kuka_fleet_config.yaml
kuka_fleet:
  connection:
    base_url: "http://192.168.10.3:10870"  # KUKA Fleet Manager 地址
    username: "admin"
    password: "Admin"
    timeout: 30
    retry_count: 3

  robots:
    - id: "101"
      type: "KMP 400i diffDrive"
      capabilities: ["MOVE", "RACK_MOVE"]

    - id: "102"
      type: "KMP 400i diffDrive"
      capabilities: ["MOVE", "RACK_MOVE"]

  map_layout:
    district: "test-test1"  # 地圖區域前綴
    nodes: 50              # 節點數量

  monitoring:
    query_cycle_time: 0.1  # 秒
    timer_period: 0.05     # 秒
```

### 便利方法清單
```python
# KukaApiClient 便利方法
client.get_all_robots()           # 獲取所有機器人
client.get_robot_by_id(robot_id)  # 根據 ID 查詢機器人
client.get_all_containers_in_map() # 獲取所有在場容器
client.get_container_by_code(code) # 根據代碼查詢容器
client.get_running_jobs()          # 獲取運行中的作業
client.get_pending_jobs()          # 獲取待執行的作業
client.is_token_valid()            # 檢查 token 有效性
client.force_relogin(user, pass)  # 強制重新登入
```

## 🚀 最佳實踐

### 任務管理
1. **使用唯一 missionCode**: 確保每個任務有唯一識別碼
2. **檢查機器人狀態**: 提交任務前確認機器人空閒
3. **處理回調更新**: 正確處理任務狀態回調
4. **錯誤重試機制**: 實作任務失敗的重試邏輯

### 效能優化
1. **連線管理**: 重用 HTTP session 減少連線開銷
2. **批量查詢**: 合併多個查詢請求
3. **狀態快取**: 短時間快取機器人狀態
4. **監控頻率**: 根據需求調整查詢週期

### 安全考量
1. **認證管理**: 安全存儲用戶名和密碼
2. **Token 刷新**: 定期檢查並刷新 token
3. **錯誤處理**: 妥善處理 API 錯誤回應
4. **日誌記錄**: 記錄所有 API 呼叫用於審計

---

**相關文檔：**
- [TAFL WCS 系統](../technical-details/tafl-wcs-integration.md) - 任務調度整合
- [技術架構](../system-architecture/dual-environment.md) - 整體系統架構
- [業務流程](../business-processes/eyewear-production.md) - AGV 運輸場景
- [故障排除](../operations/troubleshooting.md) - KUKA 整合問題排除