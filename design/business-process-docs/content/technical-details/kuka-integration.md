# KUKA Fleet 整合

## 🎯 KUKA Fleet 系統整合方案

本文檔詳細說明 RosAGV 與 KUKA Fleet Manager 的整合架構、API 介面、通訊協定和實作細節。

## 📋 整合概覽

### KUKA Fleet 系統定位
```
KUKA Fleet 在 RosAGV 生態中的角色
├── 🤖 外部機器人系統
│   ├── KUKA 工業機器人
│   ├── Fleet Manager 控制系統
│   └── 任務調度和執行
├── 🔗 系統整合層
│   ├── kuka_fleet_ws ROS 2 工作空間
│   ├── REST API 通訊介面
│   └── WebSocket 即時通訊
└── 🏭 業務協作
    ├── 任務同步和協調
    ├── 狀態資訊共享
    └── 異常處理協作
```

### 整合架構
```
整合架構圖
RosAGV AGVC 系統
├── AI WCS (統一決策引擎)
├── kuka_fleet_ws (KUKA 整合工作空間)
│   ├── KUKA Fleet Adapter
│   ├── Task Synchronization
│   └── Status Monitor
└── Web API (統一介面)
    ↕️ HTTP/WebSocket
KUKA Fleet Manager
├── Robot Control
├── Task Management
└── Status Reporting
    ↕️
KUKA Industrial Robots
```

## 🔧 技術架構

### KUKA Fleet Adapter
```python
# KUKA Fleet Adapter 核心架構
class KukaFleetAdapter:
    def __init__(self):
        self.fleet_client = KukaFleetClient()
        self.task_manager = TaskManager()
        self.status_monitor = StatusMonitor()
        
    def initialize_connection(self):
        """初始化與 KUKA Fleet 的連接"""
        pass
        
    def sync_tasks(self):
        """同步任務狀態"""
        pass
        
    def handle_events(self):
        """處理 KUKA Fleet 事件"""
        pass
```

### API 整合層
```python
# KUKA Fleet API 客戶端
class KukaFleetClient:
    def __init__(self, base_url, api_key):
        self.base_url = base_url
        self.api_key = api_key
        self.session = requests.Session()
        
    def create_task(self, task_data):
        """創建 KUKA 機器人任務"""
        endpoint = f"{self.base_url}/api/v2/tasks"
        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json"
        }
        return self.session.post(endpoint, json=task_data, headers=headers)
    
    def get_task_status(self, task_id):
        """獲取任務狀態"""
        endpoint = f"{self.base_url}/api/v2/tasks/{task_id}"
        headers = {"Authorization": f"Bearer {self.api_key}"}
        return self.session.get(endpoint, headers=headers)
```

## 🌐 通訊協定

### REST API 介面

#### 任務管理 API
```http
# 創建機器人任務
POST /api/v2/tasks
Content-Type: application/json
Authorization: Bearer {api_key}

{
    "task_type": "pick_and_place",
    "robot_id": "kuka_01",
    "parameters": {
        "pickup_location": {"x": 100, "y": 200, "z": 50},
        "dropoff_location": {"x": 300, "y": 400, "z": 50},
        "item_type": "eyewear_frame",
        "priority": "high"
    },
    "timeout": 300,
    "callback_url": "http://agvc.local:8000/kuka/callback"
}

# 回應
{
    "task_id": "task_12345",
    "status": "created",
    "estimated_duration": 120,
    "robot_assigned": "kuka_01"
}
```

#### 狀態查詢 API
```http
# 查詢任務狀態
GET /api/v2/tasks/{task_id}
Authorization: Bearer {api_key}

# 回應
{
    "task_id": "task_12345",
    "status": "executing",
    "progress": 75,
    "current_action": "moving_to_dropoff",
    "estimated_completion": "2025-12-01T10:30:00Z",
    "robot_id": "kuka_01"
}
```

#### 機器人狀態 API
```http
# 查詢機器人狀態
GET /api/v2/robots/{robot_id}/status
Authorization: Bearer {api_key}

# 回應
{
    "robot_id": "kuka_01",
    "status": "active",
    "current_task": "task_12345",
    "position": {"x": 150, "y": 250, "z": 75},
    "battery_level": 85,
    "last_update": "2025-12-01T10:25:30Z"
}
```

### WebSocket 即時通訊
```javascript
// WebSocket 連接範例
const ws = new WebSocket('ws://fleet.kuka.local:8080/ws');

ws.onmessage = function(event) {
    const data = JSON.parse(event.data);
    switch(data.type) {
        case 'task_status_update':
            handleTaskStatusUpdate(data);
            break;
        case 'robot_alert':
            handleRobotAlert(data);
            break;
        case 'system_notification':
            handleSystemNotification(data);
            break;
    }
};

// 任務狀態更新事件
{
    "type": "task_status_update",
    "task_id": "task_12345",
    "status": "completed",
    "completion_time": "2025-12-01T10:28:45Z",
    "result": "success"
}
```

## 🔄 任務同步機制

### 任務生命週期
```
KUKA 任務生命週期
1. Created (已創建)
   ├── RosAGV 發送任務請求
   └── KUKA Fleet 分配機器人
   
2. Queued (已排隊)
   ├── 等待機器人可用
   └── 任務優先級排序
   
3. Executing (執行中)
   ├── 機器人開始執行
   ├── 進度即時回報
   └── 異常狀況處理
   
4. Completed (已完成)
   ├── 任務成功完成
   ├── 結果資料回傳
   └── 資源釋放
   
5. Failed (執行失敗)
   ├── 錯誤原因分析
   ├── 重試機制觸發
   └── 異常告警處理
```

### 任務協調邏輯
```python
# 任務協調管理器
class TaskCoordinator:
    def __init__(self):
        self.agv_tasks = {}
        self.kuka_tasks = {}
        self.coordination_rules = self.load_rules()
    
    def coordinate_task(self, agv_task):
        """協調 AGV 和 KUKA 任務"""
        # 檢查是否需要 KUKA 協作
        if self.requires_kuka_collaboration(agv_task):
            kuka_task = self.create_kuka_task(agv_task)
            self.sync_task_timing(agv_task, kuka_task)
            return self.execute_coordinated_tasks(agv_task, kuka_task)
        
        return self.execute_agv_only_task(agv_task)
    
    def sync_task_timing(self, agv_task, kuka_task):
        """同步任務時序"""
        agv_arrival_time = self.calculate_agv_arrival(agv_task)
        kuka_ready_time = self.calculate_kuka_ready(kuka_task)
        
        # 調整時序確保協調
        sync_time = max(agv_arrival_time, kuka_ready_time)
        self.schedule_tasks(agv_task, kuka_task, sync_time)
```

## 🏭 業務流程整合

### 眼鏡生產線協作
```python
# 眼鏡生產線 KUKA 協作範例
class EyewearProductionKukaIntegration:
    def __init__(self):
        self.kuka_client = KukaFleetClient()
        self.agv_manager = AGVManager()
    
    def execute_quality_inspection(self, batch_id):
        """品質檢測協作流程"""
        # 1. AGV 運送產品到檢測站
        agv_task = self.agv_manager.create_transport_task(
            from_location="production_line",
            to_location="quality_station",
            item_batch=batch_id
        )
        
        # 2. KUKA 機器人執行精密檢測
        kuka_task = self.kuka_client.create_task({
            "task_type": "quality_inspection",
            "parameters": {
                "batch_id": batch_id,
                "inspection_type": "dimensional_accuracy",
                "quality_standards": "ISO_12345"
            }
        })
        
        # 3. 協調執行時序
        self.coordinate_inspection_workflow(agv_task, kuka_task)
    
    def handle_inspection_result(self, result):
        """處理檢測結果"""
        if result["quality_grade"] == "pass":
            # 合格品：AGV 運送到包裝區
            self.agv_manager.create_transport_task(
                from_location="quality_station",
                to_location="packaging_area",
                item_batch=result["batch_id"]
            )
        else:
            # 不合格品：KUKA 分揀到返工區
            self.kuka_client.create_task({
                "task_type": "sort_defective",
                "parameters": {
                    "batch_id": result["batch_id"],
                    "defect_type": result["defect_analysis"]
                }
            })
```

### 協作工作站設計
```
協作工作站配置
┌─────────────────────────────────────┐
│           協作工作站               │
├─────────────────────────────────────┤
│  AGV 停靠區                        │
│  ├── 精確定位點                   │
│  ├── 充電接口                     │
│  └── 通訊天線                     │
├─────────────────────────────────────┤
│  KUKA 機器人工作區                 │
│  ├── 6軸工業機器人                │
│  ├── 視覺系統                     │
│  ├── 精密夾具                     │
│  └── 品質檢測設備                 │
├─────────────────────────────────────┤
│  安全系統                         │
│  ├── 光柵安全系統                 │
│  ├── 緊急停止按鈕                 │
│  └── 人員檢測雷達                 │
└─────────────────────────────────────┘
```

## 🔧 實作細節

### ROS 2 整合實作
```python
# KUKA Fleet ROS 2 節點
import rclpy
from rclpy.node import Node
from kuka_interfaces.srv import CreateTask, GetTaskStatus
from kuka_interfaces.msg import TaskUpdate

class KukaFleetNode(Node):
    def __init__(self):
        super().__init__('kuka_fleet_node')
        
        # 服務伺服器
        self.create_task_service = self.create_service(
            CreateTask, 'kuka/create_task', self.create_task_callback)
        
        self.get_status_service = self.create_service(
            GetTaskStatus, 'kuka/get_status', self.get_status_callback)
        
        # 狀態發布者
        self.status_publisher = self.create_publisher(
            TaskUpdate, 'kuka/task_updates', 10)
        
        # KUKA Fleet 客戶端
        self.fleet_client = KukaFleetClient()
        
        # 狀態監控定時器
        self.status_timer = self.create_timer(1.0, self.monitor_tasks)
    
    def create_task_callback(self, request, response):
        """創建 KUKA 任務服務回調"""
        try:
            result = self.fleet_client.create_task(request.task_data)
            response.success = True
            response.task_id = result['task_id']
            response.message = "Task created successfully"
        except Exception as e:
            response.success = False
            response.message = f"Failed to create task: {str(e)}"
        
        return response
    
    def monitor_tasks(self):
        """監控任務狀態並發布更新"""
        for task_id in self.active_tasks:
            status = self.fleet_client.get_task_status(task_id)
            
            # 發布狀態更新
            update_msg = TaskUpdate()
            update_msg.task_id = task_id
            update_msg.status = status['status']
            update_msg.progress = status['progress']
            self.status_publisher.publish(update_msg)
```

### 配置管理
```yaml
# kuka_fleet_config.yaml
kuka_fleet:
  connection:
    base_url: "http://kuka-fleet.local:8080"
    api_version: "v2"
    timeout: 30
    retry_count: 3
    
  authentication:
    api_key: "${KUKA_API_KEY}"
    refresh_interval: 3600
    
  robots:
    - id: "kuka_01"
      type: "KR QUANTEC"
      capabilities: ["pick_and_place", "assembly", "inspection"]
      max_payload: 240  # kg
      
    - id: "kuka_02"
      type: "KR AGILUS"
      capabilities: ["precision_assembly", "quality_inspection"]
      max_payload: 10  # kg
      
  task_coordination:
    sync_timeout: 60  # seconds
    max_concurrent_tasks: 5
    priority_levels: ["low", "normal", "high", "critical"]
```

## 📊 監控和診斷

### 整合狀態監控
```python
# KUKA 整合狀態監控
class KukaIntegrationMonitor:
    def __init__(self):
        self.metrics = {
            'api_response_time': [],
            'task_success_rate': 0.0,
            'active_connections': 0,
            'error_count': 0
        }
    
    def check_integration_health(self):
        """檢查整合健康狀態"""
        health_status = {
            'api_connectivity': self.check_api_connectivity(),
            'task_sync_status': self.check_task_sync(),
            'robot_availability': self.check_robot_status(),
            'error_rate': self.calculate_error_rate()
        }
        
        return health_status
    
    def generate_integration_report(self):
        """生成整合狀態報告"""
        report = {
            'timestamp': datetime.now().isoformat(),
            'total_tasks': len(self.completed_tasks),
            'success_rate': self.calculate_success_rate(),
            'average_response_time': np.mean(self.metrics['api_response_time']),
            'robot_utilization': self.calculate_robot_utilization()
        }
        
        return report
```

### 故障排除
```bash
# KUKA 整合故障診斷
# 檢查 API 連接
curl -H "Authorization: Bearer $KUKA_API_KEY" \
     http://kuka-fleet.local:8080/api/v2/health

# 檢查 ROS 2 服務
ros2 service list | grep kuka
ros2 service call /kuka/get_status kuka_interfaces/GetTaskStatus

# 檢查任務狀態
ros2 topic echo /kuka/task_updates

# 檢查網路連接
ping kuka-fleet.local
telnet kuka-fleet.local 8080
```

## 🚀 最佳化建議

### 效能最佳化
1. **連接池管理**: 使用連接池減少連接開銷
2. **批量操作**: 合併多個 API 呼叫
3. **快取策略**: 快取機器人狀態和任務訊息
4. **異步處理**: 使用異步 I/O 提高併發性能

### 可靠性增強
1. **重試機制**: 實作指數退避重試
2. **熔斷器**: 防止級聯故障
3. **健康檢查**: 定期檢查整合狀態
4. **災難恢復**: 實作故障切換機制

### 安全性考量
1. **API 金鑰管理**: 安全存儲和輪替 API 金鑰
2. **HTTPS 通訊**: 使用加密通訊通道
3. **存取控制**: 實作細粒度權限控制
4. **審計日誌**: 記錄所有 API 呼叫和操作

---

**相關文檔：**
- [業務流程](../business-processes/eyewear-production.md) - KUKA 協作場景
- [AI WCS 系統](../technical-details/ai-wcs-integration.md) - 統一決策整合
- [技術架構](../system-architecture/dual-environment.md) - 整體系統架構
- [故障排除](../operations/troubleshooting.md) - KUKA 整合問題排除