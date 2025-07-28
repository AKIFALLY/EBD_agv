# ROS 2 介面設計和協定

## 🎯 適用場景
- ROS 2 自定義訊息、服務、動作設計
- 跨工作空間介面整合
- 通訊協定標準化
- 介面版本管理和相容性

## 📋 RosAGV ROS 2 介面架構

### 介面分類
```
RosAGV ROS 2 介面
├── 📨 訊息 (Messages)
│   ├── AGV 狀態訊息
│   ├── 任務指令訊息
│   ├── 感測器資料訊息
│   └── 系統事件訊息
├── 🔧 服務 (Services)
│   ├── AGV 控制服務
│   ├── 任務管理服務
│   ├── 配置管理服務
│   └── 診斷服務
└── 🎯 動作 (Actions)
    ├── 導航動作
    ├── 裝載動作
    ├── 卸載動作
    └── 充電動作
```

### 核心介面套件
- **agv_interfaces**: AGV 核心介面定義
- **shared_interfaces**: 跨系統共用介面
- **sensor_interfaces**: 感測器相關介面

## 📨 訊息介面設計

### AGV 狀態訊息
```
# AGVStatus.msg
std_msgs/Header header

# AGV 基本資訊
string agv_id
string agv_type          # cargo_mover, loader, unloader
geometry_msgs/Pose2D pose

# 狀態資訊
string current_state     # idle, moving, loading, unloading, charging
string previous_state
float32 battery_level    # 0.0 - 1.0
bool is_emergency_stop
bool is_manual_mode

# 任務資訊
string current_task_id
string target_location
float32 progress         # 0.0 - 1.0

# 硬體狀態
bool[] sensor_status     # 各感測器狀態
bool plc_connected
bool zenoh_connected

# 時間戳
builtin_interfaces/Time last_update
```

### 任務指令訊息
```
# TaskCommand.msg
std_msgs/Header header

# 任務基本資訊
string task_id
string task_type         # move, load, unload, charge
int32 priority          # 1-10, 10 為最高優先級

# 目標資訊
string target_location
geometry_msgs/Pose2D target_pose
string[] waypoints

# 任務參數
string payload_type
float32 payload_weight
bool require_confirmation

# 時間限制
builtin_interfaces/Duration timeout
builtin_interfaces/Time deadline
```

### 感測器資料訊息
```
# SensorData.msg
std_msgs/Header header

# 雷射掃描資料
sensor_msgs/LaserScan laser_scan

# 超音波感測器
float32[] ultrasonic_distances
bool[] ultrasonic_valid

# 編碼器資料
float64 left_wheel_position
float64 right_wheel_position
float64 left_wheel_velocity
float64 right_wheel_velocity

# IMU 資料
sensor_msgs/Imu imu_data

# 相機資料 (可選)
sensor_msgs/Image camera_image
```

## 🔧 服務介面設計

### AGV 控制服務
```
# AGVControl.srv

# 請求
string command           # start, stop, pause, resume, reset
string agv_id
geometry_msgs/Pose2D target_pose
string[] parameters

---

# 回應
bool success
string message
string error_code
builtin_interfaces/Time timestamp
```

### 任務管理服務
```
# TaskManagement.srv

# 請求
string action           # create, update, cancel, query
string task_id
TaskCommand task_data   # 使用上面定義的訊息

---

# 回應
bool success
string message
string task_id
string task_status      # pending, running, completed, failed, cancelled
float32 progress
```

### 配置管理服務
```
# ConfigManagement.srv

# 請求
string operation        # get, set, list, reset
string config_key
string config_value
string config_section

---

# 回應
bool success
string message
string[] config_keys
string[] config_values
```

## 🎯 動作介面設計

### 導航動作
```
# Navigate.action

# 目標
geometry_msgs/PoseStamped target_pose
string[] waypoints
float32 max_velocity
float32 tolerance
bool use_obstacle_avoidance

---

# 結果
bool success
string message
geometry_msgs/PoseStamped final_pose
float32 total_distance
builtin_interfaces/Duration total_time

---

# 回饋
geometry_msgs/PoseStamped current_pose
float32 distance_remaining
float32 estimated_time_remaining
string current_status
```

### 裝載動作
```
# LoadOperation.action

# 目標
string load_type        # pallet, box, custom
geometry_msgs/Pose target_pose
float32 approach_speed
bool use_vision_guidance

---

# 結果
bool success
string message
float32 actual_weight
geometry_msgs/Pose final_pose
bool payload_secured

---

# 回饋
string current_phase    # approaching, positioning, grasping, lifting, securing
float32 progress
geometry_msgs/Pose current_pose
bool[] sensor_readings
```

## 🔄 介面版本管理

### 版本控制策略
```
介面版本規則
├── 主版本 (Major): 不相容的變更
├── 次版本 (Minor): 向後相容的新功能
└── 修訂版本 (Patch): 向後相容的錯誤修復

範例:
agv_interfaces v2.1.3
├── v2: 主要架構變更
├── 1: 新增感測器支援
└── 3: 修復訊息格式錯誤
```

### 相容性處理
```python
# 版本檢查
def check_interface_compatibility(required_version, current_version):
    req_major, req_minor, req_patch = map(int, required_version.split('.'))
    cur_major, cur_minor, cur_patch = map(int, current_version.split('.'))
    
    # 主版本必須相同
    if req_major != cur_major:
        return False
    
    # 次版本向後相容
    if cur_minor < req_minor:
        return False
    
    return True

# 介面適配器
class InterfaceAdapter:
    def adapt_agv_status_v1_to_v2(self, old_msg):
        new_msg = AGVStatusV2()
        new_msg.agv_id = old_msg.id
        new_msg.current_state = old_msg.state
        # 新欄位使用預設值
        new_msg.agv_type = "unknown"
        new_msg.zenoh_connected = True
        return new_msg
```

## 🔧 介面實作最佳實踐

### 訊息設計原則
```
1. 明確性: 欄位名稱清晰明確
2. 完整性: 包含所有必要資訊
3. 擴展性: 預留未來擴展空間
4. 效率性: 避免不必要的資料冗餘
5. 標準化: 遵循 ROS 2 命名規範
```

### 服務設計原則
```python
# 服務回應標準格式
class StandardResponse:
    success: bool           # 操作是否成功
    message: str           # 人類可讀的訊息
    error_code: str        # 機器可讀的錯誤碼
    timestamp: Time        # 操作時間戳
    data: Any             # 具體回應資料

# 錯誤碼標準化
ERROR_CODES = {
    "SUCCESS": "00000",
    "INVALID_PARAMETER": "10001",
    "RESOURCE_NOT_FOUND": "10002",
    "PERMISSION_DENIED": "10003",
    "TIMEOUT": "20001",
    "NETWORK_ERROR": "20002",
    "HARDWARE_ERROR": "30001",
    "SENSOR_ERROR": "30002"
}
```

### 動作設計原則
```python
# 動作回饋頻率控制
class ActionFeedbackManager:
    def __init__(self, feedback_rate=10.0):  # 10 Hz
        self.feedback_rate = feedback_rate
        self.last_feedback_time = time.time()
    
    def should_send_feedback(self):
        current_time = time.time()
        if current_time - self.last_feedback_time >= 1.0 / self.feedback_rate:
            self.last_feedback_time = current_time
            return True
        return False

# 動作取消處理
def handle_action_cancellation(self):
    self.get_logger().info("Action cancellation requested")
    # 安全停止當前操作
    self.stop_current_operation()
    # 回復到安全狀態
    self.return_to_safe_state()
    # 發送取消確認
    return CancelResponse.ACCEPT
```

## 📊 介面測試和驗證

### 單元測試
```python
import unittest
from agv_interfaces.msg import AGVStatus

class TestAGVInterfaces(unittest.TestCase):
    def test_agv_status_message(self):
        msg = AGVStatus()
        msg.agv_id = "AGV001"
        msg.current_state = "idle"
        msg.battery_level = 0.85
        
        # 驗證訊息完整性
        self.assertEqual(msg.agv_id, "AGV001")
        self.assertEqual(msg.current_state, "idle")
        self.assertAlmostEqual(msg.battery_level, 0.85)
        
        # 驗證範圍限制
        self.assertGreaterEqual(msg.battery_level, 0.0)
        self.assertLessEqual(msg.battery_level, 1.0)

    def test_task_command_validation(self):
        cmd = TaskCommand()
        cmd.task_type = "move"
        cmd.priority = 5
        
        # 驗證優先級範圍
        self.assertGreaterEqual(cmd.priority, 1)
        self.assertLessEqual(cmd.priority, 10)
```

### 整合測試
```python
# 跨節點通訊測試
class IntegrationTest:
    def test_agv_control_service(self):
        # 建立服務客戶端
        client = self.create_client(AGVControl, '/agv_control')
        
        # 準備請求
        request = AGVControl.Request()
        request.command = "start"
        request.agv_id = "AGV001"
        
        # 發送請求並驗證回應
        future = client.call_async(request)
        response = self.wait_for_response(future)
        
        self.assertTrue(response.success)
        self.assertIsNotNone(response.message)

    def test_navigation_action(self):
        # 建立動作客戶端
        client = ActionClient(self, Navigate, '/navigate')
        
        # 準備目標
        goal = Navigate.Goal()
        goal.target_pose.pose.position.x = 5.0
        goal.target_pose.pose.position.y = 3.0
        
        # 發送目標並監控進度
        future = client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        result = self.wait_for_result(future)
        
        self.assertTrue(result.success)
```

## 🔗 交叉引用
- ROS 2 開發: @docs-ai/operations/development/ros2-development.md
- AGV 工作空間: @docs-ai/context/workspaces/agv-workspaces.md
- Zenoh 通訊: @docs-ai/knowledge/protocols/zenoh-rmw.md
- 技術棧: @docs-ai/context/system/technology-stack.md
