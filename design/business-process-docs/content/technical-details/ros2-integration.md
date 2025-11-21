# ROS 2 技術整合

## 🎯 ROS 2 在 RosAGV 中的核心作用

ROS 2 (Robot Operating System 2) 是 RosAGV 系統的技術核心，提供分散式機器人系統所需的通訊、協調和控制能力。

## 🤖 ROS 2 Jazzy + Zenoh RMW 架構

### 為什麼選擇 ROS 2 Jazzy？

#### 技術優勢
- **長期支援版本**：Jazzy Jalapa 是最新的 LTS 版本
- **現代化設計**：基於 DDS 的分散式通訊架構
- **即時效能**：更好的即時性能和延遲控制
- **安全性增強**：內建的安全機制和權限控制
- **工具鏈完善**：豐富的開發和除錯工具

#### 工業級特性
```python
# ROS 2 工業級配置範例
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class IndustrialAGVNode(Node):
    def __init__(self):
        super().__init__('agv_control_node')
        
        # 工業級 QoS 配置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        # 關鍵控制主題
        self.safety_publisher = self.create_publisher(
            SafetyStatus, '/agv/safety_status', qos_profile)
```

### Zenoh RMW 高效能通訊

#### 為什麼選擇 Zenoh RMW？
- **超低延遲**：本地通訊 < 100μs，跨網路 < 10ms
- **高吞吐量**：支援 > 1GB/s 資料傳輸
- **網路透明性**：無縫跨網路節點發現和通訊
- **自動故障恢復**：網路中斷後自動重連
- **QoS 保證**：支援 ROS 2 的所有 QoS 策略

#### Zenoh 配置最佳化
```json5
// /app/routerconfig.json5
{
  mode: "router",
  
  // 高效能監聽配置
  listen: {
    endpoints: ["tcp/0.0.0.0:7447"]
  },
  
  // 低延遲最佳化
  transport: {
    unicast: {
      lowlatency: true,
      qos: {
        enabled: true
      }
    }
  },
  
  // 自動服務發現
  scouting: {
    multicast: {
      enabled: true,
      address: "224.0.0.224:7446"
    }
  }
}
```

## 🏗️ 工作空間架構設計

### 雙環境工作空間分佈

#### AGV 車載工作空間（11個含共用）
```
🚗 專注於即時控制和硬體整合
├── 專用工作空間 (6個)
│   ├── agv_ws/ - 核心 AGV 控制
│   │   ├── agv_base/              # 3層狀態機基礎框架
│   │   ├── cargo_mover_agv/       # Cargo Mover 車型實作
│   │   ├── loader_agv/            # Loader 車型實作
│   │   └── unloader_agv/          # Unloader 車型實作
│   ├── agv_cmd_service_ws/        # 手動指令服務
│   ├── joystick_ws/               # 搖桿控制整合
│   ├── sensorpart_ws/             # 感測器資料處理
│   ├── uno_gpio_ws/               # GPIO 控制服務
│   └── web_api_ws/                # AGVUI 車載監控介面
├── 共用基礎設施 (4個)
│   ├── shared_constants_ws/       # 系統級常數定義
│   ├── keyence_plc_ws/            # Keyence PLC 通訊
│   ├── plc_proxy_ws/              # PLC 代理服務
│   └── path_algorithm/            # 路徑規劃演算法
└── 共用應用 (2個)
    ├── db_proxy_ws/               # 本地資料存取
    └── launch_ws/                 # ROS 2 啟動編排
```

#### AGVC 管理工作空間（13個含共用）
```
🖥️ 專注於車隊管理和系統整合
├── 專用工作空間 (7個)
│   ├── web_api_ws/ - Web API 和 Socket.IO
│   │   ├── web_api/              # 核心 API 服務
│   │   ├── agvcui/               # 管理員界面
│   │   └── opui/                 # 操作員界面
│   ├── db_proxy_ws/              # 資料庫代理服務
│   ├── ecs_ws/                   # 設備控制系統
│   ├── rcs_ws/                   # 機器人控制系統
│   ├── kuka_wcs_ws/              # KUKA WCS 系統（當前使用）
│   ├── wcs_ws/                   # WCS 工作空間（流程控制邏輯）
│   └── kuka_fleet_ws/            # KUKA Fleet 整合
├── 共用基礎設施 (4個) - 與 AGV 環境共用
│   ├── shared_constants_ws/      # 系統級常數定義
│   ├── keyence_plc_ws/           # Keyence PLC 通訊
│   ├── plc_proxy_ws/             # PLC 代理服務
│   └── path_algorithm/           # 路徑規劃演算法
├── 共用應用 (2個)
│   ├── agv_ws/                   # AGV 介面定義（監控用）
│   └── launch_ws/                # ROS 2 啟動編排
└── 已棄用 (2個)
    ├── ~~tafl_ws/~~              # ⚠️ 已棄用 - TAFL 語言核心
    └── ~~tafl_wcs_ws/~~          # ⚠️ 已棄用（已被 kuka_wcs_ws 取代）
```

### 工作空間管理最佳實踐

#### 自動載入系統
```bash
# 自動檢測並載入對應工作空間
all_source                    # 或別名: sa

# 強制載入特定環境
agv_source                   # 載入 AGV 工作空間
agvc_source                  # 載入 AGVC 工作空間

# 檢查載入狀態
echo $ROS_WORKSPACE          # 顯示當前載入的工作空間
printenv | rg ROS            # 檢查 ROS 2 環境變數
```

#### 建置管理策略
```bash
# 自動建置系統
build_all                    # 自動建置腳本

# 增量建置最佳化
colcon build --packages-select-modified
colcon build --packages-up-to target_package

# 並行建置加速
colcon build --parallel-workers 4
```

## 🔄 ROS 2 通訊模式

### 主題 (Topic) 通訊

#### 核心系統主題
```python
# AGV 狀態主題
/agv_status                  # AGV 基本狀態資訊
/agv_position               # AGV 位置和姿態
/agv_battery                # 電池狀態
/agv_safety                 # 安全狀態監控

# 控制指令主題
/move_cmd                   # 移動指令
/robot_cmd                  # 機械臂控制指令
/manual_cmd                 # 手動控制指令

# 感測器資料主題
/sensor_data                # 感測器原始資料
/lidar_scan                 # 雷射掃描資料
/camera_image               # 相機影像資料
```

#### 高效能主題配置
```python
# 關鍵控制主題的 QoS 配置
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# 安全關鍵主題
safety_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5
)

# 高頻率感測器資料
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

# 狀態監控主題
status_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)
```

### 服務 (Service) 通訊

#### 核心系統服務
```python
# PLC 通訊服務
/plc_read                   # 讀取 PLC 資料
/plc_write                  # 寫入 PLC 資料
/plc_status                 # PLC 狀態查詢

# AGV 控制服務
/agv_control                # AGV 控制指令
/path_planning              # 路徑規劃服務
/task_assignment            # 任務分配服務

# 系統管理服務
/system_config              # 系統配置服務
/diagnostics                # 系統診斷服務
/emergency_stop             # 緊急停止服務
```

#### 跨環境服務範例
```python
# AGV 環境 - 服務提供者
class PLCProxyService(Node):
    def __init__(self):
        super().__init__('plc_proxy_service')
        
        # 提供 PLC 讀取服務
        self.read_service = self.create_service(
            PLCRead, 'plc_read', self.plc_read_callback)
        
    def plc_read_callback(self, request, response):
        """處理 PLC 讀取請求"""
        try:
            # 實際 PLC 通訊邏輯
            data = self.plc_client.read_data(
                request.device_type, request.address)
            response.success = True
            response.data = data
        except Exception as e:
            response.success = False
            response.error_message = str(e)
        return response

# AGVC 環境 - 服務消費者
class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')
        
        # 建立 PLC 服務客戶端
        self.plc_client = self.create_client(PLCRead, 'plc_read')
        
    async def check_plc_status(self):
        """跨環境 PLC 狀態檢查"""
        request = PLCRead.Request()
        request.device_type = "DM"
        request.address = "2990"
        
        # 跨環境服務呼叫
        future = self.plc_client.call_async(request)
        response = await future
        
        if response.success:
            return response.data
        else:
            self.get_logger().error(f"PLC read failed: {response.error_message}")
            return None
```

### 動作 (Action) 通訊

#### 長時間任務管理
```python
# AGV 移動動作
/move_to_position           # 移動到指定位置
/rotate_rack               # Rack 旋轉動作
/pick_and_place            # 抓取和放置動作

# 複雜業務流程動作
/cargo_entrance_flow       # Cargo Mover 入口流程
/loader_process_flow       # Loader 製程流程
/unloader_batch_flow       # Unloader 批量流程
```

## 🏛️ 節點架構設計

### 3層狀態機架構

#### Base 層狀態機（生命週期管理）
```python
from lifecycle_msgs.msg import State, Transition
from rclpy_lifecycle import LifecycleNode

class BaseLifecycleNode(LifecycleNode):
    """基礎生命週期節點"""
    
    def on_configure(self, state):
        """配置狀態處理"""
        self.get_logger().info('Configuring node...')
        # 初始化資源和參數
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state):
        """激活狀態處理"""
        self.get_logger().info('Activating node...')
        # 啟動定時器和訂閱者
        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state):
        """停用狀態處理"""
        self.get_logger().info('Deactivating node...')
        # 停止活動和清理
        return TransitionCallbackReturn.SUCCESS
```

#### AGV 層狀態機（業務邏輯）
```python
class AGVControlNode(BaseLifecycleNode):
    """AGV 控制節點"""
    
    def __init__(self):
        super().__init__('agv_control_node')
        self.agv_state = 'IDLE'
        self.state_machine = self.create_state_machine()
    
    def create_state_machine(self):
        """建立 AGV 狀態機"""
        return {
            'IDLE': self.handle_idle_state,
            'MOVING': self.handle_moving_state,
            'LOADING': self.handle_loading_state,
            'UNLOADING': self.handle_unloading_state,
            'ERROR': self.handle_error_state
        }
    
    def handle_idle_state(self, event):
        """處理閒置狀態"""
        if event.type == 'MOVE_COMMAND':
            self.agv_state = 'MOVING'
            self.start_movement(event.data)
        elif event.type == 'LOAD_COMMAND':
            self.agv_state = 'LOADING'
            self.start_loading(event.data)
```

#### Robot 層狀態機（機械臂控制）
```python
class RobotControlNode(Node):
    """機械臂控制節點"""
    
    def __init__(self):
        super().__init__('robot_control_node')
        self.robot_state = 'IDLE'
        self.current_pgno = "50000"  # 閒置程式編號
    
    def execute_pgno(self, pgno, parameters=None):
        """執行 PGNO 程式"""
        try:
            # 1. 寫入參數（如有需要）
            if parameters:
                self.update_robot_parameters(parameters)
            
            # 2. 寫入 PGNO
            self.plc_client.write_data("DM", "1000", pgno)
            
            # 3. 監控執行狀態
            self.monitor_pgno_execution(pgno)
            
        except Exception as e:
            self.get_logger().error(f"PGNO execution failed: {e}")
            self.robot_state = 'ERROR'
```

## 📡 跨環境通訊實作

### Zenoh 網路發現機制

#### 自動節點發現
```python
# 跨環境節點自動發現範例
class CrossEnvironmentNode(Node):
    def __init__(self):
        super().__init__('cross_env_node')
        
        # 等待跨環境服務可用
        self.wait_for_service('/plc_read', timeout_sec=30)
        
        # 建立跨環境訂閱
        self.create_subscription(
            AGVStatus, '/agv_status', self.agv_status_callback, 10)
    
    def agv_status_callback(self, msg):
        """處理來自另一環境的 AGV 狀態"""
        self.get_logger().info(f'Received AGV status from {msg.agv_id}')
        # 跨環境資料處理邏輯
```

#### 網路容錯機制
```python
class RobustCommunicationNode(Node):
    def __init__(self):
        super().__init__('robust_comm_node')
        self.connection_lost = False
        
        # 定期檢查連接狀態
        self.create_timer(5.0, self.check_connection_health)
    
    def check_connection_health(self):
        """檢查跨環境連接健康度"""
        try:
            # 嘗試發送心跳訊息
            heartbeat = Heartbeat()
            heartbeat.timestamp = self.get_clock().now().to_msg()
            self.heartbeat_publisher.publish(heartbeat)
            
        except Exception as e:
            self.handle_connection_loss(e)
    
    def handle_connection_loss(self, error):
        """處理連接中斷"""
        if not self.connection_lost:
            self.connection_lost = True
            self.get_logger().warn(f"Connection lost: {error}")
            # 啟動本地備援機制
            self.activate_local_fallback()
```

## 🔧 開發工具和最佳實踐

### ROS 2 開發工具鏈

#### 除錯和診斷
```bash
# ROS 2 系統狀態檢查
ros2 node list                # 查看所有節點
ros2 topic list               # 查看所有主題
ros2 service list             # 查看所有服務

# 即時資料監控
ros2 topic echo /agv_status   # 監控 AGV 狀態
ros2 topic hz /sensor_data    # 檢查主題頻率
ros2 topic bw /camera_image   # 檢查主題頻寬

# 服務測試
ros2 service call /plc_read plc_interfaces/PLCRead "device_type: 'DM', address: '2990'"
```

#### 效能分析
```bash
# 節點效能分析
ros2 run rqt_top rqt_top      # 節點 CPU 使用監控
ros2 run rqt_graph rqt_graph  # 節點關係圖視覺化

# 通訊效能監控
ros2 run rqt_plot rqt_plot    # 即時資料繪圖
ros2 run rqt_console rqt_console  # 日誌監控
```

### 測試策略

#### 單元測試
```python
import unittest
from rclpy.node import Node
from agv_interfaces.srv import AGVControl

class TestAGVControlNode(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = AGVControlNode()
    
    def test_state_transition(self):
        """測試狀態轉換邏輯"""
        initial_state = self.node.agv_state
        self.assertEqual(initial_state, 'IDLE')
        
        # 模擬移動指令
        event = MoveEvent(target_position=(1.0, 2.0, 0.0))
        self.node.handle_event(event)
        
        self.assertEqual(self.node.agv_state, 'MOVING')
    
    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()
```

#### 整合測試
```python
class TestCrossEnvironmentCommunication(unittest.TestCase):
    def test_plc_service_communication(self):
        """測試跨環境 PLC 服務通訊"""
        rclpy.init()
        
        # 建立測試客戶端
        client_node = Node('test_client')
        plc_client = client_node.create_client(PLCRead, 'plc_read')
        
        # 等待服務可用
        self.assertTrue(plc_client.wait_for_service(timeout_sec=10))
        
        # 發送測試請求
        request = PLCRead.Request()
        request.device_type = 'DM'
        request.address = '2990'
        
        future = plc_client.call_async(request)
        rclpy.spin_until_future_complete(client_node, future)
        
        response = future.result()
        self.assertTrue(response.success)
        
        client_node.destroy_node()
        rclpy.shutdown()
```

## 📊 效能最佳化

### 通訊最佳化
- **QoS 調整**：根據資料特性選擇適當的 QoS 策略
- **批量處理**：高頻率資料的批量傳輸
- **壓縮技術**：大型資料（如影像）的壓縮傳輸
- **快取機制**：頻繁存取資料的本地快取

### 資源管理
- **記憶體最佳化**：節點記憶體使用最佳化
- **CPU 排程**：關鍵節點的 CPU 排程優先級
- **I/O 最佳化**：磁碟和網路 I/O 最佳化
- **垃圾回收**：Python 垃圾回收機制調整

## 🔗 相關技術資源

- [雙環境架構設計](../system-architecture/dual-environment.md) - 整體架構理解
- [Zenoh RMW 通訊機制](zenoh-communication.md) - 深入通訊技術
- [PLC 整合方案](plc-integration.md) - 工業控制整合
- [開發環境設定](../operations/development.md) - 實際開發指導

---

💡 **總結**：ROS 2 + Zenoh RMW 的技術組合為 RosAGV 提供了強大的分散式通訊和控制能力，透過精心設計的工作空間架構和跨環境通訊機制，實現了工業級 AGV 系統的高效能和高可靠性。