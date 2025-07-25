# RCS (Robot Control System) 專案完整文檔

[![RCS Version](https://img.shields.io/badge/RCS-v2.0-blue.svg)]()
[![ROS 2 Version](https://img.shields.io/badge/ROS%202-Jazzy-green.svg)]()
[![Python Version](https://img.shields.io/badge/Python-3.12-yellow.svg)]()

## 專案概述

RCS (Robot Control System) 是 RosAGV 系統的核心車隊控制系統，負責智能任務分派、AGV 車隊管理、任務狀態協調和 KUKA Fleet 整合。支援多種車型（Cargo、Loader、Unloader、KUKA400i）的混合車隊管理，提供基於房間和車型的智能分派機制。

### 系統特色
- 🚗 **多車型支援**: CT AGV (Cargo/Loader/Unloader) + KUKA400i 混合車隊
- 🧠 **智能分派**: 基於房間、車型和任務類型的智能分派邏輯
- 🔄 **狀態同步**: 與資料庫和 KUKA Fleet API 的即時狀態同步
- 📊 **配置管理**: 統一的 AGV 配置管理系統
- 🧪 **完整測試**: 單元測試、整合測試和離線測試環境
- 📚 **豐富文檔**: 完整的 API 文檔和故障排除指南

## 系統架構

```
RCS Core (rcs_core.py)
├── CT Manager                    # CT 車隊管理 (ct_manager.py)
│   ├── 房內任務: Loader02, Unloader02
│   └── 房外任務: Cargo02
├── KUKA Manager                  # KUKA 車隊管理 (kuka_manager.py)
│   ├── KUKA Dispatcher V2        # 增強版派發器 (kuka_dispatcher_v2.py)
│   ├── KUKA Robot Control        # 機器人控制 (kuka_robot.py)
│   └── KUKA Container Mgmt       # 容器管理 (kuka_container.py)
├── Configuration Manager         # 統一配置管理 (kuka_config_manager.py)
├── Testing Framework            # 測試框架 (test/)
└── Documentation               # 完整文檔 (docs/)
```

## 專案結構

```
rcs_ws/src/rcs/
├── rcs/                          # 核心系統模組
│   ├── rcs_core.py              # RCS 核心節點 - 系統協調中心
│   ├── ct_manager.py            # CT 車隊管理器 - 智能任務分派
│   ├── kuka_manager.py          # KUKA 車隊管理器 (1518行完整實現)
│   ├── kuka_dispatcher_v2.py    # 增強版 KUKA 派發器 (507行)
│   ├── kuka_robot.py            # KUKA 機器人控制
│   ├── kuka_container.py        # KUKA 容器管理
│   ├── kuka_config_manager.py   # 統一配置管理系統
│   ├── kuka_config_cli.py       # 配置管理 CLI 工具
│   └── task_status_simulator.py # 任務狀態模擬器
├── test/                        # 完整測試套件
│   ├── conftest.py             # pytest 配置和 fixtures
│   ├── test_kuka_manager.py    # KukaManager 單元測試
│   ├── test_kuka_integration.py # KUKA Fleet API 整合測試
│   ├── offline_test_server.py  # 離線測試伺服器
│   └── test_environment_manager.py # 測試環境管理
├── docs/                       # 完整文檔系統
│   ├── README.md              # 本文件 - 完整文檔
│   ├── api/                   # API 文檔
│   ├── troubleshooting/       # 故障排除指南
│   └── development/           # 開發指南
├── test_config_manager.py      # 配置管理器測試腳本
├── package.xml                 # ROS 2 套件配置
└── setup.py                   # Python 套件設定
```

---

# 📋 目錄

1. [系統概述](#系統概述)
2. [ROS 2 API 文檔](#ros-2-api-文檔)
3. [配置管理系統](#配置管理系統)
4. [測試框架](#測試框架)
5. [故障排除指南](#故障排除指南)
6. [開發指南](#開發指南)
7. [部署指南](#部署指南)
8. [監控與維護](#監控與維護)

---

# 📖 系統概述

## 核心組件

### RcsCore - 系統協調中心
**文件**: `rcs/rcs_core.py`

系統的核心協調節點，負責初始化和協調所有子系統：

```python
class RcsCore(Node):
    def __init__(self):
        # 初始化資料庫連線池
        self.db_pool = ConnectionPoolManager(
            'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'
        )
        
        # 初始化車隊管理器
        self.kuka_manager = KukaManager(self)  # KUKA 車隊
        self.ct_manager = CtManager(self)      # CT 車隊
        
        # 任務狀態模擬器
        self.task_status_simulator = TaskStatusSimulator(
            self.db_pool, self.get_logger()
        )
        
        # 1秒定時器 - 主迴圈
        self.timer_1s = self.create_timer(1.0, self.main_loop)
```

**主要職責**:
- 系統初始化和資源管理
- 車隊管理器協調
- 定時任務調度
- 資料庫連線池管理

### CtManager - CT 車隊智能分派
**文件**: `rcs/ct_manager.py`

CT 車隊（Cargo、Loader、Unloader）的智能任務分派核心：

```python
class CtManager:
    def dispatch(self):
        """CT 車隊智能任務分派"""
        # 1. 查詢待執行的 CT 任務
        ct_tasks = session.exec(
            select(Task).where(
                Task.status_id == 1,  # 待執行
                or_(
                    Task.parameters["model"].as_string() == "Cargo",
                    Task.parameters["model"].as_string() == "Loader",
                    Task.parameters["model"].as_string() == "Unloader"
                )
            ).limit(20)
        )
        
        # 2. 智能分派邏輯
        for task in ct_tasks:
            target_agv = self._select_agv_for_task(session, task)
            if target_agv:
                self._assign_task_to_agv(session, task, target_agv)
```

**分派規則:**
- 房內任務: `Loader{房間:02d}`, `Unloader{房間:02d}`
- 房外任務: `Cargo02`
- 狀態驗證: AGV 必須為 IDLE 狀態

### KukaManager - KUKA 車隊管理
**文件**: `rcs/kuka_manager.py` (1518行完整實現)

KUKA400i 機器人車隊的完整管理系統：

```python
class KukaManager:
    def dispatch(self):
        """KUKA400i AGV 智能任務派發 - 支援 WCS 四級優先度"""
        idle_kuka400i_agvs = self.kuka_fleet.select_agv(
            KukaFleetAdapter.STATUS_IDLE
        )
        
        if not idle_kuka400i_agvs:
            return
        
        # WCS 四級優先度任務查詢
        priority_tasks = self._query_priority_tasks()
        
        for task in priority_tasks:
            success = self._dispatch_task_to_kuka(task, idle_kuka400i_agvs[0])
            if success:
                idle_kuka400i_agvs.pop(0)
                if not idle_kuka400i_agvs:
                    break
```

**主要功能:**
- KUKA Fleet API 整合
- 四級優先度任務處理
- 容器和料架狀態同步
- 任務執行監控

### KukaDispatcherV2 - 增強版派發器
**文件**: `rcs/kuka_dispatcher_v2.py` (507行實現)

對原始 KukaDispatcher 的全面升級版本：

```python
class KukaDispatcherV2:
    def _execute_api_with_retry(self, api_method: str, args: list) -> Dict[str, Any]:
        """API 重試機制"""
        last_error = None
        for attempt in range(self.api_retry_config['max_attempts']):
            try:
                result = getattr(self.kuka_fleet, api_method)(*args)
                return {'success': True, 'data': result}
            except Exception as e:
                last_error = e
                if attempt < self.api_retry_config['max_attempts'] - 1:
                    wait_time = self.api_retry_config['base_delay'] * (2 ** attempt)
                    time.sleep(wait_time)
        
        return {'success': False, 'error': str(last_error)}
```

**增強功能:**
- API 重試機制與指數退避
- 完整錯誤處理
- 性能監控與統計
- 配置管理整合

### 統一配置管理系統
**文件**: `rcs/kuka_config_manager.py`

統一管理 KUKA 和 CT AGV 的配置系統：

```python
@dataclass
class UnifiedFleetConfig:
    """統一車隊配置"""
    kuka_api: KukaAPIConfig = field(default_factory=KukaAPIConfig)
    kuka_fleet: KukaFleetConfig = field(default_factory=KukaFleetConfig)
    ct_fleet: CTFleetConfig = field(default_factory=CTFleetConfig)
    agvs: Dict[str, AGVConfig] = field(default_factory=dict)
    system: Dict[str, Any] = field(default_factory=lambda: {
        'update_interval': 5.0,
        'heartbeat_timeout': 30.0,
        'task_dispatch_interval': 1.0,
        'log_level': 'INFO'
    })
```

**管理功能:**
- AGV 配置的 CRUD 操作
- 資料庫同步
- 配置檔案匯入/匯出
- 配置驗證與錯誤檢查

---

# 🔌 ROS 2 API 文檔

## 系統節點

### rcs_core 節點
**啟動命令**: `ros2 run rcs rcs_core`

#### 發布的話題 (Published Topics)

| 話題名稱 | 訊息類型 | 頻率 | 描述 |
|---------|---------|------|------|
| `/rcs/fleet_status` | `rcs_interfaces/msg/FleetStatus` | 1 Hz | 整體車隊狀態資訊 |
| `/rcs/task_assignments` | `rcs_interfaces/msg/TaskAssignment` | 事件觸發 | 任務分配結果 |
| `/rcs/system_metrics` | `rcs_interfaces/msg/SystemMetrics` | 5 Hz | 系統性能指標 |
| `/rcs/dispatch_logs` | `std_msgs/msg/String` | 事件觸發 | 分派決策日誌 |

#### 訂閱的話題 (Subscribed Topics)

| 話題名稱 | 訊息類型 | 描述 |
|---------|---------|------|
| `/agv/status_update` | `agv_interfaces/msg/AgvStatus` | AGV 狀態更新 |
| `/wcs/task_request` | `wcs_interfaces/msg/TaskRequest` | WCS 任務請求 |
| `/database/change_notification` | `db_interfaces/msg/ChangeNotification` | 資料庫變更通知 |

#### 提供的服務 (Provided Services)

| 服務名稱 | 服務類型 | 描述 |
|---------|---------|------|
| `/rcs/assign_task` | `rcs_interfaces/srv/AssignTask` | 手動任務分配 |
| `/rcs/get_fleet_status` | `rcs_interfaces/srv/GetFleetStatus` | 獲取車隊狀態 |
| `/rcs/cancel_task` | `rcs_interfaces/srv/CancelTask` | 取消任務執行 |
| `/rcs/get_agv_by_type` | `rcs_interfaces/srv/GetAgvByType` | 根據車型查詢 AGV |
| `/rcs/update_agv_config` | `rcs_interfaces/srv/UpdateAgvConfig` | 更新 AGV 配置 |
| `/rcs/validate_config` | `rcs_interfaces/srv/ValidateConfig` | 驗證系統配置 |

#### 調用的服務 (Called Services)

| 服務名稱 | 服務類型 | 描述 |
|---------|---------|------|
| `/db_proxy/get_agv_status` | `db_interfaces/srv/GetAgvStatus` | 查詢 AGV 狀態 |
| `/db_proxy/update_task_status` | `db_interfaces/srv/UpdateTaskStatus` | 更新任務狀態 |
| `/kuka_fleet/submit_mission` | `kuka_interfaces/srv/SubmitMission` | 提交 KUKA 任務 |

## 服務詳細說明

### AssignTask 服務
**路徑**: `/rcs/assign_task`
**類型**: `rcs_interfaces/srv/AssignTask`

**請求格式**:
```yaml
# AssignTaskRequest
task_id: int32
agv_name: string
force_assign: bool  # 是否強制分配（忽略 AGV 狀態）
priority: int32     # 任務優先級 (1-4)
```

**回應格式**:
```yaml
# AssignTaskResponse
success: bool
message: string
assigned_agv: string
estimated_completion_time: builtin_interfaces/msg/Time
```

**使用範例**:
```bash
# 分配任務給特定 AGV
ros2 service call /rcs/assign_task rcs_interfaces/srv/AssignTask \
  "{
    task_id: 12345,
    agv_name: 'KUKA001',
    force_assign: false,
    priority: 2
  }"
```

### GetFleetStatus 服務
**路徑**: `/rcs/get_fleet_status`
**類型**: `rcs_interfaces/srv/GetFleetStatus`

**請求格式**:
```yaml
# GetFleetStatusRequest
fleet_type: string  # "all", "kuka", "ct" 或空字串
include_metrics: bool
```

**回應格式**:
```yaml
# GetFleetStatusResponse
total_agvs: int32
idle_agvs: int32
busy_agvs: int32
error_agvs: int32
offline_agvs: int32
agv_details: rcs_interfaces/msg/AgvStatus[]
system_metrics: rcs_interfaces/msg/SystemMetrics
```

**使用範例**:
```bash
# 獲取完整車隊狀態
ros2 service call /rcs/get_fleet_status rcs_interfaces/srv/GetFleetStatus \
  "{
    fleet_type: 'all',
    include_metrics: true
  }"
```

### UpdateAgvConfig 服務
**路徑**: `/rcs/update_agv_config`
**類型**: `rcs_interfaces/srv/UpdateAgvConfig`

**請求格式**:
```yaml
# UpdateAgvConfigRequest
agv_name: string
configurations: rcs_interfaces/msg/AgvConfigUpdate[]
validate_only: bool  # 僅驗證不實際更新
```

**回應格式**:
```yaml
# UpdateAgvConfigResponse
success: bool
validation_errors: string[]
updated_fields: string[]
config_backup_path: string
```

## 訊息類型定義

### FleetStatus 訊息
**路徑**: `rcs_interfaces/msg/FleetStatus`

```yaml
# FleetStatus.msg
builtin_interfaces/msg/Time timestamp
string fleet_id
int32 total_agvs
int32 active_agvs
int32 idle_agvs
int32 error_agvs
float32 average_battery_level
float32 task_completion_rate
rcs_interfaces/msg/AgvStatus[] agv_list
```

### TaskAssignment 訊息
**路徑**: `rcs_interfaces/msg/TaskAssignment`

```yaml
# TaskAssignment.msg
builtin_interfaces/msg/Time assignment_time
int32 task_id
string agv_name
string task_type
int32 priority
string source_location
string destination_location
float32 estimated_duration
string assignment_reason
```

### SystemMetrics 訊息
**路徑**: `rcs_interfaces/msg/SystemMetrics`

```yaml
# SystemMetrics.msg
builtin_interfaces/msg/Time timestamp
float32 cpu_usage_percent
float32 memory_usage_percent
int32 active_database_connections
float32 average_dispatch_latency_ms
int32 tasks_dispatched_per_minute
int32 api_call_failures_per_minute
string[] system_warnings
```

---

# ⚙️ 配置管理系統

## 統一配置管理器 (KukaConfigManager)

### CLI 工具使用
**腳本**: `rcs/kuka_config_cli.py`

#### 基本命令

```bash
# 列出所有 AGV
python3 kuka_config_cli.py list

# 列出特定類型 AGV
python3 kuka_config_cli.py list --type kuka
python3 kuka_config_cli.py list --type ct

# 顯示特定 AGV 詳細資訊
python3 kuka_config_cli.py show KUKA001

# 驗證配置
python3 kuka_config_cli.py validate

# 顯示系統配置摘要
python3 kuka_config_cli.py config summary
```

#### AGV 管理命令

```bash
# 新增 KUKA AGV
python3 kuka_config_cli.py add KUKA004 8506999 KUKA400i \
  --description "新增的 KUKA AGV" \
  --x 1000.0 --y 2000.0 \
  --robot-id "8506999" \
  --robot-type "KMP 400i diffDrive"

# 新增 CT AGV
python3 kuka_config_cli.py add Loader03 4 Loader \
  --description "房間3 Loader" \
  --room 3 \
  --capabilities "material_handling,loader_operations"

# 更新 AGV 配置
python3 kuka_config_cli.py update KUKA001 \
  --description "更新後的描述" \
  --x 1500.0 \
  --enable true

# 移除 AGV
python3 kuka_config_cli.py remove TEST_AGV --force
```

#### 配置管理命令

```bash
# 與資料庫同步
python3 kuka_config_cli.py sync

# 匯出配置
python3 kuka_config_cli.py export --format yaml --output backup.yaml
python3 kuka_config_cli.py export --format json --output backup.json

# 顯示 KUKA API 配置
python3 kuka_config_cli.py config kuka_api

# 顯示系統配置
python3 kuka_config_cli.py config system
```

### 程式化配置管理

#### 基本使用

```python
from rcs.kuka_config_manager import KukaConfigManager, AGVConfig, AGVModel

# 初始化配置管理器
config_manager = KukaConfigManager()

# 獲取配置摘要
summary = config_manager.get_config_summary()
print(f"總 AGV 數: {summary['total_agvs']}")
print(f"KUKA AGV 數: {summary['kuka_agvs']}")
print(f"CT AGV 數: {summary['ct_agvs']}")

# 查詢特定 AGV
agv_config = config_manager.get_agv_config("KUKA001")
if agv_config:
    print(f"AGV: {agv_config.name}, 位置: ({agv_config.initial_x}, {agv_config.initial_y})")
```

#### AGV 配置操作

```python
# 新增 AGV 配置
new_agv = AGVConfig(
    id=9999,
    name="TEST_AGV",
    model=AGVModel.KUKA400I,
    description="測試用 KUKA AGV",
    initial_x=1000.0,
    initial_y=2000.0,
    kuka_robot_id="9999",
    kuka_robot_type="KMP 400i diffDrive"
)

if config_manager.add_agv_config(new_agv):
    print("AGV 配置新增成功")

# 更新 AGV 配置
updates = {
    'description': '測試用 KUKA AGV (已更新)',
    'initial_x': 1500.0,
    'battery_threshold_low': 25.0
}

if config_manager.update_agv_config("TEST_AGV", updates):
    print("AGV 配置更新成功")

# 移除 AGV 配置
if config_manager.remove_agv_config("TEST_AGV"):
    print("AGV 配置移除成功")
```

#### 查詢操作

```python
# 查詢不同類型的 AGV
kuka_agvs = config_manager.get_kuka_agvs()
ct_agvs = config_manager.get_ct_agvs()

print("KUKA AGV 列表:")
for agv in kuka_agvs:
    print(f"  - {agv.name} (Robot ID: {agv.kuka_robot_id})")

print("CT AGV 列表:")
for agv in ct_agvs:
    room_text = f"房間{agv.ct_room_assignment}" if agv.ct_room_assignment else "房外"
    print(f"  - {agv.name} ({agv.model.value}, {room_text})")

# 根據房間查詢 AGV
room_agvs = config_manager.get_agvs_by_room(2)
print(f"房間2的 AGV: {[agv.name for agv in room_agvs]}")
```

#### 配置驗證與匯出

```python
# 配置驗證
errors = config_manager.validate_config()
total_errors = sum(len(error_list) for error_list in errors.values())

if total_errors == 0:
    print("✅ 配置驗證通過")
else:
    print(f"❌ 發現 {total_errors} 個錯誤")
    for category, error_list in errors.items():
        if error_list:
            print(f"  {category}: {error_list}")

# 配置匯出
if config_manager.export_config("/tmp/config_backup.yaml", "yaml"):
    print("配置匯出成功")

# 與資料庫同步
if config_manager.sync_with_database():
    print("與資料庫同步成功")
```

### 配置檔案結構

#### 統一配置檔案格式 (YAML)
**路徑**: `/app/config/unified_fleet_config.yaml`

```yaml
# KUKA API 配置
kuka_api:
  base_url: "http://192.168.10.3:10870"
  username: "admin"
  password: "Admin"
  timeout: 30.0
  max_retries: 3
  retry_delay: 1.0
  endpoints:
    login: "/api/login"
    robot_query: "/api/amr/robotQuery"
    container_query: "/api/amr/containerQuery"
    submit_mission: "/api/amr/submitMission"
    mission_cancel: "/api/amr/missionCancel"

# KUKA 車隊配置
kuka_fleet:
  map_layout_district: "test-test1"
  mission_types:
    MOVE: "MOVE"
    RACK_MOVE: "RACK_MOVE"
  coordinate_transform:
    translation: [0.0, 0.0, 0.0]
    rotation: [0.0, 0.0, 0.0]
    scale: [1.0, 1.0, 1.0]
  status_mapping:
    IDLE: 3
    MOVING: 4
    LOADING: 4
    CHARGING: 5
    ERROR: 7

# CT 車隊配置
ct_fleet:
  supported_models: ["Cargo", "Loader", "Unloader"]
  room_dispatch_rules:
    cargo_rules:
      default_agv: "Cargo02"
      room_assignment: null
    loader_rules:
      pattern: "Loader{room:02d}"
      room_based: true
    unloader_rules:
      pattern: "Unloader{room:02d}"
      room_based: true
  task_priorities:
    emergency: 90
    high: 70
    normal: 50
    low: 30

# AGV 配置列表
agvs:
  Cargo02:
    id: 1
    name: "Cargo02"
    model: "Cargo"
    description: "走廊AGV(暫時規劃僅負責房間2)"
    initial_x: 0.0
    initial_y: 0.0
    initial_heading: 0.0
    default_status: 3  # IDLE
    enable: true
    ct_room_assignment: null
    ct_capabilities: ["transport", "corridor_navigation"]
    battery_threshold_low: 20.0
    battery_threshold_critical: 10.0
    max_payload: 50.0
    max_speed: 1.5
    
  KUKA001:
    id: 8506941
    name: "KUKA001"
    model: "KUKA400i"
    description: "在房間外負責料架搬運"
    initial_x: 3116.0
    initial_y: 1852.0
    initial_heading: 0.0
    default_status: 3  # IDLE
    enable: true
    kuka_robot_id: "8506941"
    kuka_robot_type: "KMP 400i diffDrive"
    battery_threshold_low: 20.0
    battery_threshold_critical: 10.0
    max_payload: 50.0
    max_speed: 1.5

# 系統配置
system:
  update_interval: 5.0
  heartbeat_timeout: 30.0
  task_dispatch_interval: 1.0
  log_level: "INFO"
  enable_monitoring: true
  enable_auto_recovery: true
```

---

# 🧪 測試框架

## 測試架構概述

### 測試分層
1. **單元測試**: 測試個別類別和方法
2. **整合測試**: 測試組件間的互動
3. **離線測試**: 不依賴外部服務的模擬測試
4. **端到端測試**: 完整系統流程測試

### 測試環境設定
**文件**: `test/conftest.py`

```python
@pytest.fixture
def mock_rcs_core():
    """模擬 RCS 核心節點"""
    mock_core = MagicMock()
    mock_core.get_logger.return_value = MagicMock()
    mock_core.db_pool = MagicMock()
    return mock_core

@pytest.fixture
def sample_kuka_agvs():
    """範例 KUKA AGV 資料"""
    return [
        {
            "id": "8506941",
            "name": "KUKA001",
            "status": 3,  # IDLE
            "position": {"x": 3116.0, "y": 1852.0}
        },
        {
            "id": "8506995", 
            "name": "KUKA002",
            "status": 4,  # RUNNING
            "position": {"x": 2860.0, "y": 1680.0}
        }
    ]
```

## 單元測試

### KukaManager 測試
**文件**: `test/test_kuka_manager.py`

```bash
# 執行 KukaManager 單元測試
python3 -m pytest test/test_kuka_manager.py -v

# 執行特定測試方法
python3 -m pytest test/test_kuka_manager.py::TestKukaManager::test_dispatch_with_idle_agvs -v

# 生成覆蓋率報告
python3 -m pytest test/test_kuka_manager.py --cov=rcs.kuka_manager --cov-report=html
```

**測試範例**:
```python
def test_dispatch_with_idle_agvs(self, mock_rcs_core, sample_kuka_agvs):
    """測試有空閒 AGV 時的派發邏輯"""
    manager = KukaManager(mock_rcs_core)
    
    # 模擬 KUKA Fleet API 回應
    manager.kuka_fleet.select_agv.return_value = sample_kuka_agvs[:1]  # 1台空閒
    
    # 執行派發
    result = manager.dispatch()
    
    # 驗證結果
    assert result is True
    manager.kuka_fleet.select_agv.assert_called_with(3)  # STATUS_IDLE
```

### KukaDispatcherV2 測試
**文件**: `test/test_kuka_dispatcher_v2.py`

```python
def test_api_retry_mechanism(self, mock_kuka_fleet):
    """測試 API 重試機制"""
    dispatcher = KukaDispatcherV2()
    
    # 模擬 API 失敗後成功
    mock_kuka_fleet.robot_query.side_effect = [
        Exception("Network error"),
        Exception("Timeout"),
        {"robots": []}  # 第三次成功
    ]
    
    result = dispatcher._execute_api_with_retry("robot_query", [])
    
    assert result['success'] is True
    assert mock_kuka_fleet.robot_query.call_count == 3
```

## 整合測試

### KUKA Fleet API 整合測試
**文件**: `test/test_kuka_integration.py`

```bash
# 執行整合測試
python3 -m pytest test/test_kuka_integration.py -v

# 執行需要真實 API 的測試（需要 KUKA Fleet 可用）
python3 -m pytest test/test_kuka_integration.py -m "integration" -v
```

**測試內容**:
- KUKA Fleet API 連線測試
- 任務提交和取消測試
- 機器人狀態查詢測試
- 容器管理測試

## 離線測試環境

### 離線測試伺服器
**文件**: `test/offline_test_server.py`

```bash
# 啟動離線測試伺服器
python3 test/offline_test_server.py

# 伺服器將在 http://localhost:10870 啟動
# 模擬 KUKA Fleet API 的所有端點
```

**功能特色**:
- 完整的 KUKA Fleet API 模擬
- WebSocket 即時狀態更新
- 可配置的回應延遲和錯誤
- 測試資料管理

**測試伺服器 API**:
```bash
# 測試登入 API
curl -X POST http://localhost:10870/api/login \
  -H "Content-Type: application/json" \
  -d '{"username": "admin", "password": "Admin"}'

# 查詢機器人狀態
curl -X POST http://localhost:10870/api/amr/robotQuery \
  -H "Content-Type: application/json" \
  -d '{"token": "test_token"}'

# 提交任務
curl -X POST http://localhost:10870/api/amr/submitMission \
  -H "Content-Type: application/json" \
  -d '{"token": "test_token", "mission": {...}}'
```

### 測試環境管理器
**文件**: `test/test_environment_manager.py`

```python
class TestEnvironmentManager:
    """測試環境管理器"""
    
    def setup_kuka_mock_environment(self):
        """設置 KUKA 模擬環境"""
        # 啟動模擬伺服器
        # 初始化測試資料
        # 設置環境變數
        
    def create_test_scenario(self, scenario_name: str):
        """創建測試場景"""
        scenarios = {
            "normal_operation": self._normal_operation_scenario,
            "api_failures": self._api_failures_scenario,
            "high_load": self._high_load_scenario,
            "network_issues": self._network_issues_scenario
        }
        return scenarios.get(scenario_name, self._default_scenario)()
```

## 配置管理器測試

### 獨立測試腳本
**文件**: `test_config_manager.py`

```bash
# 執行配置管理器完整測試
python3 test_config_manager.py

# 測試內容包括:
# - 配置管理器基本功能
# - AGV 管理功能（新增、更新、移除）
# - 配置持久化（儲存、匯出）
# - KUKA 特定功能
# - CT AGV 特定功能
# - API 配置更新
```

**測試輸出範例**:
```
KUKA 配置管理系統測試
==================================================
=== 測試配置管理器基本功能 ===
使用臨時目錄: /tmp/tmpxxx

1. 配置摘要測試
  總 AGV 數: 6
  KUKA AGV 數: 3
  CT AGV 數: 3

2. AGV 查詢測試
  KUKA AGV: ['KUKA001', 'KUKA002', 'KUKA003']
  CT AGV: ['Cargo02', 'Loader02', 'Unloader02']

✅ 所有測試完成
```

## 測試執行指南

### 完整測試套件執行

```bash
# 進入 AGVC 容器
docker compose -f docker-compose.agvc.yml exec agvc_server bash
source /app/setup.bash && agvc_source
cd /app/rcs_ws/src/rcs

# 執行所有測試
python3 -m pytest test/ -v

# 執行特定測試類別
python3 -m pytest test/test_kuka_manager.py -v
python3 -m pytest test/test_kuka_integration.py -v

# 生成詳細的覆蓋率報告
python3 -m pytest test/ --cov=rcs --cov-report=html --cov-report=term

# 執行長時間運行測試
python3 -m pytest test/ -m "slow" -v

# 跳過需要外部服務的測試
python3 -m pytest test/ -m "not integration" -v
```

### 持續整合測試

```bash
# CI/CD 環境測試腳本
#!/bin/bash
set -e

# 啟動離線測試環境
python3 test/offline_test_server.py &
SERVER_PID=$!
sleep 5

# 執行測試套件
python3 -m pytest test/ --cov=rcs --cov-report=xml --junitxml=test-results.xml

# 清理
kill $SERVER_PID
```

---

# 🔧 故障排除指南

## 常見問題診斷與解決

### 1. CT 任務分派問題

#### 問題現象
- CT 任務長時間處於待執行狀態
- AGV 狀態顯示 IDLE 但未接收到任務
- 分派日誌顯示找不到合適的 AGV

#### 診斷步驟

```bash
# 1. 檢查 RCS 核心節點狀態
ros2 node list | grep rcs
ros2 node info /rcs_core

# 2. 檢查 CT 管理器狀態
ros2 topic echo /rcs/fleet_status --once

# 3. 查看分派日誌
ros2 topic echo /rcs/dispatch_logs

# 4. 檢查資料庫中的任務狀態
# 進入 AGVC 容器
docker compose -f docker-compose.agvc.yml exec agvc_server bash
source /app/setup.bash && agvc_source
python3 -c "
from db_proxy.db_proxy.connection_pool_manager import ConnectionPoolManager
from sqlmodel import select
from db_proxy.db_proxy.models import Task

with ConnectionPoolManager().get_session() as session:
    tasks = session.exec(select(Task).where(Task.status_id == 1).limit(10)).all()
    for task in tasks:
        print(f'Task {task.id}: {task.parameters}')
"

# 5. 檢查 AGV 狀態
python3 -c "
from db_proxy.db_proxy.connection_pool_manager import ConnectionPoolManager
from sqlmodel import select
from db_proxy.db_proxy.models import AGV

with ConnectionPoolManager().get_session() as session:
    agvs = session.exec(select(AGV).where(AGV.model.in_(['Cargo', 'Loader', 'Unloader']))).all()
    for agv in agvs:
        print(f'{agv.name}: status={agv.status_id}, enable={agv.enable}')
"
```

#### 解決方案

1. **AGV 狀態異常**:
```bash
# 重置 AGV 狀態為 IDLE
python3 -c "
from db_proxy.db_proxy.connection_pool_manager import ConnectionPoolManager
from sqlmodel import select
from db_proxy.db_proxy.models import AGV

with ConnectionPoolManager().get_session() as session:
    agv = session.exec(select(AGV).where(AGV.name == 'Cargo02')).first()
    if agv:
        agv.status_id = 3  # IDLE
        session.add(agv)
        session.commit()
        print(f'{agv.name} 狀態已重置為 IDLE')
"
```

2. **任務參數格式錯誤**:
```python
# 檢查任務參數格式
# 正確格式應包含: model, room_id (可選)
expected_format = {
    "model": "Cargo",  # 或 "Loader", "Unloader"
    "room_id": 2       # 房內任務必需，房外任務可選
}
```

3. **重啟 RCS 服務**:
```bash
# 重啟 RCS 核心節點
ros2 lifecycle set /rcs_core shutdown
sleep 2
ros2 run rcs rcs_core
```

### 2. KUKA 車隊連線問題

#### 問題現象
- KUKA 任務無法提交
- API 調用超時或失敗
- KUKA AGV 狀態無法更新

#### 診斷步驟

```bash
# 1. 檢查 KUKA Fleet API 可達性
curl -X GET http://192.168.10.3:10870/api/status

# 2. 測試 API 登入
curl -X POST http://192.168.10.3:10870/api/login \
  -H "Content-Type: application/json" \
  -d '{"username": "admin", "password": "Admin"}'

# 3. 檢查網路連通性
ping 192.168.10.3
telnet 192.168.10.3 10870

# 4. 檢查 KUKA Manager 日誌
ros2 topic echo /rcs/system_logs | grep -i kuka

# 5. 檢查配置檔案
python3 -c "
from rcs.kuka_config_manager import get_config_manager
config = get_config_manager()
print('KUKA API Config:')
print(f'  Base URL: {config.config.kuka_api.base_url}')
print(f'  Username: {config.config.kuka_api.username}')
print(f'  Timeout: {config.config.kuka_api.timeout}')
print(f'  Max Retries: {config.config.kuka_api.max_retries}')
"
```

#### 解決方案

1. **API 連線問題**:
```bash
# 更新 KUKA API 配置
python3 kuka_config_cli.py config kuka_api

# 如需修改配置
python3 -c "
from rcs.kuka_config_manager import get_config_manager
config_manager = get_config_manager()
api_updates = {
    'timeout': 60.0,
    'max_retries': 5,
    'retry_delay': 2.0
}
config_manager.update_kuka_api_config(api_updates)
print('KUKA API 配置已更新')
"
```

2. **網路連線問題**:
```bash
# 檢查防火牆設定
sudo ufw status

# 檢查路由表
route -n

# 測試不同的 API 端點
for endpoint in "/api/login" "/api/amr/robotQuery"; do
    echo "Testing $endpoint"
    curl -X POST http://192.168.10.3:10870$endpoint \
         -H "Content-Type: application/json" \
         -d '{}' \
         --max-time 10
done
```

3. **使用離線測試環境**:
```bash
# 啟動離線測試伺服器進行調試
cd /app/rcs_ws/src/rcs
python3 test/offline_test_server.py &

# 修改配置指向本地測試伺服器
python3 -c "
from rcs.kuka_config_manager import get_config_manager
config_manager = get_config_manager()
api_updates = {'base_url': 'http://localhost:10870'}
config_manager.update_kuka_api_config(api_updates)
print('已切換到離線測試環境')
"
```

### 3. 資料庫連線問題

#### 問題現象
- 資料庫查詢失敗
- 連線池耗盡
- 任務狀態無法更新

#### 診斷步驟

```bash
# 1. 檢查 PostgreSQL 容器狀態
docker compose -f docker-compose.agvc.yml ps postgres

# 2. 檢查資料庫連線
docker compose -f docker-compose.agvc.yml exec postgres \
  psql -U agvc -d agvc -c "SELECT version();"

# 3. 檢查連線池狀態
python3 -c "
from db_proxy.db_proxy.connection_pool_manager import ConnectionPoolManager
pool = ConnectionPoolManager()
print(f'連線池狀態: {pool.get_pool_status()}')
"

# 4. 檢查活動連線
docker compose -f docker-compose.agvc.yml exec postgres \
  psql -U agvc -d agvc -c "
    SELECT pid, usename, application_name, client_addr, state, query_start 
    FROM pg_stat_activity 
    WHERE state = 'active';"

# 5. 檢查鎖定情況
docker compose -f docker-compose.agvc.yml exec postgres \
  psql -U agvc -d agvc -c "
    SELECT blocked_locks.pid AS blocked_pid,
           blocked_activity.usename AS blocked_user,
           blocking_locks.pid AS blocking_pid,
           blocking_activity.usename AS blocking_user,
           blocked_activity.query AS blocked_statement
    FROM pg_catalog.pg_locks blocked_locks
    JOIN pg_catalog.pg_stat_activity blocked_activity 
      ON blocked_activity.pid = blocked_locks.pid
    JOIN pg_catalog.pg_locks blocking_locks 
      ON blocking_locks.locktype = blocked_locks.locktype
    JOIN pg_catalog.pg_stat_activity blocking_activity 
      ON blocking_activity.pid = blocking_locks.pid
    WHERE NOT blocked_locks.granted;"
```

#### 解決方案

1. **重啟資料庫服務**:
```bash
# 重啟 PostgreSQL 容器
docker compose -f docker-compose.agvc.yml restart postgres

# 等待服務就緒
sleep 10

# 測試連線
ros2 service call /db_proxy/test_connection
```

2. **清理長時間運行的查詢**:
```bash
# 終止長時間運行的查詢（謹慎使用）
docker compose -f docker-compose.agvc.yml exec postgres \
  psql -U agvc -d agvc -c "
    SELECT pg_terminate_backend(pid) 
    FROM pg_stat_activity 
    WHERE state = 'active' 
      AND query_start < NOW() - INTERVAL '5 minutes' 
      AND pid != pg_backend_pid();"
```

3. **重置連線池**:
```python
# 重置連線池配置
from db_proxy.db_proxy.connection_pool_manager import ConnectionPoolManager

# 創建新的連線池管理器（增加池大小）
pool = ConnectionPoolManager(
    connection_string="postgresql+psycopg2://agvc:password@192.168.100.254/agvc",
    pool_size=20,  # 增加基礎池大小
    max_overflow=40  # 增加溢出池大小
)
```

### 4. 配置管理問題

#### 問題現象
- AGV 配置不一致
- 配置檔案損壞
- 資料庫同步失敗

#### 診斷步驟

```bash
# 1. 驗證配置
python3 kuka_config_cli.py validate

# 2. 檢查配置檔案
ls -la /app/config/unified_fleet_config.yaml
cat /app/config/unified_fleet_config.yaml | head -20

# 3. 檢查備份檔案
ls -la /app/config/backups/

# 4. 比較資料庫與配置檔案
python3 -c "
from rcs.kuka_config_manager import get_config_manager
config_manager = get_config_manager()

# 顯示配置檔案中的 AGV
print('配置檔案中的 AGV:')
for name, agv in config_manager.config.agvs.items():
    print(f'  {name}: ID={agv.id}, Model={agv.model.value}')

# 嘗試與資料庫同步
print('\n嘗試與資料庫同步...')
if config_manager.sync_with_database():
    print('同步成功')
else:
    print('同步失敗')
"
```

#### 解決方案

1. **恢復配置備份**:
```bash
# 列出可用備份
ls -lt /app/config/backups/

# 恢復最新備份
cp /app/config/backups/unified_fleet_config_YYYYMMDD_HHMMSS.yaml \
   /app/config/unified_fleet_config.yaml

# 驗證恢復的配置
python3 kuka_config_cli.py validate
```

2. **重建配置檔案**:
```python
# 從資料庫重建配置
from rcs.kuka_config_manager import KukaConfigManager
import os

# 備份現有配置
os.rename('/app/config/unified_fleet_config.yaml', 
          '/app/config/unified_fleet_config.yaml.broken')

# 創建新的配置管理器（將從資料庫載入）
config_manager = KukaConfigManager()

# 儲存配置
config_manager.save_config()
print("配置檔案已重建")
```

3. **手動配置同步**:
```bash
# 強制與資料庫同步
python3 kuka_config_cli.py sync

# 匯出配置進行檢查
python3 kuka_config_cli.py export --format yaml --output /tmp/current_config.yaml

# 比較配置
diff /app/config/unified_fleet_config.yaml /tmp/current_config.yaml
```

### 5. 性能問題

#### 問題現象
- 任務分派延遲過高
- 系統響應緩慢
- 記憶體使用率過高

#### 診斷步驟

```bash
# 1. 檢查系統資源使用
top -p $(pgrep -f "rcs_core")
free -h
df -h

# 2. 檢查 ROS 2 節點性能
ros2 topic hz /rcs/fleet_status
ros2 topic bw /rcs/task_assignments

# 3. 檢查資料庫性能
docker compose -f docker-compose.agvc.yml exec postgres \
  psql -U agvc -d agvc -c "
    SELECT query, calls, total_time, mean_time 
    FROM pg_stat_statements 
    ORDER BY mean_time DESC 
    LIMIT 10;"

# 4. 監控任務分派延遲
ros2 topic echo /rcs/system_metrics | grep dispatch_latency

# 5. 檢查日誌中的警告
ros2 topic echo /rcs/system_logs | grep -i "warning\|error"
```

#### 解決方案

1. **調整系統參數**:
```python
# 調整更新間隔
from rcs.kuka_config_manager import get_config_manager
config_manager = get_config_manager()

system_updates = {
    'update_interval': 10.0,      # 降低更新頻率
    'task_dispatch_interval': 2.0  # 降低分派頻率
}

# 更新系統配置
config_manager.config.system.update(system_updates)
config_manager.save_config()
print("系統參數已調整")
```

2. **最佳化資料庫查詢**:
```sql
-- 在資料庫中添加索引
CREATE INDEX IF NOT EXISTS idx_task_status_model 
  ON task (status_id) 
  WHERE (parameters->>'model') IN ('Cargo', 'Loader', 'Unloader');

CREATE INDEX IF NOT EXISTS idx_agv_status_model 
  ON agv (status_id, model) 
  WHERE enable = true;
```

3. **記憶體最佳化**:
```python
# 調整連線池大小
from db_proxy.db_proxy.connection_pool_manager import ConnectionPoolManager

# 降低連線池大小以節省記憶體
pool = ConnectionPoolManager(
    pool_size=5,      # 降低基礎池大小
    max_overflow=10,  # 降低溢出池大小
    pool_recycle=1800 # 縮短連線回收時間
)
```

## 系統監控指標

### 關鍵性能指標 (KPI)

1. **任務分派延遲**: < 3 秒
2. **AGV 利用率**: > 70%
3. **任務完成率**: > 95%
4. **系統可用時間**: > 99%
5. **API 成功率**: > 98%

### 監控命令

```bash
# 即時監控腳本
#!/bin/bash
echo "RCS 系統監控 - $(date)"
echo "============================"

# 檢查節點狀態
echo "ROS 2 節點狀態:"
ros2 node list | grep rcs

# 檢查系統指標
echo -e "\n系統指標:"
ros2 topic echo /rcs/system_metrics --once 2>/dev/null | \
  grep -E "(cpu_usage|memory_usage|dispatch_latency|tasks_dispatched)"

# 檢查車隊狀態
echo -e "\n車隊狀態:"
ros2 topic echo /rcs/fleet_status --once 2>/dev/null | \
  grep -E "(total_agvs|idle_agvs|busy_agvs|error_agvs)"

# 檢查資料庫連線
echo -e "\n資料庫連線:"
ros2 service call /db_proxy/test_connection 2>/dev/null | \
  grep -E "(success|connection_count)"

echo -e "\n============================"
```

---

# 👨‍💻 開發指南

## 新功能開發流程

### 1. 環境準備

```bash
# 進入開發環境
docker compose -f docker-compose.agvc.yml exec agvc_server bash
source /app/setup.bash && agvc_source
cd /app/rcs_ws/src/rcs

# 確認開發分支
git checkout -b feature/new-feature

# 設置開發環境變數
export RCS_DEBUG=1
export RCS_LOG_LEVEL=DEBUG
```

### 2. 新增 AGV 車型支援

**步驟 1: 更新資料模型**

```python
# 在 kuka_config_manager.py 中新增車型
class AGVModel(Enum):
    CARGO = "Cargo"
    LOADER = "Loader"
    UNLOADER = "Unloader"
    KUKA400I = "KUKA400i"
    NEW_MODEL = "NewModel"  # 新增車型
```

**步驟 2: 更新 CT Manager**

```python
# 在 ct_manager.py 中新增分派邏輯
def _determine_target_agv_name(self, model, room_id):
    if model == 'NewModel':
        return f'NewModel{room_id:02d}' if room_id else 'NewModel01'
    # ... 現有邏輯
```

**步驟 3: 更新資料庫初始化**

```python
# 在 db_proxy 專案中新增 AGV 初始化資料
default_agv.append({
    "name": "NewModel01", 
    "model": "NewModel", 
    "x": 0.0, "y": 0.0,
    "heading": 0.0, 
    "description": "新車型測試"
})
```

**步驟 4: 新增測試**

```python
# test/test_new_model.py
def test_new_model_dispatch():
    """測試新車型分派邏輯"""
    # 測試邏輯
    pass
```

### 3. 擴展 KUKA 功能

**新增自定義任務類型**:

```python
# 在 kuka_manager.py 中擴展
class CustomKukaManager(KukaManager):
    def __init__(self, rcs_core):
        super().__init__(rcs_core)
        self.custom_mission_types = {
            'CUSTOM_PICKUP': 'CUSTOM_PICKUP',
            'CUSTOM_DELIVERY': 'CUSTOM_DELIVERY'
        }
        
    def handle_custom_mission(self, mission_data):
        """處理自定義任務"""
        # 自定義任務處理邏輯
        pass
```

### 4. 新增配置選項

```python
# 擴展統一配置
@dataclass
class ExtendedFleetConfig(UnifiedFleetConfig):
    custom_settings: Dict[str, Any] = field(default_factory=dict)
    
    def __post_init__(self):
        super().__post_init__()
        self.custom_settings.update({
            'enable_advanced_routing': True,
            'custom_timeout': 120.0,
            'experimental_features': False
        })
```

### 5. 開發最佳實踐

**程式碼品質**:
- 遵循 PEP 8 編碼規範
- 使用類型提示 (Type Hints)
- 添加完整的 docstring
- 實現適當的錯誤處理

**測試覆蓋**:
```bash
# 確保新功能有充分測試覆蓋
python3 -m pytest test/test_new_feature.py --cov=rcs.new_module --cov-report=html

# 目標覆蓋率 > 90%
python3 -m pytest --cov=rcs --cov-fail-under=90
```

**文檔更新**:
- 更新 API 文檔
- 添加使用範例
- 更新故障排除指南
- 記錄設計決策

### 6. 整合測試

```bash
# 執行完整測試套件
python3 -m pytest test/ -v

# 執行整合測試
python3 -m pytest test/test_integration.py -v

# 執行效能測試
python3 test/test_performance.py

# 離線環境測試
python3 test/offline_test_server.py &
python3 -m pytest test/test_kuka_integration.py -v
```

## 程式碼審查清單

### 功能性檢查
- [ ] 新功能符合需求規格
- [ ] 錯誤處理完整且適當
- [ ] 日誌記錄詳細且有意義
- [ ] 配置選項已正確實現
- [ ] 向後相容性已驗證

### 程式碼品質
- [ ] 遵循專案編碼規範
- [ ] 類型提示完整
- [ ] 函數和類別有適當的 docstring
- [ ] 變數和函數命名清楚
- [ ] 沒有重複程式碼

### 測試覆蓋
- [ ] 單元測試覆蓋主要邏輯
- [ ] 整合測試驗證組件互動
- [ ] 邊界條件已測試
- [ ] 錯誤情況已測試
- [ ] 測試覆蓋率 > 90%

### 文檔更新
- [ ] API 文檔已更新
- [ ] 設定指南已更新
- [ ] 故障排除指南已更新
- [ ] 變更日誌已記錄

## 除錯技巧

### 使用內建除錯工具

```python
# 添加除錯日誌
self.get_logger().debug(f"Processing task: {task.id}")
self.get_logger().info(f"AGV {agv_name} assigned to task {task.id}")
self.get_logger().warning(f"No idle AGV found for task {task.id}")
self.get_logger().error(f"Failed to assign task {task.id}: {error}")

# 使用 Python debugger
import pdb
pdb.set_trace()  # 設置斷點

# 或使用 ipdb（增強版）
import ipdb
ipdb.set_trace()
```

### ROS 2 除錯工具

```bash
# 監控話題資料
ros2 topic echo /rcs/dispatch_logs
ros2 topic echo /rcs/system_metrics

# 檢查服務狀態
ros2 service list | grep rcs
ros2 service type /rcs/assign_task

# 檢查節點資訊
ros2 node info /rcs_core
ros2 param list /rcs_core

# 實時監控
ros2 topic hz /rcs/fleet_status
ros2 topic bw /rcs/task_assignments
```

### 資料庫除錯

```python
# 啟用 SQLAlchemy 除錯
import logging
logging.getLogger('sqlalchemy.engine').setLevel(logging.INFO)

# 檢查資料庫狀態
from db_proxy.db_proxy.connection_pool_manager import ConnectionPoolManager
with ConnectionPoolManager().get_session() as session:
    # 執行除錯查詢
    result = session.execute("SELECT version()")
    print(f"Database version: {result.scalar()}")
```

---

# 🚀 部署指南

## 生產環境部署

### 1. 環境準備

**系統需求**:
- Ubuntu 22.04 LTS 或更新版本
- Docker 24.0+ 與 Docker Compose V2
- 最少 4GB RAM，推薦 8GB+
- 最少 20GB 可用磁碟空間

**網路需求**:
- AGV 車載系統網段: 192.168.100.x
- AGVC 管理系統網段: 192.168.10.x
- KUKA Fleet API: 192.168.10.3:10870
- PostgreSQL: 192.168.100.254:5432

### 2. 配置檔案準備

**環境變數設定**:
```bash
# /app/.env
ENVIRONMENT=production
DB_HOST=192.168.100.254
DB_PORT=5432
DB_NAME=agvc
DB_USER=agvc
DB_PASSWORD=your_secure_password

KUKA_API_HOST=192.168.10.3
KUKA_API_PORT=10870
KUKA_API_USERNAME=admin
KUKA_API_PASSWORD=your_kuka_password

RCS_LOG_LEVEL=INFO
RCS_ENABLE_MONITORING=true
RCS_AUTO_RECOVERY=true
```

**RCS 系統配置**:
```yaml
# /app/config/rcs_production.yaml
system:
  environment: "production"
  update_interval: 5.0
  heartbeat_timeout: 30.0
  task_dispatch_interval: 1.0
  log_level: "INFO"
  enable_monitoring: true
  enable_auto_recovery: true
  max_concurrent_tasks: 10
  api_timeout: 30.0
  database_pool_size: 20
  database_max_overflow: 40
```

### 3. 部署步驟

**步驟 1: 部署 AGVC 管理系統**

```bash
# 啟動 AGVC 系統
docker compose -f docker-compose.agvc.yml up -d

# 等待服務就緒
sleep 30

# 檢查服務狀態
docker compose -f docker-compose.agvc.yml ps

# 初始化資料庫
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "
  source /app/setup.bash && agvc_source
  cd /app/db_proxy_ws/src/db_proxy/db_proxy/sql/init_data
  python3 run_all_init.py
"

# 驗證資料庫連線
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "
  source /app/setup.bash && agvc_source
  ros2 service call /db_proxy/test_connection
"
```

**步驟 2: 配置 RCS 系統**

```bash
# 進入 AGVC 容器
docker compose -f docker-compose.agvc.yml exec agvc_server bash
source /app/setup.bash && agvc_source

# 初始化 RCS 配置
cd /app/rcs_ws/src/rcs
python3 kuka_config_cli.py validate
python3 kuka_config_cli.py sync

# 測試 KUKA 連線
python3 -c "
from rcs.kuka_config_manager import get_config_manager
import requests

config = get_config_manager()
api_config = config.config.kuka_api

try:
    response = requests.get(f'{api_config.base_url}/api/status', timeout=10)
    print(f'KUKA API 狀態: {response.status_code}')
except Exception as e:
    print(f'KUKA API 連線失敗: {e}')
"
```

**步驟 3: 啟動 RCS 服務**

```bash
# 在 AGVC 容器中啟動 RCS
cd /app/rcs_ws
colcon build --packages-select rcs
source install/setup.bash

# 啟動 RCS 核心節點
ros2 run rcs rcs_core &

# 等待啟動完成
sleep 10

# 檢查節點狀態
ros2 node list | grep rcs
ros2 topic list | grep rcs
```

**步驟 4: 部署驗證**

```bash
# 系統健康檢查
ros2 service call /rcs/get_fleet_status rcs_interfaces/srv/GetFleetStatus \
  '{fleet_type: "all", include_metrics: true}'

# 檢查任務分派功能
python3 -c "
from db_proxy.db_proxy.connection_pool_manager import ConnectionPoolManager
from sqlmodel import select
from db_proxy.db_proxy.models import Task, AGV

with ConnectionPoolManager().get_session() as session:
    # 檢查 AGV 狀態
    agvs = session.exec(select(AGV).where(AGV.enable == True)).all()
    print(f'啟用的 AGV 數量: {len(agvs)}')
    
    # 檢查待執行任務
    tasks = session.exec(select(Task).where(Task.status_id == 1)).all()
    print(f'待執行任務數量: {len(tasks)}')
"

# 測試手動任務分配
ros2 service call /rcs/assign_task rcs_interfaces/srv/AssignTask \
  '{task_id: 1, agv_name: "Cargo02", force_assign: false, priority: 2}'
```

### 4. 監控與日誌

**設置系統監控**:

```bash
# 創建監控腳本
cat > /app/scripts/rcs_monitor.sh << 'EOF'
#!/bin/bash

LOG_FILE="/var/log/rcs_monitor.log"
echo "$(date): RCS 系統監控開始" >> $LOG_FILE

# 檢查 RCS 節點
if ros2 node list | grep -q rcs_core; then
    echo "$(date): RCS 核心節點運行正常" >> $LOG_FILE
else
    echo "$(date): 警告 - RCS 核心節點未運行" >> $LOG_FILE
    # 重啟邏輯
    ros2 run rcs rcs_core &
fi

# 檢查系統指標
METRICS=$(ros2 topic echo /rcs/system_metrics --once 2>/dev/null)
if [ $? -eq 0 ]; then
    echo "$(date): 系統指標正常" >> $LOG_FILE
else
    echo "$(date): 警告 - 無法獲取系統指標" >> $LOG_FILE
fi

# 檢查資料庫連線
DB_STATUS=$(ros2 service call /db_proxy/test_connection 2>/dev/null)
if echo "$DB_STATUS" | grep -q "success.*true"; then
    echo "$(date): 資料庫連線正常" >> $LOG_FILE
else
    echo "$(date): 警告 - 資料庫連線異常" >> $LOG_FILE
fi
EOF

chmod +x /app/scripts/rcs_monitor.sh

# 設置 cron 定時監控（每分鐘）
echo "* * * * * /app/scripts/rcs_monitor.sh" | crontab -
```

**日誌管理**:

```bash
# 設置日誌輪轉
cat > /etc/logrotate.d/rcs << 'EOF'
/var/log/rcs_monitor.log {
    daily
    rotate 7
    compress
    delaycompress
    missingok
    notifempty
    postrotate
        /bin/kill -HUP `cat /var/run/rsyslogd.pid 2> /dev/null` 2> /dev/null || true
    endscript
}
EOF

# ROS 2 日誌配置
cat > /app/config/ros2_logging.yaml << 'EOF'
loggers:
  rcs_core:
    level: INFO
  kuka_manager:
    level: INFO
  ct_manager:
    level: INFO
appenders:
  file_appender:
    type: file
    file_name: /var/log/ros2_rcs.log
    pattern: "[%d{yyyy-MM-dd HH:mm:ss}] [%p] [%c] %m%n"
root:
  level: INFO
  appender_refs:
    - file_appender
EOF

export ROS2_LOGGING_CONFIG_FILE=/app/config/ros2_logging.yaml
```

### 5. 備份與恢復

**自動備份腳本**:

```bash
# 創建備份腳本
cat > /app/scripts/rcs_backup.sh << 'EOF'
#!/bin/bash

BACKUP_DIR="/app/backups"
DATE=$(date +%Y%m%d_%H%M%S)

# 創建備份目錄
mkdir -p $BACKUP_DIR/$DATE

# 備份配置檔案
cp -r /app/config $BACKUP_DIR/$DATE/

# 備份資料庫
docker compose -f docker-compose.agvc.yml exec -T postgres \
  pg_dump -U agvc agvc > $BACKUP_DIR/$DATE/database_backup.sql

# 壓縮備份
tar -czf $BACKUP_DIR/rcs_backup_$DATE.tar.gz -C $BACKUP_DIR $DATE
rm -rf $BACKUP_DIR/$DATE

# 清理舊備份（保留7天）
find $BACKUP_DIR -name "rcs_backup_*.tar.gz" -mtime +7 -delete

echo "備份完成: rcs_backup_$DATE.tar.gz"
EOF

chmod +x /app/scripts/rcs_backup.sh

# 設置每日自動備份
echo "0 2 * * * /app/scripts/rcs_backup.sh" | crontab -
```

**恢復程序**:

```bash
# 恢復配置和資料庫
#!/bin/bash

BACKUP_FILE="$1"
if [ -z "$BACKUP_FILE" ]; then
    echo "用法: $0 <backup_file.tar.gz>"
    exit 1
fi

# 停止服務
docker compose -f docker-compose.agvc.yml down

# 解壓備份
tar -xzf $BACKUP_FILE -C /tmp/
BACKUP_EXTRACTED=$(ls /tmp/ | grep rcs_backup)

# 恢復配置
cp -r /tmp/$BACKUP_EXTRACTED/config/* /app/config/

# 重啟服務
docker compose -f docker-compose.agvc.yml up -d
sleep 30

# 恢復資料庫
docker compose -f docker-compose.agvc.yml exec -T postgres \
  psql -U agvc -d agvc < /tmp/$BACKUP_EXTRACTED/database_backup.sql

# 清理臨時檔案
rm -rf /tmp/$BACKUP_EXTRACTED

echo "恢復完成"
```

---

# 📊 監控與維護

## 系統監控

### 關鍵監控指標

**系統健康指標**:
- RCS 核心節點狀態
- 資料庫連線池狀態
- KUKA Fleet API 連線狀態
- 系統資源使用率（CPU、記憶體、磁碟）

**業務指標**:
- 任務分派成功率
- 平均任務分派延遲
- AGV 利用率
- 任務完成率
- 錯誤任務數量

### 監控實現

**ROS 2 監控節點**:

```python
# monitoring/rcs_monitor_node.py
import rclpy
from rclpy.node import Node
from rcs_interfaces.msg import SystemMetrics
import psutil
import time

class RcsMonitorNode(Node):
    def __init__(self):
        super().__init__('rcs_monitor')
        
        # 發布系統指標
        self.metrics_publisher = self.create_publisher(
            SystemMetrics, '/rcs/system_metrics', 10
        )
        
        # 定時器 - 每5秒發布一次指標
        self.timer = self.create_timer(5.0, self.publish_metrics)
        
        # 監控指標
        self.task_count = 0
        self.api_failures = 0
        self.dispatch_latencies = []
        
    def publish_metrics(self):
        """發布系統監控指標"""
        metrics = SystemMetrics()
        metrics.timestamp = self.get_clock().now().to_msg()
        
        # 系統資源指標
        metrics.cpu_usage_percent = psutil.cpu_percent()
        metrics.memory_usage_percent = psutil.virtual_memory().percent
        
        # 資料庫連線指標
        metrics.active_database_connections = self.get_db_connections()
        
        # 業務指標
        metrics.average_dispatch_latency_ms = (
            sum(self.dispatch_latencies) / len(self.dispatch_latencies)
            if self.dispatch_latencies else 0.0
        )
        metrics.tasks_dispatched_per_minute = self.task_count
        metrics.api_call_failures_per_minute = self.api_failures
        
        # 系統警告
        metrics.system_warnings = self.get_system_warnings()
        
        self.metrics_publisher.publish(metrics)
        
        # 重置計數器
        self.task_count = 0
        self.api_failures = 0
        self.dispatch_latencies.clear()
```

**Prometheus 整合**:

```python
# monitoring/prometheus_exporter.py
from prometheus_client import start_http_server, Gauge, Counter, Histogram
import rclpy
from rcs_interfaces.msg import SystemMetrics

class RcsPrometheusExporter:
    def __init__(self):
        # 定義 Prometheus 指標
        self.cpu_usage = Gauge('rcs_cpu_usage_percent', 'CPU usage percentage')
        self.memory_usage = Gauge('rcs_memory_usage_percent', 'Memory usage percentage')
        self.db_connections = Gauge('rcs_db_connections', 'Active database connections')
        
        self.dispatch_latency = Histogram(
            'rcs_dispatch_latency_seconds', 
            'Task dispatch latency in seconds'
        )
        
        self.tasks_dispatched = Counter(
            'rcs_tasks_dispatched_total', 
            'Total number of tasks dispatched'
        )
        
        self.api_failures = Counter(
            'rcs_api_failures_total', 
            'Total number of API failures'
        )
        
        # 啟動 Prometheus HTTP 伺服器
        start_http_server(8000)
        
    def update_metrics(self, metrics_msg: SystemMetrics):
        """更新 Prometheus 指標"""
        self.cpu_usage.set(metrics_msg.cpu_usage_percent)
        self.memory_usage.set(metrics_msg.memory_usage_percent)
        self.db_connections.set(metrics_msg.active_database_connections)
        
        self.dispatch_latency.observe(metrics_msg.average_dispatch_latency_ms / 1000.0)
        
        self.tasks_dispatched.inc(metrics_msg.tasks_dispatched_per_minute)
        self.api_failures.inc(metrics_msg.api_call_failures_per_minute)
```

### 告警系統

**告警規則定義**:

```yaml
# monitoring/alert_rules.yaml
groups:
  - name: rcs_alerts
    rules:
      - alert: RcsNodeDown
        expr: up{job="rcs_core"} == 0
        for: 30s
        labels:
          severity: critical
        annotations:
          summary: "RCS 核心節點下線"
          description: "RCS 核心節點已下線超過30秒"
          
      - alert: HighDispatchLatency
        expr: rcs_dispatch_latency_seconds > 3
        for: 1m
        labels:
          severity: warning
        annotations:
          summary: "任務分派延遲過高"
          description: "任務分派延遲超過3秒，當前值: {{ $value }}秒"
          
      - alert: DatabaseConnectionsHigh
        expr: rcs_db_connections > 50
        for: 2m
        labels:
          severity: warning
        annotations:
          summary: "資料庫連線數過高"
          description: "資料庫連線數超過50，當前值: {{ $value }}"
          
      - alert: ApiFailureRateHigh
        expr: rate(rcs_api_failures_total[5m]) > 0.1
        for: 3m
        labels:
          severity: critical
        annotations:
          summary: "API 失敗率過高"
          description: "過去5分鐘 API 失敗率超過10%"
```

**告警通知**:

```python
# monitoring/alert_manager.py
import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
import requests
import json

class AlertManager:
    def __init__(self, config):
        self.smtp_server = config['smtp']['server']
        self.smtp_port = config['smtp']['port']
        self.smtp_username = config['smtp']['username']
        self.smtp_password = config['smtp']['password']
        self.slack_webhook = config.get('slack_webhook')
        
    def send_email_alert(self, subject, message, recipients):
        """發送郵件告警"""
        msg = MIMEMultipart()
        msg['From'] = self.smtp_username
        msg['To'] = ', '.join(recipients)
        msg['Subject'] = subject
        
        msg.attach(MIMEText(message, 'plain', 'utf-8'))
        
        try:
            server = smtplib.SMTP(self.smtp_server, self.smtp_port)
            server.starttls()
            server.login(self.smtp_username, self.smtp_password)
            server.sendmail(self.smtp_username, recipients, msg.as_string())
            server.quit()
            return True
        except Exception as e:
            print(f"郵件發送失敗: {e}")
            return False
            
    def send_slack_alert(self, message):
        """發送 Slack 告警"""
        if not self.slack_webhook:
            return False
            
        payload = {
            'text': message,
            'username': 'RCS Alert Bot',
            'icon_emoji': ':warning:'
        }
        
        try:
            response = requests.post(
                self.slack_webhook,
                data=json.dumps(payload),
                headers={'Content-Type': 'application/json'},
                timeout=10
            )
            return response.status_code == 200
        except Exception as e:
            print(f"Slack 告警發送失敗: {e}")
            return False
```

## 維護作業

### 定期維護檢查清單

**每日檢查**:
- [ ] RCS 核心節點運行狀態
- [ ] 系統資源使用率
- [ ] 錯誤日誌檢查
- [ ] 任務分派成功率
- [ ] AGV 連線狀態

**每週檢查**:
- [ ] 資料庫性能分析
- [ ] 系統日誌清理
- [ ] 配置檔案備份驗證
- [ ] API 性能統計
- [ ] 磁碟空間使用檢查

**每月檢查**:
- [ ] 完整系統備份測試
- [ ] 安全更新檢查
- [ ] 性能趨勢分析
- [ ] 容量規劃評估
- [ ] 災難恢復測試

### 維護腳本

**系統健康檢查**:

```bash
#!/bin/bash
# scripts/health_check.sh

echo "RCS 系統健康檢查 - $(date)"
echo "========================================"

# 檢查 Docker 容器狀態
echo "1. Docker 容器狀態:"
docker compose -f docker-compose.agvc.yml ps

# 檢查 ROS 2 節點
echo -e "\n2. ROS 2 節點狀態:"
ros2 node list | grep -E "(rcs|db_proxy)"

# 檢查系統資源
echo -e "\n3. 系統資源使用:"
echo "CPU: $(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | cut -d'%' -f1)%"
echo "Memory: $(free | grep Mem | awk '{printf "%.1f%%", $3/$2 * 100.0}')"
echo "Disk: $(df -h /app | awk 'NR==2{print $5}')"

# 檢查資料庫連線
echo -e "\n4. 資料庫連線測試:"
db_result=$(ros2 service call /db_proxy/test_connection 2>/dev/null)
if echo "$db_result" | grep -q "success.*true"; then
    echo "✅ 資料庫連線正常"
else
    echo "❌ 資料庫連線異常"
fi

# 檢查 KUKA API 連線
echo -e "\n5. KUKA API 連線測試:"
kuka_result=$(curl -s -w "%{http_code}" http://192.168.10.3:10870/api/status -o /dev/null)
if [ "$kuka_result" = "200" ]; then
    echo "✅ KUKA API 連線正常"
else
    echo "❌ KUKA API 連線異常 (HTTP $kuka_result)"
fi

# 檢查任務執行狀況
echo -e "\n6. 任務執行統計:"
python3 -c "
from db_proxy.db_proxy.connection_pool_manager import ConnectionPoolManager
from sqlmodel import select, func
from db_proxy.db_proxy.models import Task
from datetime import datetime, timedelta

with ConnectionPoolManager().get_session() as session:
    # 今日任務統計
    today = datetime.now().date()
    
    completed = session.exec(
        select(func.count(Task.id)).where(
            Task.status_id == 4,  # COMPLETED
            func.date(Task.created_at) == today
        )
    ).first()
    
    failed = session.exec(
        select(func.count(Task.id)).where(
            Task.status_id == 5,  # FAILED
            func.date(Task.created_at) == today
        )
    ).first()
    
    pending = session.exec(
        select(func.count(Task.id)).where(Task.status_id == 1)
    ).first()
    
    print(f'今日完成任務: {completed or 0}')
    print(f'今日失敗任務: {failed or 0}')
    print(f'待執行任務: {pending or 0}')
"

echo -e "\n========================================"
echo "健康檢查完成"
```

**日誌清理腳本**:

```bash
#!/bin/bash
# scripts/log_cleanup.sh

LOG_RETENTION_DAYS=7
echo "開始清理 $LOG_RETENTION_DAYS 天前的日誌檔案..."

# 清理 ROS 2 日誌
find /var/log/ros2/ -name "*.log" -mtime +$LOG_RETENTION_DAYS -delete 2>/dev/null

# 清理 Docker 日誌
docker system prune -f --filter "until=$(($LOG_RETENTION_DAYS*24))h"

# 清理自定義日誌
find /var/log/ -name "rcs_*.log" -mtime +$LOG_RETENTION_DAYS -delete 2>/dev/null

# 清理備份檔案
find /app/config/backups/ -name "*.yaml" -mtime +$LOG_RETENTION_DAYS -delete 2>/dev/null

echo "日誌清理完成"
```

**性能優化腳本**:

```bash
#!/bin/bash
# scripts/performance_optimize.sh

echo "開始系統性能優化..."

# 優化資料庫
echo "1. 優化資料庫..."
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc -c "
    VACUUM ANALYZE;
    REINDEX DATABASE agvc;
"

# 清理 Docker 資源
echo "2. 清理 Docker 資源..."
docker system prune -f
docker volume prune -f

# 重啟服務以釋放記憶體
echo "3. 重啟 RCS 服務..."
ros2 lifecycle set /rcs_core shutdown 2>/dev/null
sleep 5
ros2 run rcs rcs_core &

echo "性能優化完成"
```

---

## 🎯 總結

這份文檔提供了 RCS 系統的完整指南，涵蓋了從基礎概念到高級部署的所有面向。主要亮點包括：

### 🔧 技術特色
- **統一配置管理**: 整合 KUKA 和 CT AGV 的配置系統
- **智能任務分派**: 基於房間和車型的智能分派機制
- **完整測試框架**: 單元測試、整合測試和離線測試環境
- **容錯機制**: API 重試、錯誤恢復和自動重啟
- **監控告警**: 完整的系統監控和告警機制

### 📚 文檔完整性
- **ROS 2 API 文檔**: 詳細的服務和話題定義
- **配置管理指南**: CLI 工具和程式化操作
- **故障排除手冊**: 常見問題的診斷和解決方案
- **開發指南**: 新功能開發和最佳實踐
- **部署指南**: 生產環境部署和維護

### 🚀 生產就緒
- **高可用性**: 自動恢復和故障轉移機制
- **可擴展性**: 支援多車型混合車隊擴展
- **安全性**: 完整的配置管理和權限控制
- **可維護性**: 豐富的監控指標和維護工具

這個 RCS 系統已經具備了企業級應用所需的所有特性，可以直接用於生產環境中的 AGV 車隊管理。

## 開發指南

### 新增車型
1. 更新 `ct_manager.py` 中的查詢條件
2. 修改 `_validate_task_parameters()` 方法
3. 更新 `_determine_target_agv_name()` 邏輯
4. 更新測試腳本和文件

### 文件管理
- 總結性文件放在 `docs/summaries/`
- 測試相關文件放在 `docs/testing/`
- 使用 Markdown 格式
- 保持文件與程式碼同步更新

### 測試開發
- 測試腳本放在專案根目錄
- 使用 `test_` 前綴命名
- 包含完整的功能驗證

## 建置與執行

```bash
# 建置專案
cd rcs_ws
colcon build

# 執行 RCS 核心
source install/setup.bash
ros2 run rcs rcs_core

# 執行測試
cd src/rcs
python test_ct_dispatch.py
```

## 注意事項

1. **路徑設定**：測試腳本已配置正確的 Python 路徑
2. **資料庫連線**：確保資料庫服務正常運行
3. **車型配置**：AGV 資料需要正確初始化
4. **狀態同步**：確保 AGV 實際狀態與資料庫一致

## 維護

- 定期更新文件
- 保持測試覆蓋率
- 監控系統效能
- 記錄重要變更

## 參考

- 參考 `agvcui` 專案的文件組織結構
- 遵循 ROS 2 套件開發規範
- 保持與其他專案的一致性
