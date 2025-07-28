# AGV 工作空間 (agv_ws)

## 📋 基本資訊

**啟動狀態**: ✅ 實際啟動 (Loader AGV 在容器啟動腳本中自動載入並執行)
**運行環境**: 🚗 AGV 車載系統 (主要)
**主要功能**: AGV 核心控制系統和狀態機架構
**依賴狀態**: 依賴多個工作空間，是 RosAGV 系統的核心模組
**實作狀態**: Loader AGV (完整) | Cargo Mover AGV (部分) | Unloader AGV (部分)

## 📋 專案概述

AGV 工作空間是 RosAGV 系統的核心模組，包含多種類型的 AGV 控制節點和基礎框架。該工作空間實現了基於狀態機的 AGV 控制架構，支援不同類型的 AGV（Loader、Cargo Mover、Unloader）以及完整的狀態管理和任務執行功能。

此工作空間作為系統的核心，整合了 PLC 通訊、感測器處理、路徑規劃和任務執行等關鍵功能。它採用雙層狀態管理架構，包含 AGV 層狀態和通用狀態，支援複雜的任務流程和狀態轉換。系統具備完整的記憶體管理（65536 words）、50ms 主迴圈控制和 1.5 秒狀態發布週期。

**⚠️ 重要說明**: 目前僅 **Loader AGV** 完全實作並自動啟動，其他車型（Cargo Mover AGV、Unloader AGV）仍在開發中，需要手動啟動且功能不完整。

## 🔗 依賴關係

### 系統套件依賴
- **ROS 2**: `rclpy`, `rclpy.node`, `rclpy.timer`, `rclpy.clock`
- **Python 標準庫**: `time`, `threading`, `abc`

### 依賴的工作空間
- **plc_proxy_ws**: 使用 `PlcClient` 進行 PLC 通訊和服務調用
- **keyence_plc_ws**: 使用 `PlcMemory` 進行記憶體管理 (65536 words = 131072 bytes)
- **path_algorithm**: 使用 `AStarAlgorithm` 進行路徑規劃和座標轉換
- **agv_cmd_service_ws**: 可能使用 AGV 命令服務 (部分整合)
- **db_proxy_ws**: 使用資料庫客戶端進行資料存取 (部分整合)

### 被依賴的工作空間
- **外部系統**: AGVC 管理系統、Web UI 等監控和控制本工作空間

### 外部依賴
- **ROS 2**: `rclpy`, `rclpy.executors`, `sensor_msgs`, `geometry_msgs`
- **Python 標準庫**: `time`, `threading`, `yaml`, `pathlib`

## 🏗️ 專案結構

```
agv_ws/
├── src/
│   ├── agv_base/                  # AGV 基礎框架套件
│   │   ├── agv_base/
│   │   │   ├── agv_node_base.py           # AGV 節點基礎類 (核心)
│   │   │   ├── agv_status.py              # AGV 狀態資料結構
│   │   │   ├── base_context.py            # 狀態機上下文管理
│   │   │   ├── context_abc.py             # 狀態機抽象介面
│   │   │   ├── event.py                   # 事件系統
│   │   │   ├── robot.py                   # 機器人基礎類別
│   │   │   ├── hokuyo_dms_8bit.py         # Hokuyo 8bit 光通訊模組處理
│   │   │   ├── agv_states/                # AGV 專用狀態實作
│   │   │   │   ├── idle_state.py          # 空閒狀態
│   │   │   │   ├── manual_state.py        # 手動模式狀態
│   │   │   │   ├── mission_select_state.py # 任務選擇狀態
│   │   │   │   └── wait_robot_state.py    # 等待機器人狀態
│   │   │   └── states/                    # 通用狀態基礎類別
│   │   │       ├── state.py               # 狀態抽象基礎類別
│   │   │       ├── idle_state.py          # 通用空閒狀態
│   │   │       ├── auto_state.py          # 自動模式狀態
│   │   │       ├── manual_state.py        # 通用手動狀態
│   │   │       ├── error_state.py         # 錯誤狀態
│   │   │       ├── write_path_state.py    # 路徑寫入狀態
│   │   │       └── test_*.py              # 測試狀態
│   │   ├── package.xml                    # 套件配置
│   │   └── setup.py                       # Python 套件設定
│   ├── agv_interfaces/            # AGV 訊息介面定義
│   │   ├── msg/
│   │   │   ├── AgvStatus.msg              # AGV 狀態訊息
│   │   │   └── AgvStateChange.msg         # AGV 狀態變更訊息
│   │   ├── CMakeLists.txt                 # CMake 建置配置
│   │   └── package.xml                    # 介面套件配置
│   ├── loader_agv/                # Loader AGV 實作套件
│   │   ├── loader_agv/
│   │   │   ├── agv_core_node.py           # Loader AGV 核心節點
│   │   │   ├── loader_context.py          # Loader 狀態機上下文
│   │   │   ├── robot_context.py           # 機器人狀態機上下文
│   │   │   └── robot_states/              # Loader 專用機器人狀態
│   │   │       └── base_robot_state.py    # 機器人狀態基礎類別
│   │   ├── launch/
│   │   │   └── launch.py                  # 主要啟動檔案 (系統入口)
│   │   ├── package.xml
│   │   └── setup.py
│   ├── cargo_mover_agv/           # Cargo Mover AGV 實作套件
│   │   ├── cargo_mover_agv/
│   │   │   ├── agv_core_node.py           # Cargo Mover AGV 核心節點
│   │   │   ├── robot_context.py           # 機器人上下文
│   │   │   └── robot_states/              # Cargo Mover 專用機器人狀態
│   │   ├── package.xml
│   │   └── setup.py
│   └── unloader_agv/              # Unloader AGV 實作套件
│       ├── unloader_agv/
│       │   ├── agv_core_node.py           # Unloader AGV 核心節點
│       │   └── robot_states/              # Unloader 專用機器人狀態
│       ├── package.xml
│       └── setup.py
├── test/                          # 測試檔案
│   └── test_hokuyo_dms_8bit.py    # Hokuyo 8bit 光通訊模組測試
└── README.md                      # 本檔案
```

## ⚙️ 主要功能

### 1. AGV 基礎框架 (agv_base)
**AgvNodebase 核心類別**:
- **PLC 通訊整合**: 透過 `PlcClient` 與 PLC 進行資料交換
- **記憶體管理**: 使用 `PlcMemory` 管理 65536 words (131072 bytes) 的 PLC 記憶體
- **狀態機架構**: 基於狀態模式的 AGV 控制邏輯，支援狀態轉換和事件處理
- **定時器系統**: 50ms 主迴圈 + 1.5 秒狀態發布週期
- **狀態發布**: 定期發布 AGV 狀態資訊到 `/agv/status` 主題

**狀態機系統**:
- **BaseContext**: 狀態機上下文管理，支援狀態轉換和事件發布
- **Event 系統**: 狀態變更事件處理和通知機制
- **狀態基礎類別**: 抽象狀態介面，支援進入、離開和處理邏輯

### 2. 雙層狀態管理架構
**AGV 層狀態 (agv_states/)**:
- **IdleState**: 空閒狀態，處理 TAG 座標請求和路徑資料檢查
- **ManualState**: 手動控制模式，支援搖桿和手動命令
- **MissionSelectState**: 任務選擇和派發狀態
- **WaitRobotState**: 等待機器人完成作業狀態

**通用狀態 (states/)**:
- **IdleState**: 通用空閒狀態，支援自動/手動模式切換
- **AutoState**: 自動模式狀態
- **ManualState**: 通用手動狀態
- **ErrorState**: 錯誤處理狀態
- **WritePathState**: 路徑寫入狀態

### 3. 多類型 AGV 支援

#### Loader AGV ✅ (完全實作)
- **功能**: 專門用於裝載作業的 AGV
- **啟動狀態**: 自動啟動 (`ros2 launch loader_agv launch.py`)
- **實作程度**: 100% - 完整的狀態機、機器人控制、PLC 整合
- **特色**: 包含完整的機器人狀態管理和裝載流程
- **業務流程**: 接收任務 → 移動到裝載點 → 執行裝載 → 移動到目標點 → 完成任務

#### Cargo Mover AGV ⚠️ (部分實作)
- **功能**: 用於貨物搬運的 AGV
- **啟動狀態**: 手動啟動 (`ros2 run cargo_mover_agv agv_core_node`)
- **實作程度**: 60% - 基礎狀態機已實作，專用功能待開發
- **已完成**: 基本 AGV 控制、狀態管理
- **待完成**: 貨物檢測、搬運邏輯、專用機器人狀態
- **業務流程**: 接收任務 → 移動到貨物點 → 🚧 搬運邏輯 (開發中) → 運輸到目標點

#### Unloader AGV ⚠️ (部分實作)
- **功能**: 專門用於卸載作業的 AGV
- **啟動狀態**: 手動啟動 (`ros2 run unloader_agv agv_core_node`)
- **實作程度**: 40% - 基礎框架已建立，核心功能待開發
- **已完成**: 基本節點結構、狀態機框架
- **待完成**: 卸載邏輯、分揀功能、專用機器人狀態、感測器整合
- **業務流程**: 接收任務 → 移動到卸載點 → 🚧 卸載邏輯 (開發中) → 🚧 分揀作業 (開發中)

### 4. 感測器整合
- **Hokuyo DMS**: 8bit 光通訊模組整合，透過 PLC 進行資料通訊
- **PGV 感測器**: 前後 PGV 感測器支援
- **SLAM 定位**: SLAM 位置資訊處理

## 🚗 車型啟動配置

### Loader AGV (生產環境)
```bash
# 自動啟動 (容器啟動時執行)
ros2 launch loader_agv launch.py

# 包含的服務
# - plc_service (PLC 通訊)
# - joy_linux_node (搖桿控制)
# - agv_core_node (AGV 核心控制)
```

**配置參數**:
- **agv_id**: `cargo02` (預設)
- **room_id**: `2` (從 agv_id 提取)
- **命名空間**: `/cargo02`
- **launch 檔案**: `/app/agv_ws/src/loader_agv/launch/launch.py`

### Cargo Mover AGV (開發環境)
```bash
# 手動啟動 (開發測試用)
ros2 run cargo_mover_agv agv_core_node

# 需要額外啟動的服務
ros2 run plc_proxy plc_service --ros-args -r __ns:=/cargo_mover
ros2 run joy_linux joy_linux_node --ros-args -r __ns:=/cargo_mover
```

**配置參數**:
- **agv_id**: 需手動配置
- **命名空間**: 需手動指定
- **狀態**: 🚧 開發中，功能不完整

### Unloader AGV (開發環境)
```bash
# 手動啟動 (開發測試用)
ros2 run unloader_agv agv_core_node

# 需要額外啟動的服務
ros2 run plc_proxy plc_service --ros-args -r __ns:=/unloader
ros2 run joy_linux joy_linux_node --ros-args -r __ns:=/unloader
```

**配置參數**:
- **agv_id**: 需手動配置
- **命名空間**: 需手動指定
- **狀態**: 🚧 開發中，基礎功能待實作

### 5. 技術特色
- **狀態發布週期**: 1.5 秒 (AGV 狀態) + 50ms 主迴圈
- **PLC 記憶體大小**: 65536 words (131072 bytes)
- **命名空間支援**: 使用 agv_id 作為 ROS 2 命名空間
- **房間 ID 提取**: 從 agv_id 後兩位數字提取 room_id
- **多執行緒支援**: 使用 MultiThreadedExecutor (4 執行緒)

## 🔧 核心 API

### AgvNodebase 基礎類別
```python
from agv_base.agv_node_base import AgvNodebase
from agv_base.states.idle_state import IdleState

class MyAgvNode(AgvNodebase):
    def __init__(self, node_name='my_agv_node'):
        super().__init__(node_name=node_name)

        # AGV 狀態可透過 self.agv_status 存取
        print(f"AGV Auto: {self.agv_status.AGV_Auto}")
        print(f"AGV Manual: {self.agv_status.AGV_MANUAL}")
        print(f"AGV Alarm: {self.agv_status.AGV_ALARM}")

        # PLC 記憶體操作
        self.dMmemory.set_int(7800, 1234, length=2)
        value = self.dMmemory.get_int(7800, length=2)

        # 狀態機操作
        current_state = self.base_context.state.__class__.__name__
        print(f"當前狀態: {current_state}")

# 啟動節點
import rclpy
from rclpy.executors import MultiThreadedExecutor

rclpy.init()
node = MyAgvNode()
executor = MultiThreadedExecutor(num_threads=4)
executor.add_node(node)
executor.spin()
```

### 狀態機使用
```python
from agv_base.base_context import BaseContext
from agv_base.states.idle_state import IdleState
from agv_base.states.auto_state import AutoState

# 建立狀態機上下文
context = BaseContext(IdleState(node))

# 狀態轉換
context.set_state(AutoState(node))

# 處理狀態邏輯
context.handle()

# 取得當前狀態
current_state = context.state.__class__.__name__
```

### 自訂狀態實作
```python
from agv_base.states.state import State
from agv_base.context_abc import ContextABC

class MyCustomState(State):
    def __init__(self, node):
        super().__init__(node)

    def enter(self):
        self.node.get_logger().info("進入自訂狀態")

    def leave(self):
        self.node.get_logger().info("離開自訂狀態")

    def handle(self, context: ContextABC):
        # 狀態處理邏輯
        if self.node.agv_status.AGV_ALARM:
            from agv_base.states.error_state import ErrorState
            context.set_state(ErrorState(self.node))
```

## 📡 訊息介面

### AgvStatus.msg
```
# AGV 基本資訊
string agv_id
string agv_id1
float32 power

# AGV 速度資訊
int32 x_speed
int32 y_speed
int32 theta_speed

# PGV 感測資訊
int32 front_pgv
int32 back_pgv

# 任務與路線資訊
int32 start_point
int32 end_point
int32 action
int32 zone

# SLAM 位置資訊
int32 slam_x
int32 slam_y
int32 slam_theta

# 狀態資訊
int32 status1
int32 status2
int32 status3

# 警報資訊
int32 alarm1-alarm6

# 樓層與雜項
int32 layer
int32 magic
```

### AgvStateChange.msg
```
string agv_id
string context_name
string from_state
string to_state
builtin_interfaces/Time timestamp
```

## 🚀 使用方法

### 1. 建置工作空間
```bash
# 載入 ROS 2 環境並建置
source /opt/ros/jazzy/setup.bash && source /opt/ws_rmw_zenoh/install/setup.bash && cd /app/agv_ws && colcon build
source install/setup.bash
```

### 2. 啟動 Loader AGV (生產環境 - 推薦)
```bash
# 使用 launch 文件啟動完整系統 (自動啟動)
ros2 launch loader_agv launch.py

# 檢查啟動狀態
ros2 node list | grep cargo02
ros2 topic list | grep cargo02

# 檢查 AGV 狀態
ros2 topic echo /agv/status --once
```

### 3. 啟動其他車型 (開發環境)

#### Cargo Mover AGV (部分功能)
```bash
# 載入所有必要的工作空間環境
source /app/setup.bash && all_source

# 啟動核心節點
ros2 run cargo_mover_agv agv_core_node

# ⚠️ 注意：需要手動啟動相關服務
ros2 run plc_proxy plc_service --ros-args -r __ns:=/cargo_mover -p agv_id:=cargo_mover
ros2 run joy_linux joy_linux_node --ros-args -r __ns:=/cargo_mover

# 檢查狀態
ros2 node list | grep cargo_mover
```

#### Unloader AGV (基礎功能)
```bash
# 載入所有必要的工作空間環境
source /app/setup.bash && all_source

# 啟動核心節點
ros2 run unloader_agv agv_core_node

# ⚠️ 注意：需要手動啟動相關服務
ros2 run plc_proxy plc_service --ros-args -r __ns:=/unloader -p agv_id:=unloader
ros2 run joy_linux joy_linux_node --ros-args -r __ns:=/unloader

# 檢查狀態
ros2 node list | grep unloader
```

### 4. 車型選擇和配置
```bash
# 檢查可用的車型
ls /app/agv_ws/src/*/setup.py

# 檢查各車型的實作狀態
ros2 pkg list | grep agv

# 查看車型特定配置
cat /app/agv_ws/src/loader_agv/launch/launch.py
```

### 4. 監控 AGV 狀態
```bash
# 監聽 AGV 狀態訊息
ros2 topic echo /agv/status

# 監聽狀態變更事件
ros2 topic echo /agv/state_change

# 檢查節點資訊
ros2 node info /cargo02/agv_core_node
```

### 5. 測試和除錯
```bash
# 測試 Hokuyo 8bit 光通訊模組
python3 /app/agv_ws/test/test_hokuyo_dms_8bit.py

# 檢查 PLC 連線
ros2 service list | grep plc

# 檢查狀態機狀態
ros2 param get /cargo02/agv_core_node room_id
```

## 🧪 測試方法

### 1. 建置和測試
```bash
# 建置工作空間
source /opt/ros/jazzy/setup.bash && source /opt/ws_rmw_zenoh/install/setup.bash && cd /app/agv_ws && colcon build

# 執行測試
source install/setup.bash && colcon test
colcon test-result --verbose
```

### 2. 狀態機功能測試
```bash
# 啟動 AGV 系統
ros2 launch loader_agv launch.py

# 測試狀態轉換
ros2 topic echo /agv/state_change

# 檢查當前狀態
ros2 topic echo /agv/status --once
```

### 3. PLC 通訊測試
```bash
# 檢查 PLC 服務
ros2 service list | grep plc

# 測試 PLC 讀取
ros2 service call /cargo02/read_continuous_byte plc_interfaces/srv/ReadContinuousByte "{device_type: 'DM', start_address: '7800', length: 10}"

# 測試 PLC 寫入
ros2 service call /cargo02/write_continuous_data plc_interfaces/srv/WriteContinuousData "{device_type: 'DM', start_address: '7800', values: ['1', '2', '3']}"
```

### 4. 感測器測試
```bash
# 測試 Hokuyo 8bit 光通訊模組
cd /app/agv_ws
python3 test/test_hokuyo_dms_8bit.py

# 檢查感測器配置
cat /app/config/hokuyo_dms_config.yaml
```

### 5. 手動驗證
```python
# 測試狀態機基本功能
import rclpy
from agv_base.agv_node_base import AgvNodebase

rclpy.init()
node = AgvNodebase('test_node')

# 檢查狀態機狀態
print(f"當前狀態: {node.base_context.state.__class__.__name__}")

# 檢查 AGV 狀態
print(f"AGV Auto: {node.agv_status.AGV_Auto}")
print(f"AGV Manual: {node.agv_status.AGV_MANUAL}")

# 檢查 PLC 記憶體
print(f"PLC 記憶體大小: {len(node.dMmemory.data)} bytes")

node.destroy_node()
rclpy.shutdown()
```

## 🔧 配置說明

### Launch 文件配置
```python
# loader_agv/launch/launch.py 主要參數
agv_id = 'cargo02'                    # AGV 識別碼
room_id = int(agv_id[-2:])           # 房間 ID (從 agv_id 提取)
param_file = "/app/config/ecs_config.yaml"  # ECS 配置檔案
agv_command_file = "/app/agv_cmd_service_ws/src/agv_cmd_service/config/agv_cmd_service.yaml"
```

### AGV 節點參數
```yaml
# AGV 核心節點參數
room_id: 2                           # 房間 ID
namespace: cargo02                   # ROS 2 命名空間
```

### PLC 記憶體配置
```python
# PLC 記憶體配置
DM_MEMORY_SIZE = 65536 * 2          # 65536 words (131072 bytes)
READ_CYCLE_MS = 50                  # 讀取週期 50ms
STATUS_PUBLISH_CYCLE = 1.5          # 狀態發布週期 1.5 秒
HEARTBEAT_ADDRESS = 'DM7800'        # 心跳位址
STATE_ADDRESS = 'DM7801'            # 狀態位址
```

### 狀態機配置
```python
# 狀態對應數值
IDLE_STATE = 1                      # 空閒狀態
MANUAL_STATE = 2                    # 手動狀態
AUTO_STATE = 3                      # 自動狀態
ERROR_STATE = 4                     # 錯誤狀態
```

## 🔗 依賴項目

- **ROS 2 Jazzy**: 機器人作業系統框架
- **plc_proxy**: PLC 通訊代理服務
- **db_proxy_interfaces**: 資料庫介面定義
- **joy_linux**: 搖桿控制支援
- **rclpy**: ROS 2 Python 客戶端庫
- **ament_python**: Python 套件建置工具

## 🧪 測試與除錯

### 狀態監控
```bash
# 監控 AGV 狀態
ros2 topic echo /agv/status

# 監控狀態變更
ros2 topic echo /agv/state_change

# 檢查節點狀態
ros2 node list
ros2 node info /cargo02/agv_core_node
```

### 日誌檢查
```bash
# 檢查 ROS 2 日誌
ros2 topic echo /rosout

# 檢查特定命名空間的日誌
ros2 topic echo /cargo02/rosout
```

## 📝 開發指南

### 新增 AGV 類型
1. 複製現有 AGV 套件（如 loader_agv）
2. 修改 `agv_core_node.py` 中的特定邏輯
3. 實現對應的 robot_states
4. 更新 setup.py 和 package.xml
5. 建立對應的 launch 文件

### 新增狀態
1. 在 `agv_base/agv_states/` 或對應 AGV 的 `robot_states/` 中新增狀態類
2. 繼承適當的基礎狀態類
3. 實現 `handle()` 方法
4. 在狀態轉換邏輯中加入新狀態

### 修改 PLC 通訊
1. 更新 `agv_status.py` 中的狀態定義
2. 修改 `agv_node_base.py` 中的讀取邏輯
3. 調整狀態處理邏輯

## 🔧 維護注意事項

1. **狀態機設計**: 確保狀態轉換邏輯清晰，避免死鎖
2. **PLC 通訊**: 注意 PLC 讀寫頻率，避免過載
3. **記憶體管理**: 定期檢查記憶體使用情況
4. **錯誤處理**: 完善異常處理機制，確保系統穩定性
5. **命名空間**: 多 AGV 環境下注意命名空間衝突

## 🔧 故障排除

### 常見問題

#### 1. AGV 節點啟動失敗
**症狀**: `ros2 launch loader_agv launch.py` 啟動失敗
**解決方法**:
```bash
# 檢查依賴工作空間是否建置
ls /app/plc_proxy_ws/install
ls /app/keyence_plc_ws/install

# 檢查配置檔案
ls /app/config/ecs_config.yaml

# 檢查 PLC 服務
ros2 service list | grep plc
```

#### 2. 狀態機無法正常運作
**症狀**: AGV 狀態不會轉換或卡在某個狀態
**解決方法**:
```bash
# 檢查狀態變更事件
ros2 topic echo /agv/state_change

# 檢查 AGV 狀態
ros2 topic echo /agv/status

# 檢查 PLC 連線狀態
ros2 service call /cargo02/read_continuous_byte plc_interfaces/srv/ReadContinuousByte "{device_type: 'DM', start_address: '7800', length: 2}"
```

#### 3. PLC 通訊問題
**症狀**: 無法與 PLC 通訊或讀取資料失敗
**解決方法**:
```bash
# 檢查 PLC 服務狀態
ros2 node info /cargo02/plc_service

# 檢查網路連線
ping 192.168.1.100  # PLC IP 位址

# 重啟 PLC 服務
ros2 lifecycle set /cargo02/plc_service shutdown
ros2 launch loader_agv launch.py
```

#### 4. 記憶體存取錯誤
**症狀**: `IndexError` 或記憶體存取超出範圍
**解決方法**:
```bash
# 檢查記憶體配置
python3 -c "from keyence_plc.keyence_plc_memory import PlcMemory; m = PlcMemory(131072); print(f'記憶體大小: {len(m.data)} bytes')"

# 檢查位址範圍
# DM 位址範圍: 0-65535 (words)
# 確保存取位址在有效範圍內
```

### 除錯工具
```bash
# 檢查所有 AGV 相關節點
ros2 node list | grep agv

# 監控系統資源
top | grep ros2

# 檢查 ROS 2 日誌
ros2 topic echo /rosout

# 檢查節點詳細資訊
ros2 node info /cargo02/agv_core_node

# 檢查參數
ros2 param list /cargo02/agv_core_node
ros2 param get /cargo02/agv_core_node room_id
```

### 日誌和診斷
```bash
# 啟用詳細日誌
export RCUTILS_LOGGING_SEVERITY=DEBUG

# 檢查系統日誌
journalctl -u ros2-agv

# 檢查 AGV 特定日誌
tail -f /tmp/agv.log

# 檢查狀態機日誌
ros2 topic echo /agv/state_change | grep -E "(IdleState|AutoState|ManualState)"
```

## � 車型開發狀態和 ToDo 清單

### 🚗 Loader AGV (完成度: 95% ✅)
**已完成功能**:
- ✅ 完整的狀態機架構
- ✅ PLC 通訊整合
- ✅ 機器人狀態管理
- ✅ 裝載業務邏輯
- ✅ 自動啟動配置
- ✅ 感測器整合 (Hokuyo)

**待完成功能**:
- [ ] **優化裝載精度** (1 週) - 高優先級
- [ ] **完善錯誤恢復機制** (1 週) - 高優先級

### 🚛 Cargo Mover AGV (完成度: 60% 🔄)
**已完成功能**:
- ✅ 基礎 AGV 節點架構
- ✅ 基本狀態機
- ✅ PLC 通訊基礎

**待完成功能**:
- [ ] **實作貨物檢測邏輯** (2 週) - 高優先級
  - 貨物感測器整合
  - 重量檢測機制
  - 貨物狀態管理
- [ ] **開發搬運專用狀態** (2 週) - 高優先級
  - PickupState (拾取狀態)
  - TransportState (運輸狀態)
  - DeliveryState (交付狀態)
- [ ] **建立 launch 配置** (3 天) - 中優先級
- [ ] **完善機器人控制邏輯** (1 週) - 中優先級

### 🏭 Unloader AGV (完成度: 40% ⏳)
**已完成功能**:
- ✅ 基礎節點結構
- ✅ 基本狀態機框架

**待完成功能**:
- [ ] **實作卸載核心邏輯** (3 週) - 高優先級
  - 卸載點定位
  - 貨物卸載機制
  - 卸載完成驗證
- [ ] **開發分揀功能** (3 週) - 高優先級
  - 貨物分類邏輯
  - 分揀路徑規劃
  - 分揀狀態管理
- [ ] **感測器整合** (2 週) - 高優先級
  - 視覺識別系統
  - 重量感測器
  - 位置感測器
- [ ] **建立專用機器人狀態** (2 週) - 中優先級
  - UnloadState (卸載狀態)
  - SortState (分揀狀態)
  - InspectState (檢查狀態)
- [ ] **建立 launch 配置** (3 天) - 中優先級

### 🎯 開發里程碑

#### 第一階段 (4 週內)
- [ ] 完成 Cargo Mover AGV 核心功能 (貨物檢測 + 搬運狀態)
- [ ] 完成 Unloader AGV 基礎卸載功能

#### 第二階段 (8 週內)
- [ ] Cargo Mover AGV 達到生產可用狀態 (80% 完成度)
- [ ] Unloader AGV 完成分揀功能開發

#### 第三階段 (12 週內)
- [ ] 所有車型達到生產可用狀態
- [ ] 建立完整的車型切換和配置系統

### 📊 車型比較表

| 功能 | Loader AGV | Cargo Mover AGV | Unloader AGV |
|------|------------|-----------------|--------------|
| 基礎狀態機 | ✅ 完成 | ✅ 完成 | ✅ 完成 |
| PLC 通訊 | ✅ 完成 | ✅ 完成 | ⚠️ 基礎 |
| 專用業務邏輯 | ✅ 完成 | 🚧 開發中 | ❌ 待開發 |
| 機器人控制 | ✅ 完成 | ⚠️ 部分 | ❌ 待開發 |
| 感測器整合 | ✅ 完成 | ❌ 待開發 | ❌ 待開發 |
| Launch 配置 | ✅ 完成 | ❌ 待開發 | ❌ 待開發 |
| 自動啟動 | ✅ 支援 | ❌ 不支援 | ❌ 不支援 |
| 生產就緒 | ✅ 是 | ❌ 否 | ❌ 否 |

## �🔗 相關文檔

- **plc_proxy_ws**: PLC 通訊代理服務，提供 PLC 讀寫功能
- **keyence_plc_ws**: Keyence PLC 記憶體管理，提供記憶體操作介面
- **agv_cmd_service_ws**: AGV 命令服務，提供手動控制功能
- **path_algorithm**: A* 路徑規劃演算法，提供路徑計算功能
- **db_proxy_ws**: 資料庫代理服務，提供資料存取功能
