# 共享常數工作空間 (shared_constants_ws)

## 📋 基本資訊

**啟動狀態**: ✅ 自動載入 (AGV 和 AGVC 環境都自動載入)
**運行環境**: 🚗🖥️ 共用 (AGV 車載系統 + AGVC 管理系統)
**主要功能**: 跨環境共享常數定義和狀態碼統一管理
**依賴狀態**: 無外部依賴，純 Python 定義
**實作狀態**: 完整實作，提供統一的常數管理

## 📋 專案概述

共享常數工作空間提供 AGV 和 AGVC 兩個環境都能使用的統一常數定義，解決跨容器依賴問題並確保系統狀態的一致性。該工作空間作為 RosAGV 系統的基礎定義層，提供任務狀態、工作 ID、錯誤碼等關鍵常數的統一管理。

此工作空間採用純 Python 實作，無需任何外部依賴，確保在任何環境下都能正確載入和使用。系統提供完整的任務狀態生命週期定義（從 REQUESTING 到 COMPLETED/ERROR）、工作類型識別碼（支援各種 AGV 操作類型）、以及狀態管理工具函數。工作空間的設計確保了跨環境的一致性，避免了因狀態定義不一致導致的系統錯誤。

**⚠️ 重要說明**: 此工作空間的任何修改都會同時影響 AGV 和 AGVC 環境，請謹慎修改並確保向後相容性。

## 🔗 依賴關係

### 系統套件依賴
- **Python 標準庫**: `enum` (列舉定義)、`typing` (型別提示)
- **ROS 2 Jazzy**: 基礎套件支援（無特殊依賴）

### 被依賴的工作空間
- **agv_ws**: AGV 核心系統 - 使用 `TaskStatus` 進行任務狀態管理
- **agvc_ws**: AGVC 管理系統 - 使用 `TaskStatus` 同步任務狀態
- **db_proxy_ws**: 資料庫代理 - 使用 `TaskStatus` 儲存任務狀態
- **web_api_ws**: Web API - 使用 `TaskStatus` 和 `WorkIds` 進行任務管理
- **rcs_ws**: 機器人控制系統 - 使用狀態常數進行任務調度
- **tafl_wcs_ws**: WCS 系統 - 使用工作 ID 進行任務分派

### 外部依賴
- 無外部依賴，完全自包含

## 🏗️ 專案結構

```
shared_constants_ws/
├── src/
│   └── shared_constants/           # 共享常數套件 (完整實作)
│       ├── shared_constants/       # 核心功能模組
│       │   ├── __init__.py         # 套件初始化和匯出
│       │   ├── task_status.py     # TaskStatus 任務狀態定義 ✅
│       │   └── work_ids.py        # WorkIds 工作類型定義 ✅
│       ├── resource/               # ROS 2 資源目錄
│       │   └── shared_constants   # ROS 2 資源標記檔案
│       ├── package.xml             # ROS 2 套件描述文件
│       ├── setup.py                # Python 套件設定
│       └── setup.cfg               # 套件安裝配置
├── CLAUDE.md                       # AI Agent 專用指導文件
└── README.md                       # 本檔案 (專案說明文件)
```

## ⚙️ 主要功能

### 1. 任務狀態管理 (TaskStatus)
**完整的任務生命週期狀態定義**：
- **請求階段**: `REQUESTING (0)` - UI 請求執行任務
- **待處理階段**: `PENDING (1)` - WCS 任務已接受，待處理
- **準備執行階段**: `READY_TO_EXECUTE (2)` - RCS 任務已派發，待執行
- **執行階段**: `EXECUTING (3)` - AGV 任務正在執行
- **完成階段**: `COMPLETED (4)` - AGV 任務已完成
- **取消流程**: `CANCELLING (5)` - 任務取消中
- **錯誤狀態**: `ERROR (6)` - 任務執行錯誤

**細分取消狀態**：
- **WCS 層取消**: `WCS_CANCELLING (51)` - WCS 層級取消處理
- **RCS 層取消**: `RCS_CANCELLING (52)` - RCS 層級取消處理
- **AGV 層取消**: `AGV_CANCELLING (53)` - AGV 層級取消處理
- **最終取消**: `CANCELLED (54)` - 任務已取消

### 2. 工作類型識別 (WorkIds)
**AGV 操作類型定義**：
- **基礎移動**: `IDLE (0)`、`MOVE_TO_CHARGER (1)`、`CHARGING (2)`
- **物料搬運**: `PICK_FROM_RACK (10)`、`PLACE_TO_RACK (11)`
- **特殊操作**: `ROTATE_RACK (20)`、`TRANSFER_MATERIAL (30)`
- **系統維護**: `EMERGENCY_STOP (99)`、`MAINTENANCE_MODE (100)`

### 3. 狀態工具函數
```python
# 狀態查詢函數
def is_terminal_status(status: int) -> bool:
    """檢查是否為終止狀態"""
    return status in [TaskStatus.COMPLETED, TaskStatus.ERROR, TaskStatus.CANCELLED]

def is_active_status(status: int) -> bool:
    """檢查是否為活動狀態"""
    return status in [TaskStatus.EXECUTING, TaskStatus.CANCELLING]

def get_status_name(status: int) -> str:
    """獲取狀態名稱"""
    return TaskStatus.get_name(status)
```

## 🔧 核心 API

### TaskStatus 類別
```python
from shared_constants.task_status import TaskStatus

# 使用狀態常數
if task.status_id == TaskStatus.PENDING:
    print("任務待處理")
elif task.status_id == TaskStatus.EXECUTING:
    print("任務執行中")
elif task.status_id == TaskStatus.COMPLETED:
    print("任務已完成")

# 狀態轉換邏輯
def update_task_status(task, new_status):
    valid_transitions = {
        TaskStatus.REQUESTING: [TaskStatus.PENDING, TaskStatus.ERROR],
        TaskStatus.PENDING: [TaskStatus.READY_TO_EXECUTE, TaskStatus.CANCELLING],
        TaskStatus.READY_TO_EXECUTE: [TaskStatus.EXECUTING, TaskStatus.CANCELLING],
        TaskStatus.EXECUTING: [TaskStatus.COMPLETED, TaskStatus.ERROR, TaskStatus.CANCELLING],
    }

    if new_status in valid_transitions.get(task.status_id, []):
        task.status_id = new_status
        return True
    return False
```

### WorkIds 類別
```python
from shared_constants.work_ids import WorkIds

# 使用工作 ID
if work.work_id == WorkIds.PICK_FROM_RACK:
    print("執行取料操作")
elif work.work_id == WorkIds.PLACE_TO_RACK:
    print("執行放料操作")

# 工作類型分類
def get_work_category(work_id):
    if work_id in [WorkIds.IDLE, WorkIds.MOVE_TO_CHARGER, WorkIds.CHARGING]:
        return "基礎操作"
    elif work_id in [WorkIds.PICK_FROM_RACK, WorkIds.PLACE_TO_RACK]:
        return "物料搬運"
    elif work_id in [WorkIds.ROTATE_RACK, WorkIds.TRANSFER_MATERIAL]:
        return "特殊操作"
    else:
        return "系統操作"
```

## 🚀 使用方法

### 1. 建置工作空間
```bash
# 載入 ROS 2 環境並建置
source /opt/ros/jazzy/setup.bash
source /opt/ws_rmw_zenoh/install/setup.bash
cd /app/shared_constants_ws
colcon build
source install/setup.bash
```

### 2. 在 Python 中使用
```python
# 匯入任務狀態
from shared_constants.task_status import TaskStatus

# 匯入工作 ID
from shared_constants.work_ids import WorkIds

# 使用範例
status = TaskStatus.PENDING
work_id = WorkIds.PICK_FROM_RACK

print(f"任務狀態: {status}")
print(f"工作類型: {work_id}")
```

### 3. 在 ROS 2 節點中使用
```python
import rclpy
from rclpy.node import Node
from shared_constants.task_status import TaskStatus
from shared_constants.work_ids import WorkIds

class TaskManagerNode(Node):
    def __init__(self):
        super().__init__('task_manager')
        self.current_status = TaskStatus.PENDING
        self.current_work = WorkIds.IDLE

    def update_status(self, new_status):
        self.get_logger().info(f'狀態更新: {self.current_status} -> {new_status}')
        self.current_status = new_status
```

### 4. 跨環境同步使用
```python
# 在 AGVC 環境中發送狀態
from shared_constants.task_status import TaskStatus
task_msg.status = TaskStatus.EXECUTING

# 在 AGV 環境中接收狀態
from shared_constants.task_status import TaskStatus
if received_msg.status == TaskStatus.EXECUTING:
    # 處理執行狀態
```

## ⚙️ 配置說明

### 狀態碼規範
```python
# 主狀態碼 (0-49)
MAIN_STATUS_RANGE = range(0, 50)

# 取消狀態碼 (50-99)
CANCEL_STATUS_RANGE = range(50, 100)

# 錯誤狀態碼 (100-199)
ERROR_STATUS_RANGE = range(100, 200)

# 預留擴展 (200+)
RESERVED_RANGE = range(200, 1000)
```

### 工作 ID 規範
```python
# 基礎操作 (0-9)
BASIC_OPERATIONS = range(0, 10)

# 物料搬運 (10-29)
MATERIAL_HANDLING = range(10, 30)

# 特殊操作 (30-49)
SPECIAL_OPERATIONS = range(30, 50)

# 系統操作 (50-99)
SYSTEM_OPERATIONS = range(50, 100)

# 預留擴展 (100+)
RESERVED_OPERATIONS = range(100, 1000)
```

## 🧪 測試方法

### 1. 建置和測試
```bash
# 建置工作空間
cd /app/shared_constants_ws
colcon build
source install/setup.bash

# 執行測試
colcon test
colcon test-result --verbose
```

### 2. 模組載入測試
```bash
# 測試 TaskStatus 載入
python3 -c "
from shared_constants.task_status import TaskStatus
print(f'✅ TaskStatus 載入成功')
print(f'PENDING = {TaskStatus.PENDING}')
print(f'EXECUTING = {TaskStatus.EXECUTING}')
print(f'COMPLETED = {TaskStatus.COMPLETED}')
"

# 測試 WorkIds 載入
python3 -c "
from shared_constants.work_ids import WorkIds
print(f'✅ WorkIds 載入成功')
print(f'PICK_FROM_RACK = {WorkIds.PICK_FROM_RACK}')
print(f'PLACE_TO_RACK = {WorkIds.PLACE_TO_RACK}')
"
```

### 3. 狀態轉換測試
```python
# 測試狀態轉換邏輯
from shared_constants.task_status import TaskStatus

# 模擬任務狀態流程
status_flow = [
    TaskStatus.REQUESTING,
    TaskStatus.PENDING,
    TaskStatus.READY_TO_EXECUTE,
    TaskStatus.EXECUTING,
    TaskStatus.COMPLETED
]

for i, status in enumerate(status_flow):
    print(f"步驟 {i+1}: 狀態 = {status}")

# 測試取消流程
cancel_flow = [
    TaskStatus.EXECUTING,
    TaskStatus.CANCELLING,
    TaskStatus.AGV_CANCELLING,
    TaskStatus.CANCELLED
]

for status in cancel_flow:
    print(f"取消流程: 狀態 = {status}")
```

### 4. 跨環境一致性測試
```bash
# 在 AGV 容器中測試
docker compose -f docker-compose.yml exec rosagv bash -c "
source /app/setup.bash && agv_source
python3 -c 'from shared_constants.task_status import TaskStatus; print(TaskStatus.PENDING)'
"

# 在 AGVC 容器中測試
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "
source /app/setup.bash && agvc_source
python3 -c 'from shared_constants.task_status import TaskStatus; print(TaskStatus.PENDING)'
"

# 確認兩個環境的值一致
```

## 🔧 故障排除

### 常見問題

#### 1. 模組載入失敗
**症狀**: `ModuleNotFoundError: No module named 'shared_constants'`
**解決方法**:
```bash
# 確認工作空間已建置
cd /app/shared_constants_ws
colcon build

# 確認環境已載入
source install/setup.bash

# 驗證 Python 路徑
python3 -c "import sys; print('\\n'.join(sys.path))"
```

#### 2. 狀態值不一致
**症狀**: AGV 和 AGVC 環境中的狀態值不同
**解決方法**:
```bash
# 重新建置兩個環境的工作空間
# AGV 環境
cd /app && agv_source && colcon build --packages-select shared_constants

# AGVC 環境
cd /app && agvc_source && colcon build --packages-select shared_constants

# 重啟服務確保載入最新版本
```

#### 3. 新增常數後無法使用
**症狀**: 新增的常數在其他模組中無法存取
**解決方法**:
```python
# 1. 確認常數已正確定義
# shared_constants/new_constants.py
class NewConstants:
    NEW_VALUE = 100

# 2. 在 __init__.py 中匯出
# shared_constants/__init__.py
from .new_constants import NewConstants

# 3. 重新建置並載入
colcon build
source install/setup.bash
```

## 📊 擴展指南

### 新增狀態常數
```python
# 在 task_status.py 中新增
class TaskStatus:
    # 現有狀態...

    # 新增狀態（使用適當的數值範圍）
    PAUSED = 7  # 暫停狀態
    RESUMING = 8  # 恢復中

    @classmethod
    def is_pauseable(cls, status):
        """檢查是否可暫停"""
        return status in [cls.EXECUTING]
```

### 新增工作類型
```python
# 在 work_ids.py 中新增
class WorkIds:
    # 現有工作類型...

    # 新增工作類型（遵循分類規範）
    SCAN_BARCODE = 31  # 條碼掃描（特殊操作類）
    WEIGHT_CHECK = 32  # 重量檢查（特殊操作類）

    @classmethod
    def requires_sensor(cls, work_id):
        """檢查是否需要感測器"""
        return work_id in [cls.SCAN_BARCODE, cls.WEIGHT_CHECK]
```

### 版本相容性維護
```python
# 保持向後相容性的方式
class TaskStatus:
    # 保留舊名稱作為別名
    RUNNING = EXECUTING  # 向後相容別名

    @classmethod
    def migrate_status(cls, old_status):
        """遷移舊狀態值到新狀態"""
        migration_map = {
            100: cls.ERROR,  # 舊錯誤碼映射
            200: cls.CANCELLED,  # 舊取消碼映射
        }
        return migration_map.get(old_status, old_status)
```

## 🔧 維護注意事項

1. **版本控制**: 任何修改都需要考慮向後相容性
2. **同步更新**: 修改後需要在所有環境中重新建置
3. **文檔更新**: 新增常數需要同步更新文檔
4. **測試覆蓋**: 確保新增的常數有對應的測試
5. **通知相關團隊**: 修改會影響多個工作空間，需要通知相關開發者

## 🔗 相關文檔

- **agv_ws**: AGV 核心系統，主要使用 TaskStatus
- **web_api_ws**: Web API，使用所有共享常數
- **db_proxy_ws**: 資料庫代理，儲存狀態資訊
- **rcs_ws**: 機器人控制系統，使用狀態進行調度
- **tafl_wcs_ws**: WCS 系統，使用工作 ID 進行任務管理