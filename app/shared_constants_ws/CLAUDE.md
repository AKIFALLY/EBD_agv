# shared_constants_ws - 共享常數工作空間

## 📚 Context Loading
../CLAUDE.md  # 引用根目錄系統文档

## 📋 工作空間概述

**共享常數工作空間** 提供 AGV 和 AGVC 兩個環境都能使用的共享常數定義，解決跨容器依賴問題。

### 核心特色
- **跨環境共享**: AGV 和 AGVC 環境都能編譯和使用
- **任務狀態統一**: 提供統一的 TaskStatus 常數定義
- **架構清晰**: 專門的共享常數套件，職責明確
- **易於擴展**: 未來其他共享常數可以加入此套件

## 📂 專案結構

```
shared_constants_ws/
├── src/
│   └── shared_constants/              # 共享常數套件
│       ├── shared_constants/          # Python 模組
│       │   ├── __init__.py
│       │   └── task_status.py         # TaskStatus 常數定義
│       ├── package.xml                # ROS 2 套件配置
│       ├── setup.py                   # Python 套件設定
│       ├── setup.cfg                  # 設定檔
│       └── resource/shared_constants  # ROS 2 資源標記
├── CLAUDE.md                          # 模組文檔
└── README.md                          # 基本說明
```

## 🚀 TaskStatus 常數

### 主要狀態
- `REQUESTING = 0` - 請求中 (UI-請求執行任務)
- `PENDING = 1` - 待處理 (WCS-任務已接受，待處理)
- `READY_TO_EXECUTE = 2` - 待執行 (RCS-任務已派發，待執行)
- `EXECUTING = 3` - 執行中 (AGV-任務正在執行)
- `COMPLETED = 4` - 已完成 (AGV-任務已完成)
- `CANCELLING = 5` - 取消中 (任務取消)
- `ERROR = 6` - 錯誤 (錯誤)

### 取消相關狀態
- `WCS_CANCELLING = 51` - WCS-取消中
- `RCS_CANCELLING = 52` - RCS-取消中
- `AGV_CANCELLING = 53` - AGV-取消中
- `CANCELLED = 54` - 已取消

## 🚀 WorkIds 常數

### KUKA 支援的工作 ID
- `KUKA_MOVE = 210001` - KUKA 移動
- `KUKA_RACK_MOVE = 220001` - KUKA 移動貨架
- `KUKA_WORKFLOW = 230001` - KUKA template 流程任務

### 其他工作 ID
- `OPUI_CALL_EMPTY = 100001` - OPUI 叫空車
- `CT_AGV_WORK = 2000102` - CT AGV 工作

### KUKA API 映射
- `KUKA_MOVE` → `move` (對應 kuka_fleet.move())
- `KUKA_RACK_MOVE` → `rack_move` (對應 kuka_fleet.rack_move())
- `KUKA_WORKFLOW` → `workflow` (對應 kuka_fleet.workflow())

## 🔧 使用方式

### 導入 TaskStatus
```python
from shared_constants.task_status import TaskStatus

# 使用狀態常數
if task.status_id == TaskStatus.PENDING:
    print("任務待處理")

# 取得狀態描述
description = TaskStatus.get_description(task.status_id)
name = TaskStatus.get_name(task.status_id)

# 檢查狀態有效性
if TaskStatus.is_valid_status(status_code):
    print("有效的狀態碼")
```

### 導入 WorkIds
```python
from shared_constants.work_ids import WorkIds

# 使用工作 ID 常數
if task.work_id == WorkIds.KUKA_MOVE:
    print("KUKA 移動任務")

# 檢查 KUKA 支援
if WorkIds.is_kuka_supported(task.work_id):
    api_type = WorkIds.get_kuka_api_type(task.work_id)
    print(f"KUKA API 類型: {api_type}")

# 取得描述
description = WorkIds.get_description(task.work_id)
print(f"工作描述: {description}")

# 取得 KUKA 支援的所有工作 ID
kuka_ids = WorkIds.get_kuka_work_ids()
print(f"KUKA 支援: {kuka_ids}")
```

### TaskStatus 實用方法
- `get_description(status_code)` - 取得中文描述
- `get_name(status_code)` - 取得英文名稱
- `is_valid_status(status_code)` - 檢查狀態有效性
- `get_all_statuses()` - 取得所有狀態
- `get_main_statuses()` - 取得主要狀態 (0-6)
- `get_cancel_statuses()` - 取得取消狀態 (50+)

### WorkIds 實用方法
- `get_description(work_id)` - 取得工作 ID 中文描述
- `get_name(work_id)` - 取得工作 ID 英文名稱
- `is_kuka_supported(work_id)` - 檢查是否為 KUKA 支援的工作 ID
- `get_kuka_api_type(work_id)` - 取得 KUKA API 類型
- `get_all_work_ids()` - 取得所有工作 ID
- `get_kuka_work_ids()` - 取得 KUKA 支援的工作 ID 列表

## 🚀 開發環境

### 建置套件
```bash
# 進入工作空間
cd /app/shared_constants_ws

# 建置套件
colcon build --packages-select shared_constants

# 載入環境
source install/setup.bash
```

### 測試導入
```python
# 測試 TaskStatus 導入
python3 -c "from shared_constants.task_status import TaskStatus; print('TaskStatus 導入成功')"

# 測試 WorkIds 導入
python3 -c "from shared_constants.work_ids import WorkIds; print('WorkIds 導入成功')"

# 測試功能
python3 -c "
from shared_constants.task_status import TaskStatus
from shared_constants.work_ids import WorkIds
print(f'PENDING = {TaskStatus.PENDING}')
print(f'描述: {TaskStatus.get_description(TaskStatus.PENDING)}')
print(f'KUKA_MOVE = {WorkIds.KUKA_MOVE}')
print(f'描述: {WorkIds.get_description(WorkIds.KUKA_MOVE)}')
"
```

## 🔗 整合點

### AGV 環境使用
- `agv_base/agv_states/mission_select_state.py` - 任務選擇狀態機 (TaskStatus)

### AGVC 環境使用
- `rcs_ws/simple_kuka_manager.py` - KUKA 任務管理和派發 (TaskStatus + WorkIds)
- `rcs_ws/simple_ct_manager.py` - CT AGV 任務管理 (TaskStatus)
- `rcs_ws/test/test_rcs_pytest.py` - RCS 系統測試 (TaskStatus + WorkIds)
- `web_api_ws/opui/core/op_ui_socket.py` - OPUI Socket.IO 通訊 (TaskStatus)
- `web_api_ws/opui/services/opui_task_service.py` - OPUI 任務服務 (TaskStatus)
- `db_proxy_ws` - 可選擇性使用 (TaskStatus)

## 📅 開發時間線

- **2025-08-04**: 建立 shared_constants_ws 工作空間
  - 解決跨容器依賴問題
  - 提供統一的 TaskStatus 和 WorkIds 常數定義
  - 支援 AGV 和 AGVC 環境共享使用

## 💡 設計原則

1. **簡單純粹**: 只包含常數定義，不依賴複雜模組
2. **向前相容**: 與原有 TaskStatus 完全相容
3. **跨環境**: AGV 和 AGVC 都能編譯使用
4. **易於維護**: 集中管理所有狀態常數

## 🔗 交叉引用
- 系統概覽: @docs-ai/context/system/rosagv-overview.md
- 雙環境架構: @docs-ai/context/system/dual-environment.md
- AGV 工作空間: @docs-ai/context/workspaces/agv-workspaces.md
- AGVC 工作空間: @docs-ai/context/workspaces/agvc-workspaces.md