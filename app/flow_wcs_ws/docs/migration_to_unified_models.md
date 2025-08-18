# flow_wcs 遷移到統一模型指南

## 背景
flow_wcs 原本使用自定義的資料庫模型，現在要遷移到使用 db_proxy 的統一模型，以便整個系統使用一致的資料結構。

## 影響分析

### 主要影響
1. **資料庫模型定義**: 不再使用 flow_wcs 自己的模型類別
2. **欄位名稱變化**: 某些欄位名稱有所不同
3. **狀態處理**: 需要在狀態名稱和狀態 ID 之間轉換

### 相容性
- ✅ **向後相容**: 所有功能都能保持，只是實作方式改變
- ✅ **資料不會遺失**: 資料庫 schema 是擴充而非替換
- ✅ **Flow Designer 不受影響**: UI 和 API 介面保持不變

## 遷移步驟

### 方案 1：最小修改（推薦）
只修改 `database.py` 的 import 部分，讓它使用 db_proxy 的模型：

```python
# 原本的 import
from sqlalchemy import create_engine, Column, Integer, String, Float, Boolean, DateTime, JSON, ForeignKey
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker, Session

# 改為
import sys
sys.path.append('/app/db_proxy_ws/src/db_proxy')
from db_proxy.models.agvc_location import Location
from db_proxy.models.rack import Rack
from db_proxy.models.agvc_task import Task, Work, FlowLog
from db_proxy.models.agvc_rcs import AGV
from db_proxy.connection_pool_manager import ConnectionPoolManager
```

然後調整 `DatabaseManager` 的方法以適應新模型。

### 方案 2：使用 database_unified.py（已提供）
使用已經創建好的 `database_unified.py`：

```python
# 在 flow_executor.py 中
# 原本：
from .database import db_manager

# 改為：
from .database_unified import db_manager
```

## 主要差異對照表

| flow_wcs 原欄位 | db_proxy 統一模型 | 轉換方式 |
|----------------|------------------|---------|
| `Location.has_rack` | `Location.rack_id` | 判斷 `rack_id` 是否為 NULL |
| `Location.status` | `Location.location_status_id` | 使用狀態 ID 而非字串 |
| `Rack.rack_id` (str) | `Rack.id` (int) | 直接使用整數 ID |
| `Rack.status` | 無此欄位 | 使用 `side_a_status` 和 `side_b_status` |
| `Task.task_metadata` | `Task.parameters` | 欄位名稱不同，功能相同 |
| `Task.status` (str) | `Task.status_id` (int) | 需要狀態名稱與 ID 轉換 |
| `AGV.type` | `AGV.model` | 欄位名稱不同 |
| `AGV.theta` | `AGV.heading` | 欄位名稱不同 |
| `AGV.battery_level` | `AGV.battery` | 欄位名稱不同 |

## 狀態對照表

### Task 狀態
| flow_wcs 狀態名稱 | db_proxy status_id | 常數定義 |
|-------------------|-------------------|----------|
| 'pending' | 1 | TaskStatus.PENDING |
| 'assigned' | 2 | TaskStatus.READY_TO_EXECUTE |
| 'executing' | 3 | TaskStatus.EXECUTING |
| 'completed' | 4 | TaskStatus.COMPLETED |
| 'failed' | 6 | TaskStatus.ERROR |
| 'cancelled' | 54 | TaskStatus.CANCELLED |

### AGV 狀態
| flow_wcs 狀態名稱 | db_proxy status_id |
|-------------------|-------------------|
| 'idle' | 1 |
| 'busy' | 2 |
| 'charging' | 3 |
| 'error' | 4 |

## 新增功能

使用統一模型後，flow_wcs 可以獲得以下新功能：

1. **Rack 雙面管理**：
   ```python
   rack.rotate(180)  # 自動交換兩面狀態
   rack.get_side_status('a')
   rack.set_side_status('b', 'occupied')
   ```

2. **FlowLog 記錄**：
   ```python
   db_manager.log_flow_execution(
       flow_id="FLOW_001",
       flow_name="測試流程",
       section="init",
       step_id="step_1",
       function="query_locations",
       status="success"
   )
   ```

3. **相容性方法**：
   ```python
   # 自動轉換為 flow_wcs 格式
   location.to_flow_wcs_dict()
   task.to_flow_wcs_dict()
   agv.to_flow_wcs_dict()
   ```

## 測試驗證

遷移後需要測試以下功能：

1. **基本查詢功能**：
   ```bash
   # 測試 flow_wcs 的查詢功能
   python3 -c "from flow_wcs.database_unified import db_manager; print(db_manager.query_locations())"
   ```

2. **任務創建和更新**：
   ```python
   task_id = db_manager.create_task(
       type="transport",
       work_id="W001",
       location_id=1,
       priority=100
   )
   db_manager.update_task(task_id, "executing")
   ```

3. **Flow 執行**：
   ```bash
   # 執行一個測試 Flow
   ros2 run flow_wcs flow_wcs_node
   ```

## 回退方案

如果需要回退，只需要：
1. 將 `flow_executor.py` 的 import 改回原本的 `from .database import db_manager`
2. flow_wcs 就會使用原本的模型定義

## 結論

這個遷移是**安全且向後相容**的：
- ✅ 不會破壞現有功能
- ✅ Flow Designer 不受影響
- ✅ 資料庫資料保持完整
- ✅ 可以隨時回退
- ✅ 獲得新功能（雙面架台、FlowLog 等）