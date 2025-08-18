# flow_wcs 和 db_proxy 資料庫整合指南

## 整合概述

本文檔說明如何整合 `flow_wcs` 和 `db_proxy` 兩個資料庫存取系統，建立統一的資料模型和存取介面。

## 整合內容

### 1. 模型擴充

#### Location 模型 (`agvc_location.py`)
- **新增欄位**：
  - `type`: 位置類型 (room_inlet, room_outlet, charging等)
  - `x`, `y`: 座標資訊
  - `rack_id`: 架台關聯 (外鍵)
  - `created_at`, `updated_at`: 時間戳記
- **相容性方法**：
  - `has_rack`: 屬性，判斷是否有架台
  - `status`: 返回狀態名稱字串
  - `to_flow_wcs_dict()`: 轉換為 flow_wcs 格式

#### Rack 模型 (`rack.py`)
- **新增欄位**：
  - `rotation_angle`: 最後旋轉角度
  - `last_rotation`: 最後旋轉時間
  - `side_a_status`, `side_b_status`: 雙面狀態管理
  - `created_at`, `updated_at`: 時間戳記
- **新增方法**：
  - `get_side_status(side)`: 取得指定面狀態
  - `set_side_status(side, status)`: 設定指定面狀態
  - `rotate(angle)`: 旋轉架台並交換兩面狀態
  - `to_flow_wcs_dict()`: 轉換為 flow_wcs 格式

#### Task 模型 (`agvc_task.py`)
- **新增欄位**：
  - `task_id`: 字串任務 ID (唯一索引)
  - `type`: 任務類型
  - `location_id`, `rack_id`: 位置和架台關聯
  - `completed_at`: 完成時間
- **相容性方法**：
  - `status`: 返回狀態名稱字串
  - `metadata`, `task_metadata`: 對應到 parameters
  - `generate_task_id()`: 生成 flow_wcs 格式任務 ID
  - `to_flow_wcs_dict()`: 轉換為 flow_wcs 格式

#### Work 模型 (`agvc_task.py`)
- **新增欄位**：
  - `work_code`: 字串工作代碼 (唯一索引)
  - `work_type`: 工作類型分類
  - `flow_definition`: Linear Flow v2 定義 (JSON)
- **相容性方法**：
  - `work_id`: 返回字串 ID

#### AGV 模型 (`agvc_rcs.py`)
- **新增欄位**：
  - `agv_id`: 字串 AGV ID (唯一索引)
  - `current_location`: 當前位置名稱
  - `current_task_id`: 當前任務 ID
- **相容性方法**：
  - `type`: 返回小寫 model
  - `theta`: 對應到 heading
  - `battery_level`: 對應到 battery
  - `status`: 返回狀態名稱字串
  - `to_flow_wcs_dict()`: 轉換為 flow_wcs 格式

#### FlowLog 模型 (`agvc_task.py`)
- **全新模型**：記錄 Linear Flow 執行日誌
- **欄位**：
  - Flow 識別資訊 (flow_id, flow_name, work_id)
  - 執行細節 (section, step_id, function)
  - 參數和結果 (params, result)
  - 狀態資訊 (status, error_message, duration)

#### Carrier 模型 (`carrier.py`)
- **新增欄位**：
  - `rack_side`: 在架台的哪一面 ('a' 或 'b')

### 2. 資料庫遷移

提供完整的資料遷移腳本：`scripts/migrate_flow_wcs_to_unified.py`

#### 使用方法
```bash
# 在 AGVC 容器內執行
cd /app/db_proxy_ws
python3 scripts/migrate_flow_wcs_to_unified.py

# 測試模式（不實際寫入）
python3 scripts/migrate_flow_wcs_to_unified.py --dry-run
```

#### 遷移內容
- Location 資料：包含座標和狀態轉換
- Rack 資料：包含雙面狀態和旋轉記錄
- Task 資料：包含字串 ID 和參數對應
- AGV 資料：包含狀態和位置資訊
- FlowLog 資料：完整保留執行記錄

### 3. 整合測試

提供完整的整合測試腳本：`scripts/test_unified_integration.py`

#### 使用方法
```bash
# 在 AGVC 容器內執行
cd /app/db_proxy_ws
python3 scripts/test_unified_integration.py
```

#### 測試項目
- Location 整合功能測試
- Rack 雙面管理和旋轉測試
- Task 字串 ID 和相容性測試
- AGV 狀態和位置測試
- FlowLog 記錄測試
- Carrier 側面管理測試

## 使用統一模型

### flow_wcs 使用方式

```python
# 使用 db_proxy 的連線池
from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.models.agvc_location import Location
from db_proxy.models.rack import Rack
from db_proxy.models.agvc_task import Task, Work, FlowLog
from db_proxy.models.agvc_rcs import AGV

# 建立連線池
pool_manager = ConnectionPoolManager("postgresql://agvc:password@192.168.100.254:5432/agvc")

# 使用統一模型
with pool_manager.get_session() as session:
    # 查詢位置
    locations = session.query(Location).filter(
        Location.type == "room_inlet",
        Location.has_rack == False
    ).all()
    
    # 轉換為 flow_wcs 格式
    flow_locations = [loc.to_flow_wcs_dict() for loc in locations]
    
    # 旋轉架台
    rack = session.query(Rack).filter(Rack.id == 1).first()
    if rack:
        rack.rotate(180.0)  # 自動交換兩面狀態
        session.commit()
    
    # 創建任務
    task = Task(
        task_id=Task().generate_task_id("W001"),
        name="Transport Task",
        type="transport",
        priority=100,
        parameters={"from": "A", "to": "B"}
    )
    session.add(task)
    session.commit()
```

### db_proxy ROS 2 服務使用

現有的 ROS 2 服務保持不變，但底層模型已擴充支援 flow_wcs 功能：

```bash
# 查詢服務仍然可用
ros2 service call /carrier_query db_proxy_interfaces/srv/CarrierQuery "query_type: 'get_all'"
ros2 service call /rack_query db_proxy_interfaces/srv/RackQuery "query_type: 'get_all'"
```

## 向後相容性

### flow_wcs 程式碼相容性

現有的 flow_wcs 程式碼可以透過以下方式保持相容：

```python
# flow_wcs 原始使用方式
from flow_wcs.database import DatabaseManager

# 改為使用統一模型
from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.models.agvc_location import Location

class DatabaseManager:
    def __init__(self):
        self.pool_manager = ConnectionPoolManager(DATABASE_URL)
    
    def query_locations(self, type=None, rooms=None, has_rack=None):
        with self.pool_manager.get_session() as session:
            query = session.query(Location)
            if type:
                query = query.filter(Location.type == type)
            if rooms:
                query = query.filter(Location.room_id.in_(rooms))
            if has_rack is not None:
                # 使用 rack_id 判斷
                if has_rack:
                    query = query.filter(Location.rack_id.isnot(None))
                else:
                    query = query.filter(Location.rack_id.is_(None))
            
            # 轉換為 flow_wcs 格式
            return [loc.to_flow_wcs_dict() for loc in query.all()]
```

## 注意事項

1. **資料庫 Schema 變更**：需要執行資料庫遷移來新增欄位
2. **索引建立**：`task_id`, `agv_id`, `work_code` 需要建立唯一索引
3. **外鍵關係**：新增的外鍵需要確保參照完整性
4. **時間戳記**：所有時間戳記使用 UTC 時區

## 後續優化建議

1. **效能優化**：
   - 為常用查詢建立複合索引
   - 優化 hybrid_property 的查詢效能
   
2. **資料驗證**：
   - 加強模型層的資料驗證
   - 實作業務邏輯驗證
   
3. **監控告警**：
   - 監控資料庫連線池使用率
   - 設定慢查詢告警

## 整合效益

1. **統一資料模型**：消除重複定義，維護單一資料來源
2. **共用連線池**：提升資源使用效率
3. **向後相容**：保持現有 API 不變
4. **功能擴充**：整合雙方優勢功能
5. **簡化維護**：減少程式碼重複，提升可維護性