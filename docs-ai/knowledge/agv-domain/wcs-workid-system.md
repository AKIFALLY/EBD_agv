# WCS Work ID 分類系統

## 🎯 適用場景
- 理解 RosAGV 系統中 Work ID 的分類和用途
- 掌握不同 Work ID 對應的任務參數格式
- 為任務創建和調度提供參考規範

## ⚠️ 重要說明
本文檔包含兩類內容：
- **✅ 實際實作**：已在程式碼中實現的功能（如 `shared_constants.work_ids.WorkIds`）
- **⚠️ 設計參考**：用於理解業務流程的設計概念（如業務流程字典映射）

請參閱文檔末尾的「[文檔說明總結](#📝-文檔說明總結)」章節了解詳細分類。

## 📋 Work ID 分類系統概覽

Work ID 是 RosAGV 系統中任務分類的核心識別碼，每個 Work ID 對應特定的任務類型和執行方式。

### 主要分類

```
Work ID 分類體系
├── 100xxx: OPUI 操作員任務
├── 210xxx: KUKA 基礎移動任務
├── 220xxx: KUKA 貨架搬運任務
├── 230xxx: KUKA 流程觸發任務
├── 2000xxx: CargoAGV 專用任務
├── 2010xxx: LoaderAGV 專用任務
├── 2050xxx: UnloaderAGV 專用任務
└── 2060xxx: 烤箱相關任務
```

## 🔧 核心 Work ID 定義

### OPUI 操作員任務 (100xxx)

#### 100001: opui-call-empty (叫空車)

**狀態**: 🛑 **已棄用** - 此功能已改為人工作業

**歷史記錄**:
- 原設計：AGV 自動從料架儲存區運送空料架至工作區
- 現行方式：作業員手動搬運空料架，透過 OPUI 「加入料架」功能登記

**原任務參數格式** (僅供參考):
```json
{
  "work_id": 100001,
  "function": "rack_move",
  "api": "submit_mission",
  "missionType": "RACK_MOVE",
  "model": "KUKA400i",
  "task_category": "opui_call_empty",
  "priority_level": 40,
  
  // OPUI 特定參數
  "task_type": "call_empty",
  "machine_id": 1,              // 機台ID
  "space_num": 1,               // 停車格編號 (1 或 2)
  "node_id": 95,                // 停車格對應的節點ID
  "client_id": "clientId",      // OPUI 客戶端ID
  
  // KUKA 參數
  "nodes": [91, 76, 95],        // 移動路徑
  "kuka_agv_id": 123,
  
  // 停車格狀態管理
  "parking_space_status": 1     // 設置為任務進行中
}
```

#### 100002: opui-dispatch-full (派滿車)

**狀態**: ✅ **已實作**

**用途**: 作業員從 OPUI 請求將 Rack 派至系統準備派車區

**任務參數格式**:
```json
{
  "work_id": 100002,
  "function": "rack_move",
  "api": "submit_mission",
  "missionType": "RACK_MOVE",
  "model": "KUKA400i",
  "task_category": "opui_dispatch_full",
  "priority_level": 40,
  
  // OPUI 特定參數
  "task_type": "dispatch_full",
  "rack_id": 1,                 // 料架ID
  "room_id": 2,                 // 目標房間ID
  "machine_id": 1,              // 機台ID
  "side": "left",               // 停車格側別（目前直接對應停車格）
  "client_id": "clientId",
  
  // 產品資訊
  "product_name": "ABC121345",
  "count": 32,
  
  // KUKA 參數
  "nodes": [95, 74, 72, 15],    // 移動路徑
  "kuka_agv_id": 123,
  
  // WCS 決策參數
  "node_id": null,              // 由 WCS 決定
  "target_area": "system_prep_area"
}
```

### KUKA 基礎任務 (2x0xxx)

#### 210001: kuka-移動
**用途**: 執行指定的 from,to(nodes) 移動至指定位置

**任務參數格式**:
```json
{
  "function": "move",
  "model": "KUKA400i",
  "work_id": 210001,
  "api": "submit_mission",
  "missionType": "MOVE",
  "nodes": [75, 74, 72, 75],
  "task_category": "basic_movement",
  "priority_level": 60,
  "description": "執行移動(參數提供2個以上node)依序經過所有nodes"
}
```

#### 220001: kuka-移動貨架
**用途**: 執行指定的 from,to(nodes) 將貨架搬至指定位置 (WCS 七大業務流程主要使用)

**任務參數格式**:
```json
{
  "function": "rack_move",
  "model": "KUKA400i",
  "work_id": 220001,
  "api": "submit_mission",
  "missionType": "RACK_MOVE",
  "nodes": [91, 76],
  "rack_id": 123,
  "task_category": "rack_transport",
  "priority_level": 80,
  "source_location": 91,
  "target_location": 76,
  "room_id": 2
}
```

**AGV 旋轉特殊用法**:
```json
{
  "function": "rack_move",
  "model": "KUKA400i",
  "work_id": 220001,
  "api": "submit_mission",
  "missionType": "RACK_MOVE",
  "nodes": [75, 76, 75],        // 3個節點的旋轉路徑
  "rack_id": 123,
  "task_category": "rotation",
  "priority_level": 100,
  "location_type": "inlet",
  "room_id": 2,
  "agv_id": 5,
  "parent_task_id": 456,
  "description": "AGV在房間入口/出口執行旋轉動作"
}
```

#### 230001: kuka-流程觸發

**狀態**: 🛑 **已棄用** - 此功能已改為人工手動管理（2025-09）

**歷史記錄**:
- 原設計：AGV 自動搬運空料架從人工回收區 (91-92) 到空料架回收區 (51-54)
- 現行方式：人工手動管理，透過 OPUI-HMI「移出系統」+ OPUI「加入料架」

**原任務參數格式** (僅供參考):
```json
{
  "function": "workflow",
  "model": "KUKA400i",
  "work_id": 230001,
  "api": "submit_mission",
  "missionType": "MOVE",
  "templateCode": "W000000001",
  "task_category": "manual_empty_recycling",
  "priority_level": 40,
  "source_location": 91,
  "target_location": 51,
  "description": "人工回收空料架區搬運到系統空料架區（已停用）"
}
```

### CargoAGV 專用任務 (2000xxx)

#### 2000102: CargoAGV放入口傳送箱
**用途**: 從料架拿 carrier 到入口傳送箱放

```json
{
  "function": "rack_move",
  "model": "KUKA400i",
  "work_id": 2000102,
  "api": "submit_mission",
  "missionType": "RACK_MOVE",
  "nodes": [],
  "task_category": "cargo_inlet",
  "priority_level": 80,
  "room_id": 2,
  "description": "從料架拿carrier到入口傳送箱放"
}
```

#### 2000201: CargoAGV拿出口傳送箱
**用途**: 從出口傳送箱拿 carrier 到料架放

```json
{
  "function": "rack_move",
  "model": "KUKA400i",
  "work_id": 2000201,
  "api": "submit_mission",
  "missionType": "RACK_MOVE",
  "nodes": [],
  "task_category": "cargo_outlet",
  "priority_level": 80,
  "room_id": 2,
  "description": "從出口傳送箱拿carrier到料架放"
}
```

## 🔄 Work ID 實際使用方式

### 業務流程與 Work ID 對應表
以下表格說明各業務流程實際使用的 Work ID：

| 業務流程 | Work ID | 說明 | 狀態 |
|---------|---------|------|------|
| **KUKA 料架搬運任務** |
| AGV旋轉檢查 | 220001 | Rack 180度轉向（房間入口/出口） | ✅ 已實作 |
| 系統準備區到房間 | 220001 | 投料調度流程 | ✅ 已實作 |
| 滿料架到人工收料區 | 220001 | 製程完成後搬運 | ❌ 待實作 |
| 空料架搬運 | 220001 | 雙決策路徑調度 | ❌ 待實作 |
| NG料架回收 | 220001 | 不良品處理 | ❌ 待實作 |
| **OPUI 操作員任務** |
| OPUI派滿車 | 100002 | 射出機滿料架派送 | ✅ 已實作 |
| ~~OPUI叫空車~~ | ~~100001~~ | 🛑 已棄用 - 改為人工搬運 | - |
| **CT AGV 任務** |
| CargoAGV入口作業 | 2000102 | 從Rack卸載到入口傳送箱 | ✅ 已實作 |
| CargoAGV出口作業 | 2000201 | 從出口傳送箱裝載到Rack | ✅ 已實作 |
| **已移除流程** |
| ~~人工收料區搬運~~ | - | 🛑 已確認移除 - 全由人工處理 | - |
| ~~人工回收空料架~~ | ~~230001~~ | 🛑 已棄用 - 改為手動管理 | - |

**關鍵觀察**：
- 大部分 KUKA 料架搬運流程都使用 **220001** (KUKA_RACK_MOVE)
- 系統透過任務參數中的 `nodes`、`rack_id`、`room_id` 等欄位區分不同業務場景
- 不同業務流程共用同一個 work_id，由 TAFL 流程控制不同的業務邏輯

### 實際程式碼實作 (shared_constants/work_ids.py)
```python
# ✅ 實際使用的 Work ID 定義 - 位於 shared_constants_ws
from shared_constants.work_ids import WorkIds

class WorkIds:
    """實際程式碼中使用的 Work ID 常數"""

    # KUKA 支援的工作 ID
    KUKA_MOVE = 210001          # KUKA 移動
    KUKA_RACK_MOVE = 220001     # KUKA 移動貨架 (主要使用)
    KUKA_WORKFLOW = 230001      # KUKA template 流程任務

    # KUKA 支援的工作 ID 列表
    KUKA_SUPPORTED_WORK_IDS = [210001, 220001, 230001]

    # 其他工作 ID
    OPUI_CALL_EMPTY = 100001    # OPUI 叫空車 (已棄用)
    CT_AGV_WORK = 2000102       # CT AGV 工作
```

### RCS 系統使用方式 (simple_kuka_manager.py)
```python
# ✅ RCS 根據 WorkIds 常數進行任務過濾和路由
from shared_constants.work_ids import WorkIds

# 1. 查詢任務時過濾支援的 Work ID
kuka_tasks = session.exec(
    select(Task).where(
        Task.status_id == TaskStatus.PENDING,
        Task.work_id.in_(WorkIds.KUKA_SUPPORTED_WORK_IDS)  # 只選擇 210001, 220001, 230001
    )
).all()

# 2. 根據 work_id 路由到對應的 KUKA API
if task.work_id == WorkIds.KUKA_MOVE:           # 210001
    result = self.kuka_fleet.move(nodes, agv_id, mission_code)
elif task.work_id == WorkIds.KUKA_RACK_MOVE:    # 220001
    result = self.kuka_fleet.rack_move(nodes, agv_id, mission_code)
elif task.work_id == WorkIds.KUKA_WORKFLOW:     # 230001
    result = self.kuka_fleet.workflow(template_code, agv_id, mission_code)
```

### TAFL 流程使用方式
```yaml
# ✅ TAFL 流程直接在 YAML 中定義 work_id 數字
# 範例：rack_rotation_room_outlet_afull_bempty.yaml
metadata:
  id: rack_rotation_room_outlet_afull_bempty
  name: 房間出口架台翻轉（A面滿B面空）

variables:
  work_id: 220001      # 直接使用數字，對應 KUKA_RACK_MOVE
  priority: 5
  model: KUKA400i

flow:
  - create:
      target: task
      with:
        work_id: ${work_id}  # 創建任務時使用此 work_id
        priority: ${priority}
```

## 🏗️ OPUI 整合邏輯

### Machine → Location → Node ID 對應關係
```python
# 機台完整 Location 配置 (每個射出機有 6 個 location)
MACHINE_LOCATION_CONFIG = {
    1: {  # 射出機1
        "operator_1": {
            "work_area_a": 91,      # 作業員 1 工作區 A
            "work_area_b": 92,      # 作業員 1 工作區 B
            "parking_space": 95     # 作業員 1 停車格
        },
        "operator_2": {
            "work_area_a": 93,      # 作業員 2 工作區 A
            "work_area_b": 94,      # 作業員 2 工作區 B
            "parking_space": 96     # 作業員 2 停車格
        },
        "name": "射出機1"
    },
    2: {  # 射出機2
        "operator_1": {
            "work_area_a": 101,     # 作業員 1 工作區 A
            "work_area_b": 102,     # 作業員 1 工作區 B
            "parking_space": 97     # 作業員 1 停車格
        },
        "operator_2": {
            "work_area_a": 103,     # 作業員 2 工作區 A
            "work_area_b": 104,     # 作業員 2 工作區 B
            "parking_space": 98     # 作業員 2 停車格
        },
        "name": "射出機2"
    }
    # 射出機 3、4 配置類似...

# Location 狀態定義
LOCATION_STATUS = {
    0: "AVAILABLE",              # 可用 - Location 空閒
    1: "OCCUPIED",               # 佔用 - Location 有 Rack
    2: "TASK_ACTIVE",            # 任務進行中 - AGV 正在前往
    3: "RESERVED"                # 預留 - 為特定任務預留
}

# 作業員 Location 管理
OPERATOR_LOCATION_MAPPING = {
    # (機台ID, 作業員ID, 側邊) -> Location ID
    (1, 1, "left"): 91,   # 機台1-作業員1-工作區A
    (1, 1, "right"): 92,  # 機台1-作業員1-工作區B
    (1, 2, "left"): 93,   # 機台1-作業員2-工作區A
    (1, 2, "right"): 94,  # 機台1-作業員2-工作區B
    # ... 其他機台配置
}
```

### OPUI任務創建邏輯
```python
def create_call_empty_task(machine_id, space_num):
    """
    叫空車邏輯 (已棄用 - 此函數保留僅供參考):
    注意：現行流程已改為人工搬運空料架
    1. 作業員手動搬運空料架到工作區
    2. 使用 OPUI 首頁「加入料架」功能登記
    3. 系統自動更新料架位置到對應工作區

    原流程 (已不使用):
    1. 檢查機台是否啟用
    2. 檢查停車格是否可用 (status = 0)
    3. 獲取停車格對應的node_id (machine.parking_space_1/2)
    4. 創建任務並設置停車格狀態為 1 (TASK_ACTIVE)
    """
    
def create_dispatch_full_task(rack_id, room_id):
    """
    派滿車邏輯:
    1. 檢查料架是否存在且有產品
    2. 創建派車任務，目標為指定房間
    3. node_id 由 WCS 決定具體位置
    """
```

### 停車格狀態管理流程
```
停車格狀態流轉 (現行流程):
1. 空料架手動處理: 作業員手動搬運到工作區A/B，透過OPUI首頁「加入料架」登記
2. 派滿車: 從工作區派送到停車格 (status 0→1→2→0)
3. 停車格專用: 僅供KUKA AGV停放滿料架使用

舊流程 (已棄用):
1. 叫空車: status 0→1 (可用→任務進行中)
2. AGV送達: status 1→2 (任務進行中→任務完成)
3. 確認取貨: status 2→0 (任務完成→可用)
```

## 📦 系統位置配置（每台射出機）

### 現況 vs 未來規劃

#### 📌 目前實作（每台射出機 2 個 location）
```python
class Machine(SQLModel, table=True):
    __tablename__ = "machine"

    machine_id: int = Field(primary_key=True)
    machine_name: str
    parking_space_1: int  # 作業員 1 停車格
    parking_space_2: int  # 作業員 2 停車格
    # 缺少：工作區 A/B 欄位
```

#### 🚀 未來規劃（每台射出機 6 個 location）
```python
class Machine(SQLModel, table=True):  # 待開發
    __tablename__ = "machine"

    machine_id: int = Field(primary_key=True)
    machine_name: str

    # 作業員 1 的 location (3個)
    workspace_a_1: int    # 待新增：工作區 A
    workspace_b_1: int    # 待新增：工作區 B
    parking_space_1: int  # 已有：停車格

    # 作業員 2 的 location (3個)
    workspace_a_2: int    # 待新增：工作區 A
    workspace_b_2: int    # 待新增：工作區 B
    parking_space_2: int  # 已有：停車格
```

### Location 使用狀況

| Location 類型 | 數量 | 目前狀態 | 未來規劃 |
|-------------|------|----------|----------|
| 工作區 A | 2 | ❌ 未實作 | 待開發 |
| 工作區 B | 2 | ❌ 未實作 | 待開發 |
| 停車格 | 2 | ✅ 已實作 | 維持 |
| **總計** | **6** | **2/6 完成** | **6/6 完成** |

## 🔧 實際任務參數構建方式

系統中的任務參數直接在 TAFL 流程或 OPUI 服務中構建，沒有統一的參數管理器。

### TAFL 流程中的參數構建
```yaml
# ✅ TAFL 直接在 create 動詞中構建任務參數
# 範例：rack_rotation_room_outlet_afull_bempty.yaml
metadata:
  id: rack_rotation_room_outlet_afull_bempty
  name: 房間出口架台翻轉（A面滿B面空）

variables:
  work_id: 220001      # 使用 KUKA_RACK_MOVE
  priority: 5
  model: KUKA400i

flow:
  - create:
      target: task
      with:
        work_id: ${work_id}
        priority: ${priority}
        status_id: 1
        parameters:
          rack_id: ${rack.id}
          rack_name: ${rack.name}
          location_id: ${location.id}
          location_name: ${location.name}
          room_id: ${location.room_id}
          model: ${model}
          rotation_angle: 180
          nodes:
            - ${location.node_id}
            - ${location.node_id + 1}
            - ${location.node_id}
```

### OPUI 服務中的參數構建
```python
# ✅ OPUI 在服務層直接構建參數 (opui_task_service.py)
# 範例：派滿車任務
def create_dispatch_full_task(self, rack_id: int, room_id: int, machine_id: int, ...):
    """創建派滿車任務"""

    task_params = {
        "work_id": 100002,  # OPUI派滿車
        "rack_id": rack_id,
        "room_id": room_id,
        "machine_id": machine_id,
        "product_name": product_name,
        "count": count,
        "model": "KUKA400i",
        "nodes": [source_node, target_node],
        "target_area": "system_prep_area",
        "client_id": client_id
    }

    # 寫入資料庫
    task = Task(**task_params)
    session.add(task)
    session.commit()
```

### 任務參數標準格式

#### KUKA 移動貨架任務 (220001)
```json
{
  "work_id": 220001,
  "priority": 5,
  "status_id": 1,
  "parameters": {
    "model": "KUKA400i",
    "function": "rack_move",
    "rack_id": 123,
    "room_id": 2,
    "location_id": 75,
    "nodes": [75, 76, 75],
    "rotation_angle": 180
  }
}
```

#### OPUI 派滿車任務 (100002)
```json
{
  "work_id": 100002,
  "priority": 40,
  "status_id": 1,
  "parameters": {
    "model": "KUKA400i",
    "function": "rack_move",
    "rack_id": 101,
    "room_id": 2,
    "machine_id": 1,
    "product_name": "ABC12345",
    "count": 32,
    "nodes": [95, 74, 72, 15],
    "target_area": "system_prep_area"
  }
}
```

## 📝 文檔說明總結

### 實際實作 ✅
以下內容在實際程式碼中已實作：

1. **Work ID 定義** (`shared_constants/work_ids.py`)
   - `WorkIds.KUKA_MOVE = 210001`
   - `WorkIds.KUKA_RACK_MOVE = 220001`
   - `WorkIds.KUKA_WORKFLOW = 230001`
   - `WorkIds.KUKA_SUPPORTED_WORK_IDS` 列表

2. **RCS 任務路由** (`rcs_ws/simple_kuka_manager.py`)
   - 根據 `work_id` 路由到對應的 KUKA API
   - 任務過濾使用 `KUKA_SUPPORTED_WORK_IDS`

3. **TAFL 流程** (`config/tafl/flows/*.yaml`)
   - 直接使用數字 work_id (220001)
   - 在 YAML 中構建完整任務參數

4. **OPUI 任務創建** (`web_api_ws/opui/`)
   - 服務層直接構建任務參數
   - 使用數字 work_id

### 推薦使用方式
- ✅ 使用 `from shared_constants.work_ids import WorkIds` 類別
- ✅ 在 TAFL 中直接使用數字 work_id (如 220001)
- ✅ 參考「業務流程與 Work ID 對應表」理解業務場景
- ✅ 任務參數在 TAFL 或 OPUI 中直接構建，不使用中間管理器

## 🔗 交叉引用
- **實際實作**:
  - shared_constants_ws: @app/shared_constants_ws/CLAUDE.md
  - RCS 系統: @app/rcs_ws/CLAUDE.md
  - TAFL WCS 實作: @app/tafl_wcs_ws/CLAUDE.md
- **設計文檔**:
  - WCS 系統設計: docs-ai/knowledge/agv-domain/wcs-system-design.md
  - 資料庫設計: docs-ai/knowledge/agv-domain/wcs-database-design.md
  - 手動 Rack 管理: docs-ai/knowledge/system/manual-rack-management.md
  - 操作手冊: docs-ai/operations/guides/rack-management-guide.md