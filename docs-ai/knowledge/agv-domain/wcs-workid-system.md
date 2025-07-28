# WCS Work ID 分類系統

## 🎯 適用場景
- 理解 RosAGV 系統中 Work ID 的分類和用途
- 掌握不同 Work ID 對應的任務參數格式
- 為任務創建和調度提供參考規範

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
**用途**: 作業員從 OPUI 請求將空 Rack 派至人工作業準備區

**任務參數格式**:
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
  "side": "left",               // 停車格側別
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
**用途**: 執行指定的 workflow 流程觸發 (僅用於人工回收空料架)

**任務參數格式**:
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
  "description": "人工回收空料架區搬運到系統空料架區"
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

## 🔄 業務流程與Work ID映射

### WCS 決策引擎使用的 Work ID
```python
BUSINESS_FLOW_WORK_IDS = {
    # WCS 決策引擎使用的 Work IDs (大部分使用 kuka-移動貨架)
    'agv_rotation': 220001,              # AGV旋轉檢查 → kuka-移動貨架
    'ng_rack_recycling': 220001,         # NG料架回收 → kuka-移動貨架
    'full_rack_to_manual': 220001,       # 滿料架到人工收料區 → kuka-移動貨架
    'manual_area_transport': 220001,     # 人工收料區搬運 → kuka-移動貨架
    'system_to_room': 220001,           # 系統準備區到房間 → kuka-移動貨架
    'empty_rack_transfer': 220001,      # 空料架搬運 → kuka-移動貨架
    'manual_empty_recycling': 230001,   # 人工回收空料架 → kuka-流程觸發 ⭐唯一特殊
    
    # OPUI 手動任務
    'opui_call_empty': 100001,          # OPUI叫空車
    'opui_dispatch_full': 100002,       # OPUI派滿車
    
    # Cargo AGV 任務
    'cargo_inlet': 2000102,             # CargoAGV放入口傳送箱
    'cargo_outlet': 2000201,            # CargoAGV拿出口傳送箱
}
```

### 條件檢查Work ID對應
```python
CONDITION_CHECK_WORK_IDS = {
    # 所有料架移動操作都使用 220001 (kuka-移動貨架)
    'agv_rotation': '220001',           # AGV旋轉重複任務檢查
    'ng_rack_recycling': '220001',       # NG料架回收重複任務檢查
    'full_rack_to_manual': '220001',     # 滿料架搬運重複任務檢查
    'manual_area_transport': '220001',   # 人工收料區搬運重複任務檢查
    'system_to_room': '220001',         # 系統準備區搬運重複任務檢查
    'empty_rack_transfer': '220001',    # 空料架搬運重複任務檢查
    
    # 唯一特殊例外：人工回收空料架使用 230001 (kuka-流程觸發)
    'manual_empty_recycling': '230001',  # 人工回收空料架重複任務檢查
}
```

## 🏗️ OPUI 整合邏輯

### Machine → Location → Node ID 對應關係
```python
# 機台停車格配置 (實際初始化資料)
MACHINE_PARKING_CONFIG = {
    1: {"parking_space_1": 95, "parking_space_2": 96, "name": "射出機1"},
    2: {"parking_space_1": 97, "parking_space_2": 98, "name": "射出機2"}, 
    3: {"parking_space_1": 1005, "parking_space_2": 1006, "name": "射出機3"},
    4: {"parking_space_1": 1007, "parking_space_2": 1008, "name": "射出機4"}
}

# 停車格狀態定義
PARKING_STATUS = {
    0: "PARKING_AVAILABLE",      # 可用 - 停車格空閒，可以叫車
    1: "PARKING_TASK_ACTIVE",    # 任務進行中 - 已叫車，等待AGV送達  
    2: "PARKING_TASK_COMPLETED"  # 任務完成 - 車輛已送達，等待確認取貨
}
```

### OPUI任務創建邏輯
```python
def create_call_empty_task(machine_id, space_num):
    """
    叫空車邏輯:
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
停車格狀態流轉
1. 叫空車: status 0→1 (可用→任務進行中)
2. AGV送達: status 1→2 (任務進行中→任務完成)
3. 確認取貨: status 2→0 (任務完成→可用)
4. 派滿車: 直接使用已停靠的料架，不改變停車格狀態
```

## 🔧 Work ID 參數管理器

### UnifiedTaskManager 中的參數管理
```python
class WorkIDParameterManager:
    """Work ID 參數管理器"""
    
    def format_task_parameters(self, work_id: str, decision: TaskDecision) -> dict:
        """根據 work_id 格式化任務參數"""
        
        if work_id == '220001':  # kuka-移動貨架
            return self._format_rack_move_parameters(decision)
        elif work_id == '230001':  # kuka-流程觸發
            return self._format_workflow_parameters(decision)
        elif work_id == '100001':  # OPUI叫空車
            return self._format_opui_call_empty_parameters(decision)
        elif work_id == '100002':  # OPUI派滿車
            return self._format_opui_dispatch_full_parameters(decision)
        else:
            return self._format_default_parameters(decision)
    
    def _format_rack_move_parameters(self, decision: TaskDecision) -> dict:
        """格式化 kuka-移動貨架 參數"""
        return {
            "function": "rack_move",
            "model": "KUKA400i",
            "api": "submit_mission",
            "missionType": "RACK_MOVE",
            "nodes": decision.nodes,
            "rack_id": decision.rack_id,
            "task_category": decision.task_category,
            "priority_level": decision.priority,
            "source_location": decision.source_location,
            "target_location": decision.target_location,
            "room_id": decision.room_id
        }
```

## 🔗 交叉引用
- WCS 系統設計: @docs-ai/knowledge/agv-domain/wcs-system-design.md
- 資料庫設計: @docs-ai/knowledge/agv-domain/wcs-database-design.md
- AI WCS 實作: @app/ai_wcs_ws/CLAUDE.md
- 任務管理: @docs-ai/operations/development/task-management.md