# WCS 資料庫設計

## 🎯 適用場景
- 理解 RosAGV 系統的完整資料庫架構
- 掌握各資料表之間的關聯關係
- 為資料庫操作和查詢最佳化提供參考

## 📋 資料庫架構概覽

WCS 系統的資料庫設計支援完整的 Rack 生命週期管理、任務調度和狀態追蹤。

### 核心資料表關係
```
資料表關聯圖
├── Machine (機台配置)
├── Room (房間配置) ←→ ProcessSettings
├── Rack (料架管理) ←→ Product, Location, AGV
├── Carrier (載具管理) ←→ Rack, Room
├── Task (任務管理) ←→ TaskStatus, Work, Room, Node, AGV
├── Location (位置管理) ←→ LocationStatus
└── Product (產品管理) ←→ ProcessSettings
```

## 🏭 機台和房間配置

### Machine 表
```python
class Machine(SQLModel, table=True):
    __tablename__ = "machine"
    id: Optional[int] = Field(default=None, primary_key=True)
    parking_space_1: Optional[int] = Field(default=None, foreign_key="node.id")
    parking_space_2: Optional[int] = Field(default=None, foreign_key="node.id")
    parking_space_1_status: Optional[int] = Field(default=0)
    parking_space_2_status: Optional[int] = Field(default=0)
    workspace_1: Optional[List[int]] = Field(default=None)  # 左側工作區位置陣列
    workspace_2: Optional[List[int]] = Field(default=None)  # 右側工作區位置陣列
    name: str
    description: Optional[str] = None
    enable: int = Field(default=1)
```

**用途**: 管理作業區機台配置，每台機台有兩個停車格和兩個工作區
**關鍵欄位**:
- `parking_space_1/2`: 對應到 Node 表的停車格位置
- `parking_space_1/2_status`: 停車格狀態 (0=可用, 1=任務中, 2=完成)
- `workspace_1`: 左側工作區的 Location ID 陣列（作業員1使用）
- `workspace_2`: 右側工作區的 Location ID 陣列（作業員2使用）

### Room 表
```python
class Room(SQLModel, table=True):
    __tablename__ = "room"
    id: Optional[int] = Field(default=None, primary_key=True)
    process_settings_id: int = Field(foreign_key="process_settings.id")
    name: str
    description: Optional[str] = None
    enable: int = Field(default=1)
    enter_location_id: Optional[int] = Field(default=None)
    exit_location_id: Optional[int] = Field(default=None)
```

**用途**: 管理生產房間配置，關聯製程設定
**關鍵欄位**:
- `process_settings_id`: 關聯到製程設定，用於驗證 Rack 產品是否相符
- `enter_location_id/exit_location_id`: 房間入口和出口位置

### ProcessSettings 表
```python
class ProcessSettings(SQLModel, table=True):
    __tablename__ = "process_settings"
    id: Optional[int] = Field(default=None, primary_key=True)
    soaking_times: int
    description: Optional[str] = None
```

**用途**: 定義製程參數，確保 Rack 產品與房間製程匹配

## 🚛 料架和載具管理

### Rack 表
```python
class Rack(SQLModel, table=True):
    __tablename__ = "rack"
    id: Optional[int] = Field(default=None, primary_key=True)
    name: str
    room_id: Optional[int] = Field(default=None, foreign_key="room.id")
    agv_id: Optional[int] = Field(default=None, foreign_key="agv.id")
    location_id: Optional[int] = Field(default=None, foreign_key="location.id")
    product_id: Optional[int] = Field(default=None, foreign_key="product.id")
    is_carry: Optional[int] = None
    is_in_map: Optional[int] = None
    is_docked: Optional[int] = None
    status_id: Optional[int] = Field(default=None, foreign_key="rack_status.id")
    direction: int = Field(default=0)  # 0=90度A面, 180=-90度B面
```

**用途**: 核心料架管理，追蹤料架狀態和位置
**關鍵欄位**:
- `room_id`: 目標房間，用於製程驗證
- `direction`: 料架朝向，決定 A/B 面狀態 (0=A面, 180=B面)
- `location_id`: 當前位置
- `product_id`: 關聯產品，用於容量計算和製程驗證

### Carrier 表
```python
class Carrier(SQLModel, table=True):
    __tablename__ = "carrier"
    id: Optional[int] = Field(default=None, primary_key=True)
    room_id: Optional[int] = None  # FK room.id
    rack_id: Optional[int] = None  # FK rack.id
    port_id: Optional[int] = None  # FK eqp_port.id
    rack_index: Optional[int] = None  # 1-16=A面, 17-32=B面
    status_id: Optional[int] = None
    created_at: datetime = Field(
        sa_column=Column(DateTime(timezone=True), nullable=False),
        default_factory=lambda: datetime.now(timezone.utc))
    updated_at: Optional[datetime] = Field(
        default=None,
        sa_column=Column(DateTime(timezone=True), nullable=True))
```

**用途**: 管理料架上的載具 (Carrier)，支援 A/B 面管理
**關鍵欄位**:
- `rack_index`: 載具在料架上的位置
  - 1-16: A面位置
  - 17-32: B面位置
- `status_id`: 載具狀態，用於 NG 檢測和作業判斷

### CarrierStatus 表
```python
class CarrierStatus(SQLModel, table=True):
    __tablename__ = "carrier_status"
    id: Optional[int] = Field(default=None, primary_key=True)
    name: str
    description: Optional[str] = None
    color: Optional[str] = None  # UI 顯示顏色
```

**用途**: 定義載具的各種狀態
**狀態定義**:
| ID | 名稱 | 說明 | 作業判斷 |
|----|------|------|----------|
| 1 | 空閒 | 載具空閒，可以使用 | **有待作業** (需要開始處理) |
| 2 | 使用中 | 載具正在使用中 | **有待作業** |
| 3 | 故障 | 載具發生故障 | **特殊處理** |
| 4 | 待處理 | 載具等待處理 | **有待作業** |
| 5 | 處理中 | 載具正在處理製程 | **有待作業** (還在處理中) |
| 6 | NG | 載具處理結果不良 | **特殊處理** |
| 7 | 維護中 | 載具正在維護 | **特殊處理** |
| 8 | 已完成 | 載具處理完成 | **無待作業** (已經完成) |
| 101-603 | 各站點製程狀態 | 詳細製程追蹤 | **有待作業** |

**架台翻轉判斷邏輯**:
- A面全部完成: 所有 A 面載具 `status_id = 8`
- B面有待作業: 任何 B 面載具 `status_id != 8`
- 翻轉條件: A面全部完成 AND B面有待作業

### Product 表
```python
class Product(SQLModel, table=True):
    __tablename__ = "product"
    id: Optional[int] = Field(default=None, primary_key=True)
    name: str
    size: str  # S或L尺寸
    process_settings_id: int = Field(foreign_key="process_settings.id")
    created_at: datetime = Field(
        sa_column=Column(DateTime(timezone=True), nullable=False),
        default_factory=lambda: datetime.now(timezone.utc))
    updated_at: Optional[datetime] = Field(
        default=None,
        sa_column=Column(DateTime(timezone=True), nullable=True))
```

**用途**: 產品定義，決定料架容量和製程需求
**關鍵欄位**:
- `size`: 產品尺寸，影響料架容量 (S=32個, L=16個)
- `process_settings_id`: 製程需求，必須與房間製程匹配

## 📋 任務管理系統

### Task 表
```python
class Task(SQLModel, table=True):
    __tablename__ = "task"
    id: Optional[int] = Field(default=None, primary_key=True)
    parent_task_id: Optional[int] = Field(default=None, foreign_key="task.id")
    work_id: Optional[int] = Field(default=None, foreign_key="work.id")
    status_id: Optional[int] = Field(default=None, foreign_key="task_status.id")
    room_id: Optional[int] = Field(default=None, foreign_key="room.id")
    node_id: Optional[int] = Field(default=None, foreign_key="node.id")
    name: str
    description: Optional[str] = None
    mission_code: Optional[str] = None  # Kuka 系統的任務代碼
    agv_id: Optional[int] = Field(default=None, foreign_key="agv.id")
    priority: int = Field(default=0)
    parameters: Optional[Dict[str, Any]] = Field(sa_column=Column(JSON))
    created_at: datetime = Field(
        sa_column=Column(DateTime(timezone=True), nullable=False),
        default_factory=lambda: datetime.now(timezone.utc))
    updated_at: Optional[datetime] = Field(
        default=None,
        sa_column=Column(DateTime(timezone=True), nullable=True))
```

**用途**: 核心任務管理，記錄所有 WCS 決策和執行狀態
**關鍵欄位**:
- `work_id`: 任務類型識別碼 (詳見 Work ID 系統)
- `parameters`: JSON 格式的任務參數
- `parent_task_id`: 支援任務階層 (如 AGV 旋轉的子任務)

### TaskStatus 表
```python
class TaskStatus(SQLModel, table=True):
    __tablename__ = "task_status"
    id: Optional[int] = Field(default=None, primary_key=True)
    name: str
    description: Optional[str] = None

    # 任務狀態常數定義
    REQUESTING: ClassVar[int] = 0          # 請求中
    PENDING: ClassVar[int] = 1             # 待處理
    READY_TO_EXECUTE: ClassVar[int] = 2    # 待執行
    EXECUTING: ClassVar[int] = 3           # 執行中
    COMPLETED: ClassVar[int] = 4           # 已完成
    CANCELLING: ClassVar[int] = 5          # 取消中
    ERROR: ClassVar[int] = 6               # 錯誤

    # 取消相關狀態
    WCS_CANCELLING: ClassVar[int] = 51     # WCS-取消中
    RCS_CANCELLING: ClassVar[int] = 52     # RCS-取消中
    AGV_CANCELLING: ClassVar[int] = 53     # AGV-取消中
    CANCELLED: ClassVar[int] = 54          # 已取消
```

**用途**: 定義任務生命週期的各個狀態
**狀態流轉**: 0(請求) → 1(待處理) → 2(待執行) → 3(執行中) → 4(完成)

## 📍 位置管理系統

### Location 表
```python
class Location(SQLModel, table=True):
    __tablename__ = "location"
    id: Optional[int] = Field(default=None, primary_key=True)
    location_status_id: Optional[int] = Field(
        default=None, foreign_key="location_status.id")
    room_id: Optional[int] = Field(default=None)
    node_id: Optional[int] = None  # 可能參考 node.id 或 kuka_node.id，無外鍵約束
    name: str  # 系統空料車停車區, 系統準備派車區, 人工收料區, NG料車區等
    description: Optional[str] = None
    type: Optional[str] = Field(default="enter_or_exit")  # 位置類型
    rack_id: Optional[int] = Field(default=None, foreign_key="rack.id")

    # 架台旋轉點配置 (2025-10-01 新增)
    rotation_node_id: Optional[int] = Field(
        default=None,
        description="架台在此位置旋轉時使用的中間轉向點 (參考 kuka_node.id，用於 room_inlet/room_outlet 類型)"
    )
```

**用途**: 管理系統中所有位置的狀態和配置，包括架台旋轉點設定

**關鍵欄位**:
- `node_id`: 位置對應的導航節點
  - 可能參考 `node.id` (通用節點) 或 `kuka_node.id` (KUKA 專用節點)
  - **無外鍵約束**：保持靈活性，支援兩種不同的 node 表
- `rotation_node_id`: 架台旋轉使用的中間轉向點 (2025-10-01 新增)
  - 實際參考 `kuka_node.id` 表
  - **無外鍵約束**：kuka_node 表由外部 KUKA Fleet Manager 軟體管理和匯入
  - 用於房間入口 (room_inlet) 和出口 (room_outlet) 的架台旋轉任務
  - 每個房間的入口和出口各有專屬的旋轉點配置

**外鍵設計考量** (2025-10-01 更新):
1. **為何不綁定外鍵**:
   - `kuka_node` 表由外部 KUKA Fleet Manager 軟體編輯後匯入
   - 設置外鍵約束會限制外部資料匯入的靈活性
   - `node_id` 需要同時支援 `node` 和 `kuka_node` 兩個不同的表
2. **資料完整性保證**:
   - 透過初始化腳本統一設定 (08_locations.py)
   - 應用層驗證 node_id 和 rotation_node_id 的有效性
   - TAFL 流程執行前檢查節點存在性

**架台旋轉點配置範例**:
```python
# 房間 1 的入口和出口配置 (來自 08_locations.py)
{"id": 10001, "room_id": 1, "node_id": 10001, "rotation_node_id": 10003,
    "name": "room01 Loader Box", "type": "room_inlet"},
{"id": 10002, "room_id": 1, "node_id": 10002, "rotation_node_id": 10004,
    "name": "room01 Unloader Box", "type": "room_outlet"}
```

### LocationStatus 表
```python
class LocationStatus(SQLModel, table=True):
    __tablename__ = "location_status"
    id: Optional[int] = Field(default=None, primary_key=True)
    name: str
    description: Optional[str] = None

    # 位置狀態常數定義
    UNKNOWN: ClassVar[int] = 1          # 未知狀態
    UNOCCUPIED: ClassVar[int] = 2       # 未佔用 (空位)
    OCCUPIED: ClassVar[int] = 3         # 佔用 (有料架)
```

**用途**: 管理系統中所有位置的佔用狀態
**重要位置編號**:
- 2-9: 系統準備區
- 11-13: 系統空車停放區
- 21-22: 人工收料區
- 71-72: NG回收區 (🛑 已棄用 - OCR NG 改為房間入口即時處理，不再搬運到此區域)
- 91-92: 人工回收空料架區 (🛑 已棄用 - 改為手動管理，不再透過 AGV 搬運)

## 🔄 資料表關聯邏輯

### Rack 狀態判斷機制
```python
def get_rack_status(rack_id: int) -> dict:
    """取得 Rack 完整狀態資訊"""
    rack = get_rack_by_id(rack_id)
    carriers = get_carriers_by_rack_id(rack_id)
    product = get_product_by_id(rack.product_id)
    
    # 判斷產品尺寸與容量
    max_capacity = 32 if product.size == 'S' else 16
    
    # 分析A面/B面狀態
    a_side_carriers = [c for c in carriers if 1 <= c.rack_index <= 16]
    b_side_carriers = [c for c in carriers if 17 <= c.rack_index <= 32]
    
    # 檢查NG狀態
    ng_carriers = [c for c in carriers if is_carrier_ng(c.status_id)]
    has_ng = len(ng_carriers) > 0
    
    # 依據direction判斷當前朝向
    current_side = 'A' if rack.direction == 90 else 'B'
    
    return {
        'total_carriers': len(carriers),
        'max_capacity': max_capacity,
        'a_side_count': len(a_side_carriers),
        'b_side_count': len(b_side_carriers),
        'current_side': current_side,
        'has_ng': has_ng,
        'is_empty': len(carriers) == 0,
        'is_full': len(carriers) == max_capacity,
        'needs_rotation': needs_rack_rotation(rack, carriers)
    }
```

### 製程驗證邏輯
```python
def validate_process_compatibility(rack: Rack, room: Room) -> bool:
    """驗證 Rack 產品與房間製程相符性"""
    try:
        product = get_product_by_id(rack.product_id)
        
        # room.process_settings_id 應該對應到 product.process_settings_id
        if room.process_settings_id != product.process_settings_id:
            # 設定Rack狀態為不合格
            update_rack_status(rack.id, "PROCESS_MISMATCH")
            log_error(f"Rack {rack.id} 產品製程 {product.process_settings_id} 與房間 {room.id} 製程 {room.process_settings_id} 不符")
            return False
            
        return True
        
    except Exception as e:
        log_error(f"製程驗證失敗: {e}")
        update_rack_status(rack.id, "VALIDATION_ERROR")
        return False
```

### OPUI 停車格狀態管理
```python
# 機台停車格配置
MACHINE_PARKING_CONFIG = {
    1: {"parking_space_1": 95, "parking_space_2": 96, "name": "射出機1"},
    2: {"parking_space_1": 97, "parking_space_2": 98, "name": "射出機2"}, 
    3: {"parking_space_1": 1005, "parking_space_2": 1006, "name": "射出機3"},
    4: {"parking_space_1": 1007, "parking_space_2": 1008, "name": "射出機4"}
}

# 停車格狀態定義
PARKING_STATUS = {
    0: "PARKING_AVAILABLE",      # 可用
    1: "PARKING_TASK_ACTIVE",    # 任務進行中
    2: "PARKING_TASK_COMPLETED"  # 任務完成
}
```

## 🚀 查詢最佳化策略

### 批次位置狀態檢查
```sql
-- 批次檢查多組位置狀態
-- 用於 WCS 決策引擎的批次最佳化
SELECT 'ng_recycling' as group_name,
       CASE WHEN COUNT(*) > 0 THEN 'True' ELSE 'False' END as available,
       CASE WHEN COUNT(*) > 0 THEN MIN(id) ELSE NULL END as location_id
FROM location 
WHERE id = ANY (ARRAY[71,72]) AND location_status_id = 2

UNION ALL

SELECT 'manual_area' as group_name,
       CASE WHEN COUNT(*) > 0 THEN 'True' ELSE 'False' END as available,
       CASE WHEN COUNT(*) > 0 THEN MIN(id) ELSE NULL END as location_id
FROM location 
WHERE id = ANY (ARRAY[51,52,53,54,55]) AND location_status_id = 2
```

### 任務衝突批次檢查
```sql
-- 批次檢查任務衝突
-- 減少重複查詢，提升決策效率
SELECT '220001_10001' as check_key,
       COUNT(*) as conflict_count
FROM task 
WHERE work_id = '220001' 
AND (node_id = 10001 OR status_id IN (0,1,2))

UNION ALL

SELECT '220001_20001' as check_key,
       COUNT(*) as conflict_count
FROM task 
WHERE work_id = '220001' 
AND (node_id = 20001 OR status_id IN (0,1,2))
```

## 📊 關鍵統計查詢

### Rack 狀態統計
```sql
-- Rack 狀態分布統計
SELECT 
    rs.name as status_name,
    COUNT(r.id) as rack_count,
    ROUND(COUNT(r.id) * 100.0 / SUM(COUNT(r.id)) OVER(), 2) as percentage
FROM rack r
JOIN rack_status rs ON r.status_id = rs.id
GROUP BY rs.id, rs.name
ORDER BY rack_count DESC;
```

### 位置佔用率統計
```sql
-- 系統位置佔用率統計
SELECT 
    l.name as location_name,
    ls.name as status_name,
    COUNT(l.id) as location_count
FROM location l
JOIN location_status ls ON l.location_status_id = ls.id
WHERE l.id BETWEEN 11 AND 92  -- 主要系統位置
GROUP BY l.name, ls.name
ORDER BY l.name;
```

### 任務執行效率統計
```sql
-- 任務執行時間統計
SELECT 
    w.name as work_type,
    ts.name as status_name,
    COUNT(t.id) as task_count,
    AVG(EXTRACT(EPOCH FROM (t.updated_at - t.created_at))/60) as avg_duration_minutes
FROM task t
JOIN work w ON t.work_id = w.id
JOIN task_status ts ON t.status_id = ts.id
WHERE t.created_at >= NOW() - INTERVAL '24 hours'
GROUP BY w.name, ts.name
ORDER BY w.name, task_count DESC;
```

## 🔗 交叉引用
- WCS 系統設計: docs-ai/knowledge/agv-domain/wcs-system-design.md
- Work ID 系統: docs-ai/knowledge/agv-domain/wcs-workid-system.md
- **當前 WCS 實作**:
  - KUKA WCS: @app/kuka_wcs_ws/CLAUDE.md（當前使用）
  - WCS 工作空間: @app/wcs_ws/CLAUDE.md（流程控制邏輯）
- **歷史參考**: ~~TAFL WCS 實作: @app/tafl_wcs_ws/~~（⚠️ 已棄用）
- 資料庫操作: docs-ai/operations/development/database-operations.md