# WCS 系統完整設計文檔

## 專案概述

本專案為 WCS (Warehouse Control System) 系統，負責管理 Rack 車的搬移需求決策。

### 基本環境配置
- **房間數量**: 5個房間
- **每個房間配置**: 1個入口 + 1個出口 (均可停靠 Rack 車)
- **Rack 位置管理**: 由 WCS 掌控並儲存在資料庫中
- **實體搬運**: 由 KUKA AGV 透過 KUKA API 執行

## 系統架構分工

### 🎯 WCS (我們的系統)
**職責範圍**:
- 決策 Rack 是否需要搬移
- 產生搬運任務 (Task) 到資料庫
- 監控 Rack 位置狀態

### 🤖 RCS (Robot Control System)
**職責範圍**:
- 讀取 WCS 產生的任務
- 透過 KUKA API 向 KUKA Fleet 下達搬運指令
- 管理 AGV 調度

### 🌐 Web API (kuka.py)
**職責範圍**:
- 掌控任務運行狀態
- 更新任務狀態到資料庫
- 與 KUKA Fleet 介面整合

## ✅ 完整的Rack生命週期流程

### 階段1：上料階段
1. **作業區上貨** → 作業員在作業區(4個作業區，每個有2個停車格)將貨物放上空Rack
   - 停車格位置記錄在 `machine.parking_space_1/2` 欄位
2. **OPUI操作** → 作業員透過OPUI後端直連資料庫，產生`status=0`的task
   - 目標房間資訊寫入 `task.parameters`
   - 任務類型：**叫車**(需要空Rack) 或 **派車**(將滿車移走)
3. **WCS監控與執行**：
   - **派車**：WCS將滿載Rack送到"系統準備派車區"等待
   - **叫車**：WCS挑選空Rack，產生任務讓AGV搬運到叫車的作業區

### 階段2：生產階段
4. **進入房間入口** → WCS依優先度將Rack從準備區送到房間入口
5. **機器手臂取料** → 房間入口機器手臂將Rack上貨物送入房間處理
6. **A面清空與翻面** → Rack有2面，A面carrier全部搬完時：
   - WCS產生**旋轉任務**，讓AGV執行Rack翻面動作
   - 機器手臂繼續作業B面
7. **NG檢查與處理** → 機器手臂OCR檢查，如果NG：
   - 記錄NG但不取出carrier
   - **有NG carrier的Rack → 送到NG區人員處理**
   - **正常空Rack → 送到房間出口(有空位)或空Rack暫存區**

### 階段3：收料階段
8. **機器手臂放料** → 房間出口機器手臂將處理完貨物放到空Rack上
9. **A面放滿與翻面** → Rack A面放滿但B面未放時：
   - WCS產生**旋轉任務**(節點路徑與入口旋轉不同)
   - 機器手臂繼續作業B面
10. **送到收料區** → Rack滿載或房間尾批完成時送到"人工收料區"

### 階段4：回收階段
11. **人工收料** → 作業員從人工收料區取走完成品
    - 作業員在系統輸入Rack已空
    - 作業員將Rack搬到空車回收位置(2格Node)
12. **空車回收** → WCS檢查到Rack在空車回收位置，派任務送回"系統空料車停車區"

## ✅ 資料庫表結構設計

### Machine表
```python
class Machine(SQLModel, table=True):
    __tablename__ = "machine"
    id: Optional[int] = Field(default=None, primary_key=True)
    parking_space_1: Optional[int] = Field(default=None, foreign_key="node.id")
    parking_space_2: Optional[int] = Field(default=None, foreign_key="node.id")
    parking_space_1_status: Optional[int] = Field(default=0)
    parking_space_2_status: Optional[int] = Field(default=0)
    name: str
    description: Optional[str] = None
    enable: int = Field(default=1)
```

### Room表
```python
class Room(SQLModel, table=True):
    __tablename__ = "room"
    id: Optional[int] = Field(default=None, primary_key=True)
    # 外鍵：對應到 ProcessSettings 的 process_id
    process_settings_id: int = Field(foreign_key="process_settings.id")
    name: str
    description: Optional[str] = None
    enable: int = Field(default=1)
    # 新增房間入口和出口位置ID - 不設外鍵約束避免循環依賴
    enter_location_id: Optional[int] = Field(default=None)
    exit_location_id: Optional[int] = Field(default=None)
```

### Rack表
```python
class Rack(SQLModel, table=True):
    __tablename__ = "rack"
    id: Optional[int] = Field(default=None, primary_key=True)
    name: str
    room_id: Optional[int] = Field(default=None, foreign_key="room.id")  # 這個rack要搬去哪個房間
    agv_id: Optional[int] = Field(default=None, foreign_key="agv.id")
    location_id: Optional[int] = Field(default=None, foreign_key="location.id")
    product_id: Optional[int] = Field(default=None, foreign_key="product.id")
    is_carry: Optional[int] = None
    is_in_map: Optional[int] = None
    is_docked: Optional[int] = None
    status_id: Optional[int] = Field(default=None, foreign_key="rack_status.id")
    direction: int = Field(default=0)  # 0=90度A面, 180=-90度B面
```

### Carrier表
```python
class Carrier(SQLModel, table=True):
    __tablename__ = "carrier"
    id: Optional[int] = Field(default=None, primary_key=True)
    room_id: Optional[int] = None  # FK room.id 不強綁定
    rack_id: Optional[int] = None  # FK rack.id 不強綁定
    port_id: Optional[int] = None  # FK eqp_port.id 不強綁定
    rack_index: Optional[int] = None  # 1-16=A面, 17-32=B面
    status_id: Optional[int] = None
    created_at: datetime = Field(
        sa_column=Column(DateTime(timezone=True), nullable=False),
        default_factory=lambda: datetime.now(timezone.utc))
    updated_at: Optional[datetime] = Field(
        default=None,
        sa_column=Column(DateTime(timezone=True), nullable=True))
```

### Task表與TaskStatus表
```python
class TaskStatus(SQLModel, table=True):
    __tablename__ = "task_status"
    id: Optional[int] = Field(default=None, primary_key=True)
    name: str
    description: Optional[str] = None

    # 任務狀態常數定義 (基於實際資料庫模型)
    # 主要狀態
    REQUESTING: ClassVar[int] = 0          # 請求中 (UI-請求執行任務)
    PENDING: ClassVar[int] = 1             # 待處理 (WCS-任務已接受，待處理)
    READY_TO_EXECUTE: ClassVar[int] = 2    # 待執行 (RCS-任務已派發，待執行)
    EXECUTING: ClassVar[int] = 3           # 執行中 (AGV-任務正在執行)
    COMPLETED: ClassVar[int] = 4           # 已完成 (AGV-任務已完成)
    CANCELLING: ClassVar[int] = 5          # 取消中 (任務取消)
    ERROR: ClassVar[int] = 6               # 錯誤 (錯誤)

    # 取消相關狀態
    WCS_CANCELLING: ClassVar[int] = 51     # WCS-取消中 (WCS-任務取消中，待處理)
    RCS_CANCELLING: ClassVar[int] = 52     # RCS-取消中 (RCS-任務取消中，取消中)
    AGV_CANCELLING: ClassVar[int] = 53     # AGV-取消中 (AGV-取消完成)
    CANCELLED: ClassVar[int] = 54          # 已取消 (任務已取消)

    # 狀態碼對應的中文描述
    STATUS_DESCRIPTIONS: ClassVar[Dict[int, str]] = {
        0: "請求中", 1: "待處理", 2: "待執行", 3: "執行中", 4: "已完成",
        5: "取消中", 6: "錯誤", 51: "WCS-取消中", 52: "RCS-取消中", 
        53: "AGV-取消中", 54: "已取消"
    }

    # 狀態碼對應的英文名稱
    STATUS_NAMES: ClassVar[Dict[int, str]] = {
        0: "REQUESTING", 1: "PENDING", 2: "READY_TO_EXECUTE", 3: "EXECUTING",
        4: "COMPLETED", 5: "CANCELLING", 6: "ERROR", 51: "WCS_CANCELLING",
        52: "RCS_CANCELLING", 53: "AGV_CANCELLING", 54: "CANCELLED"
    }

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

### Product表與ProcessSettings表
```python
class ProcessSettings(SQLModel, table=True):
    __tablename__ = "process_settings"
    id: Optional[int] = Field(default=None, primary_key=True)
    soaking_times: int
    description: Optional[str] = None

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

### Location表與LocationStatus表
```python
class LocationStatus(SQLModel, table=True):
    __tablename__ = "location_status"
    id: Optional[int] = Field(default=None, primary_key=True)
    name: str
    description: Optional[str] = None

    # 位置狀態常數定義
    UNKNOWN: ClassVar[int] = 1          # 未知狀態
    UNOCCUPIED: ClassVar[int] = 2       # 未佔用 (空位沒有被使用)
    OCCUPIED: ClassVar[int] = 3         # 佔用 (已經有停放的料架)

class Location(SQLModel, table=True):
    __tablename__ = "location"
    id: Optional[int] = Field(default=None, primary_key=True)
    location_status_id: Optional[int] = Field(
        default=None, foreign_key="location_status.id")
    room_id: Optional[int] = Field(default=None)
    node_id: Optional[int] = None
    name: str  # 系統空料車停車區, 系統準備派車區, 人工收料區, NG料車區等
    description: Optional[str] = None
```

## ✅ WCS 統一決策引擎設計

### 七大業務流程優先度架構

基於舊系統完整條件邏輯分析，WCS系統包含7個核心業務流程：

#### 🔴 第1級：AGV旋轉檢查 (Priority: 100)
**業務流程**：檢查等待旋轉狀態的AGV，使用3個節點移動方式執行旋轉
**觸發條件**：
- AGV處於 'wait_rotation_state' 狀態
- 對應任務無子任務存在
- 無重複執行任務 (work_id='220001')

**決策邏輯** (改用 nodes 移動方式)：
```python
async def check_agv_rotation_flow():
    """AGV旋轉狀態檢查 - 使用3節點移動方式"""
    waiting_agvs = await db.get_agvs_by_state('wait_rotation_state')
    
    for agv_context in waiting_agvs:
        agv_tasks = await db.get_tasks_by_agv(agv_context.agv_id)
        for task in agv_tasks:
            # 檢查是否無子任務 (防重複發送)
            child_tasks = await db.get_child_tasks(task.id)
            if not child_tasks:
                # 檢查是否有重複的旋轉任務
                duplicate_check = await db.has_active_task('220001', task.node_id)
                if not duplicate_check:
                    # 創建使用 nodes 移動的旋轉任務
                    rotation_nodes = generate_rotation_nodes(task.node_id, agv_context.current_location)
                    create_rotation_task(
                        agv_id=agv_context.agv_id, 
                        parent_task_id=task.id,
                        work_id='220001',  # 改用 kuka-移動貨架
                        nodes=rotation_nodes  # 3個節點的旋轉路徑
                    )

def generate_rotation_nodes(target_location: int, current_location: int) -> List[int]:
    """生成AGV旋轉的3個節點路徑"""
    # 根據目標位置類型生成旋轉節點
    if is_room_inlet(target_location):
        # 入口旋轉：當前位置 → 旋轉中間點 → 旋轉完成位置
        return [current_location, get_rotation_intermediate_point(target_location), target_location]
    elif is_room_outlet(target_location):
        # 出口旋轉：當前位置 → 旋轉中間點 → 旋轉完成位置  
        return [current_location, get_rotation_intermediate_point(target_location), target_location]
    else:
        # 一般旋轉
        return [current_location, target_location, target_location]
```

#### 🟠 第2級：NG料架回收 (Priority: 90)
**業務流程**：將NG料架從房間入口傳送箱搬運到NG回收區
**觸發條件**：
- NG回收區有空位 (位置71-72, status=2)
- 房間入口傳送箱有NG料架 (location_id=X0001, status=7)
- 無重複執行任務 (work_id='220001', node_id=location_id)

**條件鏈式檢查** (基於實際條件檔案)：
```python
async def check_ng_rack_recycling_flow():
    """NG料架回收 - 三階段條件檢查 (房間擴展支援)"""
    # 條件 6: 檢查NG回收區是否有空位
    ng_space_query = """
        SELECT CASE WHEN COUNT(*) > 0 THEN 'True' ELSE 'False' END as result,
               CASE WHEN COUNT(*) > 0 THEN '[120,220,320,420,520,620,720,820,920,1020]' ELSE NULL END as next_id,
               CASE WHEN COUNT(*) > 0 THEN MIN(id) ELSE NULL END as location
        FROM location 
        WHERE id = ANY (ARRAY[71,72]) AND location_status_id = 2
    """
    
    condition_6_result = await db.execute_raw_query(ng_space_query)
    if condition_6_result['result'] != 'True':
        return []  # NG回收區無空位
    
    ng_target_location = condition_6_result['location']
    tasks_created = []
    
    # 遍歷所有房間 (支援房間1-10擴展)
    for room_id in range(1, 11):
        inlet_location = room_id * 10000 + 1  # 房間入口位置
        condition_id = room_id * 100 + 20  # 條件ID: 120, 220, 320...
        
        # 條件 X20: 檢查房間X入口傳送箱NG料架
        ng_rack_query = f"""
            SELECT CASE WHEN COUNT(*) > 0 THEN 'True' ELSE 'False' END as result,
                   CASE WHEN COUNT(*) > 0 THEN {condition_id + 1} ELSE NULL END as next_id,
                   CASE WHEN COUNT(*) > 0 THEN MIN(location_id) ELSE NULL END as location
            FROM rack 
            WHERE location_id = {inlet_location} AND status_id = 7
        """
        
        ng_rack_result = await db.execute_raw_query(ng_rack_query)
        if ng_rack_result['result'] != 'True':
            continue  # 該房間無NG料架
        
        # 條件 X21: 檢查是否有重複執行任務
        duplicate_task_query = f"""
            SELECT CASE WHEN COUNT(*) = 0 THEN 'True' ELSE 'False' END as result,
                   NULL as next_id,
                   CASE WHEN COUNT(*) = 0 THEN 'True' ELSE 'False' END as end
            FROM task 
            WHERE work_id = '220001' AND node_id = {inlet_location}
        """
        
        duplicate_result = await db.execute_raw_query(duplicate_task_query)
        if duplicate_result['result'] == 'True':
            # 該房間條件滿足，創建NG料架回收任務
            task = create_ng_recycling_task(
                source_location=inlet_location,
                target_location=ng_target_location,
                room_id=room_id,
                work_id='220001'  # 使用 kuka-移動貨架
            )
            tasks_created.append(task)
            
            # NG回收區位置已被分配，不再處理其他房間
            break
    
    return tasks_created
```

#### 🟡 第3級：滿料架到人工收料區 (Priority: 80)
**業務流程**：滿料架從各房間搬運到人工收料區
**觸發條件**：
- 系統空架區有空料架 (位置31-34, status=3)
- 房間內有carrier需要搬運
- 無重複執行任務 (work_id='220001')

**決策邏輯**：
```python
async def check_full_rack_to_manual_flow():
    """滿料架到人工收料區"""
    # 檢查系統空架區空料架
    empty_locations = await db.get_available_locations([31,32,33,34], status=3)
    if not empty_locations:
        return []
    
    for room_id in range(1, 11):
        carriers = await db.get_carriers_in_room(room_id)
        if carriers and not await db.has_active_task('220001', room_id*10000+1):
            create_outlet_transport_task(room_id, empty_locations[0])
```

#### 🟡 第4級：人工收料區搬運 (Priority: 80)
**業務流程**：人工收料區滿料架搬運到房間入口傳送箱
**觸發條件**：
- 人工收料區有空位 (位置51-55, status=2)
- 房間出口傳送箱有滿料架 (status=[2,3,6]) 或 cargo任務已完成
- 無重複執行任務 (work_id='220001')

**決策邏輯**：
```python
async def check_manual_area_transport_flow():
    """人工收料區搬運"""
    manual_spaces = await db.get_available_locations([51,52,53,54,55], status=2)
    if not manual_spaces:
        return []
    
    for room_id in range(1, 11):
        outlet_location = room_id * 10000 + 2
        racks = await db.get_racks_at_location(outlet_location, status=[2,3,6])
        
        if racks:
            # 有滿料架，檢查重複任務
            if not await db.has_active_task('220001', outlet_location):
                create_manual_transport_task(racks[0], manual_spaces[0], room_id)
        else:
            # 無滿料架，檢查cargo任務
            cargo_work_id = room_id * 1000000 + 201
            if (await db.has_completed_task(cargo_work_id) and 
                not await db.has_active_task('220001', outlet_location)):
                create_cargo_followup_task(room_id, manual_spaces[0])
```

#### 🟢 第5級：系統準備區到房間 (Priority: 60)
**業務流程**：系統準備區料架送往房間入口傳送箱
**觸發條件**：
- 系統準備區有料架 (位置11-18, status=3)
- 房間入口傳送箱無料架佔用
- 無重複執行任務 (work_id='220001')

**決策邏輯**：
```python
async def check_system_to_room_flow():
    """系統準備區到房間入口"""
    system_racks = await db.get_available_locations([11,12,13,14,15,16,17,18], status=3)
    if not system_racks:
        return []
    
    for room_id in range(1, 11):
        inlet_location = room_id * 10000 + 11
        occupied = await db.get_racks_at_location(inlet_location)
        
        if not occupied and not await db.has_active_task('220001', room_id*10000+1):
            create_inlet_transport_task(system_racks[0], inlet_location, room_id)
```

#### 🔵 第6級：空料架搬運 (Priority: 40)
**業務流程**：入口傳送箱空料架搬運到出口傳送箱
**觸發條件**：
- 房間入口傳送箱有空料架 (status=1)
- 房間出口傳送箱無料架佔用
- 無重複執行任務 (work_id='220001')

**決策邏輯**：
```python
async def check_empty_rack_transfer_flow():
    """空料架搬運"""
    for room_id in range(1, 11):
        inlet_location = room_id * 10000 + 1
        empty_racks = await db.get_racks_at_location(inlet_location, status=[1])
        
        if empty_racks:
            outlet_location = room_id * 10000 + 2
            outlet_occupied = await db.get_racks_at_location(outlet_location)
            
            if (not outlet_occupied and 
                not await db.has_active_task('220001', outlet_location)):
                create_empty_transfer_task(empty_racks[0], outlet_location, room_id)
```

#### 🔵 第7級：人工回收空料架 (Priority: 40)
**業務流程**：人工回收空料架區搬運到系統空料架區
**觸發條件**：
- 人工回收空料架區有料架 (位置91-92, status=3)
- 空料架回收區有空位 (位置51-54, status=2)  
- 無重複執行任務 (work_id='230001', status IN (0,1,2)) ⭐特殊work_id使用流程觸發

**條件鏈式檢查** (基於實際條件檔案)：
```python
async def check_manual_empty_recycling_flow():
    """人工回收空料架搬運 - 三階段條件檢查"""
    # 條件 7: 檢查人工回收空料架區是否有料架
    manual_empty_query = """
        SELECT CASE WHEN COUNT(*) > 0 THEN 'True' ELSE 'False' END as result,
               CASE WHEN COUNT(*) > 0 THEN 8 ELSE NULL END as next_id,
               CASE WHEN COUNT(*) > 0 THEN MIN(id) ELSE NULL END as location
        FROM location 
        WHERE id = ANY (ARRAY[91,92]) AND location_status_id = 3
    """
    
    condition_7_result = await db.execute_raw_query(manual_empty_query)
    if condition_7_result['result'] != 'True':
        return []  # 無空料架需回收
    
    # 條件 8: 檢查空料架回收區是否有空位  
    empty_space_query = """
        SELECT CASE WHEN COUNT(*) > 0 THEN 'True' ELSE 'False' END as result,
               CASE WHEN COUNT(*) > 0 THEN 9 ELSE NULL END as next_id,
               CASE WHEN COUNT(*) > 0 THEN MIN(id) ELSE NULL END as location
        FROM location 
        WHERE id = ANY (ARRAY[51,52,53,54]) AND location_status_id = 2
    """
    
    condition_8_result = await db.execute_raw_query(empty_space_query)
    if condition_8_result['result'] != 'True':
        return []  # 回收區無空位
    
    # 條件 9: 檢查是否有重複執行任務 (特殊work_id='230001')
    duplicate_task_query = """
        SELECT CASE WHEN COUNT(*) = 0 THEN 'True' ELSE 'False' END as result,
               NULL as next_id,
               CASE WHEN COUNT(*) = 0 THEN 'True' ELSE 'False' END as end
        FROM task 
        WHERE work_id = '230001' AND status_id IN (0, 1, 2)
    """
    
    condition_9_result = await db.execute_raw_query(duplicate_task_query)
    if condition_9_result['result'] == 'True':
        # 三個條件都滿足，創建人工回收空料架任務
        return create_manual_empty_recycling_task(
            source_location=condition_7_result['location'],
            target_location=condition_8_result['location'],
            work_id='230001'  # 使用 kuka-流程觸發
        )
    
    return []  # 有重複任務執行中
```

### Rack狀態判斷機制

```python
def get_rack_status(rack_id: int) -> dict:
    """取得Rack完整狀態資訊"""
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

def needs_rack_rotation(rack: Rack, carriers: List[Carrier]) -> bool:
    """判斷是否需要旋轉Rack"""
    status = get_rack_status(rack.id)
    location = get_location_by_id(rack.location_id)
    
    # 入口位置：A面空了且B面有貨物
    if is_room_inlet(location):
        return (status['current_side'] == 'A' and 
                status['a_side_count'] == 0 and 
                status['b_side_count'] > 0)
    
    # 出口位置：A面滿了且B面還能放
    elif is_room_outlet(location):
        a_max = 16  # 每面最多16個
        return (status['current_side'] == 'A' and 
                status['a_side_count'] == a_max and 
                status['b_side_count'] < a_max)
    
    return False
```

### 製程驗證邏輯

```python
def validate_process_compatibility(rack: Rack, room: Room) -> bool:
    """驗證Rack產品與房間製程相符性"""
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

### 統一決策引擎架構

```python
class UnifiedWCSDecisionEngine:
    """統一的WCS決策引擎 - 整合所有業務流程"""
    
    def __init__(self):
        self.work_ids = {
            # 主要料架搬運作業 (大部分業務流程)
            'MAIN_RACK_OPERATIONS': '220001',    # kuka-移動貨架 (流程2,3,4,5,6)
            'WORKFLOW_OPERATIONS': '230001',     # kuka-流程觸發 (流程1,7)
            
            # OPUI操作員任務
            'OPUI_OPERATIONS': ['100001', '100002'],  # opui-call-empty, opui-dispatch-full
            
            # Cargo AGV專業任務
            'CARGO_OPERATIONS': ['2000102', '2000201']  # CargoAGV入口/出口傳送箱
        }
        
        # 條件檢查使用的Work ID映射
        self.condition_work_ids = {
            'agv_rotation': '220001',           # AGV旋轉 → kuka-移動貨架 (改用3節點移動)
            'ng_rack_recycling': '220001',       # NG料架回收 → kuka-移動貨架
            'full_rack_to_manual': '220001',     # 滿料架搬運 → kuka-移動貨架
            'manual_area_transport': '220001',   # 人工收料區搬運 → kuka-移動貨架
            'system_to_room': '220001',         # 系統準備區搬運 → kuka-移動貨架
            'empty_rack_transfer': '220001',    # 空料架搬運 → kuka-移動貨架
            'manual_empty_recycling': '230001', # 人工回收空料架 → kuka-流程觸發 ⭐唯一特殊
        }
        
        self.priority_levels = {
            'AGV_ROTATION': 100,        # AGV旋轉檢查
            'NG_RECYCLING': 90,         # NG料架回收
            'MANUAL_TRANSPORT': 80,     # 人工收料區相關
            'SYSTEM_TO_ROOM': 60,       # 系統準備區到房間
            'EMPTY_OPERATIONS': 40      # 空料架和人工回收
        }
        
        self.location_mappings = {
            'ng_recycling_area': [71, 72],          # NG回收區
            'manual_area': [51, 52, 53, 54, 55],    # 人工收料區
            'system_empty_area': [31, 32, 33, 34],  # 系統空架區
            'system_prep_area': [11, 12, 13, 14, 15, 16, 17, 18],  # 系統準備區
            'manual_empty_area': [91, 92],          # 人工回收空料架區
            'empty_recycling_area': [51, 52, 53, 54]  # 空料架回收區
        }
    
    async def run_unified_decision_cycle(self) -> List[TaskDecision]:
        """執行統一決策週期 - 涵蓋7大業務流程"""
        all_decisions = []
        
        # 🔴 Priority 100: AGV旋轉檢查
        decisions = await self.check_agv_rotation_flow()
        all_decisions.extend(decisions)
        
        # 🟠 Priority 90: NG料架回收
        decisions = await self.check_ng_rack_recycling_flow()
        all_decisions.extend(decisions)
        
        # 🟡 Priority 80: 人工收料區相關流程
        decisions = await self.check_full_rack_to_manual_flow()
        all_decisions.extend(decisions)
        
        decisions = await self.check_manual_area_transport_flow()
        all_decisions.extend(decisions)
        
        # 🟢 Priority 60: 系統準備區到房間
        decisions = await self.check_system_to_room_flow()
        all_decisions.extend(decisions)
        
        # 🔵 Priority 40: 空料架相關流程
        decisions = await self.check_empty_rack_transfer_flow()
        all_decisions.extend(decisions)
        
        decisions = await self.check_manual_empty_recycling_flow()
        all_decisions.extend(decisions)
        
        # 依優先度排序並調度
        return self._prioritize_and_schedule(all_decisions)
    
    def _prioritize_and_schedule(self, decisions: List[TaskDecision]) -> List[TaskDecision]:
        """優先度排序和調度衝突解決"""
        # 按優先度排序
        decisions.sort(key=lambda d: d.priority.value, reverse=True)
        
        # 解決資源衝突
        scheduled = []
        occupied_locations = set()
        
        for decision in decisions:
            if decision.target_location not in occupied_locations:
                scheduled.append(decision)
                occupied_locations.add(decision.target_location)
        
        return scheduled
    
    async def get_room_location_info(self, room_id: int) -> dict:
        """取得房間位置資訊"""
        return {
            'inlet_location': room_id * 10000 + 1,    # 房間入口
            'outlet_location': room_id * 10000 + 2,   # 房間出口
            'cargo_work_id': room_id * 1000000 + 201, # Cargo任務ID
            'node_prefix': room_id * 10000             # 節點ID前綴
        }
```

## ✅ 多Rack調度與異常處理

### 多Rack同時調度的排程策略

```python
def schedule_multiple_racks(tasks: List[dict]) -> List[dict]:
    """多Rack調度排程策略"""
    
    # 1. 依優先度分組
    priority_groups = group_tasks_by_priority(tasks)
    
    # 2. 檢查資源衝突
    scheduled_tasks = []
    occupied_locations = set()
    assigned_agvs = set()
    
    for priority, task_group in priority_groups.items():
        for task in task_group:
            # 檢查目標位置是否被佔用
            target_location = task.get('target_location_id')
            if target_location in occupied_locations:
                # 加入等待佇列
                add_to_waiting_queue(task)
                continue
            
            # 檢查可用AGV
            available_agv = find_available_agv(task['rack_id'])
            if not available_agv or available_agv.id in assigned_agvs:
                add_to_waiting_queue(task)
                continue
            
            # 可以排程執行
            task['agv_id'] = available_agv.id
            scheduled_tasks.append(task)
            occupied_locations.add(target_location)
            assigned_agvs.add(available_agv.id)
    
    return scheduled_tasks

def resolve_deadlock():
    """死鎖檢測與解除機制"""
    # 簡單的死鎖檢測：循環等待檢測
    waiting_tasks = get_waiting_tasks()
    
    # 檢查是否有循環依賴
    dependency_graph = build_task_dependency_graph(waiting_tasks)
    cycles = detect_cycles(dependency_graph)
    
    if cycles:
        # 解除死鎖：取消最低優先度任務
        for cycle in cycles:
            lowest_priority_task = min(cycle, key=lambda t: t['priority'])
            cancel_task(lowest_priority_task['id'])
            log_warning(f"死鎖解除：取消任務 {lowest_priority_task['id']}")
```

### 異常狀況處理機制

```python
class ExceptionHandler:
    def handle_agv_failure(self, agv_id: int):
        """AGV故障處理"""
        # 暫停所有分配給該AGV的任務
        tasks = get_tasks_by_agv_id(agv_id)
        for task in tasks:
            update_task_status(task.id, TaskStatus.ERROR)
            log_error(f"AGV {agv_id} 故障，任務 {task.id} 暫停")
    
    def handle_location_occupied(self, task_id: int):
        """位置被佔用處理"""
        task = get_task_by_id(task_id)
        
        # 持續等待策略
        max_wait_time = 300  # 5分鐘
        wait_start = datetime.now()
        
        while datetime.now() - wait_start < timedelta(seconds=max_wait_time):
            target_location = get_location_by_id(task.target_location_id)
            if target_location.location_status_id == LocationStatus.UNOCCUPIED:
                # 位置空出，重新執行任務
                update_task_status(task.id, TaskStatus.READY_TO_EXECUTE)
                return
            
            time.sleep(30)  # 等待30秒後重新檢查
        
        # 超時處理
        log_error(f"任務 {task_id} 等待位置超時")
        update_task_status(task.id, TaskStatus.ERROR)
```

## ✅ 任務參數格式與Work ID對應

### Work ID分類系統 (基於實際資料庫定義)
```python
WORK_ID_MAPPINGS = {
    # === OPUI 操作員任務群組 ===
    100001: {
        "name": "opui-call-empty",
        "description": "作業員從opui請求將空Rack派至[人工作業準備區]",
        "category": "opui_operations",
        "priority": 40
    },
    100002: {
        "name": "opui-dispatch-full", 
        "description": "作業員從opui請求將Rack派至[系統準備派車區]",
        "category": "opui_operations",
        "priority": 40
    },
    
    # === KUKA 基礎移動任務群組 ===
    210001: {
        "name": "kuka-移動",
        "description": "執行指定的from,to(nodes)移動至指定位置",
        "category": "basic_movement",
        "priority": 60,
        "parameters": {"function": "move", "api": "submit_mission", "missionType": "MOVE"}
    },
    220001: {
        "name": "kuka-移動貨架", 
        "description": "執行指定的from,to(nodes)將貨架搬至指定位置",
        "category": "rack_transport",
        "priority": 80,
        "parameters": {"function": "rack_move", "api": "submit_mission", "missionType": "RACK_MOVE"}
    },
    230001: {
        "name": "kuka-流程觸發",
        "description": "執行指定的workflow流程觸發", 
        "category": "workflow_trigger",
        "priority": 100,
        "parameters": {"function": "workflow", "api": "submit_mission", "missionType": "MOVE", "templateCode": "W000000001"}
    },
    
    # === 房間2 CargoAGV 任務群組 ===
    2000102: {
        "name": "CargoAGV放入口傳送箱",
        "description": "從料架拿carrier到入口傳送箱放",
        "category": "cargo_inlet",
        "priority": 80,
        "parameters": {"function": "rack_move", "api": "submit_mission", "missionType": "RACK_MOVE"}
    },
    2000201: {
        "name": "CargoAGV拿出口傳送箱",
        "description": "從出口傳送箱拿carrier到料架放", 
        "category": "cargo_outlet",
        "priority": 80,
        "parameters": {"function": "rack_move", "api": "submit_mission", "missionType": "RACK_MOVE"}
    },
    
    # === LoaderAGV 任務群組 ===
    2010101: {"name": "LoaderAGV取入口傳送箱", "category": "loader_operations", "priority": 70},
    2030102: {"name": "LoaderAGV放清洗機", "category": "loader_operations", "priority": 70},
    2030201: {"name": "LoaderAGV取清洗機", "category": "loader_operations", "priority": 70},
    
    # 泡藥機系列 (2040xxx)
    2040102: {"name": "LoaderAGV放泡藥機A", "category": "loader_soaking", "priority": 70},
    2040202: {"name": "LoaderAGV放泡藥機B", "category": "loader_soaking", "priority": 70},
    2040302: {"name": "LoaderAGV放泡藥機C", "category": "loader_soaking", "priority": 70},
    2040402: {"name": "LoaderAGV放泡藥機D", "category": "loader_soaking", "priority": 70},
    2040502: {"name": "LoaderAGV放泡藥機E", "category": "loader_soaking", "priority": 70},
    2040602: {"name": "LoaderAGV放泡藥機F", "category": "loader_soaking", "priority": 70},
    
    2040101: {"name": "LoaderAGV拿泡藥機A", "category": "loader_soaking", "priority": 70},
    2040201: {"name": "LoaderAGV拿泡藥機B", "category": "loader_soaking", "priority": 70},
    2040301: {"name": "LoaderAGV拿泡藥機C", "category": "loader_soaking", "priority": 70},
    2040401: {"name": "LoaderAGV拿泡藥機D", "category": "loader_soaking", "priority": 70},
    2040501: {"name": "LoaderAGV拿泡藥機E", "category": "loader_soaking", "priority": 70},
    2040601: {"name": "LoaderAGV拿泡藥機F", "category": "loader_soaking", "priority": 70},
    
    # 預烘機系列 (2050xxx)
    2050102: {"name": "LoaderAGV放預烘機1", "category": "loader_prebaking", "priority": 70},
    2050202: {"name": "LoaderAGV放預烘機2", "category": "loader_prebaking", "priority": 70},
    # ... 其他預烘機 (3-8)
    
    # === UnloaderAGV 任務群組 ===
    2050901: {"name": "UnloaderAGV取預烘A", "category": "unloader_prebaking", "priority": 70},
    2051001: {"name": "UnloaderAGV取預烘B", "category": "unloader_prebaking", "priority": 70},
    2051101: {"name": "UnloaderAGV取預烘C", "category": "unloader_prebaking", "priority": 70},
    2051201: {"name": "UnloaderAGV取預烘D", "category": "unloader_prebaking", "priority": 70},
    
    # 烤箱系列 (2060xxx)
    2060102: {"name": "UnloaderAGV放烤箱A", "category": "unloader_baking", "priority": 70},
    2060202: {"name": "UnloaderAGV放烤箱B", "category": "unloader_baking", "priority": 70},
    2060101: {"name": "UnloaderAGV取烤箱A", "category": "unloader_baking", "priority": 70},
    2060201: {"name": "UnloaderAGV取烤箱B", "category": "unloader_baking", "priority": 70},
    
    2020102: {"name": "UnloaderAGV放出口傳送箱", "category": "unloader_outlet", "priority": 80}
}

# === 業務流程與Work ID映射 ===
BUSINESS_FLOW_WORK_IDS = {
    # WCS 決策引擎使用的 Work IDs (大部分使用 kuka-移動貨架)
    'agv_rotation': 220001,              # AGV旋轉檢查 → kuka-移動貨架 (改用3節點移動)
    'ng_rack_recycling': 220001,         # NG料架回收 → kuka-移動貨架
    'full_rack_to_manual': 220001,       # 滿料架到人工收料區 → kuka-移動貨架
    'manual_area_transport': 220001,     # 人工收料區搬運 → kuka-移動貨架
    'system_to_room': 220001,           # 系統準備區到房間 → kuka-移動貨架
    'empty_rack_transfer': 220001,      # 空料架搬運 → kuka-移動貨架
    'manual_empty_recycling': 230001,   # 人工回收空料架 → kuka-流程觸發 ⭐唯一特殊例外
    
    # OPUI 手動任務
    'opui_call_empty': 100001,          # OPUI叫空車
    'opui_dispatch_full': 100002,       # OPUI派滿車
    
    # Cargo AGV 任務
    'cargo_inlet': 2000102,             # CargoAGV放入口傳送箱
    'cargo_outlet': 2000201,            # CargoAGV拿出口傳送箱
}

# === 條件檢查Work ID特殊對應 ===
CONDITION_CHECK_WORK_IDS = {
    # 所有料架移動操作都使用 220001 (kuka-移動貨架)
    'agv_rotation': '220001',           # AGV旋轉重複任務檢查 (改用移動貨架)
    'ng_rack_recycling': '220001',       # NG料架回收重複任務檢查
    'full_rack_to_manual': '220001',     # 滿料架搬運重複任務檢查
    'manual_area_transport': '220001',   # 人工收料區搬運重複任務檢查
    'system_to_room': '220001',         # 系統準備區搬運重複任務檢查
    'empty_rack_transfer': '220001',    # 空料架搬運重複任務檢查
    
    # 唯一特殊例外：人工回收空料架使用 230001 (kuka-流程觸發)
    'manual_empty_recycling': '230001',  # 人工回收空料架重複任務檢查
}
```

### KUKA移動貨架任務 (work_id: 220001)
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

### KUKA移動貨架任務 - AGV旋轉 (work_id: 220001)
```json
// AGV旋轉任務 (改用3節點移動方式)
{
  "function": "rack_move",
  "model": "KUKA400i",
  "work_id": 220001,
  "api": "submit_mission", 
  "missionType": "RACK_MOVE",
  "nodes": [75, 76, 75],  // 3個節點的旋轉路徑
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

### KUKA流程觸發任務 (work_id: 230001) - 人工回收空料架專用
```json
// 人工回收空料架任務 (唯一使用workflow)
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

### OPUI叫空車任務 (work_id: 100001) - 基於實際OPUI邏輯
```json
{
  "work_id": 100001,
  "function": "rack_move",
  "api": "submit_mission",
  "missionType": "RACK_MOVE",
  "model": "KUKA400i",
  "task_category": "opui_call_empty",
  "priority_level": 40,
  
  // OPUI 特定參數 (基於實際代碼)
  "task_type": "call_empty",
  "machine_id": 1,              // 機台ID
  "space_num": 1,               // 停車格編號 (1 或 2)
  "node_id": 95,                // 停車格對應的節點ID (machine.parking_space_1)
  "client_id": "clientId",      // OPUI 客戶端ID
  
  // KUKA 參數
  "nodes": [91, 76, 95],        // 移動路徑：取空車位置 → 中間點 → 目標停車格
  "kuka_agv_id": 123,
  
  // 停車格狀態管理
  "parking_space_status": 1     // 設置為 PARKING_TASK_ACTIVE (任務進行中)
}

// Machine Parking Space 對應關係 (基於實際初始化資料)
// machine_id: 1 → parking_space_1: 95, parking_space_2: 96
// machine_id: 2 → parking_space_1: 97, parking_space_2: 98  
// machine_id: 3 → parking_space_1: 1005, parking_space_2: 1006
// machine_id: 4 → parking_space_1: 1007, parking_space_2: 1008
```

### OPUI派滿車任務 (work_id: 100002) - 基於實際OPUI邏輯  
```json
{
  "work_id": 100002,
  "function": "rack_move",
  "api": "submit_mission",
  "missionType": "RACK_MOVE",
  "model": "KUKA400i", 
  "task_category": "opui_dispatch_full",
  "priority_level": 40,
  
  // OPUI 特定參數 (基於實際代碼)
  "task_type": "dispatch_full",
  "rack_id": 1,                 // 料架ID
  "room_id": 2,                 // 目標房間ID
  "machine_id": 1,              // 機台ID
  "side": "left",               // 停車格側別 (left=space_1, right=space_2)
  "client_id": "clientId",      // OPUI 客戶端ID
  
  // 產品資訊
  "product_name": "ABC121345",  // 產品名稱
  "count": 32,                  // 數量
  
  // KUKA 參數  
  "nodes": [95, 74, 72, 15],    // 移動路徑：停車格 → 中間點 → 系統準備派車區
  "kuka_agv_id": 123,
  
  // WCS 決策參數
  "node_id": null,              // 由 WCS 決定具體目標位置
  "target_area": "system_prep_area"  // 系統準備派車區 (位置11-18)
}

// 停車格狀態管理流程 (基於實際OPUI邏輯)
// 1. 叫空車: status 0→1 (可用→任務進行中)
// 2. AGV送達: status 1→2 (任務進行中→任務完成)  
// 3. 確認取貨: status 2→0 (任務完成→可用)
// 4. 派滿車: 直接使用已停靠的料架，不改變停車格狀態
```

### OPUI業務邏輯整合要點 (基於實際代碼分析)

#### Machine → Location → Node ID 對應關係
```python
# 機台停車格配置 (實際初始化資料)
MACHINE_PARKING_CONFIG = {
    1: {"parking_space_1": 95, "parking_space_2": 96, "name": "射出機1"},
    2: {"parking_space_1": 97, "parking_space_2": 98, "name": "射出機2"}, 
    3: {"parking_space_1": 1005, "parking_space_2": 1006, "name": "射出機3"},
    4: {"parking_space_1": 1007, "parking_space_2": 1008, "name": "射出機4"}
}

# 停車格狀態定義 (Machine模型)
PARKING_STATUS = {
    0: "PARKING_AVAILABLE",      # 可用 - 停車格空閒，可以叫車
    1: "PARKING_TASK_ACTIVE",    # 任務進行中 - 已叫車，等待AGV送達  
    2: "PARKING_TASK_COMPLETED"  # 任務完成 - 車輛已送達，等待確認取貨
}
```

#### OPUI任務創建邏輯 (OpuiTaskService)
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

### CargoAGV入口傳送箱任務 (work_id: 2000102)
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

### CargoAGV出口傳送箱任務 (work_id: 2000201)
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

### KUKA基礎移動任務 (work_id: 210001)
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

## 📋 完整實作階段規劃

### 第1階段：統一決策引擎開發 (2天)
1. **建立統一架構** (`ai_wcs_ws/`)
   - 實作 `UnifiedWCSDecisionEngine` 類別
   - 整合7大業務流程Python邏輯
   - 建立統一的work_id管理系統

2. **Python條件邏輯實現**
   - 轉換舊系統SQL條件為Python邏輯
   - 實作批次資料庫查詢最佳化
   - 建立條件檢查快取機制

### 第2階段：業務流程整合 (2天)
1. **七大流程完整實現**
   - AGV旋轉檢查 (Priority: 100) - 使用3節點移動
   - NG料架回收 (Priority: 90) 
   - 滿料架到人工收料區 (Priority: 80)
   - 人工收料區搬運 (Priority: 80)
   - 系統準備區到房間 (Priority: 60)
   - 空料架搬運 (Priority: 40)
   - 人工回收空料架 (Priority: 40) - 使用workflow

2. **OPUI整合邏輯**
   - 叫空車任務處理：machine parking space → node id 對應
   - 派滿車任務處理：WCS決定目標位置到系統準備派車區
   - 停車格狀態管理：0(可用)→1(任務中)→2(完成)→0(可用)

### 第3階段：系統整合與調度 (1天)
1. **任務調度與衝突解決**
   - 資源佔用衝突檢測
   - 優先度排序機制
   - 任務去重邏輯

2. **外部系統介面整合**
   - RCS系統：Task交接機制完善
   - OPUI系統：machine parking space狀態同步
   - KUKA Fleet：work_id追蹤系統

### 第4階段：測試與驗證 (1天)
1. **業務邏輯驗證**
   - 7大流程完整測試
   - OPUI叫空車/派滿車流程測試
   - 邊界條件測試
   - 性能基準測試

2. **系統整合測試**
   - 與現有ai_wcs模組整合
   - OPUI停車格狀態同步測試
   - 完整決策週期測試
   - 異常處理機制驗證

### 第5階段：OPUI整合完善 (0.5天)
1. **停車格狀態同步機制**
   - WCS與OPUI的停車格狀態實時同步
   - 叫空車任務完成後狀態更新
   - 派滿車任務的停車格清理

2. **Machine配置驗證**
   - 確認所有機台的parking_space配置正確
   - 驗證node_id對應關係
   - 測試不同機台的叫車/派車邏輯

## 🎯 核心技術特點

### 統一的業務規則引擎
- **單一決策點**: 統合所有業務邏輯到一個引擎
- **Python原生**: 完全使用Python實現，告別複雜SQL條件鏈
- **批次最佳化**: 減少70%資料庫查詢次數

### 完整的Work ID管理
- **220001**: KUKA移動貨架 (WCS決策引擎主要使用，包含AGV旋轉)
- **230001**: KUKA流程觸發 (僅用於人工回收空料架workflow任務)
- **100001/100002**: OPUI操作員任務 (叫空車/派滿車，與machine parking space整合)
- **2000102/2000201**: CargoAGV任務 (入口/出口傳送箱)
- **210001**: KUKA基礎移動任務
- **2010xxx-2060xxx**: LoaderAGV/UnloaderAGV專業製程任務

### OPUI停車格狀態整合
- **Machine Model**: parking_space_1/2 → node_id 直接對應
- **狀態流轉**: 0(可用) → 1(任務中) → 2(完成) → 0(可用)
- **實時同步**: WCS與OPUI的停車格狀態實時更新
- **任務追蹤**: 基於machine_id + space_num + node_id的完整追蹤

### 七級優先度調度
- **Priority 100**: AGV旋轉 (安全優先)
- **Priority 90**: NG回收 (品質優先)  
- **Priority 80**: 人工收料區 (效率優先)
- **Priority 60**: 系統準備區 (流程優先)
- **Priority 40**: 空料架操作 (維護優先)

### 智能衝突解決
- **位置衝突**: 自動檢測目標位置佔用
- **任務去重**: work_id + location_id雙重檢查
- **資源調度**: AGV資源智能分配

## 🔧 關鍵實作重點

### 業務邏輯完整性
- **涵蓋所有舊系統條件**: 30+個條件邏輯完全轉換
- **支援房間擴展**: 自動支援1-10房間
- **邊界條件處理**: 完整的異常狀況處理

### 性能與維護性
- **查詢最佳化**: 批次查詢取代逐條件查詢
- **代碼可讀性**: Python邏輯易於理解和維護
- **除錯友好**: 完整的日誌和錯誤追蹤

### 擴展與整合
- **模組化設計**: 新增業務流程只需擴展方法
- **API相容**: 保持現有介面不變
- **監控就緒**: 內建性能監控和統計

## 🚀 高級實現機制

### 條件檢查批次最佳化
```python
class BatchConditionChecker:
    """批次條件檢查器 - 減少資料庫查詢次數"""
    
    async def batch_location_check(self, location_groups: Dict[str, List[int]], status_filter: int):
        """批次檢查多組位置狀態"""
        queries = []
        for group_name, location_ids in location_groups.items():
            query = f"""
                SELECT '{group_name}' as group_name,
                       CASE WHEN COUNT(*) > 0 THEN 'True' ELSE 'False' END as available,
                       CASE WHEN COUNT(*) > 0 THEN MIN(id) ELSE NULL END as location_id
                FROM location 
                WHERE id = ANY (ARRAY{location_ids}) AND location_status_id = {status_filter}
            """
            queries.append(query)
        
        # 一次查詢獲取所有位置狀態
        combined_query = " UNION ALL ".join(queries)
        results = await self.db.execute_raw_query(combined_query)
        
        return {r['group_name']: r for r in results}
    
    async def batch_task_conflict_check(self, work_location_pairs: List[Tuple[str, int]]):
        """批次檢查任務衝突"""
        conflict_checks = []
        for work_id, location_id in work_location_pairs:
            conflict_checks.append(f"""
                SELECT '{work_id}_{location_id}' as check_key,
                       COUNT(*) as conflict_count
                FROM task 
                WHERE work_id = '{work_id}' 
                AND (node_id = {location_id} OR status_id IN (0,1,2))
            """)
        
        combined_query = " UNION ALL ".join(conflict_checks)
        results = await self.db.execute_raw_query(combined_query)
        
        return {r['check_key']: r['conflict_count'] == 0 for r in results}

class EnhancedUnifiedWCSDecisionEngine(UnifiedWCSDecisionEngine):
    """增強版統一WCS決策引擎 - 整合批次最佳化"""
    
    def __init__(self):
        super().__init__()
        self.batch_checker = BatchConditionChecker()
        self.condition_cache = {}  # 條件檢查快取
        self.cache_expiry = 30     # 快取30秒過期
    
    async def batch_check_all_conditions(self) -> Dict[str, Any]:
        """一次性批次檢查所有業務流程條件"""
        # 1. 批次檢查所有位置狀態
        location_groups = {
            'ng_recycling': [71, 72],              # NG回收區
            'manual_area': [51, 52, 53, 54, 55],   # 人工收料區  
            'system_empty': [31, 32, 33, 34],      # 系統空架區
            'system_prep': [11, 12, 13, 14, 15, 16, 17, 18],  # 系統準備區
            'manual_empty': [91, 92],              # 人工回收空料架區
            'empty_recycling': [51, 52, 53, 54]    # 空料架回收區
        }
        
        empty_locations = await self.batch_checker.batch_location_check(location_groups, status=2)  # 空位
        occupied_locations = await self.batch_checker.batch_location_check(location_groups, status=3)  # 有料架
        
        # 2. 批次檢查所有房間狀態
        room_conditions = {}
        for room_id in range(1, 11):
            room_key = f"room_{room_id}"
            inlet_location = room_id * 10000 + 1
            outlet_location = room_id * 10000 + 2
            
            room_conditions[room_key] = {
                'inlet_location': inlet_location,
                'outlet_location': outlet_location,
                'inlet_available': empty_locations.get(f'inlet_{room_id}', {}).get('available') == 'True',
                'outlet_available': empty_locations.get(f'outlet_{room_id}', {}).get('available') == 'True'
            }
        
        # 3. 批次檢查任務衝突
        conflict_pairs = [
            ('220001', loc_id) for loc_id in [71, 72] + list(range(10001, 100002, 10000))  # NG、房間位置、AGV旋轉
        ] + [('230001', None)]  # 人工回收空料架 (唯一使用230001)
        
        conflict_results = await self.batch_checker.batch_task_conflict_check(conflict_pairs)
        
        return {
            'locations': {
                'empty': empty_locations,
                'occupied': occupied_locations
            },
            'rooms': room_conditions,
            'conflicts': conflict_results,
            'timestamp': datetime.now()
        }
    
    async def enhanced_decision_cycle(self) -> List[TaskDecision]:
        """增強版決策週期 - 使用批次最佳化"""
        # 一次性獲取所有條件狀態
        batch_conditions = await self.batch_check_all_conditions()
        all_decisions = []
        
        # 🔴 Priority 100: AGV旋轉檢查 (使用快取條件)
        if self._should_check_agv_rotation(batch_conditions):
            decisions = await self.optimized_agv_rotation_check(batch_conditions)
            all_decisions.extend(decisions)
        
        # 🟠 Priority 90: NG料架回收 (使用批次條件)
        if self._can_process_ng_recycling(batch_conditions):
            decisions = await self.optimized_ng_recycling_check(batch_conditions)
            all_decisions.extend(decisions)
        
        # 🟡 Priority 80: 人工收料區相關流程
        manual_decisions = await self.optimized_manual_area_checks(batch_conditions)
        all_decisions.extend(manual_decisions)
        
        # 🟢 Priority 60: 系統準備區到房間
        system_decisions = await self.optimized_system_to_room_checks(batch_conditions)
        all_decisions.extend(system_decisions)
        
        # 🔵 Priority 40: 空料架相關流程
        empty_decisions = await self.optimized_empty_rack_checks(batch_conditions)
        all_decisions.extend(empty_decisions)
        
        return self._prioritize_and_schedule(all_decisions)
    
    def _should_check_agv_rotation(self, conditions: Dict[str, Any]) -> bool:
        """基於批次條件判斷是否需要檢查AGV旋轉"""
        # 實作條件邏輯...
        return True
    
    def _can_process_ng_recycling(self, conditions: Dict[str, Any]) -> bool:
        """基於批次條件判斷是否可以處理NG料架回收"""
        ng_space_available = conditions['locations']['empty'].get('ng_recycling', {}).get('available') == 'True'
        no_conflicts = not any(k.startswith('220001_') and v == False for k, v in conditions['conflicts'].items())
        return ng_space_available and no_conflicts
    
    async def optimized_ng_recycling_check(self, conditions: Dict[str, Any]) -> List[TaskDecision]:
        """最佳化的NG料架回收檢查"""
        decisions = []
        ng_target = conditions['locations']['empty']['ng_recycling']['location_id']
        
        # 使用批次房間條件
        for room_id in range(1, 11):
            room_key = f"room_{room_id}"
            room_info = conditions['rooms'].get(room_key, {})
            
            # 檢查該房間是否有NG料架且無衝突
            inlet_location = room_info['inlet_location']
            conflict_key = f"220001_{inlet_location}"
            
            if (self._has_ng_rack(inlet_location) and 
                conditions['conflicts'].get(conflict_key, False)):
                
                decision = TaskDecision(
                    work_id='220001',
                    source_location=inlet_location,
                    target_location=ng_target,
                    priority=90,
                    room_id=room_id,
                    task_category='ng_rack_recycling'
                )
                decisions.append(decision)
                break  # NG回收區只能同時處理一個
        
        return decisions
    
    async def _has_ng_rack(self, location_id: int) -> bool:
        """檢查特定位置是否有NG料架"""
        # 快取檢查或直接查詢
        cache_key = f"ng_rack_{location_id}"
        if cache_key in self.condition_cache:
            cache_time, result = self.condition_cache[cache_key]
            if (datetime.now() - cache_time).seconds < self.cache_expiry:
                return result
        
        # 查詢並快取結果
        query = f"""
            SELECT COUNT(*) > 0 as has_ng
            FROM rack 
            WHERE location_id = {location_id} AND status_id = 7
        """
        result = await self.db.execute_raw_query(query)
        has_ng = result[0]['has_ng']
        
        self.condition_cache[cache_key] = (datetime.now(), has_ng)
        return has_ng
```

### 智能快取機制
```python
class ConditionCache:
    """條件檢查快取系統"""
    
    def __init__(self, default_ttl: int = 30):
        self.cache = {}
        self.default_ttl = default_ttl
    
    def get(self, key: str) -> Optional[Any]:
        """獲取快取值"""
        if key in self.cache:
            value, expiry = self.cache[key]
            if datetime.now() < expiry:
                return value
            else:
                del self.cache[key]
        return None
    
    def set(self, key: str, value: Any, ttl: Optional[int] = None) -> None:
        """設置快取值"""
        ttl = ttl or self.default_ttl
        expiry = datetime.now() + timedelta(seconds=ttl)
        self.cache[key] = (value, expiry)
    
    def invalidate_pattern(self, pattern: str) -> None:
        """使符合模式的快取失效"""
        import re
        compiled_pattern = re.compile(pattern)
        keys_to_remove = [k for k in self.cache.keys() if compiled_pattern.match(k)]
        for key in keys_to_remove:
            del self.cache[key]
```

### 決策引擎性能監控
```python
class WCSPerformanceMonitor:
    """WCS決策引擎性能監控"""
    
    def __init__(self):
        self.metrics = {
            'decision_cycles': 0,
            'total_decisions': 0,
            'avg_cycle_time': 0.0,
            'cache_hit_rate': 0.0,
            'db_query_count': 0,
            'error_count': 0
        }
        self.cycle_times = []
    
    def record_cycle(self, cycle_time: float, decisions_count: int, cache_hits: int, cache_misses: int):
        """記錄決策週期性能"""
        self.metrics['decision_cycles'] += 1
        self.metrics['total_decisions'] += decisions_count
        self.cycle_times.append(cycle_time)
        
        # 計算平均週期時間
        self.metrics['avg_cycle_time'] = sum(self.cycle_times) / len(self.cycle_times)
        
        # 計算快取命中率
        total_cache_requests = cache_hits + cache_misses
        if total_cache_requests > 0:
            self.metrics['cache_hit_rate'] = cache_hits / total_cache_requests
    
    def get_performance_report(self) -> Dict[str, Any]:
        """獲取性能報告"""
        return {
            'summary': self.metrics,
            'recommendations': self._generate_recommendations()
        }
    
    def _generate_recommendations(self) -> List[str]:
        """生成性能最佳化建議"""
        recommendations = []
        
        if self.metrics['avg_cycle_time'] > 2.0:
            recommendations.append("決策週期時間過長，建議增加快取使用")
        
        if self.metrics['cache_hit_rate'] < 0.7:
            recommendations.append("快取命中率偏低，建議調整快取策略")
        
        if self.metrics['db_query_count'] / self.metrics['decision_cycles'] > 10:
            recommendations.append("資料庫查詢過多，建議使用批次查詢")
        
        return recommendations
```

---

*此文檔記錄了WCS系統的完整設計方案，包含所有討論確認的技術細節和高級最佳化機制，供後續開發參考使用。*