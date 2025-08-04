# KUKA 任務判斷系統架構分析

## 概述

RosAGV 系統中的 KUKA 任務判斷機制是透過 `task_condition` 資料表儲存複雜的 SQL 條件語法實現的。這些條件會被持續查詢執行，查詢結果儲存在 `results` 欄位（JSONB 格式）中，用於 AGV 倉儲自動化的業務決策。

## 核心機制

### 資料表結構

#### TaskCondition (主要表)
```python
class TaskCondition(SQLModel, table=True):
    __tablename__ = "task_condition"
    
    id: Optional[int] = Field(default=None, primary_key=True, description="主鍵")
    conditions: str = Field(sa_column=Column(Text), description="條件內容（TEXT 格式）")
    results: Optional[Dict[str, Any]] = Field(default=None, sa_column=Column(JSON), description="結果資料（JSONB 格式）")
    description: Optional[str] = Field(default=None, max_length=500, description="條件描述")
```

#### 支援表
- **TaskConditionHistory**：提供完整的條件歷史追蹤，包含時間戳、過期時間、錯誤訊息等
- **TaskConditionCache**：提供高效能的條件查詢快取，包含快取管理機制

### 執行流程

1. **條件定義階段**：在 `conditions/` 目錄中定義各種業務條件的 SQL 查詢
2. **條件初始化**：透過 `init_data/21_task_condition.py` 將條件載入到資料庫
3. **條件執行**：
   - 透過 ROS 2 服務 `/agvc/sql_query` 執行 SQL 查詢
   - 使用 `handle_sql_query` 處理查詢請求
   - 執行 `session.exec(text(query_string))` 執行 SQL
4. **結果處理**：
   - 查詢結果轉換為 JSON 格式
   - 透過 `task_condition_crud.update_results()` 更新結果到 `results` 欄位
   - 結果以 JSONB 格式儲存在資料庫中

## 七大業務流程模組

### 1. 空料架搬運到出口傳送箱流程 (empty_rack_to_boxout.py)

**業務目標**：將系統空料架區的空料架搬運到出口傳送箱

**關鍵條件**：
- **條件 1**：系統空架區-有空料架（起始條件）
  - 檢查 location 表中 ID 為 31-34 且狀態為 3（有空料架）的位置
  - 返回 next_id 指向房間檢查列表 [101,201,301,401,501,601,701,801,901,1001]

- **條件 201-202**：出口傳送箱空間檢查
  - 檢查 location 表中出口傳送箱1-2的可用空間（location_status_id = 2 未佔用）
  - 確認出口傳送箱是否有空間接收空料架

- **條件 203**：重複任務檢查
  - 防止相同 work_id 和 node_id 的重複任務產生

### 2. 滿料架搬運到人工收料區流程 (full_rack_to_manual.py)

**業務目標**：將房間出口傳送箱的滿料架搬運到人工收料區

**關鍵條件**：
- **條件 2**：滿料架搬運到人工收料區 - 檢查人工收料區是否有空位
  - 檢查 location 表中 ID 為 51-55 且狀態為 2（空位）的位置
  - next_id 指向 [105,205,305,405,505,605,705,805,905,1005]

- **條件 205**：房間2出口傳送箱滿料架檢查 - 檢查是否有滿料架需要搬運
  - 查詢 rack 表中 location_id = 20002 且狀態為滿料架的記錄
  - 有料架跳 206，無料架直接結束

- **條件 206**：房間2重複任務檢查（滿料架搬運到人工收料區流程最終確認）
  - 最終的重複任務檢查，end='True'
  - 防止重複的滿料架搬運任務產生

### 3. 準備區料架到入口傳送流程 (ready_rack_to_boxin.py)

**業務目標**：將系統準備區的料架搬運到入口傳送箱

**關鍵條件**：
- **條件 4**：準備區料架檢查
  - 檢查 location 表中 ID 為 11-18 且狀態為 3（有料架）的位置
  - 返回 next_id 指向入口傳送箱檢查列表 [110,210,310,410,510,610,710,810,910,1010]

- **條件 210**：房間2入口傳送箱空間檢查
  - 查詢 rack 表中 location_id = 20001 的記錄，確保位置空閒可接收料架

- **條件 211**：重複任務檢查
  - 防止相同的準備區料架到入口傳送執行中任務

### 4. 空料架搬運流程 (empty_rack_to_empty.py)

**業務目標**：將入口傳送箱的空料架搬運到出口傳送箱

**關鍵條件**：
- **條件 5**：入口傳送箱空料架檢查
  - 檢查入口傳送箱是否有空料架需要轉移

- **條件 215-216**：出口傳送箱空間檢查
  - 確認出口傳送箱有空間接收空料架

- 重複任務檢查：防止重複的空料架轉移任務

### 5. NG料架回收流程 (ng_rack_recycling.py)

**業務目標**：將入口傳送箱的NG料架搬運到NG回收區

**關鍵條件**：
- **條件 6**：NG回收區空位檢查
  - 檢查 location 表中 ID 為 71-72 且狀態為 2（空位）的位置
  - 返回 next_id 指向房間檢查列表 [120,220,320,420,520,620,720,820,920,1020]

- **條件 220**：房間2入口傳送箱NG料架檢查
  - 查詢 rack 表中 location_id = 20001 且狀態為 7（NG料架）的記錄

- **條件 221**：重複任務檢查
  - 防止相同的NG料架搬運執行中任務

### 6. AGV旋轉狀態檢查 (agv_rotation_check.py)

**業務目標**：監控 AGV 旋轉狀態並防止重複任務

**關鍵條件**：
- **條件 3**：檢查等待旋轉狀態的 AGV 是否可以發送任務
  - 查詢 agv_context 和 task 表，檢查是否已有子任務存在
  - 使用 UNION ALL 處理有/無等待旋轉 AGV 的情況

**SQL 查詢範例**：
```sql
SELECT * FROM (
    SELECT 
        ac.agv_id,
        t.id as task_id,
        CASE 
            WHEN NOT EXISTS (
                SELECT 1 
                FROM task t2 
                WHERE t2.parent_task_id = t.id
            ) THEN 'True'
            ELSE 'False'
        END as result,
        NULL as next_id,
        CASE 
            WHEN NOT EXISTS (
                SELECT 1 
                FROM task t2 
                WHERE t2.parent_task_id = t.id
            ) THEN 'True'
            ELSE 'False'
        END as end
    FROM agv_context ac
    INNER JOIN task t ON ac.agv_id = t.agv_id
    WHERE ac.current_state = 'wait_rotation_state'
    UNION ALL
    SELECT 
        -1 as agv_id,
        0 as task_id,
        'False' as result,
        NULL as next_id,
        'False' as end
    WHERE NOT EXISTS (
        SELECT 1 
        FROM agv_context ac2 
        INNER JOIN task t2 ON ac2.agv_id = t2.agv_id 
        WHERE ac2.current_state = 'wait_rotation_state'
    )
) subquery 
ORDER BY agv_id
```

### 7. 人工空料架回收 (manual_empty_rack_recycling.py)

**業務目標**：將人工回收空料架區的空料架搬運到系統空料架區

**關鍵條件**：
- **條件 7**：人工回收空料架區料架檢查
  - 檢查 location 表中 ID 為 91-92 且狀態為 3（有空料架）的位置

- **條件 8**：空料架回收區空位檢查
  - 檢查 location 表中 ID 為 51-54 且狀態為 2（空位）的位置

- **條件 9**：重複任務檢查
  - 防止相同的人工回收空料架搬運執行中任務

## 技術實作特點

### 條件鏈式結構
每個條件包含 `next_id` 欄位，形成決策樹流程：
- 支援複雜的業務邏輯分支
- 條件結果決定下一步執行的條件 ID
- 形成完整的業務流程鏈

### 防重複機制
透過檢查 `task` 表中的執行中任務：
- 檢查相同 work_id 和 node_id 的任務
- 防止重複任務產生
- 確保系統執行的安全性

### 多資料表聯查
整合多個核心資料表進行綜合判斷：
- **location**：位置狀態管理（1=未知狀態, 2=未佔用, 3=佔用, 4=任務佔用中）
- **rack**：料架狀態管理
- **task**：任務執行狀態
- **agv_context**：AGV 當前狀態

### 即時狀態監控
- 條件會持續執行查詢
- 即時反映倉儲系統的動態狀態變化
- 支援複雜的業務邏輯決策

### ROS 2 服務整合

#### SQL 查詢服務
```
# Request
string query_string
---
# Response
string json_result
bool success
string message
```

#### 服務實作
```python
def handle_sql_query(self, request, response):
    """處理 SQL 查詢請求的主要服務"""
    try:
        self.get_logger().info(f"---- {request.query_string}")
        
        with self.pool_agvc.get_session() as session:
            # 執行 SQL 查詢
            results = session.exec(text(request.query_string)).all()
            
            # 將結果轉換為 JSON 格式
            json_rows = [dict(r._mapping) for r in results]
            response.json_result = json.dumps(json_rows, default=str)
            
            response.success = True
            response.message = f"{len(results)} record(s) found"
    except Exception as e:
        response.success = False
        response.message = str(e)
        response.json_result = "[]"
    return response
```

## 關鍵檔案位置

- **模型定義**：`models/task_condition_history.py`
- **CRUD 操作**：`crud/task_condition_crud.py`
- **SQL 執行服務**：`agvc_database_node.py`
- **條件定義**：`sql/init_data/conditions/`
- **使用範例**：`examples/task_condition_example.py`

## 系統優勢

1. **高度彈性**：透過 SQL 條件定義，可以靈活調整業務邏輯
2. **模組化設計**：七大業務流程模組，清晰的職責分離
3. **即時響應**：持續查詢機制確保系統狀態的即時性
4. **擴展性強**：支援新增條件和業務流程
5. **整合性佳**：與 ROS 2 生態系統緊密結合

## 總結

這套 KUKA 任務判斷系統提供了高度彈性且強大的任務條件管理機制，能夠處理複雜的 AGV 倉儲自動化場景下的決策邏輯。透過 SQL 條件的動態執行和結果儲存，系統可以即時響應倉儲環境的變化，確保 AGV 任務的正確執行和系統的穩定運行。