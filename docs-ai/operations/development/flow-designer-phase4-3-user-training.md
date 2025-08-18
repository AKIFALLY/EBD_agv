# Flow Designer Phase 4.3 使用者培訓文檔

## 🎯 培訓目標
為 RosAGV YAML DSL Flow Designer 提供完整的使用者培訓體系，確保使用者能夠熟練使用視覺化流程設計器和 YAML DSL 語言進行 WCS 決策流程開發。

## 📚 培訓內容概覽

### 培訓模組結構
1. **基礎入門** - Flow Designer 介面和基本操作
2. **節點系統** - 四種節點類型和 38 個 WCS 函數
3. **視覺化設計** - 拖拽式流程圖設計
4. **YAML DSL** - 程式碼模式和雙向轉換
5. **高級功能** - 複雜流程設計和最佳實踐
6. **故障排除** - 常見問題和解決方案

## 📖 模組一：基礎入門指南

### 1.1 Flow Designer 簡介

#### 什麼是 Flow Designer？
Flow Designer 是 RosAGV 系統中的視覺化決策流程設計器，支援：
- **視覺化設計**: 拖拽式節點編輯器
- **YAML DSL**: 程式碼化流程定義語言
- **雙向轉換**: 視覺 ↔ 程式碼自由切換
- **WCS 整合**: 直接整合 38 個 WCS 決策函數

#### 介面佈局導覽
```
┌─────────────────────────────────────────────────────────┐
│ 頂部工具列: [新建] [儲存] [載入] [匯出YAML] [匯入YAML]    │
├─────────────────────────────────────────────────────────┤
│ 左側面板: 節點選板                    │ 主編輯區域:      │
│ ┌─ Condition Nodes (9個)           │ 流程圖畫布        │
│ ├─ Logic Nodes (5個)              │                  │
│ ├─ Action Nodes (4個)             │                  │
│ └─ Script Nodes (控制結構)          │                  │
├─────────────────────────────────────┤                  │
│ 屬性面板: 節點配置                    │                  │
│ - 節點名稱                          │                  │
│ - 參數設定                          │                  │
│ - 輸入輸出                          │                  │
└─────────────────────────────────────┴──────────────────┘
```

### 1.2 第一個流程設計

#### 實踐練習：建立簡單的 AGV 任務流程
```yaml
目標: 設計一個基本的 AGV 貨架搬運流程
步驟:
1. 檢查 AGV 是否在指定位置
2. 檢查貨架是否可用
3. 建立搬運任務
4. 更新任務狀態
```

#### 操作步驟詳解
1. **新增條件節點**
   - 從左側面板拖拽 "is_agv_at_location" 到畫布
   - 雙擊節點配置參數：agv_id = "agv01", location = "pickup_point"

2. **新增邏輯節點**
   - 拖拽 "check_rack_availability" 節點
   - 連接到第一個節點的輸出

3. **新增動作節點**
   - 拖拽 "create_task_from_decision" 節點
   - 配置任務類型和參數

4. **測試流程**
   - 點擊 "匯出 YAML" 查看產生的程式碼
   - 點擊 "匯入 YAML" 驗證雙向轉換

#### 預期結果
```yaml
# 產生的 YAML DSL 程式碼範例
flow_name: "basic_agv_transport"
description: "基本 AGV 貨架搬運流程"

variables:
  agv_id: "agv01"
  location: "pickup_point"
  rack_id: ""

steps:
  - name: "檢查AGV位置"
    function: "is_agv_at_location"
    type: "condition_nodes"
    parameters:
      agv_id: "{{ agv_id }}"
      location: "{{ location }}"
    outputs:
      result: "location_check_result"

  - name: "檢查貨架可用性"
    function: "check_rack_availability"
    type: "condition_nodes"
    parameters:
      location: "{{ location }}"
    outputs:
      available: "rack_available"

  - name: "建立搬運任務"
    function: "create_task_from_decision"
    type: "action_nodes"
    conditions:
      - "{{ location_check_result }} == true"
      - "{{ rack_available }} == true"
    parameters:
      agv_id: "{{ agv_id }}"
      task_type: "transport"
      source_location: "{{ location }}"
```

## 📖 模組二：節點系統詳解

### 2.1 四種節點類型概覽

#### Condition Nodes (條件節點) - 藍色
**用途**: 條件判斷和狀態檢查
**數量**: 9個函數
**顏色**: #dbeafe (淺藍色)

| 函數名 | 功能描述 | 主要參數 | 回傳值 |
|--------|----------|----------|--------|
| `is_agv_at_location` | 檢查 AGV 是否在指定位置 | agv_id, location | boolean |
| `check_agv_rotation_flow` | 檢查 AGV 旋轉流向 | agv_id, direction | boolean |
| `check_rack_availability` | 檢查貨架可用性 | location, rack_type | boolean |
| `is_task_completed` | 檢查任務是否完成 | task_id | boolean |
| `check_equipment_status` | 檢查設備狀態 | equipment_id | boolean |

#### Logic Nodes (邏輯節點) - 黃色
**用途**: 資料處理和邏輯運算
**數量**: 5個函數
**顏色**: #fef3c7 (淺黃色)

| 函數名 | 功能描述 | 主要參數 | 回傳值 |
|--------|----------|----------|--------|
| `get_agv_current_location` | 獲取 AGV 目前位置 | agv_id | location |
| `get_room_inlet_point` | 獲取房間入口點 | room_id | point |
| `calculate_optimal_path` | 計算最佳路徑 | start, end | path |
| `get_available_agv` | 獲取可用 AGV | criteria | agv_id |
| `determine_task_priority` | 確定任務優先順序 | task_list | priority |

#### Action Nodes (動作節點) - 綠色
**用途**: 執行操作和狀態變更
**數量**: 4個函數
**顏色**: #d1fae5 (淺綠色)

| 函數名 | 功能描述 | 主要參數 | 回傳值 |
|--------|----------|----------|--------|
| `create_task_from_decision` | 從決策建立任務 | decision_data | task_id |
| `update_task_status` | 更新任務狀態 | task_id, status | success |
| `assign_agv_to_task` | 分配 AGV 到任務 | agv_id, task_id | success |
| `send_agv_command` | 發送 AGV 指令 | agv_id, command | success |

#### Script Nodes (腳本節點) - 紫色
**用途**: 控制流程和複雜邏輯
**數量**: 控制結構
**顏色**: #ede9fe (淺紫色)

| 結構類型 | 功能描述 | 語法 | 用途 |
|----------|----------|------|------|
| `if_else` | 條件分支 | if-then-else | 條件執行 |
| `for_loop` | 迴圈結構 | for item in list | 批次處理 |
| `while_loop` | 條件迴圈 | while condition | 持續執行 |
| `try_catch` | 例外處理 | try-catch-finally | 錯誤處理 |

### 2.2 節點配置詳解

#### 通用節點屬性
```javascript
// 每個節點都包含以下基本屬性
{
  id: "node_001",           // 唯一識別
  name: "檢查AGV位置",       // 顯示名稱
  function: "is_agv_at_location", // WCS函數名
  type: "condition_nodes",  // 節點類型
  position: { x: 100, y: 200 }, // 畫布位置
  parameters: {             // 函數參數
    agv_id: "agv01",
    location: "pickup_point"
  },
  inputs: ["agv_id", "location"],  // 輸入介面
  outputs: ["result"],      // 輸出介面
  conditions: [],           // 執行條件
  description: "檢查指定AGV是否在目標位置"
}
```

#### 參數類型和驗證
```yaml
參數類型系統:
  string:     # 字串類型
    - 支援變數替換: "{{ variable_name }}"
    - 支援字串拼接: "prefix_{{ var }}_suffix"
    - 驗證: 非空字串
    
  number:     # 數字類型
    - 整數: 1, 2, 3, -1
    - 浮點數: 1.5, 3.14, -2.7
    - 驗證: 數值範圍檢查
    
  boolean:    # 布林類型
    - true, false
    - 運算式: "{{ result }} == true"
    - 驗證: 布林值或運算式
    
  array:      # 陣列類型
    - ["item1", "item2", "item3"]
    - 支援動態陣列: "{{ dynamic_list }}"
    - 驗證: 陣列格式和元素類型
    
  object:     # 物件類型
    - { key: "value", nested: { sub: "data" } }
    - 支援巢狀結構
    - 驗證: JSON 格式驗證
```

## 📖 模組三：視覺化設計進階

### 3.1 複雜流程設計模式

#### 分支流程設計
```yaml
場景: 多條件決策流程
設計模式:
1. 條件檢查節點 (Condition)
2. 分支邏輯節點 (Logic)
3. 多個動作節點 (Action)
4. 合併點 (Script)

實現方法:
- 使用 if_else 腳本節點建立分支
- 配置多個輸出連接
- 每個分支獨立配置動作
- 最終合併到統一的結束節點
```

#### 迴圈流程設計
```yaml
場景: 批次處理任務
設計模式:
1. 初始化節點 (Logic)
2. 迴圈控制節點 (Script: for_loop)
3. 處理邏輯節點 (Action)
4. 條件判斷節點 (Condition)
5. 迴圈結束節點 (Logic)

實現方法:
- for_loop 節點配置迴圈清單
- 迴圈體內包含處理邏輯
- 使用變數傳遞迴圈狀態
- 支援迴圈中斷和繼續
```

#### 例外處理設計
```yaml
場景: 容錯流程設計
設計模式:
1. 主要處理節點 (Action)
2. 例外捕獲節點 (Script: try_catch)
3. 錯誤處理節點 (Action)
4. 恢復邏輯節點 (Logic)
5. 最終清理節點 (Action)

實現方法:
- try_catch 包裝關鍵操作
- catch 分支處理不同錯誤類型
- finally 分支執行清理操作
- 記錄錯誤日誌和恢復狀態
```

### 3.2 最佳實踐和設計原則

#### 命名規範
```yaml
節點命名:
  - 使用描述性名稱: "檢查AGV位置" 而不是 "檢查1"
  - 動詞開頭: "獲取可用貨架", "更新任務狀態"
  - 一致性: 同類型節點使用相似命名模式
  
變數命名:
  - 駱駝命名: agvId, taskStatus, currentLocation
  - 或底線: agv_id, task_status, current_location
  - 避免縮寫: 使用 location 而不是 loc
  
流程命名:
  - 業務描述: "貨架搬運流程", "設備維護檢查"
  - 版本控制: "cargo_transport_v1.2"
  - 環境識別: "production_flow", "test_flow"
```

#### 設計原則
```yaml
單一職責原則:
  - 每個節點專注一個功能
  - 避免過度複雜的參數配置
  - 優先組合簡單節點而非複雜節點

可讀性原則:
  - 邏輯流向清晰 (從左到右, 從上到下)
  - 適當的節點間距和對齊
  - 使用註解節點說明複雜邏輯

可維護性原則:
  - 模組化設計, 可重複使用的子流程
  - 參數化配置, 避免硬編碼
  - 版本控制和變更記錄

效能原則:
  - 避免不必要的條件檢查
  - 合理使用快取和狀態
  - 最佳化迴圈和遞迴邏輯
```

#### 佈局和視覺最佳化
```yaml
節點佈局:
  - 網格對齊: 使用 20px 網格對齊節點
  - 層次分明: 主流程居中, 分支流程分佈兩側
  - 空間利用: 合理利用畫布空間, 避免重疊

連接線最佳化:
  - 減少交叉: 調整節點位置減少連接線交叉
  - 路徑清晰: 使用合適的連接點和彎曲度
  - 方向一致: 保持主要流程方向一致

顏色系統:
  - 功能區分: 不同類型節點使用不同顏色
  - 狀態識別: 錯誤節點紅色邊框, 完成節點綠色邊框
  - 優先序: 關鍵路徑使用醒目顏色
```

## 📖 模組四：YAML DSL 程式碼模式

### 4.1 YAML DSL 語法詳解

#### 基本文檔結構
```yaml
# 完整的 YAML DSL 文檔結構
flow_name: "流程名稱"           # 必需: 流程識別名稱
description: "流程描述"         # 可選: 流程功能描述  
version: "1.0"                 # 可選: 版本號
created_by: "user_name"        # 可選: 建立者
created_at: "2025-08-15"       # 可選: 建立時間
tags: ["production", "agv"]    # 可選: 標籤分類

# 變數定義部分 (可選)
variables:
  agv_id: "agv01"              # 字串變數
  max_retry: 3                 # 數字變數
  enable_debug: true           # 布林變數
  locations: ["A1", "B2", "C3"] # 陣列變數
  config:                      # 物件變數
    timeout: 30
    retry_interval: 5

# 流程步驟部分 (必需)
steps:
  - name: "步驟名稱"             # 必需: 步驟顯示名稱
    function: "函數名"           # 必需: WCS 函數名
    type: "節點類型"             # 必需: condition/logic/action/script
    description: "步驟描述"      # 可選: 步驟功能說明
    
    # 輸入參數 (可選)
    parameters:
      param1: "value1"
      param2: "{{ variable_name }}"  # 變數參考
      
    # 執行條件 (可選)
    conditions:
      - "{{ some_var }} == true"     # 條件運算式
      - "{{ retry_count }} < {{ max_retry }}"
      
    # 輸出定義 (可選)
    outputs:
      result: "output_variable_name"
      
    # 錯誤處理 (可選)
    on_error:
      action: "retry"           # retry/skip/abort
      max_retries: 3
      retry_delay: 5
```

#### 變數系統詳解
```yaml
# 變數類型和用法
variables:
  # 1. 基本類型
  string_var: "hello world"
  number_var: 42
  float_var: 3.14
  boolean_var: true
  null_var: null
  
  # 2. 複合類型
  array_var: [1, 2, 3, "four", true]
  object_var:
    nested_string: "value"
    nested_number: 100
    nested_array: ["a", "b", "c"]
    
  # 3. 運算式變數
  computed_var: "{{ string_var }}_suffix"
  math_var: "{{ number_var + 10 }}"
  condition_var: "{{ boolean_var and true }}"

# 變數參考語法
steps:
  - name: "使用變數的步驟"
    function: "example_function"
    type: "logic_nodes"
    parameters:
      # 直接參考
      simple_ref: "{{ string_var }}"
      
      # 屬性參考
      nested_ref: "{{ object_var.nested_string }}"
      
      # 陣列參考
      array_item: "{{ array_var[0] }}"
      
      # 運算式參考
      expression: "{{ number_var > 10 ? 'large' : 'small' }}"
      
      # 字串插值
      template: "AGV {{ agv_id }} is at {{ current_location }}"
      
      # 函數呼叫
      uppercase: "{{ string_var.upper() }}"
      length: "{{ array_var.length }}"
```

#### 控制結構語法
```yaml
# 1. 條件分支 (if_else)
steps:
  - name: "條件判斷"
    function: "if_else"
    type: "script_nodes"
    parameters:
      condition: "{{ agv_status }} == 'ready'"
      then_steps:
        - name: "執行任務"
          function: "create_task_from_decision"
          type: "action_nodes"
          parameters:
            agv_id: "{{ agv_id }}"
      else_steps:
        - name: "等待就緒"
          function: "wait_for_agv_ready"
          type: "action_nodes"
          parameters:
            agv_id: "{{ agv_id }}"
            timeout: 30

# 2. 迴圈結構 (for_loop)
steps:
  - name: "批次處理"
    function: "for_loop"
    type: "script_nodes"
    parameters:
      items: "{{ rack_list }}"
      item_var: "current_rack"
      loop_steps:
        - name: "檢查貨架"
          function: "check_rack_availability"
          type: "condition_nodes"
          parameters:
            rack_id: "{{ current_rack }}"
        - name: "處理貨架"
          function: "process_rack"
          type: "action_nodes"
          parameters:
            rack_id: "{{ current_rack }}"

# 3. 條件迴圈 (while_loop)
steps:
  - name: "等待完成"
    function: "while_loop"
    type: "script_nodes"
    parameters:
      condition: "{{ task_status }} != 'completed'"
      max_iterations: 100
      loop_steps:
        - name: "檢查狀態"
          function: "get_task_status"
          type: "logic_nodes"
          parameters:
            task_id: "{{ current_task_id }}"
          outputs:
            status: "task_status"
        - name: "等待"
          function: "sleep"
          type: "action_nodes"
          parameters:
            duration: 5

# 4. 例外處理 (try_catch)
steps:
  - name: "安全執行"
    function: "try_catch"
    type: "script_nodes"
    parameters:
      try_steps:
        - name: "執行操作"
          function: "risky_operation"
          type: "action_nodes"
          parameters:
            param: "value"
      catch_steps:
        - name: "錯誤處理"
          function: "handle_error"
          type: "action_nodes"
          parameters:
            error_message: "{{ error.message }}"
      finally_steps:
        - name: "清理資源"
          function: "cleanup_resources"
          type: "action_nodes"
```

### 4.2 雙向轉換機制

#### 視覺 → YAML 轉換
```javascript
// 轉換過程說明
1. 遍歷畫布上的所有節點
2. 提取節點屬性和參數配置
3. 分析節點間的連接關係
4. 生成變數定義部分
5. 按流程順序生成步驟定義
6. 格式化輸出標準 YAML

// 轉換範例
視覺節點:
  [檢查AGV位置] → [獲取貨架資訊] → [建立搬運任務]
           ↓
YAML程式碼:
  steps:
    - name: "檢查AGV位置"
      function: "is_agv_at_location"
      type: "condition_nodes"
    - name: "獲取貨架資訊"  
      function: "get_rack_info"
      type: "logic_nodes"
    - name: "建立搬運任務"
      function: "create_task_from_decision"
      type: "action_nodes"
```

#### YAML → 視覺轉換
```javascript
// 轉換過程說明
1. 解析 YAML 文檔結構
2. 驗證語法和語義正確性
3. 建立對應的視覺節點
4. 建立節點間的連接關係
5. 套用佈局演算法排列節點
6. 渲染到畫布並支援互動

// 轉換範例
YAML程式碼:
  variables:
    agv_id: "agv01"
  steps:
    - name: "步驟1"
      function: "check_status"
      parameters:
        id: "{{ agv_id }}"
           ↓
視覺節點:
  [變數面板顯示: agv_id = "agv01"]
  [畫布節點: "步驟1" - check_status函數]
  [參數配置: id = "{{ agv_id }}"]
```

#### 轉換一致性保證
```yaml
一致性驗證機制:
1. 資料完整性檢查
   - 節點屬性完整性
   - 參數類型一致性  
   - 連接關係正確性

2. 語義等價性驗證
   - 執行邏輯等價
   - 變數參考一致
   - 控制流程匹配

3. 自動化測試
   - 往返轉換測試 (Visual → YAML → Visual)
   - 邊界條件測試
   - 複雜流程驗證

4. 使用者反饋機制
   - 轉換成功/失敗提示
   - 差異對比顯示
   - 修復建議提供
```

## 📖 模組五：高級功能和實際應用

### 5.1 實際業務場景應用

#### 場景一：眼鏡生產線 AGV 調度
```yaml
# 完整的眼鏡生產線流程範例
flow_name: "eyewear_production_agv_dispatch"
description: "眼鏡生產線 AGV 自動調度流程"
version: "2.1"

variables:
  production_line: "eyewear_line_1"
  agv_pool: ["agv01", "agv02", "agv03"]
  injection_stations: ["IM001", "IM002", "IM003"]
  kuka_robot_station: "KUKA_ROBOT_001"
  quality_check_point: "QC_STATION_001"

steps:
  # 1. 檢查射出機完成狀態
  - name: "檢查射出機狀態"
    function: "check_equipment_status"
    type: "condition_nodes"
    parameters:
      equipment_type: "injection_machine"
      station_list: "{{ injection_stations }}"
    outputs:
      completed_stations: "ready_stations"

  # 2. 分配可用 AGV
  - name: "獲取可用AGV"
    function: "get_available_agv"
    type: "logic_nodes"
    conditions:
      - "{{ ready_stations.length }} > 0"
    parameters:
      agv_pool: "{{ agv_pool }}"
      criteria: "nearest_to_station"
      target_stations: "{{ ready_stations }}"
    outputs:
      assigned_agv: "selected_agv"

  # 3. 建立取料任務
  - name: "建立取料任務"
    function: "create_task_from_decision"
    type: "action_nodes"
    conditions:
      - "{{ selected_agv }} != null"
    parameters:
      agv_id: "{{ selected_agv }}"
      task_type: "pickup"
      source_location: "{{ ready_stations[0] }}"
      target_location: "{{ kuka_robot_station }}"
      priority: "high"
    outputs:
      task_id: "pickup_task_id"

  # 4. 監控任務執行
  - name: "監控任務進度"
    function: "while_loop"
    type: "script_nodes"
    parameters:
      condition: "{{ task_status }} != 'completed'"
      max_iterations: 120  # 最多等待10分鐘
      loop_steps:
        - name: "檢查任務狀態"
          function: "get_task_status"
          type: "logic_nodes"
          parameters:
            task_id: "{{ pickup_task_id }}"
          outputs:
            status: "task_status"
            current_location: "agv_location"
        - name: "等待更新"
          function: "sleep"
          type: "action_nodes"
          parameters:
            duration: 5

  # 5. KUKA 機器人互動
  - name: "通知KUKA機器人"
    function: "send_kuka_notification"
    type: "action_nodes"
    conditions:
      - "{{ task_status }} == 'completed'"
      - "{{ agv_location }} == '{{ kuka_robot_station }}'"
    parameters:
      robot_id: "{{ kuka_robot_station }}"
      notification_type: "material_ready"
      agv_id: "{{ selected_agv }}"
      material_info:
        batch_id: "{{ current_batch }}"
        part_type: "eyeglass_frame"

  # 6. 品質檢查流程
  - name: "安排品檢"
    function: "if_else"
    type: "script_nodes"
    parameters:
      condition: "{{ enable_quality_check }} == true"
      then_steps:
        - name: "建立品檢任務"
          function: "create_task_from_decision"
          type: "action_nodes"
          parameters:
            agv_id: "{{ selected_agv }}"
            task_type: "transport"
            source_location: "{{ kuka_robot_station }}"
            target_location: "{{ quality_check_point }}"
            priority: "medium"
      else_steps:
        - name: "直接入庫"
          function: "create_storage_task"
          type: "action_nodes"
          parameters:
            agv_id: "{{ selected_agv }}"
            storage_location: "finished_goods_area"
```

#### 場景二：設備維護檢查流程
```yaml
flow_name: "equipment_maintenance_check"
description: "設備預防性維護檢查流程"

variables:
  maintenance_schedule: "weekly"
  equipment_list: []
  maintenance_team: "team_alpha"
  
steps:
  # 1. 獲取維護設備清單
  - name: "獲取待維護設備"
    function: "get_maintenance_equipment_list"
    type: "logic_nodes"
    parameters:
      schedule_type: "{{ maintenance_schedule }}"
      current_date: "{{ today }}"
    outputs:
      equipment_list: "maintenance_equipment"

  # 2. 批次檢查設備狀態
  - name: "批次設備檢查"
    function: "for_loop"
    type: "script_nodes"
    parameters:
      items: "{{ maintenance_equipment }}"
      item_var: "current_equipment"
      loop_steps:
        - name: "設備狀態檢查"
          function: "check_equipment_status"
          type: "condition_nodes"
          parameters:
            equipment_id: "{{ current_equipment.id }}"
            check_type: "comprehensive"
          outputs:
            status: "equipment_status"
            issues: "found_issues"

        - name: "記錄檢查結果"
          function: "log_maintenance_result"
          type: "action_nodes"
          parameters:
            equipment_id: "{{ current_equipment.id }}"
            status: "{{ equipment_status }}"
            issues: "{{ found_issues }}"
            inspector: "{{ maintenance_team }}"

        - name: "處理問題"
          function: "if_else"
          type: "script_nodes"
          parameters:
            condition: "{{ found_issues.length }} > 0"
            then_steps:
              - name: "建立維修工單"
                function: "create_maintenance_ticket"
                type: "action_nodes"
                parameters:
                  equipment_id: "{{ current_equipment.id }}"
                  issues: "{{ found_issues }}"
                  priority: "{{ issue_priority }}"
                  assigned_team: "{{ maintenance_team }}"
```

### 5.2 效能最佳化和監控

#### 流程執行效能監控
```yaml
# 效能監控配置範例
flow_name: "performance_monitored_flow"
description: "帶效能監控的流程範例"

# 效能配置
performance_config:
  enable_monitoring: true
  log_execution_time: true
  alert_threshold: 30  # 秒
  retry_policy:
    max_retries: 3
    retry_delay: 5
    exponential_backoff: true

variables:
  start_time: null
  step_times: {}

steps:
  # 效能監控開始
  - name: "開始效能監控"
    function: "start_performance_monitoring"
    type: "action_nodes"
    parameters:
      flow_id: "{{ flow_name }}"
      monitoring_level: "detailed"
    outputs:
      start_time: "flow_start_time"

  # 業務邏輯步驟
  - name: "執行業務邏輯"
    function: "business_logic_step"
    type: "logic_nodes"
    parameters:
      input_data: "{{ business_data }}"
    performance_tracking:
      step_timeout: 15
      alert_on_slow: true
    outputs:
      result: "business_result"

  # 效能監控結束
  - name: "結束效能監控"
    function: "end_performance_monitoring"
    type: "action_nodes"
    parameters:
      flow_id: "{{ flow_name }}"
      start_time: "{{ flow_start_time }}"
      generate_report: true
```

#### 大規模流程最佳化技巧
```yaml
大規模流程設計技巧:

1. 模組化設計:
   - 將複雜流程分解為可重複使用的子流程
   - 使用子流程呼叫減少重複程式碼
   - 獨立的錯誤處理和恢復機制

2. 並行執行:
   - 識別可以並行執行的步驟
   - 使用並行分支提高執行效率
   - 合理設定並發控制和資源限制

3. 快取策略:
   - 快取頻繁查詢的資料
   - 使用會話級和流程級快取
   - 設定合理的快取過期時間

4. 資源管理:
   - 合理分配系統資源
   - 設定執行優先序和佇列管理
   - 監控資源使用情況和瓶頸

5. 錯誤恢復:
   - 設計完善的錯誤處理機制
   - 實現自動重試和降級策略
   - 保證資料一致性和流程完整性
```

## 📖 模組六：故障排除和常見問題

### 6.1 常見問題診斷

#### 問題分類和診斷流程
```yaml
問題分類體系:

1. 語法錯誤 (Syntax Errors):
   - YAML 格式錯誤
   - 變數參考錯誤
   - 函數名拼寫錯誤
   - 參數類型不匹配

2. 邏輯錯誤 (Logic Errors):
   - 條件判斷錯誤
   - 流程控制錯誤
   - 變數作用域問題
   - 死迴圈或無限遞迴

3. 執行階段錯誤 (Runtime Errors):
   - 外部系統連接失敗
   - 資料存取例外
   - 逾時錯誤
   - 資源不足

4. 效能問題 (Performance Issues):
   - 執行速度慢
   - 記憶體佔用過高
   - 回應時間長
   - 並發處理問題
```

#### 問題診斷工具
```javascript
// Flow Designer 內建診斷工具
1. 語法檢查器:
   - 即時 YAML 語法驗證
   - 變數參考檢查
   - 函數簽名驗證
   - 參數類型檢查

2. 流程模擬器:
   - 步進式執行模擬
   - 變數狀態追蹤
   - 條件評估預覽
   - 輸出結果預測

3. 效能分析器:
   - 執行時間統計
   - 記憶體使用監控
   - 瓶頸識別
   - 最佳化建議

4. 除錯主控台:
   - 中斷點設定
   - 變數檢視
   - 執行日誌
   - 錯誤堆疊
```

### 6.2 具體問題解決方案

#### 語法錯誤解決
```yaml
# 問題1: YAML 縮排錯誤
❌ 錯誤範例:
variables:
agv_id: "agv01"  # 缺少縮排
  location: "A1"

✅ 正確範例:
variables:
  agv_id: "agv01"
  location: "A1"

# 問題2: 變數參考錯誤  
❌ 錯誤範例:
parameters:
  agv_id: "{ agv_id }"      # 缺少雙大括號
  location: "{{location}"   # 缺少閉合大括號

✅ 正確範例:
parameters:
  agv_id: "{{ agv_id }}"
  location: "{{ location }}"

# 問題3: 函數名錯誤
❌ 錯誤範例:
function: "is_agv_at_position"  # 應該是 is_agv_at_location

✅ 正確範例:
function: "is_agv_at_location"

# 問題4: 參數類型錯誤
❌ 錯誤範例:
parameters:
  retry_count: "three"  # 應該是數字

✅ 正確範例:
parameters:
  retry_count: 3
```

#### 邏輯錯誤解決
```yaml
# 問題1: 條件判斷錯誤
❌ 錯誤範例:
conditions:
  - "{{ agv_status = 'ready' }}"  # 應該使用 == 比較

✅ 正確範例:
conditions:
  - "{{ agv_status == 'ready' }}"

# 問題2: 死迴圈問題
❌ 錯誤範例:
- name: "等待AGV"
  function: "while_loop"
  type: "script_nodes"
  parameters:
    condition: "{{ agv_busy == true }}"  # 條件永遠不變
    loop_steps:
      - name: "等待"
        function: "sleep"
        parameters:
          duration: 1

✅ 正確範例:
- name: "等待AGV"
  function: "while_loop"
  type: "script_nodes"
  parameters:
    condition: "{{ agv_busy == true }}"
    max_iterations: 60  # 新增最大迭代次數
    loop_steps:
      - name: "檢查AGV狀態"
        function: "get_agv_status"
        type: "logic_nodes"
        parameters:
          agv_id: "{{ agv_id }}"
        outputs:
          busy: "agv_busy"  # 更新條件變數
      - name: "等待"
        function: "sleep"
        parameters:
          duration: 5
```

#### 執行階段錯誤解決
```yaml
# 問題1: 外部系統連接失敗
解決方案:
1. 新增連接重試機制
2. 實現降級處理邏輯
3. 增加逾時控制
4. 記錄詳細錯誤日誌

範例:
- name: "安全的外部呼叫"
  function: "try_catch"
  type: "script_nodes"
  parameters:
    try_steps:
      - name: "呼叫外部系統"
        function: "call_external_api"
        type: "action_nodes"
        parameters:
          url: "{{ external_api_url }}"
          timeout: 30
          retry_count: 3
    catch_steps:
      - name: "降級處理"
        function: "fallback_logic"
        type: "action_nodes"
        parameters:
          fallback_data: "{{ default_response }}"

# 問題2: 資料存取例外
解決方案:
1. 驗證資料存在性
2. 新增資料格式檢查
3. 實現預設值機制
4. 提供資料修復邏輯

範例:
- name: "安全的資料存取"
  function: "if_else"
  type: "script_nodes"
  parameters:
    condition: "{{ data != null and data.length > 0 }}"
    then_steps:
      - name: "處理資料"
        function: "process_data"
        type: "logic_nodes"
        parameters:
          input_data: "{{ data }}"
    else_steps:
      - name: "使用預設資料"
        function: "use_default_data"
        type: "logic_nodes"
        parameters:
          default_value: "{{ fallback_data }}"
```

### 6.3 除錯技巧和工具

#### 除錯模式使用
```yaml
# 啟用除錯模式
flow_name: "debug_enabled_flow"
debug_config:
  enable_debug: true
  debug_level: "detailed"  # basic/detailed/verbose
  log_variables: true
  log_execution_time: true
  breakpoint_enabled: true

steps:
  - name: "除錯步驟"
    function: "example_function"
    type: "logic_nodes"
    debug_settings:
      breakpoint: true       # 在此步驟設定中斷點
      log_inputs: true       # 記錄輸入參數
      log_outputs: true      # 記錄輸出結果
      performance_tracking: true
    parameters:
      input_param: "{{ debug_data }}"
```

#### 日誌分析技巧
```yaml
日誌分析方法:

1. 執行軌跡分析:
   - 檢視步驟執行順序
   - 識別跳過的步驟
   - 分析條件判斷結果
   - 追蹤變數值變化

2. 效能分析:
   - 統計每步執行時間
   - 識別效能瓶頸
   - 分析資源使用情況
   - 最佳化建議生成

3. 錯誤模式識別:
   - 統計錯誤類型和頻率
   - 分析錯誤發生位置
   - 識別錯誤傳播路徑
   - 提供修復建議

4. 資料流分析:
   - 追蹤變數傳遞過程
   - 驗證資料轉換正確性
   - 識別資料遺失問題
   - 檢查資料類型匹配
```

## 📖 模組七：培訓測驗和認證

### 7.1 理論測驗題目

#### 基礎知識測驗 (選擇題)
```yaml
題目1: Flow Designer 支援哪些節點類型？
A. Condition, Logic, Action
B. Condition, Logic, Action, Script  ✓
C. Input, Process, Output
D. Start, Middle, End

題目2: YAML DSL 中變數參考的正確語法是？
A. {variable_name}
B. {{variable_name}}  ✓ 
C. $(variable_name)
D. %{variable_name}

題目3: 以下哪個函數屬於 Logic Nodes？
A. is_agv_at_location
B. create_task_from_decision
C. get_agv_current_location  ✓
D. if_else

題目4: 批次處理應該使用哪種控制結構？
A. if_else
B. for_loop  ✓
C. while_loop
D. try_catch

題目5: Flow Designer 的雙向轉換是指？
A. YAML 轉 JSON
B. 視覺化介面和 YAML DSL 程式碼互相轉換  ✓
C. 輸入輸出轉換
D. 中英文轉換
```

#### 進階應用測驗 (填空題)
```yaml
題目1: 完成以下 YAML DSL 變數定義
variables:
  agv_id: "agv01"
  max_retry: _____ (填入數字類型)
  enable_debug: _____ (填入布林類型)
  
答案: 3, true

題目2: 完成條件判斷語法
conditions:
  - "{{ agv_status _____ 'ready' }}" (填入比較運算子)
  - "{{ retry_count _____ max_retry }}" (填入比較運算子)
  
答案: ==, <

題目3: 完成迴圈結構定義
- name: "批次處理貨架"
  function: "_____" (填入函數名)
  type: "script_nodes"
  parameters:
    items: "{{ rack_list }}"
    item_var: "_____" (填入變數名)
    
答案: for_loop, current_rack
```

### 7.2 實踐操作測驗

#### 操作測驗一：基礎流程設計
```yaml
任務描述:
使用 Flow Designer 設計一個 AGV 狀態檢查流程，要求：

1. 檢查 AGV 是否在指定位置
2. 如果在位置，獲取 AGV 目前任務狀態
3. 如果有任務在執行，等待任務完成
4. 如果沒有任務，分配新的搬運任務

評分標準:
- 節點類型選擇正確 (25分)
- 節點連接邏輯正確 (25分)
- 參數配置完整 (25分)
- YAML 程式碼生成正確 (25分)

預期完成時間: 15分鐘
```

#### 操作測驗二：複雜流程設計
```yaml
任務描述:
設計一個眼鏡生產線 AGV 調度流程，要求：

1. 監控多個射出機狀態
2. 當射出機完成時，分配最近的可用 AGV
3. AGV 取料後運輸到 KUKA 機器人工作站
4. 通知 KUKA 機器人處理
5. 根據配置決定是否進行品質檢查
6. 實現例外處理和重試機制

評分標準:
- 業務邏輯理解正確 (30分)
- 流程設計合理 (25分)
- 例外處理完善 (25分)
- 程式碼品質良好 (20分)

預期完成時間: 45分鐘
```

### 7.3 認證體系

#### 認證級別
```yaml
Flow Designer 使用者認證體系:

1. 初級使用者認證:
   - 理論測驗: 80分以上
   - 實踐操作: 基礎流程設計完成
   - 有效期: 1年
   - 權限: 可設計簡單流程，需要審核後部署

2. 中級使用者認證:
   - 理論測驗: 85分以上
   - 實踐操作: 複雜流程設計完成
   - 專案案例: 至少1個生產環境流程
   - 有效期: 2年
   - 權限: 可獨立設計和部署中等複雜度流程

3. 高級使用者認證:
   - 理論測驗: 90分以上
   - 實踐操作: 全部測驗完成
   - 專案案例: 至少3個生產環境流程
   - 教學能力: 能夠培訓其他使用者
   - 有效期: 3年
   - 權限: 可設計任何複雜度流程，可審核他人流程

4. 專家級認證:
   - 系統架構理解: 深入了解 Simple WCS 整合
   - 效能最佳化能力: 能夠最佳化複雜流程效能
   - 故障排除專家: 能夠解決各種技術問題
   - 培訓認證講師: 具有正式培訓資質
   - 有效期: 永久
   - 權限: 系統管理，架構設計，高級故障排除
```

#### 持續教育要求
```yaml
認證維護要求:

1. 年度培訓: 每年至少參加8小時相關培訓
2. 實踐專案: 每年至少完成2個實際專案
3. 知識更新: 及時學習系統更新和新功能
4. 經驗分享: 參與技術分享和最佳實踐討論

重新認證條件:
- 認證過期後需要重新參加測驗
- 長期未使用系統(超過6個月)需要重新培訓
- 系統重大更新後需要補充培訓
- 出現重大操作失誤需要重新認證
```

## 📚 附錄：參考資料

### A.1 WCS 函數完整清單
```yaml
# 38個 WCS 函數詳細說明參考
詳細文檔位置:
- Simple WCS 系統文檔: @docs-ai/knowledge/system/flow-wcs-system.md
- WCS 函數註冊器: app/simple_wcs_ws/src/simple_wcs/dsl_function_registry.py
- 函數測試用例: app/simple_wcs_ws/src/simple_wcs/test/

函數分類索引:
1. Condition Nodes (9個): 條件判斷和狀態檢查
2. Logic Nodes (5個): 資料處理和邏輯運算  
3. Action Nodes (4個): 執行操作和狀態變更
4. Script Nodes: 控制結構 (if_else, for_loop, while_loop, try_catch)
```

### A.2 系統整合文檔
```yaml
# 相關系統文檔參考
1. RosAGV 系統概覽: @docs-ai/context/system/rosagv-overview.md
2. Web API 工作空間: app/web_api_ws/CLAUDE.md
3. Simple WCS 開發: @docs-ai/operations/development/flow-wcs-development.md
4. 容器開發環境: @docs-ai/operations/development/docker-development.md
```

### A.3 技術支援聯絡方式
```yaml
# 獲取技術支援的方式
1. 線上文檔: RosAGV 專案 docs-ai/ 目錄
2. 系統診斷: 使用 r 命令工具進行系統檢查
3. 日誌分析: 查看 AGVC 容器內的執行日誌
4. 問題回報: 透過系統管理員回報技術問題
```

---

## 🎓 培訓總結

Flow Designer YAML DSL 系統是一個功能強大的視覺化決策流程設計工具，透過本培訓文檔，使用者可以：

1. **掌握基礎操作**: 熟練使用視覺化介面設計流程
2. **理解節點系統**: 正確使用 38 個 WCS 函數和四種節點類型
3. **學會 YAML DSL**: 編寫和維護程式碼化的流程定義
4. **應用實際場景**: 解決真實的業務流程設計需求
5. **排除常見故障**: 獨立診斷和解決技術問題

透過系統性的學習和實踐，使用者將能夠充分發揮 Flow Designer 的強大功能，為 RosAGV 系統建立高效、可靠的決策流程。