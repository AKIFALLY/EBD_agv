# WCS Flow Designer YAML DSL 設計計劃

## 🎯 專案概述

### 目標
設計一種基於 YAML 的 DSL (Domain Specific Language) 程式語言，讓：
- **Simple WCS** 作為執行引擎，像解釋器一樣執行 YAML 腳本
- **Flow Designer** 作為視覺化 IDE，提供拖拽式編程界面
- **業務人員** 可以通過簡單的 YAML 語法定義複雜的 WCS 業務邏輯
- **三種節點類型** 完美整合：condition_nodes（輸入檢查）、logic_nodes（邏輯控制）、action_nodes（任務執行）

### 背景
目前 RosAGV 系統中：
- Simple WCS 使用靜態的 YAML 配置檔案 (如 `rack_rotation_inlet.yaml`)
- Flow Designer 提供視覺化界面但格式不統一
- 業務邏輯變更需要修改程式碼或複雜的配置
- 存在三種節點定義：condition_nodes.yaml、logic_nodes.yaml、action_nodes.yaml
- 需要流程邏輯轉換，而不只是單一節點轉換

### 創新點
1. **程式語言化配置** - 將靜態配置升級為可執行腳本
2. **視覺化程式設計** - 拖拽節點生成程式碼
3. **雙向編輯** - 視覺化設計 ↔ 程式碼編輯無縫切換
4. **業務導向** - 專門針對 WCS 任務生成的領域語言
5. **三層節點架構** - 整合 condition、logic、action 三種節點類型
6. **流程邏輯轉換** - 基於節點組合的完整流程邏輯生成

## 🏗️ 技術架構設計

### 系統架構
```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│  Flow Designer  │    │    YAML DSL     │    │  Simple WCS    │
│   (Visual IDE)  │◄──►│   Script File    │◄──►│  (Interpreter)  │
│                 │    │                  │    │                 │
└─────────────────┘    └──────────────────┘    └─────────────────┘
        │                        │                        │
        ▼                        ▼                        ▼
   節點拖拽編輯              YAML 程式腳本              Python 函數調用
   參數配置界面              變數和流程控制              任務創建執行
   即時程式碼生成            條件邏輯表達                WCS 業務邏輯
```

### DSL 語言特性

#### 核心語句類型
1. **function_call** - 函數調用語句
2. **create_task** - 任務創建語句 (簡化語法)
3. **if_condition** - 條件分支語句
4. **set_variable** - 變數賦值語句
5. **log** - 日誌輸出語句

#### 變數和參數系統
- 變數定義：`variables` 區塊
- 變數引用：`${variable_name}`
- 函數結果：`${step_result.field}`
- 簡單運算：`${room_id * 10 + 1}`

#### 流程控制
- 順序執行：script 步驟按順序執行
- 條件執行：每個步驟可設定執行條件
- 結果儲存：步驟結果可儲存為變數
- 分支邏輯：if_condition 支援 then/else

## 📋 具體語法設計

### 基於實際系統的 DSL 格式設計

#### 現有格式分析（基於 rack_rotation_inlet.yaml）
```yaml
# 現有的靜態配置格式
name: "Rack旋轉檢查-房間入口"
description: "當 Rack A面完成後，檢查是否需要旋轉處理 B面"
priority: 100
work_id: "220001"
enabled: true

# 觸發條件 - 所有條件必須為 true 才執行
trigger_conditions:
  - condition: "rack_at_location_exists"
    description: "房間入口位置有 Rack"
    parameters:
      location_type: "room_inlet"
  - condition: "rack_side_completed" 
    description: "Rack A面已完成"
    parameters:
      side: "A"
  - condition: "rack_has_b_side_work"
    description: "Rack B面有待處理工作"
    parameters:
      side: "B"

# 執行動作
action:
  type: "create_task"
  task_type: "rack_rotation"
  function: "rack_move"
  model: "KUKA400i"
  api: "submit_mission"
  mission_type: "RACK_MOVE"
  path:
    type: "inlet_rotation"

# 適用房間
applicable_rooms: [1, 2, 3, 4, 5]

# 調試選項
debug:
  enabled: true
  log_conditions: true
  dry_run: false
```

#### 三種節點類型分析（基於實際 nodes/ 目錄）

**⚠️ 重要發現：基於實際系統檔案的精確分析**

**Condition Nodes (輸入檢查節點)** - `/app/config/wcs/nodes/condition_nodes.yaml`:
- **節點數量**: 17個條件檢查節點
- **主要功能**:
  - 位置和可用性檢查：`check_locations_available`, `check_ng_rack_at_location`, `check_carriers_in_room`, `check_racks_at_location`
  - 七大業務流程檢查：`check_agv_rotation_flow` (優先級100), `check_ng_rack_recycling_flow` (優先級90), `check_full_rack_to_manual_flow` (優先級80), `check_manual_area_transport_flow` (優先級80), `check_system_to_room_flow` (優先級60), `check_empty_rack_transfer_flow` (優先級40), `check_manual_empty_recycling_flow` (優先級40)
  - OPUI請求檢查：`check_opui_requests_flow`
- **函數來源**: `enhanced_database_client` (位置檢查), `unified_decision_engine` (業務流程檢查)
- **輸出類型**: `bool`, `List[Dict[str, Any]]`, `List[TaskDecision]`
- **節點特性**: category=`"input"`, color=`"#3B82F6"` (藍色), 支援優先級參數配置

**Logic Nodes (邏輯控制節點)** - `/app/config/wcs/nodes/logic_nodes.yaml`:
- **節點數量**: 15個邏輯處理節點
- **主要功能**:
  - 決策引擎：`prioritize_and_schedule` (優先級排序和調度), `get_decision_statistics` (統計資料)
  - 房間位置處理：`get_room_location_info` (房間ID 1-10), `is_room_inlet`, `is_room_outlet` (位置類型判斷)
  - OPUI請求處理：`process_opui_call_empty_request`, `process_opui_dispatch_full_request`
  - AGV路徑生成：`generate_rotation_nodes` (3個節點路徑)
  - Work ID參數建構：`build_kuka_rack_move_parameters` (220001), `build_kuka_workflow_parameters` (230001), `build_opui_call_empty_parameters` (100001), `build_opui_dispatch_full_parameters` (100002), `build_cargo_agv_parameters` (2000102/2000201)
- **函數來源**: `unified_decision_engine`, `unified_task_manager.WorkIDParameterManager`
- **輸出類型**: `List[TaskDecision]`, `Dict[str, int]`, `Optional[TaskDecision]`, `List[int]`, `Dict[str, Any]`
- **節點特性**: category=`"control"`, color=`"#F59E0B"` (橙色), 支援複雜參數配置

**Action Nodes (任務執行節點)** - `/app/config/wcs/nodes/action_nodes.yaml`:
- **節點數量**: 21個任務執行節點
- **主要功能**:
  - 統一決策引擎：`task_decision_to_dict`, `run_unified_decision_cycle`
  - 七大業務流程執行：與condition nodes對應的action版本，支援完整的任務創建
  - 統一任務管理：`create_tasks_from_decisions` (批次), `create_task_from_decision` (單一)
  - 資料庫任務創建：`create_task_from_decision_dict` (直接字典創建)
  - OPUI任務管理：`update_existing_opui_task` (狀態更新)
  - 停車格狀態管理：`update_machine_parking_status`, `batch_update_parking_status`
- **函數來源**: `unified_decision_engine`, `unified_task_manager`, `enhanced_database_client`
- **輸出類型**: `Dict[str, Any]`, `List[TaskDecision]`, `List[TaskCreationResult]`, `TaskCreationResult`, `Optional[int]`, `bool`
- **節點特性**: category=`"output"`, color=`"#10B981"` (綠色), 支援任務創建的完整參數配置

**節點間的流程邏輯關係**:
1. **三層數據流**: condition → logic → action
2. **優先級體系**: AGV旋轉(100) > NG回收(90) > 滿料架/人工區(80) > 系統到房間(60) > 空料架/人工回收(40)
3. **Work ID 對應**: 每種業務流程都有特定的Work ID (如220001 KUKA移動, 230001 KUKA流程, 100001/100002 OPUI)
4. **參數傳遞**: 條件檢查結果 → 邏輯處理參數 → 動作執行參數，形成完整的數據pipeline

#### Flow Designer 整合點分析

**現有 Flow Designer 實作**（基於 `/app/web_api_ws/src/agvcui/agvcui/routers/flow_designer.py`）:
- **核心類別**: `FlowFileOperations` - 完整的 YAML 檔案 CRUD 操作
- **API 端點**: 9個 RESTful API 端點，支援流程的建立、讀取、更新、刪除
- **檔案管理**: 自動處理 `/app/config/wcs/flows/` 目錄下的 YAML 檔案
- **資料結構**: 支援 `flows`, `nodes`, `connections` 結構
- **視覺化支援**: 內建 `designer_data` 區塊支援，可儲存節點位置和連接關係
- **錯誤處理**: 完整的檔案操作錯誤處理和日誌記錄

**Simple WCS 解析器**（基於 `/app/simple_wcs_ws/src/simple_wcs/simple_wcs/flow_parser.py`）:
- **核心類別**: `FlowParser` - 支援多檔案 YAML 解析
- **資料結構**: 完整定義的 dataclass 結構
  - `BusinessFlow`: 業務流程主體，包含 name, priority, work_id, enabled 等
  - `TriggerCondition`: 觸發條件，包含 condition, description, parameters
  - `FlowAction`: 執行動作，包含 type, task_type, function, model, api 等
- **解析能力**: 支援 flows/ 目錄下所有 YAML 檔案的批量解析
- **驗證功能**: 內建 `validate_flows` 方法，檢查配置完整性和重複優先級
- **彈性支援**: 支援 `applicable_rooms` 和新增的 `applicable_locations` 配置
- **錯誤處理**: 完整的解析錯誤處理和日誌記錄

**現有格式相容性分析**（基於 `/app/config/wcs/flows/rack_rotation_inlet.yaml`）:
- **基本結構**: 標準的 YAML 格式，包含 name, description, priority, work_id, enabled
- **觸發條件**: `trigger_conditions` 陣列，每個條件包含 condition, description, parameters
- **執行動作**: 單一 `action` 物件，包含完整的任務創建參數
- **適用範圍**: `applicable_rooms` 陣列，支援多房間配置
- **調試支援**: `debug` 區塊，包含 enabled, log_conditions, dry_run 選項
- **擴展性**: 結構設計允許添加新欄位，向後相容性良好

#### YAML DSL 程式語言格式（整合三種節點）
```yaml
# YAML DSL 程式語言格式 - 基於實際節點類型
name: "Rack旋轉檢查-房間入口"
description: "當 Rack A面完成後，檢查是否需要旋轉處理 B面"
version: "1.0"
priority: 100
work_id: "220001"
enabled: true

# 變數定義區
variables:
  location_type: "room_inlet"
  completed_side: "A"
  target_side: "B"
  task_work_id: "220001"
  robot_model: "KUKA400i"
  room_id: 1

# 程式腳本主體 - 整合三種節點類型的流程邏輯
script:
  # === 條件檢查階段 (Condition Nodes) ===
  
  # 步驟1：檢查位置可用性 (condition_node)
  - step: "check_locations_available"
    type: "condition_call"
    node_type: "condition"
    function: "check_locations_available"  # 來自 condition_nodes.yaml
    parameters:
      location_ids: [${room_id * 10 + 1}]  # 計算房間入口位置ID
      status: 0  # 0=可用
    store_result: "location_available"
    description: "檢查房間入口位置可用性"
    
  # 步驟2：檢查料架狀態 (condition_node)
  - step: "check_racks_at_location"
    type: "condition_call"
    node_type: "condition"
    function: "check_racks_at_location"  # 來自 condition_nodes.yaml
    parameters:
      location_id: ${room_id * 10 + 1}
      status_filter: ["completed_a_side", "has_b_side_work"]
    store_result: "rack_status"
    condition: "${location_available} == true"
    description: "檢查位置料架狀態"
    
  # 步驟3：AGV旋轉流程檢查 (condition_node - 統一決策引擎)
  - step: "check_agv_rotation_flow"
    type: "condition_call"
    node_type: "condition"
    function: "check_agv_rotation_flow"  # 來自 condition_nodes.yaml
    parameters:
      priority: ${priority}
    store_result: "rotation_decisions"
    condition: "${rack_status.count} > 0"
    description: "AGV旋轉流程檢查 (優先級:100)"
    
  # === 邏輯處理階段 (Logic Nodes) ===
  
  # 步驟4：房間位置資訊處理 (logic_node)
  - step: "get_room_location_info"
    type: "logic_call"
    node_type: "logic"
    function: "get_room_location_info"  # 來自 logic_nodes.yaml
    parameters:
      room_id: ${room_id}
    store_result: "location_info"
    condition: "${rotation_decisions.length} > 0"
    description: "計算房間入口/出口位置和相關ID"
    
  # 步驟5：AGV旋轉路徑生成 (logic_node)
  - step: "generate_rotation_nodes"
    type: "logic_call"
    node_type: "logic"
    function: "generate_rotation_nodes"  # 來自 logic_nodes.yaml
    parameters:
      target_location: ${location_info.inlet_id}
      current_location: ${location_info.outlet_id}
    store_result: "rotation_path"
    condition: "${location_info} != null"
    description: "生成AGV旋轉的3個節點路徑"
    
  # 步驟6：優先級排序和調度 (logic_node)
  - step: "prioritize_and_schedule"
    type: "logic_call"
    node_type: "logic"
    function: "prioritize_and_schedule"  # 來自 logic_nodes.yaml
    parameters:
      task_decisions: ${rotation_decisions}
      sort_order: "priority_desc"
    store_result: "scheduled_decisions"
    condition: "${rotation_path.length} > 0"
    description: "按優先度排序決策並解決資源衝突"
    
  # 步驟7：KUKA移動貨架參數建構 (logic_node)
  - step: "build_kuka_rack_move_parameters"
    type: "logic_call"
    node_type: "logic"
    function: "build_kuka_rack_move_parameters"  # 來自 logic_nodes.yaml
    parameters:
      task_decision: ${scheduled_decisions[0]}
      work_id: ${task_work_id}  # "220001"
    store_result: "task_parameters"
    condition: "${scheduled_decisions.length} > 0"
    description: "建立KUKA移動貨架任務參數 (work_id: 220001)"
    
  # === 任務執行階段 (Action Nodes) ===
  
  # 步驟8：從決策創建任務 (action_node)
  - step: "create_task_from_decision"
    type: "action_call"
    node_type: "action"
    function: "create_task_from_decision"  # 來自 action_nodes.yaml
    parameters:
      decision: ${scheduled_decisions[0]}
      task_parameters: ${task_parameters}
    store_result: "task_creation_result"
    condition: "${task_parameters} != null"
    description: "從單一決策創建資料庫任務"
    
  # 步驟9：任務決策轉換 (action_node)
  - step: "task_decision_to_dict"
    type: "action_call"
    node_type: "action"
    function: "task_decision_to_dict"  # 來自 action_nodes.yaml
    parameters:
      task_decision: ${task_creation_result.task_decision}
    store_result: "task_dict"
    condition: "${task_creation_result.success} == true"
    description: "將TaskDecision轉換為Task表格式"

# 適用範圍
scope:
  applicable_rooms: [1, 2, 3, 4, 5]
  applicable_locations: []

# 調試和監控
debug:
  enabled: true
  log_conditions: true
  log_variables: true
  dry_run: false

# Flow Designer 視覺化數據 - 基於三種節點類型
designer_data:
  layout_version: "1.0"
  node_types_mapping:
    condition_call: "condition_nodes"
    logic_call: "logic_nodes"
    action_call: "action_nodes"
  
  nodes:
    # === 條件檢查層 (Condition Nodes) ===
    - id: "check_locations_available"
      type: "condition_call"
      node_category: "input"
      position: {x: 100, y: 100}
      color: "#3B82F6"
      icon: "📍"
      source_file: "condition_nodes.yaml"
      
    - id: "check_racks_at_location"
      type: "condition_call"
      node_category: "input"
      position: {x: 300, y: 100}
      color: "#3B82F6"
      icon: "📦"
      source_file: "condition_nodes.yaml"
      
    - id: "check_agv_rotation_flow"
      type: "condition_call"
      node_category: "input"
      position: {x: 500, y: 100}
      color: "#3B82F6"
      icon: "🔄"
      source_file: "condition_nodes.yaml"
      
    # === 邏輯處理層 (Logic Nodes) ===
    - id: "get_room_location_info"
      type: "logic_call"
      node_category: "control"
      position: {x: 100, y: 250}
      color: "#F59E0B"
      icon: "🏠"
      source_file: "logic_nodes.yaml"
      
    - id: "generate_rotation_nodes"
      type: "logic_call"
      node_category: "control"
      position: {x: 300, y: 250}
      color: "#F59E0B"
      icon: "🔄"
      source_file: "logic_nodes.yaml"
      
    - id: "prioritize_and_schedule"
      type: "logic_call"
      node_category: "control"
      position: {x: 500, y: 250}
      color: "#F59E0B"
      icon: "⚖️"
      source_file: "logic_nodes.yaml"
      
    - id: "build_kuka_rack_move_parameters"
      type: "logic_call"
      node_category: "control"
      position: {x: 700, y: 250}
      color: "#F59E0B"
      icon: "🏗️"
      source_file: "logic_nodes.yaml"
      
    # === 任務執行層 (Action Nodes) ===
    - id: "create_task_from_decision"
      type: "action_call"
      node_category: "output"
      position: {x: 100, y: 400}
      color: "#10B981"
      icon: "✏️"
      source_file: "action_nodes.yaml"
      
    - id: "task_decision_to_dict"
      type: "action_call"
      node_category: "output"
      position: {x: 300, y: 400}
      color: "#10B981"
      icon: "🔄"
      source_file: "action_nodes.yaml"
      
  connections:
    # 條件檢查層內部連接
    - from: "check_locations_available"
      to: "check_racks_at_location"
      condition: "success"
      connection_type: "condition_chain"
      
    - from: "check_racks_at_location"
      to: "check_agv_rotation_flow"
      condition: "success"
      connection_type: "condition_chain"
      
    # 條件檢查層到邏輯處理層
    - from: "check_agv_rotation_flow"
      to: "get_room_location_info"
      condition: "has_decisions"
      connection_type: "layer_transition"
      
    # 邏輯處理層內部連接
    - from: "get_room_location_info"
      to: "generate_rotation_nodes"
      condition: "success"
      connection_type: "logic_chain"
      
    - from: "generate_rotation_nodes"
      to: "prioritize_and_schedule"
      condition: "success"
      connection_type: "logic_chain"
      
    - from: "prioritize_and_schedule"
      to: "build_kuka_rack_move_parameters"
      condition: "success"
      connection_type: "logic_chain"
      
    # 邏輯處理層到任務執行層
    - from: "build_kuka_rack_move_parameters"
      to: "create_task_from_decision"
      condition: "parameters_ready"
      connection_type: "layer_transition"
      
    # 任務執行層內部連接
    - from: "create_task_from_decision"
      to: "task_decision_to_dict"
      condition: "task_created"
      connection_type: "action_chain"
      
  flow_metadata:
    entry_point: "check_locations_available"
    layer_sequence: ["condition", "logic", "action"]
    success_path: [
      "check_locations_available", "check_racks_at_location", "check_agv_rotation_flow",
      "get_room_location_info", "generate_rotation_nodes", "prioritize_and_schedule", "build_kuka_rack_move_parameters",
      "create_task_from_decision", "task_decision_to_dict"
    ]
    failure_handling: "stop_on_first_failure"
    node_type_integration:
      condition_nodes_source: "/app/config/wcs/nodes/condition_nodes.yaml"
      logic_nodes_source: "/app/config/wcs/nodes/logic_nodes.yaml"
      action_nodes_source: "/app/config/wcs/nodes/action_nodes.yaml"
```

### 語法規範定義

#### 1. 基本語句結構（整合三種節點類型）
```yaml
script:
  - step: "步驟識別名稱"           # 必填：步驟唯一識別
    type: "語句類型"              # 必填：condition_call, logic_call, action_call, if_condition, set_variable, log
    node_type: "節點類型"         # 必填：condition, logic, action (對應三種節點檔案)
    condition: "執行條件"          # 可選：${variable} == value
    description: "步驟描述"        # 可選：人類可讀的描述
    
    # 節點特定參數
    function: "函數名稱"           # 來自對應 nodes YAML 檔案的函數
    parameters: {...}            # 函數參數，基於節點定義的參數規範
    store_result: "結果變數名"     # 可選：儲存執行結果
    source_file: "來源檔案"       # 可選：指定節點定義來源 (condition_nodes.yaml 等)
```

#### 2. 變數系統
```yaml
variables:
  # 基本變數
  simple_var: "value"
  number_var: 100
  array_var: [1, 2, 3]
  object_var: {key: "value"}

# 變數引用語法
parameters:
  location: ${simple_var}               # 基本引用
  calculated: ${number_var * 2 + 1}     # 簡單運算
  array_item: ${array_var[0]}           # 陣列索引
  object_field: ${object_var.key}       # 物件欄位
  result_field: ${step_result.success}  # 函數結果欄位
```

#### 3. 條件表達式
```yaml
# 支援的條件運算子
condition: "${var1} == ${var2}"        # 等於
condition: "${var1} != ${var2}"        # 不等於
condition: "${var1} > ${var2}"         # 大於
condition: "${var1} < ${var2}"         # 小於
condition: "${var1} >= ${var2}"        # 大於等於
condition: "${var1} <= ${var2}"        # 小於等於
condition: "${var1} && ${var2}"        # 且
condition: "${var1} || ${var2}"        # 或
condition: "!${var1}"                  # 非
```

#### 4. 語句類型詳細定義

##### condition_call - 條件檢查調用
```yaml
- step: "check_system_condition"
  type: "condition_call"
  node_type: "condition"
  function: "check_agv_rotation_flow"    # 來自 condition_nodes.yaml
  parameters:                            # 基於節點定義的參數規範
    priority: ${priority}
  store_result: "condition_result"       # 儲存檢查結果
  condition: "${previous_step} == true"  # 執行條件
  source_file: "condition_nodes.yaml"   # 節點定義來源
```

##### logic_call - 邏輯處理調用
```yaml
- step: "process_business_logic"
  type: "logic_call"
  node_type: "logic"
  function: "prioritize_and_schedule"    # 來自 logic_nodes.yaml
  parameters:                            # 基於節點定義的參數規範
    task_decisions: ${condition_result}
    sort_order: "priority_desc"
  store_result: "logic_result"          # 儲存處理結果
  condition: "${condition_result.length} > 0"
  source_file: "logic_nodes.yaml"       # 節點定義來源
```

##### action_call - 任務執行調用
```yaml
- step: "execute_business_action"
  type: "action_call"
  node_type: "action"
  function: "create_task_from_decision"  # 來自 action_nodes.yaml
  parameters:                            # 基於節點定義的參數規範
    decision: ${logic_result[0]}
  store_result: "action_result"          # 儲存執行結果
  condition: "${logic_result} != null"
  source_file: "action_nodes.yaml"      # 節點定義來源
```

##### create_task - 任務創建
```yaml
- step: "create_agv_task"
  type: "create_task"
  parameters:
    task_type: "rack_rotation"
    model: "KUKA400i"
    work_id: ${work_id}
    priority: ${priority}
    # 其他任務參數...
  condition: "${all_checks_passed} == true"
```

##### if_condition - 條件分支
```yaml
- step: "conditional_logic"
  type: "if_condition"
  condition: "${check_result.count} > 0"
  then:
    - step: "action_on_success"
      type: "create_task"
      parameters: {...}
  else:
    - step: "action_on_failure"
      type: "log"
      level: "info"
      message: "條件不符合，跳過執行"
```

##### set_variable - 變數賦值
```yaml
- step: "calculate_target"
  type: "set_variable"
  variable: "target_location"
  value: "${room_id * 10 + 1}"
  condition: "${room_id} > 0"
```

##### log - 日誌輸出
```yaml
- step: "log_progress"
  type: "log"
  level: "info"                         # debug, info, warning, error
  message: "正在處理房間 ${room_id} 的旋轉檢查"
  variables: ["room_id", "check_result"] # 可選：輸出變數值
```

## 🔧 系統整合方案

### Simple WCS 解釋器設計（基於實際 FlowParser）

#### FlowParser 擴展（基於 `/app/simple_wcs_ws/src/simple_wcs/simple_wcs/flow_parser.py`）

**⚠️ 重要：基於實際系統檔案的精確實作設計**

```python
# 擴展現有的 FlowParser 類別
class YAMLDSLParser(FlowParser):
    """YAML DSL 解析器 - 擴展原有 FlowParser，整合三種節點類型
    
    基於實際的 FlowParser 實作：
    - 繼承現有的多檔案解析能力 (_parse_flows_directory)
    - 保持原有的 BusinessFlow, TriggerCondition, FlowAction 資料結構
    - 新增 DSL 腳本解析和執行能力
    - 整合三種節點類型的定義載入
    """
    
    def __init__(self, flows_dir: str = None, nodes_dir: str = None):
        super().__init__(flows_dir)
        self.nodes_dir = nodes_dir or "/app/config/wcs/nodes"
        self.variable_resolver = VariableResolver()
        self.expression_evaluator = ExpressionEvaluator()
        self.node_definitions = self.load_node_definitions()
        self.logger = logging.getLogger('yaml_dsl_parser')
    
    def load_node_definitions(self) -> Dict[str, Dict]:
        """載入三種節點類型的定義 - 基於實際檔案結構"""
        node_definitions = {}
        
        # 載入實際的節點定義檔案
        node_files = {
            "condition_nodes": "condition_nodes.yaml",  # 17個條件檢查節點
            "logic_nodes": "logic_nodes.yaml",          # 15個邏輯處理節點  
            "action_nodes": "action_nodes.yaml"         # 21個任務執行節點
        }
        
        for node_type, filename in node_files.items():
            yaml_file = Path(self.nodes_dir) / filename
            if yaml_file.exists():
                try:
                    with open(yaml_file, 'r', encoding='utf-8') as f:
                        data = yaml.safe_load(f)
                        if data and node_type in data:
                            node_definitions[node_type] = data[node_type]
                            self.logger.info(f"載入 {len(data[node_type])} 個 {node_type}")
                except Exception as e:
                    self.logger.error(f"載入節點定義失敗 {filename}: {e}")
        
        return node_definitions
    
    def parse_dsl_script(self, script_data: Dict[str, Any]) -> ExecutableScript:
        """解析 DSL 腳本為可執行物件"""
        
        # 解析變數定義
        variables = script_data.get('variables', {})
        
        # 解析腳本步驟
        script_steps = []
        for step_data in script_data.get('script', []):
            step = self.parse_script_step(step_data)
            script_steps.append(step)
        
        return ExecutableScript(
            name=script_data.get('name', 'unnamed'),
            variables=variables,
            steps=script_steps,
            debug_config=script_data.get('debug', {})
        )
    
    def parse_script_step(self, step_data: Dict[str, Any]) -> ScriptStep:
        """解析單個腳本步驟 - 支援三種節點類型"""
        
        step_type = step_data['type']
        
        if step_type == 'condition_call':
            return ConditionCallStep(
                step_id=step_data['step'],
                function_name=step_data['function'],
                node_type='condition',
                parameters=step_data.get('parameters', {}),
                condition=step_data.get('condition'),
                store_result=step_data.get('store_result'),
                description=step_data.get('description', ''),
                source_file=step_data.get('source_file', 'condition_nodes.yaml'),
                node_definition=self.get_node_definition('condition_nodes', step_data['function'])
            )
        elif step_type == 'logic_call':
            return LogicCallStep(
                step_id=step_data['step'],
                function_name=step_data['function'],
                node_type='logic',
                parameters=step_data.get('parameters', {}),
                condition=step_data.get('condition'),
                store_result=step_data.get('store_result'),
                description=step_data.get('description', ''),
                source_file=step_data.get('source_file', 'logic_nodes.yaml'),
                node_definition=self.get_node_definition('logic_nodes', step_data['function'])
            )
        elif step_type == 'action_call':
            return ActionCallStep(
                step_id=step_data['step'],
                function_name=step_data['function'],
                node_type='action',
                parameters=step_data.get('parameters', {}),
                condition=step_data.get('condition'),
                store_result=step_data.get('store_result'),
                description=step_data.get('description', ''),
                source_file=step_data.get('source_file', 'action_nodes.yaml'),
                node_definition=self.get_node_definition('action_nodes', step_data['function'])
            )
        elif step_type == 'if_condition':
            return ConditionalStep(
                step_id=step_data['step'],
                condition=step_data['condition'],
                then_steps=self.parse_sub_steps(step_data.get('then', [])),
                else_steps=self.parse_sub_steps(step_data.get('else', [])),
                description=step_data.get('description', '')
            )
        # 其他步驟類型...
    
    def get_node_definition(self, node_type: str, function_name: str) -> Optional[Dict]:
        """獲取節點定義資訊"""
        if node_type in self.node_definitions:
            return self.node_definitions[node_type].get(function_name)
        return None
    
    def validate_node_parameters(self, step: Union[ConditionCallStep, LogicCallStep, ActionCallStep]) -> bool:
        """驗證節點參數是否符合定義"""
        if not step.node_definition:
            return True  # 如果沒有定義，跳過驗證
        
        required_params = []
        if 'parameters' in step.node_definition:
            required_params = [
                param['name'] for param in step.node_definition['parameters'] 
                if param.get('required', False)
            ]
        
        for required_param in required_params:
            if required_param not in step.parameters:
                raise ValueError(f"缺少必需參數 '{required_param}' in step '{step.step_id}'")
        
        return True
```

#### DSL 執行引擎
```python
class YAMLDSLExecutor:
    """YAML DSL 執行引擎"""
    
    def __init__(self, wcs_functions: Dict[str, callable]):
        self.wcs_functions = wcs_functions
        self.variable_context = {}
        self.logger = logging.getLogger('yaml_dsl_executor')
    
    def execute_script(self, script: ExecutableScript) -> ExecutionResult:
        """執行完整的 DSL 腳本"""
        
        # 初始化變數環境
        self.variable_context = script.variables.copy()
        
        # 按順序執行腳本步驟
        for step in script.steps:
            try:
                # 檢查執行條件
                if not self.should_execute_step(step):
                    self.logger.debug(f"跳過步驟 {step.step_id}：條件不符合")
                    continue
                
                # 執行步驟
                result = self.execute_step(step)
                
                # 儲存結果
                if hasattr(step, 'store_result') and step.store_result:
                    self.variable_context[step.store_result] = result
                
                self.logger.info(f"步驟 {step.step_id} 執行完成")
                
            except Exception as e:
                self.logger.error(f"步驟 {step.step_id} 執行失敗: {e}")
                if script.debug_config.get('stop_on_error', True):
                    break
        
        return ExecutionResult(
            success=True,
            variables=self.variable_context,
            execution_log=self.get_execution_log()
        )
    
    def execute_step(self, step: ScriptStep) -> Any:
        """執行單個腳本步驟"""
        
        if isinstance(step, ConditionCallStep):
            return self.execute_condition_call(step)
        elif isinstance(step, LogicCallStep):
            return self.execute_logic_call(step)
        elif isinstance(step, ActionCallStep):
            return self.execute_action_call(step)
        elif isinstance(step, ConditionalStep):
            return self.execute_conditional(step)
        # 其他步驟類型...
    
    def execute_condition_call(self, step: ConditionCallStep) -> Any:
        """執行條件檢查步驟"""
        # 解析參數中的變數引用
        resolved_params = self.resolve_parameters(step.parameters)
        
        # 從 unified_decision_engine 或 enhanced_database_client 調用函數
        function_source = step.node_definition.get('conditions', [{}])[0].get('source', 'enhanced_database_client')
        
        if function_source == 'unified_decision_engine':
            # 調用統一決策引擎的條件檢查函數
            if step.function_name in self.wcs_functions['unified_decision_engine']:
                function = self.wcs_functions['unified_decision_engine'][step.function_name]
                return function(**resolved_params)
        elif function_source == 'enhanced_database_client':
            # 調用增強資料庫客戶端的條件檢查函數
            if step.function_name in self.wcs_functions['enhanced_database_client']:
                function = self.wcs_functions['enhanced_database_client'][step.function_name]
                return function(**resolved_params)
        
        raise ValueError(f"找不到條件檢查函數: {step.function_name} in {function_source}")
    
    def execute_logic_call(self, step: LogicCallStep) -> Any:
        """執行邏輯處理步驟"""
        # 解析參數中的變數引用
        resolved_params = self.resolve_parameters(step.parameters)
        
        # 從對應的來源調用邏輯函數
        function_source = step.node_definition.get('logic', [{}])[0].get('source', 'unified_decision_engine')
        
        if function_source.startswith('unified_decision_engine'):
            if step.function_name in self.wcs_functions['unified_decision_engine']:
                function = self.wcs_functions['unified_decision_engine'][step.function_name]
                return function(**resolved_params)
        elif function_source.startswith('unified_task_manager'):
            if step.function_name in self.wcs_functions['unified_task_manager']:
                function = self.wcs_functions['unified_task_manager'][step.function_name]
                return function(**resolved_params)
        
        raise ValueError(f"找不到邏輯處理函數: {step.function_name} in {function_source}")
    
    def execute_action_call(self, step: ActionCallStep) -> Any:
        """執行任務執行步驟"""
        # 解析參數中的變數引用
        resolved_params = self.resolve_parameters(step.parameters)
        
        # 從對應的來源調用動作函數
        function_source = step.node_definition.get('actions', [{}])[0].get('source', 'unified_task_manager')
        
        if function_source == 'unified_task_manager':
            if step.function_name in self.wcs_functions['unified_task_manager']:
                function = self.wcs_functions['unified_task_manager'][step.function_name]
                return function(**resolved_params)
        elif function_source == 'enhanced_database_client':
            if step.function_name in self.wcs_functions['enhanced_database_client']:
                function = self.wcs_functions['enhanced_database_client'][step.function_name]
                return function(**resolved_params)
        elif function_source.startswith('unified_decision_engine'):
            if step.function_name in self.wcs_functions['unified_decision_engine']:
                function = self.wcs_functions['unified_decision_engine'][step.function_name]
                return function(**resolved_params)
        
        raise ValueError(f"找不到任務執行函數: {step.function_name} in {function_source}")
    
    def execute_function_call(self, step: FunctionCallStep) -> Any:
        """執行函數調用步驟"""
        
        # 解析參數中的變數引用
        resolved_params = self.resolve_parameters(step.parameters)
        
        # 調用 WCS 函數
        if step.function_name in self.wcs_functions:
            function = self.wcs_functions[step.function_name]
            return function(**resolved_params)
        else:
            raise ValueError(f"未知的 WCS 函數: {step.function_name}")
    
    def execute_create_task(self, step: CreateTaskStep) -> Any:
        """執行任務創建步驟"""
        
        # 解析任務參數
        resolved_params = self.resolve_parameters(step.task_parameters)
        
        # 調用任務創建函數
        task_creator = self.wcs_functions.get('create_task_from_parameters')
        if task_creator:
            return task_creator(resolved_params)
        else:
            raise ValueError("任務創建函數未註冊")
```

#### 變數解析器
```python
class VariableResolver:
    """變數和表達式解析器"""
    
    def __init__(self):
        self.expression_pattern = re.compile(r'\$\{([^}]+)\}')
    
    def resolve_value(self, value: Any, context: Dict[str, Any]) -> Any:
        """解析值中的變數引用"""
        
        if isinstance(value, str):
            return self.resolve_string_expressions(value, context)
        elif isinstance(value, dict):
            return {k: self.resolve_value(v, context) for k, v in value.items()}
        elif isinstance(value, list):
            return [self.resolve_value(item, context) for item in value]
        else:
            return value
    
    def resolve_string_expressions(self, text: str, context: Dict[str, Any]) -> Any:
        """解析字串中的變數表達式"""
        
        # 檢查是否為純表達式 (整個字串都是 ${...})
        if text.startswith('${') and text.endswith('}') and text.count('${') == 1:
            expression = text[2:-1]
            return self.evaluate_expression(expression, context)
        
        # 替換字串中的變數引用
        def replace_expression(match):
            expression = match.group(1)
            result = self.evaluate_expression(expression, context)
            return str(result)
        
        return self.expression_pattern.sub(replace_expression, text)
    
    def evaluate_expression(self, expression: str, context: Dict[str, Any]) -> Any:
        """評估表達式"""
        
        # 簡單變數引用
        if expression in context:
            return context[expression]
        
        # 物件欄位存取 (如: obj.field)
        if '.' in expression:
            parts = expression.split('.')
            value = context.get(parts[0])
            for part in parts[1:]:
                if hasattr(value, part):
                    value = getattr(value, part)
                elif isinstance(value, dict) and part in value:
                    value = value[part]
                else:
                    raise ValueError(f"無法存取 {expression}")
            return value
        
        # 陣列索引存取 (如: arr[0])
        if '[' in expression and ']' in expression:
            var_name = expression.split('[')[0]
            index_str = expression.split('[')[1].split(']')[0]
            
            array = context.get(var_name)
            index = int(index_str) if index_str.isdigit() else self.evaluate_expression(index_str, context)
            
            return array[index]
        
        # 數學運算 (簡單支援)
        try:
            # 安全的數學表達式評估
            allowed_names = {
                k: v for k, v in context.items() 
                if isinstance(v, (int, float, bool))
            }
            allowed_names.update({
                '__builtins__': {},
                'True': True,
                'False': False,
                'None': None
            })
            
            return eval(expression, {"__builtins__": {}}, allowed_names)
        except:
            raise ValueError(f"無法評估表達式: {expression}")
```

### Flow Designer 整合設計

#### 節點類型定義（基於三種節點類型）
```typescript
// Flow Designer 節點類型定義 - 整合三種節點類型
interface DSLNode {
  id: string;
  type: 'condition_call' | 'logic_call' | 'action_call' | 'if_condition' | 'set_variable' | 'log';
  node_type: 'condition' | 'logic' | 'action' | 'control' | 'utility';
  node_category: 'input' | 'control' | 'output';
  position: { x: number, y: number };
  data: {
    step_id: string;
    description?: string;
    condition?: string;
    parameters: Record<string, any>;
    // 節點特定參數
    function_name?: string;      // 來自節點定義檔案的函數名稱
    store_result?: string;       // 結果儲存變數名
    source_file?: string;        // 節點定義來源檔案
    node_definition?: NodeDefinition;  // 完整的節點定義資訊
    // 向後相容的參數
    variable_name?: string;      // set_variable 專用
    variable_value?: any;        // set_variable 專用
    log_level?: string;          // log 專用
    log_message?: string;        // log 專用
  };
  style: {
    backgroundColor: string;
    borderColor: string;
    color: string;
  };
}

// 節點定義介面（基於實際 nodes/ YAML 結構）
interface NodeDefinition {
  name: string;
  description: string;
  category: 'input' | 'control' | 'output';
  icon: string;
  color: string;
  inputs: string[];
  outputs?: string[];
  parameters: NodeParameter[];
  conditions?: NodeFunction[];  // condition_nodes 專用
  logic?: NodeFunction[];       // logic_nodes 專用
  actions?: NodeFunction[];     // action_nodes 專用
}

interface NodeParameter {
  name: string;
  type: 'string' | 'integer' | 'list' | 'select' | 'object';
  required?: boolean;
  default?: any;
  options?: any[];
  min?: number;
  max?: number;
  description: string;
}

interface NodeFunction {
  function: string;
  source: string;
  returns: string;
}

interface DSLConnection {
  id: string;
  source: string;
  target: string;
  type: 'success' | 'failure' | 'always';
  data: {
    condition?: string;
  };
}

interface DSLFlow {
  id: string;
  name: string;
  description: string;
  variables: Record<string, any>;
  nodes: DSLNode[];
  connections: DSLConnection[];
  metadata: {
    version: string;
    priority: number;
    work_id: string;
    enabled: boolean;
    applicable_rooms: number[];
  };
}
```

#### 視覺化編輯器組件
```javascript
// Flow Designer DSL 編輯器主組件
class DSLFlowEditor {
  constructor(container, options = {}) {
    this.container = container;
    this.options = options;
    this.flowData = null;
    this.nodeTypes = this.initializeNodeTypes();
    this.init();
  }
  
  initializeNodeTypes() {
    return {
      // === 條件檢查節點 (Condition Nodes) ===
      condition_call: {
        label: '條件檢查',
        icon: '🔍',
        color: '#3B82F6',
        node_category: 'input',
        inputs: [],
        outputs: ['success', 'failure'],
        source_file: 'condition_nodes.yaml',
        available_functions: this.loadNodeFunctions('condition_nodes'),
        parameters: [
          { name: 'function_name', type: 'select', options: this.getConditionFunctions(), required: true },
          { name: 'parameters', type: 'object' },
          { name: 'store_result', type: 'string' },
          { name: 'condition', type: 'expression' }
        ]
      },
      
      // === 邏輯處理節點 (Logic Nodes) ===
      logic_call: {
        label: '邏輯處理',
        icon: '⚙️',
        color: '#F59E0B',
        node_category: 'control',
        inputs: ['trigger'],
        outputs: ['success', 'failure'],
        source_file: 'logic_nodes.yaml',
        available_functions: this.loadNodeFunctions('logic_nodes'),
        parameters: [
          { name: 'function_name', type: 'select', options: this.getLogicFunctions(), required: true },
          { name: 'parameters', type: 'object' },
          { name: 'store_result', type: 'string' },
          { name: 'condition', type: 'expression' }
        ]
      },
      
      // === 任務執行節點 (Action Nodes) ===
      action_call: {
        label: '任務執行',
        icon: '🎯',
        color: '#10B981',
        node_category: 'output',
        inputs: ['trigger'],
        outputs: ['success', 'failure'],
        source_file: 'action_nodes.yaml',
        available_functions: this.loadNodeFunctions('action_nodes'),
        parameters: [
          { name: 'function_name', type: 'select', options: this.getActionFunctions(), required: true },
          { name: 'parameters', type: 'object' },
          { name: 'store_result', type: 'string' },
          { name: 'condition', type: 'expression' }
        ]
      },
      // === 向後相容的節點類型 ===
      create_task: {
        label: '創建任務 (舊格式)',
        icon: '📝',
        color: '#6B7280',
        node_category: 'output',
        inputs: ['trigger'],
        outputs: ['success', 'failure'],
        deprecated: true,
        migration_suggestion: 'use action_call with create_task_from_decision function',
        parameters: [
          { name: 'task_type', type: 'select', options: ['rack_rotation', 'rack_move', 'carrier_transport'] },
          { name: 'model', type: 'select', options: ['KUKA400i', 'cargo', 'loader', 'unloader'] },
          { name: 'work_id', type: 'string' },
          { name: 'priority', type: 'number' },
          { name: 'condition', type: 'expression' }
        ]
      },
      if_condition: {
        label: '條件分支',
        icon: '🔀',
        color: '#F59E0B',
        inputs: ['trigger'],
        outputs: ['then', 'else'],
        parameters: [
          { name: 'condition', type: 'expression', required: true }
        ]
      },
      set_variable: {
        label: '設定變數',
        icon: '💾',
        color: '#8B5CF6',
        inputs: ['trigger'],
        outputs: ['success'],
        parameters: [
          { name: 'variable_name', type: 'string', required: true },
          { name: 'variable_value', type: 'any', required: true },
          { name: 'condition', type: 'expression' }
        ]
      },
      log: {
        label: '日誌輸出',
        icon: '📋',
        color: '#6B7280',
        inputs: ['trigger'],
        outputs: ['success'],
        parameters: [
          { name: 'log_level', type: 'select', options: ['debug', 'info', 'warning', 'error'] },
          { name: 'log_message', type: 'string', required: true },
          { name: 'condition', type: 'expression' }
        ]
      }
    };
  }
  
  // === 節點函數載入方法 ===
  
  loadNodeFunctions(nodeType) {
    // 從對應的 nodes YAML 檔案載入可用函數
    return fetch(`/api/nodes/${nodeType}/functions`)
      .then(response => response.json())
      .catch(() => []);
  }
  
  getConditionFunctions() {
    // 從 condition_nodes.yaml 獲取可用的條件檢查函數
    return [
      'check_locations_available',
      'check_ng_rack_at_location', 
      'check_carriers_in_room',
      'check_racks_at_location',
      'check_agv_rotation_flow',
      'check_ng_rack_recycling_flow',
      'check_full_rack_to_manual_flow',
      'check_manual_area_transport_flow',
      'check_system_to_room_flow',
      'check_empty_rack_transfer_flow',
      'check_manual_empty_recycling_flow',
      'check_opui_requests_flow'
    ];
  }
  
  getLogicFunctions() {
    // 從 logic_nodes.yaml 獲取可用的邏輯處理函數
    return [
      'prioritize_and_schedule',
      'get_room_location_info',
      'process_opui_call_empty_request',
      'process_opui_dispatch_full_request',
      'generate_rotation_nodes',
      'is_room_inlet',
      'is_room_outlet',
      'get_decision_statistics',
      'build_kuka_rack_move_parameters',
      'build_kuka_workflow_parameters',
      'build_opui_call_empty_parameters',
      'build_opui_dispatch_full_parameters',
      'build_cargo_agv_parameters'
    ];
  }
  
  getActionFunctions() {
    // 從 action_nodes.yaml 獲取可用的任務執行函數
    return [
      'task_decision_to_dict',
      'run_unified_decision_cycle',
      'check_agv_rotation_flow',
      'check_ng_rack_recycling_flow',
      'check_full_rack_to_manual_flow',
      'check_manual_area_transport_flow',
      'check_system_to_room_flow',
      'check_empty_rack_transfer_flow',
      'check_manual_empty_recycling_flow',
      'check_opui_requests_flow',
      'create_tasks_from_decisions',
      'create_task_from_decision',
      'create_task_from_decision_dict',
      'update_existing_opui_task',
      'update_machine_parking_status',
      'batch_update_parking_status'
    ];
  }
  
  // 生成 YAML DSL 代碼
  generateYAMLCode() {
    const yamlData = {
      name: this.flowData.name,
      description: this.flowData.description,
      version: "1.0",
      priority: this.flowData.metadata.priority,
      work_id: this.flowData.metadata.work_id,
      enabled: this.flowData.metadata.enabled,
      
      variables: this.flowData.variables,
      
      script: this.generateScriptSteps(),
      
      scope: {
        applicable_rooms: this.flowData.metadata.applicable_rooms,
        applicable_locations: []
      },
      
      debug: {
        enabled: true,
        log_conditions: true,
        log_variables: true,
        dry_run: false
      },
      
      designer_data: {
        layout_version: "1.0",
        nodes: this.flowData.nodes.map(node => ({
          id: node.id,
          type: node.type,
          position: node.position,
          color: node.style.backgroundColor,
          icon: this.nodeTypes[node.type].icon
        })),
        connections: this.flowData.connections.map(conn => ({
          from: conn.source,
          to: conn.target,
          condition: conn.data.condition || conn.type
        })),
        flow_metadata: {
          entry_point: this.findEntryPoint(),
          success_path: this.generateSuccessPath(),
          failure_handling: "stop_on_first_failure"
        }
      }
    };
    
    return yaml.dump(yamlData, { indent: 2, lineWidth: -1 });
  }
  
  generateScriptSteps() {
    // 根據節點順序和連接關係生成腳本步驟
    const steps = [];
    const executionOrder = this.calculateExecutionOrder();
    
    for (const nodeId of executionOrder) {
      const node = this.flowData.nodes.find(n => n.id === nodeId);
      if (!node) continue;
      
      const step = {
        step: node.data.step_id || node.id,
        type: node.type,
        description: node.data.description
      };
      
      // 添加執行條件
      if (node.data.condition) {
        step.condition = node.data.condition;
      }
      
      // 根據節點類型添加特定參數
      switch (node.type) {
        case 'condition_call':
          step.function = node.data.function_name;
          step.node_type = 'condition';
          step.parameters = node.data.parameters;
          step.source_file = 'condition_nodes.yaml';
          if (node.data.store_result) {
            step.store_result = node.data.store_result;
          }
          break;
          
        case 'logic_call':
          step.function = node.data.function_name;
          step.node_type = 'logic';
          step.parameters = node.data.parameters;
          step.source_file = 'logic_nodes.yaml';
          if (node.data.store_result) {
            step.store_result = node.data.store_result;
          }
          break;
          
        case 'action_call':
          step.function = node.data.function_name;
          step.node_type = 'action';
          step.parameters = node.data.parameters;
          step.source_file = 'action_nodes.yaml';
          if (node.data.store_result) {
            step.store_result = node.data.store_result;
          }
          break;
          
        case 'create_task':
          // 向後相容的任務創建格式
          step.type = 'action_call';  // 自動轉換為新格式
          step.function = 'create_task_from_decision_dict';
          step.node_type = 'action';
          step.source_file = 'action_nodes.yaml';
          step.parameters = {
            task_type: node.data.task_type,
            model: node.data.model || 'KUKA400i',
            work_id: node.data.work_id,
            priority: node.data.priority,
            name: `${node.data.task_type}_task`,
            description: `自動創建的${node.data.task_type}任務`,
            ...node.data.parameters
          };
          break;
          
        case 'if_condition':
          step.condition = node.data.condition;
          step.then = this.generateConditionalBranch(node, 'then');
          step.else = this.generateConditionalBranch(node, 'else');
          break;
          
        case 'set_variable':
          step.variable = node.data.variable_name;
          step.value = node.data.variable_value;
          break;
          
        case 'log':
          step.level = node.data.log_level || 'info';
          step.message = node.data.log_message;
          break;
      }
      
      steps.push(step);
    }
    
    return steps;
  }
  
  // 從 YAML DSL 代碼載入流程
  loadFromYAML(yamlCode) {
    try {
      const yamlData = yaml.load(yamlCode);
      
      this.flowData = {
        id: yamlData.name.replace(/\s+/g, '_'),
        name: yamlData.name,
        description: yamlData.description || '',
        variables: yamlData.variables || {},
        nodes: [],
        connections: [],
        metadata: {
          version: yamlData.version || '1.0',
          priority: yamlData.priority || 50,
          work_id: yamlData.work_id || '',
          enabled: yamlData.enabled !== false,
          applicable_rooms: yamlData.scope?.applicable_rooms || []
        }
      };
      
      // 轉換腳本步驟為節點
      if (yamlData.script) {
        this.convertScriptToNodes(yamlData.script);
      }
      
      // 載入視覺化數據
      if (yamlData.designer_data) {
        this.loadDesignerData(yamlData.designer_data);
      }
      
      this.renderFlow();
      
    } catch (error) {
      console.error('載入 YAML 失敗:', error);
      throw new Error(`YAML 格式錯誤: ${error.message}`);
    }
  }
}
```

#### 參數配置界面
```javascript
// 節點參數配置面板
class NodeParameterPanel {
  constructor(node, nodeType, onUpdate) {
    this.node = node;
    this.nodeType = nodeType;
    this.onUpdate = onUpdate;
    this.element = this.createElement();
  }
  
  createElement() {
    const panel = document.createElement('div');
    panel.className = 'node-parameter-panel';
    
    // 基本資訊
    panel.appendChild(this.createBasicInfoSection());
    
    // 執行條件
    panel.appendChild(this.createConditionSection());
    
    // 特定參數
    panel.appendChild(this.createParameterSection());
    
    return panel;
  }
  
  createParameterSection() {
    const section = document.createElement('div');
    section.className = 'parameter-section';
    
    const title = document.createElement('h4');
    title.textContent = '參數配置';
    section.appendChild(title);
    
    // 根據節點類型動態生成參數表單
    for (const param of this.nodeType.parameters) {
      const formGroup = this.createParameterInput(param);
      section.appendChild(formGroup);
    }
    
    return section;
  }
  
  createParameterInput(param) {
    const formGroup = document.createElement('div');
    formGroup.className = 'form-group';
    
    const label = document.createElement('label');
    label.textContent = param.label || param.name;
    if (param.required) {
      label.classList.add('required');
    }
    formGroup.appendChild(label);
    
    let input;
    
    switch (param.type) {
      case 'string':
        input = document.createElement('input');
        input.type = 'text';
        input.value = this.node.data[param.name] || param.default || '';
        break;
        
      case 'number':
        input = document.createElement('input');
        input.type = 'number';
        input.value = this.node.data[param.name] || param.default || 0;
        break;
        
      case 'select':
        input = document.createElement('select');
        for (const option of param.options || []) {
          const optionElement = document.createElement('option');
          optionElement.value = option.value || option;
          optionElement.textContent = option.label || option;
          input.appendChild(optionElement);
        }
        input.value = this.node.data[param.name] || param.default || '';
        break;
        
      case 'expression':
        input = document.createElement('textarea');
        input.className = 'expression-input';
        input.placeholder = '例如: ${variable} == true';
        input.value = this.node.data[param.name] || param.default || '';
        break;
        
      case 'object':
        input = document.createElement('textarea');
        input.className = 'json-input';
        input.placeholder = '請輸入 JSON 格式的物件';
        const objValue = this.node.data[param.name] || param.default || {};
        input.value = JSON.stringify(objValue, null, 2);
        break;
        
      default:
        input = document.createElement('input');
        input.type = 'text';
        input.value = this.node.data[param.name] || param.default || '';
    }
    
    input.addEventListener('change', () => {
      let value = input.value;
      
      // 特殊處理不同類型的值
      if (param.type === 'number') {
        value = parseFloat(value) || 0;
      } else if (param.type === 'object') {
        try {
          value = JSON.parse(value);
        } catch (e) {
          console.warn('無效的 JSON 格式');
          return;
        }
      }
      
      this.node.data[param.name] = value;
      this.onUpdate(this.node);
    });
    
    formGroup.appendChild(input);
    return formGroup;
  }
}
```

## 🚀 實施階段規劃

### 第一階段：DSL 語法定義和解析器 (2-3週)

#### 里程碑 1.1：語法規範定義
- **目標**: 完成 YAML DSL 語法規範文檔
- **交付物**:
  - 語法規範文檔 (BNF 格式)
  - 語句類型定義和範例
  - 變數系統和表達式規範
  - 錯誤處理規範
- **驗收標準**: 語法規範可以完整描述所有預期功能

#### 里程碑 1.2：基礎解析器實作
- **目標**: 實作 YAML DSL 解析器
- **交付物**:
  - `YAMLDSLParser` 類別實作
  - `VariableResolver` 變數解析器
  - `ExpressionEvaluator` 表達式求值器
  - 基本的語法錯誤檢測
- **驗收標準**: 可以正確解析 DSL 腳本為內部物件結構

#### 里程碑 1.3：執行引擎核心
- **目標**: 完成 DSL 執行引擎核心功能
- **交付物**:
  - `YAMLDSLExecutor` 執行引擎
  - 步驟執行邏輯 (function_call, create_task)
  - 變數環境管理
  - 執行日誌和除錯支援
- **驗收標準**: 可以執行簡單的 DSL 腳本並調用 WCS 函數

### 第二階段：Simple WCS 整合 (2-3週)

#### 里程碑 2.1：FlowParser 擴展
- **目標**: 擴展現有 FlowParser 支援 DSL 格式
- **交付物**:
  - 向後兼容的 FlowParser 擴展
  - DSL 格式檢測和自動切換
  - 現有 trigger_conditions 格式轉換
  - 單元測試覆蓋
- **驗收標準**: Simple WCS 可以同時支援舊格式和新 DSL 格式

#### 里程碑 2.2：WCS 函數註冊系統
- **目標**: 建立 WCS 函數註冊和管理系統
- **交付物**:
  - 函數註冊機制
  - 函數參數驗證
  - 函數執行結果標準化
  - 錯誤處理和日誌記錄
- **驗收標準**: DSL 腳本可以調用所有現有 WCS 函數

#### 里程碑 2.3：執行環境整合
- **目標**: 將 DSL 執行引擎整合到 Simple WCS 中
- **交付物**:
  - SimpleWCSEngine 中的 DSL 支援
  - 決策循環中的 DSL 腳本執行
  - 執行狀態監控和報告
  - 效能最佳化
- **驗收標準**: Simple WCS 可以穩定執行 DSL 腳本並產生任務

### 第三階段：Flow Designer 視覺化編輯器 (3-4週)

#### 里程碑 3.1：節點類型定義
- **目標**: 完成所有 DSL 節點類型的視覺化定義
- **交付物**:
  - 節點類型庫 (function_call, create_task, if_condition, set_variable, log)
  - 節點樣式和圖示設計
  - 節點參數配置介面規範
  - 連接規則定義
- **驗收標準**: 所有 DSL 語句類型都有對應的視覺化節點

#### 里程碑 3.2：拖拽編輯器實作
- **目標**: 實作拖拽式 DSL 流程編輯器
- **交付物**:
  - 節點拖拽和放置功能
  - 節點連接和連線編輯
  - 參數配置面板
  - 即時預覽和驗證
- **驗收標準**: 可以通過拖拽操作建立完整的 DSL 流程

#### 里程碑 3.3：YAML 代碼生成
- **目標**: 實作視覺化設計到 YAML 代碼的轉換
- **交付物**:
  - 視覺化流程 → YAML DSL 轉換器
  - 代碼格式化和最佳化
  - 語法驗證和錯誤提示
  - 即時代碼預覽
- **驗收標準**: 視覺化編輯器可以生成正確的 YAML DSL 代碼

#### 里程碑 3.4：雙向同步
- **目標**: 實現視覺化編輯和代碼編輯的雙向同步
- **交付物**:
  - YAML DSL → 視覺化流程解析器
  - 代碼變更檢測和同步機制
  - 衝突解決策略
  - 變更歷史追蹤
- **驗收標準**: 可以在視覺化編輯和代碼編輯間無縫切換

### 第四階段：測試、最佳化和部署 (2-3週)

#### 里程碑 4.1：系統測試
- **目標**: 完成完整系統的功能和整合測試
- **交付物**:
  - 完整的測試套件 (單元測試、整合測試、端到端測試)
  - 效能測試和基準測試
  - 相容性測試 (舊格式支援)
  - 使用者接受測試
- **驗收標準**: 所有測試通過，系統穩定可靠

#### 里程碑 4.2：文檔和培訓
- **目標**: 完成用戶文檔和培訓材料
- **交付物**:
  - 使用者手冊和教學文檔
  - API 文檔和開發者指南
  - 最佳實踐指南
  - 培訓影片和範例
- **驗收標準**: 用戶可以獨立使用系統完成業務流程設計

#### 里程碑 4.3：生產部署
- **目標**: 完成系統的生產環境部署
- **交付物**:
  - 部署腳本和配置
  - 監控和日誌系統
  - 備份和恢復機制
  - 維護手冊
- **驗收標準**: 系統在生產環境中穩定運行

## 🧪 測試和驗證計劃

### 單元測試計劃

#### DSL 解析器測試
```python
class TestYAMLDSLParser:
    def test_parse_basic_script(self):
        """測試基本腳本解析"""
        yaml_content = """
        name: "測試腳本"
        variables:
          test_var: "test_value"
        script:
          - step: "test_step"
            type: "function_call"
            function: "test_function"
            parameters:
              param1: ${test_var}
        """
        
        parser = YAMLDSLParser()
        script = parser.parse_dsl_script(yaml.safe_load(yaml_content))
        
        assert script.name == "測試腳本"
        assert script.variables["test_var"] == "test_value"
        assert len(script.steps) == 1
        assert script.steps[0].function_name == "test_function"
    
    def test_variable_resolution(self):
        """測試變數解析"""
        resolver = VariableResolver()
        context = {"var1": 10, "var2": 20, "obj": {"field": "value"}}
        
        # 基本變數引用
        assert resolver.resolve_value("${var1}", context) == 10
        
        # 數學運算
        assert resolver.resolve_value("${var1 + var2}", context) == 30
        
        # 物件欄位存取
        assert resolver.resolve_value("${obj.field}", context) == "value"
        
        # 字串模板
        assert resolver.resolve_value("result: ${var1}", context) == "result: 10"
    
    def test_expression_evaluation(self):
        """測試條件表達式求值"""
        evaluator = ExpressionEvaluator()
        context = {"a": 10, "b": 20, "flag": True}
        
        assert evaluator.evaluate("${a} > ${b}", context) == False
        assert evaluator.evaluate("${a} < ${b}", context) == True
        assert evaluator.evaluate("${flag} == true", context) == True
        assert evaluator.evaluate("${a} + ${b} == 30", context) == True
```

#### DSL 執行器測試
```python
class TestYAMLDSLExecutor:
    def setUp(self):
        # 模擬 WCS 函數
        self.mock_functions = {
            'test_function': lambda param1: {"result": f"processed_{param1}"},
            'create_task_from_parameters': lambda params: {"task_id": 12345},
            'check_condition': lambda: {"success": True, "count": 5}
        }
        self.executor = YAMLDSLExecutor(self.mock_functions)
    
    def test_execute_function_call(self):
        """測試函數調用執行"""
        step = FunctionCallStep(
            step_id="test_call",
            function_name="test_function",
            parameters={"param1": "test_input"},
            store_result="call_result"
        )
        
        result = self.executor.execute_step(step)
        assert result["result"] == "processed_test_input"
    
    def test_execute_conditional_step(self):
        """測試條件步驟執行"""
        # 設定變數環境
        self.executor.variable_context = {"condition_result": {"success": True}}
        
        then_step = FunctionCallStep(
            step_id="then_action",
            function_name="test_function",
            parameters={"param1": "then_executed"}
        )
        
        conditional_step = ConditionalStep(
            step_id="test_condition",
            condition="${condition_result.success} == true",
            then_steps=[then_step],
            else_steps=[]
        )
        
        result = self.executor.execute_step(conditional_step)
        assert result is not None
    
    def test_variable_context_management(self):
        """測試變數環境管理"""
        script_data = {
            "name": "變數測試",
            "variables": {"initial_var": "initial_value"},
            "script": [
                {
                    "step": "set_var",
                    "type": "function_call",
                    "function": "test_function",
                    "parameters": {"param1": "${initial_var}"},
                    "store_result": "function_result"
                }
            ]
        }
        
        parser = YAMLDSLParser()
        script = parser.parse_dsl_script(script_data)
        result = self.executor.execute_script(script)
        
        assert "initial_var" in result.variables
        assert "function_result" in result.variables
        assert result.variables["function_result"]["result"] == "processed_initial_value"
```

### 整合測試計劃

#### Simple WCS 整合測試
```python
class TestSimpleWCSIntegration:
    def setUp(self):
        # 設定測試環境
        self.test_flows_dir = "/tmp/test_flows"
        os.makedirs(self.test_flows_dir, exist_ok=True)
        
        # 準備測試用的 DSL 檔案
        self.create_test_dsl_file()
    
    def create_test_dsl_file(self):
        """創建測試用的 DSL 檔案"""
        dsl_content = """
        name: "整合測試流程"
        version: "1.0"
        priority: 100
        work_id: "999999"
        enabled: true
        
        variables:
          location_type: "test_location"
          robot_model: "KUKA400i"
        
        script:
          - step: "check_location"
            type: "function_call"
            function: "rack_at_location_exists"
            parameters:
              location_type: ${location_type}
            store_result: "location_check"
            
          - step: "create_test_task"
            type: "create_task"
            condition: "${location_check} == true"
            parameters:
              task_type: "test_task"
              model: ${robot_model}
              work_id: "999999"
              priority: 100
        """
        
        with open(f"{self.test_flows_dir}/test_flow.yaml", 'w') as f:
            f.write(dsl_content)
    
    def test_flow_parser_dsl_support(self):
        """測試 FlowParser 的 DSL 支援"""
        parser = FlowParser(self.test_flows_dir)
        flows = parser.parse()
        
        assert len(flows) > 0
        test_flow = flows[0]
        assert test_flow.name == "整合測試流程"
        assert test_flow.priority == 100
    
    def test_wcs_engine_dsl_execution(self):
        """測試 WCS 引擎的 DSL 執行"""
        # 這裡需要模擬完整的 WCS 環境
        # 包括資料庫連接、函數註冊等
        pass
```

#### Flow Designer 整合測試
```javascript
// Flow Designer 整合測試
describe('Flow Designer DSL Integration', () => {
  let editor;
  
  beforeEach(() => {
    const container = document.createElement('div');
    editor = new DSLFlowEditor(container);
  });
  
  test('should load DSL from YAML', () => {
    const yamlCode = `
      name: "測試流程"
      version: "1.0"
      variables:
        test_var: "test_value"
      script:
        - step: "test_step"
          type: "function_call"
          function: "test_function"
          parameters:
            param1: \${test_var}
    `;
    
    editor.loadFromYAML(yamlCode);
    
    expect(editor.flowData.name).toBe("測試流程");
    expect(editor.flowData.variables.test_var).toBe("test_value");
    expect(editor.flowData.nodes).toHaveLength(1);
    expect(editor.flowData.nodes[0].type).toBe("function_call");
  });
  
  test('should generate YAML from visual design', () => {
    // 創建測試用的視覺化流程
    editor.flowData = {
      name: "視覺化測試",
      description: "測試生成 YAML",
      variables: { test_var: "value" },
      nodes: [
        {
          id: "node1",
          type: "function_call",
          data: {
            step_id: "test_call",
            function_name: "test_function",
            parameters: { param1: "test" }
          }
        }
      ],
      connections: [],
      metadata: {
        priority: 50,
        work_id: "test_work",
        enabled: true,
        applicable_rooms: [1, 2, 3]
      }
    };
    
    const generatedYAML = editor.generateYAMLCode();
    
    expect(generatedYAML).toContain("name: \"視覺化測試\"");
    expect(generatedYAML).toContain("test_var: value");
    expect(generatedYAML).toContain("step: test_call");
    expect(generatedYAML).toContain("function: test_function");
  });
  
  test('should handle node parameter updates', () => {
    const node = {
      id: "test_node",
      type: "function_call",
      data: { function_name: "old_function" }
    };
    
    const panel = new NodeParameterPanel(node, editor.nodeTypes.function_call, (updatedNode) => {
      expect(updatedNode.data.function_name).toBe("new_function");
    });
    
    // 模擬參數更新
    node.data.function_name = "new_function";
    panel.onUpdate(node);
  });
});
```

### 端到端測試計劃

#### 完整工作流程測試
```python
class TestEndToEndWorkflow:
    """端到端工作流程測試"""
    
    def test_complete_dsl_workflow(self):
        """測試完整的 DSL 工作流程"""
        
        # 1. 在 Flow Designer 中創建流程
        flow_design = self.create_visual_flow()
        
        # 2. 生成 YAML DSL 代碼
        yaml_code = self.generate_yaml_from_design(flow_design)
        
        # 3. 驗證 YAML 語法正確性
        parsed_data = yaml.safe_load(yaml_code)
        assert parsed_data['name'] is not None
        assert 'script' in parsed_data
        
        # 4. Simple WCS 載入並解析 DSL
        parser = YAMLDSLParser()
        script = parser.parse_dsl_script(parsed_data)
        assert script is not None
        
        # 5. 執行 DSL 腳本
        executor = YAMLDSLExecutor(self.get_mock_wcs_functions())
        result = executor.execute_script(script)
        assert result.success
        
        # 6. 驗證任務創建結果
        assert "task_creation_result" in result.variables
        
    def test_error_handling_workflow(self):
        """測試錯誤處理工作流程"""
        
        # 創建包含錯誤的 DSL 腳本
        invalid_yaml = """
        name: "錯誤測試"
        script:
          - step: "invalid_step"
            type: "unknown_type"
            function: "non_existent_function"
        """
        
        # 測試解析錯誤處理
        parser = YAMLDSLParser()
        try:
            script = parser.parse_dsl_script(yaml.safe_load(invalid_yaml))
            assert False, "應該拋出解析錯誤"
        except ValueError as e:
            assert "unknown_type" in str(e) or "non_existent_function" in str(e)
        
    def test_performance_workflow(self):
        """測試效能工作流程"""
        
        # 創建大型的 DSL 腳本
        large_script = self.create_large_dsl_script(100)  # 100個步驟
        
        # 測試解析效能
        start_time = time.time()
        parser = YAMLDSLParser()
        script = parser.parse_dsl_script(large_script)
        parse_time = time.time() - start_time
        
        assert parse_time < 1.0, f"解析時間過長: {parse_time}秒"
        
        # 測試執行效能
        start_time = time.time()
        executor = YAMLDSLExecutor(self.get_mock_wcs_functions())
        result = executor.execute_script(script)
        execution_time = time.time() - start_time
        
        assert execution_time < 5.0, f"執行時間過長: {execution_time}秒"
```

### 性能測試計劃

#### DSL 解析性能測試
- 測試大型 DSL 檔案的解析時間
- 測試複雜變數表達式的求值性能
- 測試記憶體使用情況

#### DSL 執行性能測試
- 測試長腳本的執行時間
- 測試並發執行多個 DSL 腳本
- 測試函數調用的開銷

#### Flow Designer 性能測試
- 測試大型流程圖的渲染性能
- 測試節點拖拽操作的響應時間
- 測試 YAML 代碼生成的速度

## ⚠️ 風險評估和緩解策略

### 技術風險

#### 風險1：DSL 語法複雜度過高
- **風險描述**: DSL 語法設計過於複雜，學習成本過高
- **影響程度**: 高
- **緩解策略**:
  - 採用漸進式設計，先實現基本功能
  - 提供豐富的範例和模板
  - 設計語法提示和自動完成功能
  - 進行用戶可用性測試

#### 風險2：執行性能不符合要求
- **風險描述**: DSL 執行速度慢於現有系統
- **影響程度**: 中
- **緩解策略**:
  - 早期進行性能基準測試
  - 實施執行結果快取機制
  - 最佳化變數解析和表達式求值
  - 提供執行模式選擇（快速模式 vs 完整模式）

#### 風險3：與現有系統相容性問題
- **風險描述**: 新 DSL 系統與現有 Simple WCS 不相容
- **影響程度**: 高
- **緩解策略**:
  - 維持完整的向後相容性
  - 實施漸進式遷移策略
  - 提供格式轉換工具
  - 建立完整的測試覆蓋

#### 風險4：Flow Designer 使用者體驗不佳
- **風險描述**: 視覺化編輯器不夠直觀好用
- **影響程度**: 中
- **緩解策略**:
  - 進行早期原型測試
  - 收集用戶反饋並快速迭代
  - 參考業界最佳實踐
  - 提供詳細的操作指南

### 專案風險

#### 風險1：開發時程延遲
- **風險描述**: 技術複雜度超出預期，開發時程延遲
- **影響程度**: 中
- **緩解策略**:
  - 採用敏捷開發方法，分階段交付
  - 早期識別技術難點並提前解決
  - 預留緩衝時間
  - 必要時調整功能範圍

#### 風險2：團隊技能不足
- **風險描述**: 開發團隊缺乏相關技術經驗
- **影響程度**: 中
- **緩解策略**:
  - 提供必要的技術培訓
  - 引入外部專家支援
  - 進行知識分享和配對程式設計
  - 建立技術文檔和最佳實踐

#### 風險3：需求變更頻繁
- **風險描述**: 業務需求經常變更，影響開發進度
- **影響程度**: 中
- **緩解策略**:
  - 充分的前期需求分析
  - 設計靈活的架構支援變更
  - 建立需求變更控制流程
  - 與業務方保持密切溝通

### 業務風險

#### 風險1：用戶接受度低
- **風險描述**: 用戶不願意學習和使用新系統
- **影響程度**: 高
- **緩解策略**:
  - 充分的用戶培訓和支援
  - 展示明確的業務價值
  - 提供平滑的遷移路徑
  - 收集用戶反饋並持續改進

#### 風險2：維護成本增加
- **風險描述**: 新系統增加了維護複雜度和成本
- **影響程度**: 中
- **緩解策略**:
  - 設計良好的監控和日誌系統
  - 提供完整的文檔和培訓
  - 建立自動化測試和部署流程
  - 設計自我診斷和修復機制

## 📈 成功指標

### 技術指標
- **DSL 解析性能**: 1000行 DSL 腳本解析時間 < 100ms
- **DSL 執行性能**: 100步驟腳本執行時間 < 1s
- **系統穩定性**: 99.9% 可用時間
- **向後相容性**: 100% 支援現有 YAML 格式

### 使用者指標
- **學習時間**: 新用戶 2小時內掌握基本操作
- **操作效率**: 建立業務流程時間減少 50%
- **錯誤率**: 用戶操作錯誤率 < 5%
- **滿意度**: 用戶滿意度評分 > 4.0/5.0

### 業務指標
- **開發效率**: 新業務流程開發時間減少 60%
- **維護成本**: 業務邏輯維護時間減少 40%
- **系統靈活性**: 支援 95% 的業務場景變更
- **部署速度**: 新流程上線時間 < 30分鐘

## 📚 參考資料和標準

### 技術標準
- [YAML 1.2 規範](https://yaml.org/spec/1.2/spec.html)
- [JSON Schema 規範](https://json-schema.org/)
- [OpenAPI 3.0 規範](https://swagger.io/specification/)

### 設計參考
- [Node-RED 視覺化程式設計](https://nodered.org/)
- [Microsoft Power Automate](https://powerautomate.microsoft.com/)
- [AWS Step Functions](https://aws.amazon.com/step-functions/)
- [Zapier 工作流程自動化](https://zapier.com/)

### 程式語言設計
- [領域特定語言設計模式](https://martinfowler.com/books/dsl.html)
- [ANTLR 語言工具](https://www.antlr.org/)
- [PEG.js 解析器生成器](https://pegjs.org/)

## 🎯 結論

YAML DSL 程式語言設計將為 RosAGV 系統帶來革命性的改進：

1. **業務邏輯程式化** - 將靜態配置升級為可執行的業務邏輯腳本
2. **視覺化程式設計** - 讓非技術人員也能參與業務流程設計
3. **系統靈活性提升** - 快速響應業務需求變更，無需修改程式碼
4. **維護成本降低** - 業務邏輯外部化，降低系統維護複雜度

這個設計將 RosAGV 從傳統的配置驅動系統演進為現代的程式化配置系統，為未來的擴展和發展打下堅實基礎。

---

*本文檔為 YAML DSL 程式語言設計的完整計劃，將作為後續開發工作的指導文檔。*