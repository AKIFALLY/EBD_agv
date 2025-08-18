# Flow WCS 系統架構和技術詳解

## 🎯 適用場景
- 理解 Flow WCS Linear Flow v2 架構設計理念
- 為 WCS 相關開發提供技術參考
- 解決流程設計和執行相關問題

## 📋 Flow WCS 系統概述

### 系統定位
**Flow WCS** 是 RosAGV 唯一的倉庫控制系統實作，採用 Linear Flow v2 架構，取代了原有的節點圖形式和其他實驗性系統。

### 架構演進
- **舊版系統**: simple_wcs_ws (節點圖)、ai_wcs_ws (實驗性)、wcs_ws (已棄用)
- **現行系統**: flow_wcs_ws (Linear Flow v2) - 唯一的生產系統

## 🏗️ Linear Flow v2 架構

### 核心設計理念
```yaml
# Linear Flow v2 格式
meta:
  system: linear_flow_v2
  version: "2.0.0"
  
flow:
  id: "flow_id"
  name: "流程名稱"
  work_id: "220001"
  enabled: true
  
workflow:
  - section: "區段名稱"
    steps:
      - id: "step_id"
        exec: "function.name"
        params:
          key: value
        store: "variable_name"
        skip_if: "${condition}"
```

### 與舊系統的差異
| 特性 | 舊版 (Node-based) | Linear Flow v2 |
|------|------------------|----------------|
| **結構** | 節點和連接的圖形 | 線性的區段和步驟 |
| **可讀性** | 需要理解圖形結構 | 直觀的順序流程 |
| **維護性** | 複雜的連接管理 | 簡單的步驟管理 |
| **執行邏輯** | 基於圖遍歷 | 基於順序執行 |
| **變數管理** | 複雜的資料流 | 簡單的變數儲存 |

## 🔧 系統組件

### 核心模組
```
flow_wcs_ws/src/flow_wcs/
├── flow_executor.py      # 執行引擎 (43個內建函數)
├── flow_monitor.py       # 監控服務
├── flow_validator.py     # 驗證器
├── database.py          # 直接資料庫存取
├── decorators.py        # 裝飾器函數註冊
└── functions/           # 內建函數庫
```

### 內建函數庫 (43個函數)

#### Query Functions (查詢類)
- `query.locations` - 查詢位置資料
- `query.racks` - 查詢架台資料
- `query.tasks` - 查詢任務資料
- `query.agvs` - 查詢 AGV 資料

#### Check Functions (檢查類)
- `check.empty` - 檢查資料是否為空
- `check.location_available` - 檢查位置可用性
- `check.rack_status` - 檢查架台狀態
- `check.system_ready` - 檢查系統就緒
- `check.task_exists` - 檢查任務存在

#### Task Functions (任務類)
- `task.create` - 創建任務
- `task.update` - 更新任務狀態
- `task.assign` - 分配任務給 AGV
- `task.cancel` - 取消任務

#### Action Functions (動作類)
- `action.log` - 記錄日誌
- `action.notify` - 發送通知
- `action.rotate_rack` - 旋轉架台
- `action.optimize_batch` - 優化批次

#### Control Functions (控制類)
- `control.wait` - 等待指定時間
- `control.foreach` - 迴圈處理
- `control.switch` - 分支控制
- `control.stop` - 停止流程

## 🔄 執行機制

### 變數解析系統
```yaml
steps:
  - id: "query_step"
    exec: "query.locations"
    params:
      type: "rack"
    store: "rack_locations"    # 儲存結果
    
  - id: "use_variable"
    exec: "check.empty"
    params:
      data: "${rack_locations}" # 使用變數
```

### Enhanced Expression Resolution
The Flow WCS executor now supports advanced expression resolution:
- **Mathematical Operations**: `${variable + 1}`, `${a * b}`, `${x / 2}`
- **Deep Property Access**: `${object.nested.property}`
- **Array Indexing**: `${array[0]}`, `${items[index]}`

Example:
```yaml
steps:
  - id: "calculate_nodes"
    exec: "task.create"
    params:
      metadata:
        nodes: 
          - "${location.node_id}"        # Start node
          - "${location.node_id + 1}"    # Calculated turn node
          - "${location.node_id}"        # End node
```

### 條件執行
```yaml
steps:
  - id: "conditional_step"
    exec: "action.notify"
    params:
      message: "條件滿足時執行"
    skip_if: "${is_empty}"      # 條件為真時跳過
    
  - id: "inverse_condition"
    exec: "action.log"
    params:
      message: "條件不滿足時執行"
    skip_if_not: "${has_data}"  # 條件為假時跳過
```

### Enhanced Logical Operators
The condition evaluator now supports complex logical expressions:
- **OR Operator**: `${condition1 || condition2}`
- **AND Operator**: `${condition1 && condition2}`
- **NOT Operator**: `!${condition}`
- **Combined**: `${!empty && (status == 'ready' || priority > 5)}`

Example:
```yaml
steps:
  - id: "complex_condition"
    exec: "task.create"
    skip_if: "!${rack.side_a_completed} || ${rack.side_b_completed}"
```

### ForEach 迴圈 with Context Stack
```yaml
steps:
  - id: "foreach_locations"
    exec: "foreach"
    items: "${locations}"
    var: "location"
    steps:
      - id: "process_location"
        exec: "action.log"
        params:
          message: "處理位置: ${location.name}"
```

**Important**: ForEach loops now use a context stack to properly isolate variable scopes. This prevents inner loop variables from overwriting outer loop variables in nested foreach structures.

Example of nested foreach with proper scoping:
```yaml
steps:
  - id: "outer_loop"
    type: "foreach"
    collection: "${locations}"
    var: "location"
    body:
      - id: "inner_loop"
        type: "foreach"
        collection: "${location.racks}"
        var: "rack"
        body:
          - id: "use_both"
            exec: "task.create"
            params:
              location_id: "${location.id}"  # Parent scope preserved
              rack_id: "${rack.id}"          # Current loop variable
```

### 平行執行
```yaml
steps:
  - id: "parallel_tasks"
    exec: "parallel"
    branches:
      - name: "branch1"
        steps:
          - id: "task1"
            exec: "task.create"
      - name: "branch2"
        steps:
          - id: "task2"
            exec: "task.create"
```

## 🗄️ 資料庫整合

### 直接資料庫存取
Flow WCS 使用直接的 PostgreSQL 連接，不依賴 db_proxy 服務：

```python
DATABASE_URL = 'postgresql://agvc:password@192.168.100.254:5432/agvc'
```

### 資料模型
- **Location**: 位置資訊
- **Rack**: 架台狀態
- **Task**: 任務管理
- **AGV**: AGV 狀態
- **FlowLog**: 執行日誌
- **Work**: 工作定義 (包含 parameters 欄位)

### Parameter Merging in Task Creation
When creating tasks, the system now merges parameters from multiple sources:

```python
def create_task(self, type, work_id, metadata=None, **kwargs):
    # Get work parameters as base
    work = session.query(Work).filter_by(id=work_id).first()
    task_parameters = {}
    
    # 1. Start with work parameters (base configuration)
    if work and work.parameters:
        task_parameters.update(work.parameters)
    
    # 2. Override/extend with task metadata
    if metadata:
        task_parameters.update(metadata)
    
    # Create task with merged parameters
    task = Task(
        type=type,
        work_id=work_id,
        parameters=task_parameters,  # Merged parameters
        **kwargs
    )
```

This ensures that:
- Work-level default parameters are preserved
- Task-specific metadata can override defaults
- All required parameters are included in the final task

## 🎨 裝飾器系統

### 自動函數註冊
```python
@flow_function(
    category="query",
    description="查詢位置",
    params=["type", "rooms", "has_rack"],
    returns="array",
    defaults={"type": "rack"}
)
def locations(self, params):
    # 函數實作
    pass
```

### 函數發現機制
- 使用裝飾器自動註冊函數
- 支援參數預設值
- 自動生成函數文檔
- API 端點自動暴露

## 📁 流程檔案管理

### 檔案位置
```
/app/config/wcs/flows/
├── rack_rotation_room_inlet.yaml   # work_id: 220001
├── rack_rotation_room_outlet.yaml  # work_id: 220002
└── [其他業務流程檔案]
```

### Work ID 系統
- **220001**: 房間入口架台輪轉
- **220002**: 房間出口架台輪轉
- **230001**: AGV 任務分配
- **100001**: 系統維護任務
- **100002**: 緊急處理任務

## 🔌 API 整合

### Flow API 端點
```
GET  /api/flow/functions     # 獲取函數庫
POST /api/flow/validate      # 驗證流程
POST /api/flow/execute       # 執行流程
GET  /api/flow/status/{id}   # 查詢狀態
```

### ROS 2 服務
```python
# 執行流程
/flow_wcs/execute_flow

# 停止流程
/flow_wcs/stop_flow

# 查詢狀態
/flow_wcs/get_status
```

## 🔍 與 Linear Flow Designer 整合

### 視覺化編輯器
- 位置: `http://localhost:8001/linear-flow/designer`
- 功能: 視覺化編輯、YAML 編輯、流程驗證、匯入匯出

### 設計器特性
- 拖放式流程設計
- 即時流程驗證
- 函數庫瀏覽
- 變數自動完成
- 流程模擬執行

### Recent Enhancements
- **Expression Resolution**: Support for mathematical operations in variable expressions
- **Logical Operators**: Full support for ||, &&, ! in conditions
- **Context Stack**: Proper variable scoping in nested foreach loops
- **Parameter Merging**: Automatic merging of Work and Task parameters
- **KUKA Integration**: Special support for KUKA AGV rack rotation tasks

## 📊 效能特性

### 執行效率
- 線性執行: O(n) 複雜度
- 變數解析: 快取機制
- 資料庫: 連接池管理
- 並行處理: 異步執行

### 可擴展性
- 函數熱載入
- 流程動態載入
- 水平擴展支援
- 分散式執行準備

## 🚀 部署和運行

### 部署步驟
```bash
# 進入容器
docker compose -f docker-compose.agvc.yml exec agvc_server bash

# 部署系統
cd /app/flow_wcs_ws
./deploy.sh full

# 檢查狀態
./deploy.sh status
```

### 監控和維護
```bash
# 查看執行日誌
ros2 topic echo /flow_wcs/events

# 監控系統狀態
ros2 topic echo /flow_wcs/status

# 查看函數呼叫
ros2 topic echo /flow_wcs/function_calls
```

## 💡 最佳實踐

### 流程設計原則
1. **單一職責**: 每個流程專注一個業務目標
2. **變數命名**: 使用描述性的變數名稱
3. **錯誤處理**: 加入適當的檢查步驟
4. **日誌記錄**: 關鍵步驟加入日誌

### 函數使用建議
1. **查詢優先**: 先查詢再操作
2. **檢查驗證**: 操作前進行條件檢查
3. **批次處理**: 使用 foreach 處理集合
4. **並行優化**: 獨立任務使用 parallel

## 🔗 交叉引用
- WCS 架構設計: @docs-ai/knowledge/agv-domain/wcs-system-design.md
- 開發指導: @docs-ai/operations/development/flow-wcs-development.md
- 模組索引: @docs-ai/context/structure/module-index.md
- Work ID 系統: @docs-ai/knowledge/agv-domain/wcs-workid-system.md
- Linear Flow Advanced Features: @docs-ai/knowledge/system/linear-flow-advanced-features.md
- KUKA AGV Rack Rotation: @docs-ai/knowledge/protocols/kuka-agv-rack-rotation.md
- Troubleshooting Cases: @docs-ai/operations/development/linear-flow-troubleshooting-cases.md