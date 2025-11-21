# ⚠️ 已棄用並歸檔 (DEPRECATED & ARCHIVED)

**棄用日期**: 2025-11-18
**歸檔原因**: TAFL WCS 系統已被 KUKA WCS 完全取代
**替代方案**: 使用 `kuka_wcs_ws` 進行倉儲控制系統開發
**遷移指南**: 參見 /home/ct/RosAGV/docs-ai/guides/migration-from-tafl-to-kuka-wcs.md

本文檔已移至 archived 目錄，僅供歷史參考。不應再用於新的開發工作。

---

# TAFL配置規範 v1.1.2 (Task Automation Flow Configuration)

## 🎯 適用場景
- RosAGV 系統的任務流程配置
- WCS/AGV 基礎流程自動化
- 簡化重複性任務定義
- 需要技術背景的YAML配置語法
- 基礎的同步執行和狀態檢查

## 📋 TAFL v1.1.2 概述

### 配置定位
**TAFL (Task Automation Flow Configuration) v1.1.2** 是基於YAML格式的任務配置規範，採用順序執行模式和基礎變數替換，實現簡化的任務定義和狀態檢查。

### v1.1.2 設計原則
1. **基礎語法集** - 10個核心動詞，涵蓋基本資料庫操作
2. **YAML一致性** - 遵循標準YAML語法規範
3. **簡單組合** - 基礎if-then-else和迴圈結構
4. **有限擴充** - 固定動詞集合，擴展性受限
5. **穩定執行** - 同步執行避免記憶體問題
6. **簡單變數** - 基礎的變數替換機制

### v1.1.2 實際能力
- **基礎性**: 10個核心動詞支援簡單資料庫操作
- **獨立系統**: 專注於RosAGV任務配置需求
- **YAML語法**: 標準YAML格式配置
- **基礎邏輯**: 簡單的if-then-else和迴圈結構
- **同步執行**: 順序執行模式，避免記憶體問題
- **基礎變數**: 簡單的變數替換和作用域

## 🔧 語言規範

### 1. 核心動詞（10個）

| 動詞 | 用途 | 範例 | 實作狀態 |
|------|------|------|------|
| **query** | 資料庫查詢 | `query: locations` | ✅ 基礎實作 |
| **check** | 條件檢查（必須as參數） | `check: {condition: expr, as: var}` | ✅ 基礎實作 |
| **create** | 資料庫創建 | `create: task` | ✅ 基礎實作 |
| **update** | 資料庫更新 | `update: rack` | ✅ 基礎實作 |
| **if** | 簡單條件判斷 | `if: ${condition}` | ✅ 基礎實作 |
| **for** | 基礎迴圈 | `for: ${collection}` | ✅ 基礎實作 |
| **switch** | 多分支選擇 | `switch: ${expression}` | ✅ 基礎實作 |
| **set** | 變數設定 | `set: {count: 0}` | ✅ 基礎實作 |
| **stop** | 流程停止 | `stop: "reason"` | ✅ 基礎實作 |
| **notify** | 日誌輸出 | `notify: message` | ✅ 基礎實作 |

**限制**: 所有動詞只支援基礎功能，複雜的進階特性尚未實作。`log` 動詞未實作，請使用 `notify` 替代。

### 2. TAFL v1.1.2 程式結構

TAFL v1.1.2 採用 6 段式結構，順序執行模式：

```yaml
metadata:         # 可選：程式元資料
  id: flow_001
  name: Sample Flow
  version: 1.1.2
  enabled: true   # v1.1.2 新增：控制流程是否啟用自動執行
  description: TAFL v1.1.2 範例流程

settings:         # 可選：執行設定
  timeout: 3600
  max_retries: 3

preload:          # 新增：資料預載段（Phase 1）
  active_rooms:
    query:
      target: rooms
      where:
        status: active

rules:            # 新增：規則定義段（Phase 2）
  min_tasks: 1
  max_tasks: 10
  default_timeout: 300

variables:        # Phase 3：變數初始化
  room_id: 1
  task_count: 0

flow:            # Phase 4：主要流程執行
  - query:
      target: locations
      as: locations
  - set:
      task_count: "${task_count + 1}"
```

#### 執行階段說明
1. **Preload**: 先行資料查詢和存储
2. **Rules**: 全域規則參數定義
3. **Variables**: 變數初始化
4. **Flow**: 主流程順序執行

#### 變數作用域
1. **Rules**: 全域規則參數（唯讀）
2. **Preload**: 預載資料結果
3. **Global**: 全域變數
4. **Flow**: 流程變數
5. **Loop**: 迴圈變數

### 3. 統一語法結構

所有語句遵循統一模式，支援三種語法格式：

#### 簡化格式（適用於部分動詞）
```yaml
<verb>: <expression>
# 範例（注意：SET 不支援簡化格式）
for: item in ${collection}
stop: "reason"
```

#### 結構化格式（推薦用於複雜操作）
```yaml
<verb>:
  <modifier>: <value>
  <modifier>: <value>
# 範例
query:
  target: locations
  where:
    room_id: "${room_id}"
  as: locations
```

#### SET 標準格式（v1.1.2 統一規範）
```yaml
# SET 必須使用物件格式（即使只有單一變數）
set:
  var1: value1
  
# 多變數範例
set:
  task_count: 0
  priority: high
  timeout: "${rules.default_timeout}"
```

**實作說明**: SET 動詞統一使用物件格式，不支援單行字串格式。其他動詞仍可使用簡化或結構化格式。

### 4. v1.1 增強表達式系統

#### 數學運算（v1.1 修復）
```yaml
# 現在正確支援數學表達式
set:
  total: "${count + 1}"
set:
  average: "${sum / count}"
if:
  condition: "${score >= threshold * 0.8}"
```

#### 物件屬性存取
```yaml
# 支援深層物件屬性
set:
  location_id: "${item.location.id}"
set:
  priority: "${task.metadata.priority}"
```

#### 規則引用（v1.1 新增）
```yaml
# 引用 rules 段定義的規則
if:
  condition: "${task_count < rules.max_tasks}"
  then:
    - create:
        target: task
        params:
          timeout: "${rules.default_timeout}"
```

#### 預載資料引用（v1.1 新增）
```yaml
# 引用 preload 段的快取資料
query:
  target: locations
  where:
    room_active: "${active_rooms[room_id].status}"
```

### 5. 資料類型

```yaml
# 基本類型
number: 42, 3.14, -10
string: "hello", 'world'
boolean: true, false
null: null

# 複合類型
array: [1, 2, 3]
object: {name: "rack", id: 101}

# 變數引用
variable: ${variable_name}
property: ${object.property}
index: ${array[0]}
expression: ${a + b * 2}
```

### 4. 變數替換系統

#### 基礎變數替換
```yaml
${variable_name}     # 基礎變數
${object.property}   # 屬性存取
${array[0]}          # 陣列索引
```

#### 有限的數學運算
```yaml
${a + b}    # 簡單加法
${a - b}    # 簡單減法
# 注意: 乘除和餘數運算可能不穩定
```

#### 基礎比較運算
```yaml
${a == b}   # 等於
${a != b}   # 不等於
# 注意: 複雜邏輯運算和集合運算可能不支援
```

#### 不支援的功能
```yaml
# 以下功能尚未實作，請勿使用
${a && b}, ${a || b}, ${!a}    # 複雜邏輯
${a > b}, ${a < b}, ${a >= b}  # 大小比較
${array.length}, ${array.first} # 陣列屬性
```

### 5. 實際可用功能

**目前實作的功能**:
```yaml
# 基礎變數操作
${variable_name}     # 變數替換
${object.property}   # 屬性存取
${array[index]}      # 陣列索引

# 基礎數學運算 (限定支援)
${a + b}             # 簡單加法
${a - b}             # 簡單減法

# 邏輯比較
${a == b}            # 等於比較
${a != b}            # 不等於比較
```

**尚未實作的功能** (切勿使用):
```yaml
# 以下函數都不存在，請勿使用
empty(), exists(), valid()
count(), sum(), avg(), max(), min()
concat(), upper(), lower(), trim()
now(), today(), timestamp()
```

## 📝 語法詳解

### query - 查詢操作

```yaml
# 基本查詢
- query: locations
  as: all_locations

# 條件查詢
- query: racks
  where:
    status: available
    location_id: ${location.id}
  as: available_racks

# 進階查詢
- query: tasks
  where:
    priority: > 5
    status: pending
  order: priority desc
  limit: 10
  as: urgent_tasks
```

### check - 檢查操作

**⚠️ 重要**: `as` 參數為必要欄位，用於儲存檢查結果

```yaml
# ✅ 正確格式：必須包含 as 參數
- check:
    condition: ${expression}
    as: result_variable  # 必要參數

# 檢查存在性
- check: task_exists
  where:
    rack_id: ${rack.id}
    status: pending
  as: has_pending_task  # 必要參數

# 檢查狀態
- check: rack_status
  where:
    id: ${rack.id}
  as: rack_state  # 必要參數

# ❌ 錯誤格式：不允許省略 as
# - check: "${counter < 100}"  # 缺少 as 參數
```

### create - 創建操作

```yaml
# 創建任務
- create: task
  with:
    name: "Rack rotation - ${rack.id}"
    work_id: 220001
    location_id: ${location.id}
    rack_id: ${rack.id}
    priority: 5
    metadata:
      model: "KUKA400i"
      rotation_angle: 180
      nodes: [${location.node_id}, ${location.node_id + 1}, ${location.node_id}]
  as: new_task

# 創建通知
- create: notification
  with:
    type: "alert"
    message: "Task created: ${new_task.id}"
    recipients: ["supervisor", "operator"]
```

### update - 更新操作

```yaml
# 更新單一資料
- update: rack
  where:
    id: ${rack.id}
  set:
    status: "processing"
    side_completed: true

# 批量更新
- update: tasks
  where:
    status: "pending"
    priority: < 5
  set:
    priority: ${priority + 1}
    escalated: true
```

### if - 條件判斷

```yaml
# 簡單條件
- if: ${count > 0}
  then:
    - create: task

# 完整條件
- if: ${rack.side_a_completed && !rack.side_b_completed}
  then:
    - create: rotation_task
    - notify: operator
  else:
    - notify:
        level: info
        message: "No rotation needed"
```

### for - 迴圈處理

```yaml
# 基本迴圈
- for: ${locations}
  as: location
  do:
    - query: racks
      where:
        location_id: ${location.id}
      as: racks

# 帶過濾條件的迴圈 (v1.1 新功能)
- for: ${tasks}
  as: task
  filter: ${task.priority > 5}  # 只處理高優先級任務
  do:
    - notify:
        level: info
        message: "Processing high priority task: ${task.id}"
    - update: task
      set:
        status: "processing"
```

### switch - 多分支

**重要更新 (v1.1.1)**: default 現在作為特殊的 case 處理，統一資料結構。

```yaml
- switch:
    expression: ${task.priority}
    cases:
      - when: "> 8"        # 條件必須用引號包裹
        do:
          - notify: alarm
            message: "Critical task!"
          - create: urgent_dispatch
      
      - when: "5..8"       # 範圍條件
        do:
          - create: normal_dispatch
      
      - when: "< 5"        # 比較條件
        do:
          - create: scheduled_dispatch
      
      - when: "default"    # default 作為特殊 case（必須在最後）
        do:
          - notify:
              level: warning
              message: "Unknown priority"
```

**Switch 規則**：
- `expression`: 要評估的變數或表達式
- `cases`: case 陣列，按順序評估
- `when`: 條件字串，必須用引號包裹（避免 YAML 解析問題）
- `when: "default"`: 特殊值，表示預設分支，必須放在最後
- 每個 switch 最多只能有一個 default case

**實作狀態 (2025-09-09)**: 
- ✅ Parser 和 Executor 已更新支援 v1.1.1 格式
- ✅ 保持向後相容舊格式 (on/cases/default)
- ✅ TAFL Editor 已支援新格式生成

### set - 變數設定

```yaml
# 標準格式（單一變數）
- set:
    count: 0
    
# 多變數設定
- set:
    task_count: 0
    priority: high
    timeout: "${rules.default_timeout}"
    
# 表達式賦值
- set:
    total: "${count + 1}"
    average: "${sum / count}"
    
# 複雜賦值
- set:
    summary: 
      total: count(${racks})
      pending: count(${racks.filter(r => r.status == "pending")})
      completed: count(${racks.filter(r => r.status == "completed")})
```

### stop - 停止流程

```yaml
# 條件停止
- if: empty(${locations})
  then:
    - stop: "No locations found"

# 錯誤停止
- check: system_status
  as: status
- if: ${status.error}
  then:
    - stop: "System error: ${status.message}"
```

### notify - 通知操作

```yaml
# 資訊通知
- notify: info
  message: "Process started"

# 警告通知
- notify: warning
  message: "Low inventory: ${count}"

# 警報通知
- notify: alarm
  message: "Critical error!"
  recipients: ["admin", "supervisor"]
```

## 🚀 完整範例

### 範例 1: 架台旋轉流程（完整版）

```yaml
name: "Rack Rotation Flow"
version: "TAFL-v4"
trace: true

# 全域配置
config:
  target_rooms: [1, 2, 3, 4, 5]
  rotation_angle: 180
  max_tasks_per_location: 5
  work_id: 220001

flow:
  # 查詢所有房間入口位置
  - query: locations
    where:
      type: "room_inlet"
      room_id: in ${config.target_rooms}
    as: inlet_locations
  
  # 檢查是否有位置
  - if: empty(${inlet_locations})
    then:
      - stop: "No inlet locations found"
  
  # 記錄找到的位置數量
  - set:
      total_locations: count(${inlet_locations})
  - notify: info
    message: "Found ${total_locations} inlet locations"
  
  # 初始化總任務計數
  - set:
      total_tasks_created: 0
  
  # 處理每個位置
  - for: ${inlet_locations}
    as: location
    do:
      # 查詢該位置的架台
      - query: racks
        where:
          location_id: ${location.id}
          status: available
        as: location_racks
      
      # 如果沒有架台，記錄並處理下一個位置
      - if: empty(${location_racks})
        then:
          - notify: info
            message: "No racks at ${location.name}"
          # TAFL 不支援 continue，使用條件判斷控制流程
      
      # 初始化位置任務計數
      - set:
          location_task_count: 0
      
      # 處理每個架台
      - for: ${location_racks}
        as: rack
        do:
          # 檢查是否需要旋轉（A面完成，B面未處理）
          - if: ${rack.side_a_completed && !rack.side_b_completed}
            then:
              # 檢查任務數量限制
              - if: ${location_task_count < config.max_tasks_per_location}
                then:
                  # 檢查是否已有任務
                  - check: task_exists
                    where:
                      rack_id: ${rack.id}
                      status: in ["pending", "processing"]
                    as: existing_task
                  
                  # 只在沒有現存任務時創建
                  - if: !${existing_task}
                    then:
                      # 創建旋轉任務
                      - create: task
                        with:
                          name: "Rack rotation - ${rack.id}"
                          work_id: ${config.work_id}
                          location_id: ${location.id}
                          rack_id: ${rack.id}
                          room_id: ${location.room_id}
                          priority: 5
                          metadata:
                            model: "KUKA400i"
                            rotation_angle: ${config.rotation_angle}
                            nodes: [
                              ${location.node_id},
                              ${location.node_id + 1},
                              ${location.node_id}
                            ]
                            operation_type: "rack_rotation"
                        as: new_task
                      
                      # 更新計數器
                      - set:
                          location_task_count: "${location_task_count + 1}"
                      - set:
                          total_tasks_created: "${total_tasks_created + 1}"
                      
                      # 記錄任務創建
                      - notify: info
                        message: "Task ${new_task.id} created for Rack ${rack.id} at ${location.name}"
                else:
                  - notify: warning
                    message: "Task limit reached for ${location.name}"
                  - set:
                      skip_remaining_racks: true  # 設置旗標以跳過剩餘架台
  
  # 總結報告
  - switch:
      expression: ${total_tasks_created}
      cases:
        - when: "0"
          do:
            - notify: info
              message: "No rotation tasks needed"
        
        - when: "> 10"
          do:
            - notify: warning
              message: "Created ${total_tasks_created} rotation tasks - high volume!"
        
        - when: "default"
          do:
            - notify: info
              message: "Successfully created ${total_tasks_created} rotation tasks"
```


### 範例 2: 真實的Rack狀態檢查

基於實際業務場景的簡單範例：

```yaml
metadata:
  id: "rack_status_check"
  name: "房間入口架台狀態檢查"
  enabled: true

settings:
  execution_interval: 10

variables:
  work_id: 220001
  priority: 5

flow:
  # 查詢房間入口位置
  - query:
      target: locations
      where:
        type: "room_inlet"
      as: inlet_locations

  # 檢查每個位置
  - for:
      in: "${inlet_locations}"
      as: location
      do:
        # 查詢該位置的架台
        - query:
            target: racks
            where:
              location_id: "${location.id}"
            as: location_racks

        # 如果有架台，檢查狀態
        - if:
            condition: "${location_racks}"
            then:
              - set:
                  rack: "${location_racks[0]}"

              # 簡單的狀態檢查
              - if:
                  condition: "${rack.status_id == 2}"
                  then:
                    # 創建基礎任務
                    - create:
                        target: task
                        with:
                          work_id: "${work_id}"
                          rack_id: "${rack.id}"
                          priority: "${priority}"
                          status_id: 1
                        as: new_task

                    - notify:
                        message: "已創建任務 ${new_task.id}"
            else:
              - notify:
                  message: "位置 ${location.id} 無架台"
```

## 🔄 版本相容性說明

### Switch 語句格式演進

#### v1.0 格式（舊版，仍支援）
```yaml
- switch:
    on: ${expression}          # 使用 'on' 欄位
    cases:
      1: [action1]             # 字典格式的 cases
      2: [action2]
    default: [default_action]  # 獨立的 default 欄位
```

#### v1.1.1 格式（推薦使用）
```yaml
- switch:
    expression: ${expression}  # 使用 'expression' 欄位
    cases:                     # 陣列格式的 cases
      - when: "1"
        do: [action1]
      - when: "2"  
        do: [action2]
      - when: "default"        # default 作為特殊 case
        do: [default_action]
```

**關鍵差異**：
- 欄位名稱：`on` → `expression`
- Cases 結構：字典 → 陣列
- Default 處理：獨立欄位 → 特殊 case
- 條件支援：僅精確匹配 → 支援條件表達式


### 範例 3: 基礎任務狀態更新

另一個真實業務場景範例：

```yaml
metadata:
  id: "task_status_update"
  name: "任務狀態更新檢查"
  enabled: false

variables:
  max_tasks: 5

flow:
  # 查詢進行中的任務
  - query:
      target: tasks
      where:
        status_id: 2
      limit: "${max_tasks}"
      as: active_tasks

  # 處理每個任務
  - for:
      in: "${active_tasks}"
      as: task
      do:
        # 簡單的完成檢查
        - if:
            condition: "${task.updated_at}"
            then:
              - update:
                  target: task
                  where:
                    id: "${task.id}"
                  set:
                    status_id: 3

              - notify:
                  message: "任務 ${task.id} 已更新"
```

**證明該範例的實用性**：
- 只使用已實作的動詞和功能
- 基於真實的資料庫結構
- 可以直接在系統中執行
- 避免使用不存在的函數和特性

## 🚀 實作狀態

### 第一階段：核心解析器（Week 1-2）
1. 創建 TAFL 語法解析器
2. 實作 10 個核心動詞
3. 支援表達式系統
4. 變數作用域管理

### 第二階段：執行引擎（Week 3-4）
1. 實作語句執行邏輯
2. 內建函數庫
3. 錯誤處理機制
4. 效能優化

### 第三階段：工具整合（Week 5-6）
1. TAFL Editor 界面增強
2. 驗證工具完善
3. 錯誤提示優化

### 第四階段：工具支援（Week 7-8）
1. 語法高亮和自動完成
2. 測試框架擴展
3. 文檔生成器
4. 效能監控工具

## 📊 實際現況

### 開發現況
- **學習成本**: 需要掌握YAML語法和10個動詞規則
- **適用範圍**: 主要用於簡單的資料庫操作流程
- **技術依賴**: 需要技術背景進行配置和維護

### 功能限制
- **基礎功能**: 僅支援10個核心動詞的基本操作
- **語法限制**: 複雜邏輯需要多個步驟組合實現
- **擴展性**: 新功能需要修改核心執行引擎

### 系統狀況
- **執行模式**: 同步執行避免記憶體洩漏問題
- **穩定性**: 基本功能運作穩定
- **維護需求**: 需要持續維護和功能擴展

## 💡 最佳實踐

### 命名規範
- 變數使用 snake_case: `inlet_locations`
- 常數使用 UPPER_CASE: `MAX_RETRIES`
- 配置使用點號分隔: `config.max_tasks`

### 程式組織
- 使用 config 區塊管理全域配置
- 邏輯相關的步驟放在一起
- 適當使用註解說明業務邏輯

### 錯誤處理
- 使用 try-catch 處理可能失敗的操作
- 提供有意義的錯誤訊息
- 實作優雅的降級策略

## 🚀 實作狀態與發現

### 已實作功能
- ✅ 基礎YAML解析器 - 支援TAFL格式
- ✅ 同步執行器 - 順序執行模式
- ✅ 基本格式驗證 - 語法檢查
- ✅ 10個核心動詞基礎支援
- ✅ 變數插值系統 (`${}`)
- ✅ 迴圈變數作用域隔離
- ✅ 兩種語法格式支援（簡化與結構化）

### 實作中的調整
1. **語法格式彈性**: 支援簡化與結構化兩種格式混用，提高靈活性
2. **迴圈變數作用域**: 明確實作為迴圈內隔離，避免變數污染
3. **註解欄位**: 所有語句都支援 `comment` 欄位，改善可讀性
4. **字串引號處理**: 簡化格式中字串可能保留引號，建議複雜字串使用結構化格式

### v1.1 已完成功能 ✅
- ✅ **4-Phase 執行模型**: preload → rules → variables → flow
- ✅ **5-Level 變數作用域**: rules, preload, global, flow, loop
- ✅ **增強表達式解析器**: 修復數學運算 `${variable + 1}`
- ✅ **多變數 Set 語句**: `set: {var1: value1, var2: value2}`
- ✅ **通用 Notify 函數**: 支援 generic notify 功能
- ✅ **增強 For 迴圈**: 支援 filter 和改良作用域
- ✅ **Preload 資料預載**: 基礎資料預載機制
- ✅ **Rules 規則定義**: 全域規則和約束管理

### 待實作功能
- ⏳ 標準函數庫 (empty, exists, count, sum, avg等)
- ⏳ Try-Catch-Finally 錯誤處理
- ⏳ Break/Continue 流程控制
- ⏳ 外部函數註冊系統完整整合

### 實作狀態
- **當前版本**: TAFL v1.1 基礎功能實作
- **實作程式碼**: `/home/ct/RosAGV/app/tafl_ws/`
- **文檔位置**: `/home/ct/RosAGV/app/tafl_ws/docs/`
- **測試狀態**: 基本功能可運行，部分高級功能待完善
- **驗證工具**: `r tafl-validate` 支援基礎格式驗證

## 🔄 遷移指南

### v1.1.2 SET 格式遷移
從 TAFL v1.1.2 開始，SET 動詞不再支援單行字串格式。所有 SET 語句必須使用物件格式。

#### 遷移範例
```yaml
# ❌ 舊格式（不再支援）
set: "task_status = completed"
set: "counter = ${counter} + 1"
set: "message = Task ${task_id} completed"

# ✅ 新格式（必須使用）
set: {task_status: "completed"}
set: {counter: "${counter + 1}"}
set: {message: "Task ${task_id} completed"}

# ✅ 多行格式（保持不變）
set:
  task_status: "completed"
  counter: "${counter + 1}"
  message: "Task ${task_id} completed"
```

#### 自動遷移腳本
可以使用以下正則表達式進行批量轉換：
- 搜尋: `^(\s*)set:\s*"([^=]+)\s*=\s*([^"]+)"\s*$`
- 替換: `$1set: {$2: "$3"}`

#### 注意事項
1. 物件格式確保語法一致性
2. 支援更好的語法高亮和驗證
3. 與 TAFL Editor 實作保持一致
4. 多變數設定語法保持不變

## 🔗 相關文檔
- TAFL 開發歷史: docs-ai/knowledge/system/tafl/tafl-development-history.md
- TAFL 使用者指南: docs-ai/knowledge/system/tafl/tafl-user-guide.md
- TAFL API 參考: docs-ai/knowledge/system/tafl/tafl-api-reference.md
- TAFL 編輯器規格: docs-ai/knowledge/system/tafl/tafl-editor-specification.md

## 📅 版本記錄
- **2025-09-10**: TAFL v1.1.2 SET 語法標準化
  - **重大變更**: 移除單行字串格式 `set: "variable = value"`
  - 統一使用物件格式：單變數 `set: {variable: value}` 或多行 YAML
  - 確保 TAFL Editor 實作與規範一致性
  - **遷移指南**：
    - 舊格式: `set: "task_status = completed"`
    - 新格式: `set: {task_status: "completed"}`
    - 多變數格式保持不變
- **2025-08-21**: TAFL v1.1 語言規範完成，包含：
  - 4階段執行流程（settings, preload, rules, variables, flow）
  - 5層變數作用域管理（rules, preload, global, flow, loop）
  - 基礎數學運算修復（加減法）
  - Preload 和 Rules 段基礎支援
  - 多變數 Set 語句格式
  - 基礎 Notify 日誌功能
- **2025-08-21**: TAFL v1.0 語言規範正式發佈
- **2025-08-21**: 根據實作經驗更新規格，加入雙語法格式支援說明
- **設計者**: Claude AI Assistant + 人類夥伴
- **狀態**: RosAGV系統的獨立任務配置方案