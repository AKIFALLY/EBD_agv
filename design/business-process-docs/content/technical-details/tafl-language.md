# TAFL 語言規範

## 🎯 TAFL 語言介紹

TAFL (Task Automation Flow Language) v1.1.2 是專為工業自動化設計的流程定義語言，提供清晰、結構化的方式來描述自動化流程。

## 📋 語言結構

### 基本結構
每個 TAFL 檔案包含以下主要部分：

```yaml
# 1. 元資料區塊
metadata:
  id: flow_unique_id        # 唯一識別碼 (必要)
  name: 流程名稱            # 人類可讀名稱 (必要)
  version: "1.1.2"          # 版本號
  author: 作者名稱          # 可選
  enabled: true             # 是否啟用 (預設 true)
  description: 流程說明      # 可選的詳細說明

# 2. 設定區塊 (可選)
settings:
  timeout: 300              # 超時時間（秒）
  retry_on_failure: false   # 失敗時是否重試
  max_retries: 3           # 最大重試次數
  log_level: INFO          # 日誌級別

# 3. 變數區塊 (可選)
variables:
  task_id: 1
  priority: 5
  status: "pending"

# 4. 預載入區塊 (可選)
preload:
  available_agvs:
    query:
      target: agvs
      where:
        battery: "> 30"

# 5. 規則區塊 (可選)
rules:
  priority_multiplier:
    value: 1.5
    description: "優先級倍數"

# 6. 流程定義區塊
flow:
  - [語句...]
```

## 🔧 核心動詞系統 (10個支援動詞)

TAFL v1.1.2 支援以下 10 個核心動詞：

### 1. query - 查詢資料
查詢資料庫或系統中的資料。

```yaml
# 簡單查詢
- query:
    target: locations
    as: all_locations

# 條件查詢
- query:
    target: tasks
    where:
      status_id: 1
      priority: "> 5"
    limit: 10
    as: pending_tasks
```

**參數說明**：
- `target`: 查詢目標 (必要)
- `where`: 查詢條件 (可選)
- `limit`: 限制返回數量 (可選)
- `as`: 儲存結果的變數名稱 (可選)

### 2. check - 條件檢查
檢查條件並將結果儲存到變數。

```yaml
# TAFL v1.1.3: 'as' 參數是必要的
- check:
    condition: "${battery_level} > 30"
    as: battery_sufficient

# 使用檢查結果
- if:
    condition: "${battery_sufficient}"
    then:
      - notify:
          level: info
          message: "電量充足"
```

**參數說明**：
- `condition`: 要檢查的條件表達式 (必要)
- `as`: 儲存結果的變數名稱 (必要，v1.1.3)

### 3. create - 創建資源
創建新的資料實體。

```yaml
- create:
    target: task
    with:
      work_id: "${work_id}"
      priority: 5
      name: "新任務"
      status_id: 1
    as: new_task
```

**參數說明**：
- `target`: 創建目標類型 (必要)
- `with`: 創建參數 (可選)
- `as`: 儲存創建結果的變數名稱 (可選)

### 4. update - 更新資源
更新現有資料實體。

```yaml
- update:
    target: task
    where:
      id: "${task_id}"
    set:
      status_id: 2
      priority: 10
      updated_at: "2025-01-18T10:00:00"
```

**參數說明**：
- `target`: 更新目標類型 (必要)
- `where`: 更新條件 (可選)
- `set`: 要更新的欄位 (可選)

### 5. set - 設定變數
設定一個或多個變數值。

```yaml
# 單一變數設定
- set:
    counter: 0

# 多個變數設定
- set:
    task_count: 5
    current_index: 0
    is_running: true

# 使用表達式
- set:
    next_index: "${current_index} + 1"
    task_name: "Task ${task_id}"
```

**注意**：TAFL v1.1.2 只接受字典格式，不支援 "variable = value" 字串格式。

### 6. if - 條件分支
根據條件執行不同的分支。

```yaml
- if:
    condition: "${priority} > 5"
    then:
      - notify:
          level: warning
          message: "高優先級任務"
      - set:
          urgent: true
    else:
      - notify:
          level: info
          message: "一般任務"
      - set:
          urgent: false
```

**參數說明**：
- `condition`: 判斷條件 (必要)
- `then`: 條件為真時執行的語句 (必要)
- `else`: 條件為假時執行的語句 (可選)

### 7. for - 迴圈迭代
對集合進行迭代處理。

```yaml
- for:
    as: task
    in: "${pending_tasks}"
    filter: "${task.priority} > 3"
    do:
      - update:
          target: task
          where:
            id: "${task.id}"
          set:
            assigned: true
      - notify:
          level: info
          message: "處理任務 ${task.name}"
```

**參數說明**：
- `as`: 迭代變數名稱 (必要)
- `in`: 要迭代的集合 (必要)
- `filter`: 過濾條件 (可選)
- `do`: 每次迭代執行的語句 (必要)

### 8. switch - 多分支選擇
根據表達式值選擇不同的分支執行。

```yaml
- switch:
    expression: "${task.priority}"
    cases:
      - when: 1
        do:
          - set:
              level: "low"
      - when: 5
        do:
          - set:
              level: "medium"
      - when: 10
        do:
          - set:
              level: "high"
      - when: "default"
        do:
          - set:
              level: "unknown"
```

**參數說明**：
- `expression`: 要評估的表達式 (必要)
- `cases`: 分支列表 (必要)
  - `when`: 匹配值或 "default"
  - `do`: 該分支要執行的語句

### 9. stop - 停止執行
停止流程執行。

```yaml
# 無條件停止
- stop:
    reason: "任務完成"

# 條件停止
- stop:
    reason: "電量不足"
    if: "${battery_level} < 10"
```

**參數說明**：
- `reason`: 停止原因 (可選)
- `if` 或 `when`: 停止條件 (可選)

### 10. notify - 發送通知
發送通知或記錄訊息。

```yaml
# 簡單通知
- notify:
    level: info
    message: "任務開始執行"

# 詳細通知
- notify:
    level: warning
    message: "電量低於 ${battery_level}%"
    recipients: ["operator", "supervisor"]
    details:
      agv_id: "${agv.id}"
      location: "${current_location}"
```

**參數說明**：
- `level`: 通知級別 (info/warning/error/critical/alarm)
- `message`: 通知訊息
- `recipients`: 接收者列表 (可選)
- `details`: 額外詳細資訊 (可選)

## 📊 變數和表達式

### 變數引用
使用 `${}` 來引用變數：

```yaml
# 簡單變數
- set:
    message: "任務 ${task_id} 已完成"

# 物件屬性
- set:
    agv_name: "${agv.name}"
    task_priority: "${task.priority}"

# 陣列索引
- set:
    first_item: "${items[0]}"
    last_item: "${items[-1]}"
```

### 支援的運算符

#### 算術運算符
- `+` 加法
- `-` 減法
- `*` 乘法
- `/` 除法

#### 比較運算符
- `==` 等於
- `!=` 不等於
- `>` 大於
- `<` 小於
- `>=` 大於等於
- `<=` 小於等於

#### 邏輯運算符
- `and` 且
- `or` 或
- `not` 非

### 表達式範例
```yaml
# 算術表達式
- set:
    total: "${count} * ${price}"
    next_id: "${current_id} + 1"

# 比較表達式
- check:
    condition: "${battery} > 30 and ${status} == 'idle'"
    as: can_dispatch

# 複合表達式
- if:
    condition: "${priority} > 5 or ${urgent} == true"
    then:
      - notify:
          level: warning
          message: "需要立即處理"
```

## 🎯 完整範例

### 任務分配流程
```yaml
metadata:
  id: task_assignment
  name: 任務分配流程
  version: "1.1.2"
  description: 根據 AGV 狀態分配任務

variables:
  min_battery: 30
  task_limit: 10

preload:
  available_agvs:
    query:
      target: agvs
      where:
        status: "idle"

flow:
  # 1. 查詢待處理任務
  - query:
      target: tasks
      where:
        status_id: 1
      limit: "${task_limit}"
      as: pending_tasks

  # 2. 檢查是否有任務
  - check:
      condition: "${pending_tasks.length} > 0"
      as: has_tasks

  - if:
      condition: "${has_tasks}"
      then:
        # 3. 迭代處理每個任務
        - for:
            as: task
            in: "${pending_tasks}"
            do:
              # 尋找合適的 AGV
              - for:
                  as: agv
                  in: "${available_agvs}"
                  filter: "${agv.battery} > ${min_battery}"
                  do:
                    # 分配任務
                    - update:
                        target: task
                        where:
                          id: "${task.id}"
                        set:
                          assigned_agv: "${agv.id}"
                          status_id: 2

                    # 更新 AGV 狀態
                    - update:
                        target: agv
                        where:
                          id: "${agv.id}"
                        set:
                          status: "busy"
                          current_task: "${task.id}"

                    # 發送通知
                    - notify:
                        level: info
                        message: "任務 ${task.id} 分配給 AGV ${agv.id}"

                    # 停止內層迴圈（找到合適 AGV 後）
                    - stop:
                        reason: "任務已分配"
      else:
        - notify:
            level: info
            message: "沒有待處理的任務"

  # 4. 完成通知
  - notify:
      level: info
      message: "任務分配流程完成"
```

## 💡 最佳實踐

### 1. 結構化設計
- 使用清晰的 metadata 描述流程
- 合理使用 variables 定義常數
- 善用 preload 預載入常用資料

### 2. 錯誤處理
- 在關鍵操作前使用 check 檢查先決條件
- 使用 if/else 處理不同情況
- 適當使用 stop 終止異常流程

### 3. 程式碼品質
- 使用有意義的變數名稱
- 添加適當的 notify 記錄重要事件
- 保持流程簡潔清晰

### 4. 效能考量
- 使用 query 的 limit 參數限制資料量
- 在 for 迴圈中使用 filter 減少迭代次數
- 避免不必要的巢狀迴圈

## ⚠️ 注意事項

### 不支援的功能
TAFL v1.1.2 **不支援**以下功能：
- ❌ `delete` 動詞（刪除資源）
- ❌ `while` 迴圈（條件迴圈）
- ❌ `call` 函數呼叫
- ❌ `execute` 執行外部命令
- ❌ `wait` 等待
- ❌ `emit` 發送事件
- ❌ `break`/`continue` 迴圈控制（使用 stop 替代）
- ❌ `return` 返回值
- ❌ 平行執行（parallel execution）

### 版本差異
- **v1.1.2**: `set` 只接受字典格式
- **v1.1.3**: `check` 必須包含 `as` 參數

## 🔗 相關資源
- [TAFL 系統概覽](./tafl-system.md)
- [TAFL Editor 使用指南](./tafl-editor.md)
- [TAFL WCS 整合](./tafl-wcs-integration.md)