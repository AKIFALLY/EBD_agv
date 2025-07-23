# 任務條件查詢服務使用說明

## 📋 概述

任務條件查詢服務是一個專門用於執行 `task_condition` 資料表中條件查詢的服務。它會讀取資料表中的 `conditions` 欄位作為 SQL 查詢語句執行，並將結果儲存到 `results` 欄位中。

## 🏗️ 架構組成

### 核心模組

1. **TaskConditionQueryService** (`task_condition_query_service.py`)
   - 核心查詢服務邏輯
   - SQL 安全性驗證
   - 查詢執行和結果處理

2. **TaskConditionQueryNode** (`task_condition_query_node.py`)
   - ROS 2 節點實作
   - 定時執行和手動觸發
   - 結果發布和狀態監控

3. **TaskConditionQueryCLI** (`task_condition_query_cli.py`)
   - 命令列工具
   - 手動管理和測試功能

## 🔒 安全機制

### SQL 安全性驗證

- **允許的關鍵字**: SELECT, FROM, WHERE, AND, OR, JOIN 等查詢相關關鍵字
- **禁止的關鍵字**: INSERT, UPDATE, DELETE, DROP, CREATE 等修改操作
- **查詢限制**: 僅允許 SELECT 查詢，禁止多語句執行
- **超時機制**: 預設 30 秒查詢超時

### 錯誤處理

- SQL 語法錯誤捕獲
- 資料庫連接錯誤處理
- 查詢超時處理
- 結果序列化錯誤處理

## 🚀 使用方式

### 1. ROS 2 節點模式

#### 啟動節點

```bash
# 使用預設參數啟動
ros2 run wcs_base task_condition_query_node

# 使用 launch 檔案啟動
ros2 launch wcs_base task_condition_query.launch.py

# 自訂參數啟動
ros2 launch wcs_base task_condition_query.launch.py \
    db_url_agvc:="postgresql+psycopg2://user:pass@host/db" \
    auto_execution_interval:=600.0 \
    enable_auto_execution:=true
```

#### 手動觸發執行

```bash
# 手動觸發執行所有條件
ros2 service call /task_condition_query/manual_execute std_srvs/srv/Trigger
```

#### 監控結果和狀態

```bash
# 監控執行結果
ros2 topic echo /task_condition_query/results

# 監控節點狀態
ros2 topic echo /task_condition_query/status
```

### 2. 命令列工具模式

#### 列出所有條件

```bash
task_condition_query_cli list
```

#### 新增條件

```bash
# 新增簡單條件
task_condition_query_cli add "SELECT COUNT(*) FROM agv WHERE status = 'IDLE'"

# 新增帶描述的條件
task_condition_query_cli add "SELECT * FROM task WHERE priority > 5" --description "高優先級任務查詢"
```

#### 執行指定條件

```bash
# 執行 ID 為 1 的條件
task_condition_query_cli execute 1
```

#### 執行所有條件

```bash
task_condition_query_cli execute-all
```

#### 測試 SQL 查詢

```bash
task_condition_query_cli test "SELECT COUNT(*) FROM location WHERE status = 'OCCUPIED'"
```

## 📊 結果格式

### 成功查詢結果

```json
{
  "success": true,
  "data": [
    {
      "column1": "value1",
      "column2": "value2"
    }
  ],
  "row_count": 1,
  "columns": ["column1", "column2"],
  "timestamp": "2024-01-01T12:00:00Z"
}
```

### 失敗查詢結果

```json
{
  "success": false,
  "error": "錯誤描述",
  "timestamp": "2024-01-01T12:00:00Z"
}
```

## ⚙️ 配置參數

### ROS 2 節點參數

| 參數名稱 | 預設值 | 說明 |
|---------|--------|------|
| `db_url_agvc` | `postgresql+psycopg2://agvc:password@192.168.100.254/agvc` | 資料庫連接字串 |
| `auto_execution_interval` | `300.0` | 自動執行間隔（秒） |
| `enable_auto_execution` | `true` | 是否啟用自動執行 |

### 服務配置

| 設定項目 | 預設值 | 說明 |
|---------|--------|------|
| 查詢超時 | 30 秒 | SQL 查詢最大執行時間 |
| 狀態發布間隔 | 30 秒 | 節點狀態發布頻率 |

## 📝 使用範例

### 範例 1: 監控 AGV 狀態

```sql
-- 條件內容
SELECT 
    id, 
    status, 
    location, 
    battery_level 
FROM agv 
WHERE status IN ('IDLE', 'BUSY', 'CHARGING')
```

### 範例 2: 檢查高優先級任務

```sql
-- 條件內容
SELECT 
    task_id, 
    priority, 
    status, 
    created_at 
FROM task 
WHERE priority > 7 AND status = 'PENDING'
ORDER BY priority DESC, created_at ASC
```

### 範例 3: 統計載具使用情況

```sql
-- 條件內容
SELECT 
    status, 
    COUNT(*) as count,
    AVG(load_weight) as avg_weight
FROM carrier 
GROUP BY status
```

## 🔧 故障排除

### 常見問題

1. **SQL 驗證失敗**
   - 檢查是否使用了禁止的關鍵字
   - 確認查詢以 SELECT 開頭
   - 避免使用分號分隔多個語句

2. **查詢超時**
   - 優化 SQL 查詢效能
   - 檢查資料表索引
   - 考慮增加超時時間

3. **資料庫連接失敗**
   - 檢查資料庫連接字串
   - 確認資料庫服務正常運行
   - 檢查網路連接

### 日誌分析

```bash
# 檢視節點日誌
ros2 node info task_condition_query_node

# 檢視詳細執行日誌
ros2 launch wcs_base task_condition_query.launch.py log_level:=debug
```

## 🔄 維護和監控

### 定期維護

1. **清理舊結果**: 定期清理過期的查詢結果
2. **效能監控**: 監控查詢執行時間和資源使用
3. **錯誤分析**: 分析失敗的查詢並優化

### 監控指標

- 查詢成功率
- 平均執行時間
- 錯誤頻率
- 資源使用情況

## 🚨 注意事項

1. **資料安全**: 僅允許 SELECT 查詢，但仍需注意敏感資料的存取控制
2. **效能影響**: 複雜查詢可能影響資料庫效能，建議在低峰時段執行
3. **結果大小**: 大量查詢結果可能影響記憶體使用，考慮使用 LIMIT 限制結果數量
4. **並發控制**: 避免同時執行大量查詢，可能導致資料庫連接池耗盡

## 📈 擴展功能

### 未來可能的擴展

1. **查詢排程**: 支援 cron 表達式的定時執行
2. **結果快取**: 實作查詢結果快取機制
3. **查詢模板**: 支援參數化查詢模板
4. **結果通知**: 查詢完成後的通知機制
5. **效能分析**: 查詢效能分析和優化建議
