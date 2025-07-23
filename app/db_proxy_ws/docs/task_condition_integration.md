# TaskCondition 資料表整合說明

## 📋 概述

`task_condition` 資料表是一個簡化的任務條件儲存表，用於快速記錄和查詢各種任務相關的條件和結果。

## 🗄️ 資料表結構

```sql
CREATE TABLE task_condition ( 
    id SERIAL PRIMARY KEY, 
    conditions TEXT NOT NULL, 
    results JSONB 
);
```

### 欄位說明

| 欄位名稱 | 資料型別 | 說明 | 約束 |
|---------|---------|------|------|
| `id` | SERIAL | 主鍵，自動遞增 | PRIMARY KEY |
| `conditions` | TEXT | 條件內容，支援複雜的條件描述 | NOT NULL |
| `results` | JSONB | 結果資料，以 JSON 格式儲存 | 可為 NULL |

## 🔗 與現有系統的關係

### 現有相關資料表

1. **`task_condition_history`** - 任務條件歷史表
   - 提供完整的條件歷史追蹤
   - 包含時間戳、過期時間、錯誤訊息等詳細資訊
   - 適用於需要完整審計追蹤的場景

2. **`task_condition_cache`** - 任務條件快取表
   - 提供高效能的條件查詢快取
   - 包含快取管理機制（過期時間、命中次數等）
   - 適用於頻繁查詢的條件快取

3. **`task_condition`** - 任務條件表（新增）
   - 提供簡化的條件儲存
   - 結構簡單，適合快速開發和原型設計
   - 適用於基本的條件記錄需求

### 使用場景比較

| 使用場景 | 建議使用的資料表 | 原因 |
|---------|----------------|------|
| 簡單條件記錄 | `task_condition` | 結構簡單，開發快速 |
| 需要歷史追蹤 | `task_condition_history` | 完整的時間戳和審計功能 |
| 高頻查詢快取 | `task_condition_cache` | 專門的快取管理機制 |
| 複合需求 | 組合使用 | 根據具體需求選擇合適的表 |

## 🛠️ 實作細節

### SQLModel 模型定義

```python
class TaskCondition(SQLModel, table=True):
    """任務條件表 - 簡化版本"""
    __tablename__ = "task_condition"

    id: Optional[int] = Field(default=None, primary_key=True)
    conditions: str = Field(sa_column=Column(Text))
    results: Optional[Dict[str, Any]] = Field(default=None, sa_column=Column(JSON))
    description: Optional[str] = Field(default=None, max_length=500, description="條件描述")
```

### CRUD 操作

提供完整的 CRUD 操作，包括：
- `create_condition()` - 建立條件記錄
- `get_by_conditions()` - 根據條件內容查詢
- `search_conditions()` - 搜尋包含關鍵字的條件
- `update_results()` - 更新結果資料
- `delete_by_conditions()` - 根據條件內容刪除

### 資料庫初始化

- 自動建立資料表（透過 SQLModel）
- 提供範例資料初始化
- 包含在現有的初始化流程中

## 📊 使用範例

### 基本使用

```python
from db_proxy.crud.task_condition_crud import task_condition_crud

# 建立條件記錄
condition = task_condition_crud.create_condition(
    session,
    "AGV_STATUS = 'IDLE' AND LOCATION = 'STATION_A'",
    {"status": "ready", "timestamp": "2024-01-01T00:00:00Z"},
    "AGV 待機狀態檢查"
)

# 查詢條件
found = task_condition_crud.get_by_conditions(
    session, 
    "AGV_STATUS = 'IDLE' AND LOCATION = 'STATION_A'"
)

# 搜尋條件
results = task_condition_crud.search_conditions(session, "AGV_STATUS")
```

### 進階使用

```python
from db_proxy.examples.task_condition_example import TaskConditionManager

# 建立管理器
manager = TaskConditionManager(db_url)

# 新增 AGV 條件
agv_condition = manager.add_agv_condition("AGV001", "IDLE", "STATION_A")

# 新增任務條件
task_condition = manager.add_task_condition(
    "TRANSPORT", 8, 
    {"from": "STATION_A", "to": "STATION_B"}
)

# 取得摘要資訊
summary = manager.get_all_conditions_summary()
```

## 🔧 維護和最佳實踐

### 索引建議

雖然基本表結構簡單，但建議根據使用模式添加索引：

```sql
-- 條件內容的全文搜尋索引
CREATE INDEX idx_task_condition_conditions_gin ON task_condition USING gin(to_tsvector('english', conditions));

-- 結果資料的 GIN 索引（用於 JSONB 查詢）
CREATE INDEX idx_task_condition_results_gin ON task_condition USING gin(results);
```

### 資料清理

定期清理不需要的條件記錄：

```python
# 刪除特定條件
task_condition_crud.delete_by_conditions(session, "OLD_CONDITION")

# 批量清理（可根據需求實作）
```

### 效能考量

1. **條件字串標準化** - 建議使用統一的條件格式
2. **結果資料結構** - 保持 JSON 結構的一致性
3. **查詢最佳化** - 根據查詢模式添加適當索引

## 🚀 部署和測試

### 測試

```bash
# 執行單元測試
pytest db_proxy_ws/src/db_proxy/db_proxy/tests/test_task_condition.py

# 執行範例
python db_proxy_ws/src/db_proxy/db_proxy/examples/task_condition_example.py
```

### 部署

1. 確保資料庫連接正常
2. 執行資料庫初始化
3. 驗證資料表建立成功
4. 測試基本 CRUD 操作

## 📝 注意事項

1. **資料一致性** - 與現有的 task_condition_history 和 task_condition_cache 保持資料一致性
2. **效能監控** - 監控查詢效能，必要時添加索引
3. **資料備份** - 定期備份重要的條件資料
4. **版本相容性** - 確保與現有 WCS 系統的相容性

## 🔄 未來擴展

可能的擴展方向：
1. 添加時間戳欄位（created_at, updated_at）
2. 添加條件類型分類
3. 與現有條件表的關聯外鍵
4. 條件執行狀態追蹤
5. 條件優先級管理
