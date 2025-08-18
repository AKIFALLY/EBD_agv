# FlowLog 模型修復報告

## 執行日期: 2025-01-12

## 問題描述
FlowLog 模型在 db_proxy 中的結構與 flow_wcs 的 `database_unified.py` 不匹配，導致 `log_flow_execution` 方法無法正常運作。

### 原始問題
- db_proxy 的 FlowLog 模型缺少 flow_wcs 需要的多個欄位
- 資料庫 schema 中的 NOT NULL 約束導致插入失敗

## 解決方案

### 1. 更新 FlowLog 模型
**檔案**: `/app/db_proxy_ws/src/db_proxy/db_proxy/models/agvc_task.py`

修改 FlowLog 模型以包含所有必要欄位：
```python
class FlowLog(SQLModel, table=True):
    """Flow 執行日誌模型 - flow_wcs 整合"""
    __tablename__ = "flow_log"
    
    id: Optional[int] = Field(default=None, primary_key=True)
    flow_id: str
    flow_name: Optional[str] = None
    work_id: Optional[int] = Field(default=None, foreign_key="work.id")
    section: Optional[str] = None
    step_id: Optional[str] = None
    function: Optional[str] = None
    params: Optional[Dict[str, Any]] = Field(
        default=None,
        sa_column=Column(JSON)
    )
    result: Optional[Dict[str, Any]] = Field(
        default=None,
        sa_column=Column(JSON)
    )
    status: str  # success, failed, skipped
    error_message: Optional[str] = None
    duration: Optional[float] = None  # execution time in seconds
    created_at: datetime = Field(
        sa_column=Column(DateTime(timezone=True), nullable=False),
        default_factory=lambda: datetime.now(timezone.utc))
    
    # Legacy fields (for backward compatibility if needed)
    step_index: Optional[int] = None
    step_type: Optional[str] = None
    message: Optional[str] = None
    flow_metadata: Optional[Dict[str, Any]] = Field(
        default=None,
        sa_column=Column(JSON, name='metadata')
    )
```

### 2. 資料庫 Schema 更新
執行 SQL 新增欄位並修改約束：
```sql
-- 新增缺失欄位
ALTER TABLE flow_log ADD COLUMN IF NOT EXISTS flow_name VARCHAR(200);
ALTER TABLE flow_log ADD COLUMN IF NOT EXISTS work_id INTEGER REFERENCES work(id);
ALTER TABLE flow_log ADD COLUMN IF NOT EXISTS section VARCHAR(100);
ALTER TABLE flow_log ADD COLUMN IF NOT EXISTS step_id VARCHAR(100);
ALTER TABLE flow_log ADD COLUMN IF NOT EXISTS function VARCHAR(100);
ALTER TABLE flow_log ADD COLUMN IF NOT EXISTS params JSON;
ALTER TABLE flow_log ADD COLUMN IF NOT EXISTS result JSON;
ALTER TABLE flow_log ADD COLUMN IF NOT EXISTS error_message TEXT;
ALTER TABLE flow_log ADD COLUMN IF NOT EXISTS duration FLOAT;

-- 移除 NOT NULL 約束 (legacy 欄位)
ALTER TABLE flow_log ALTER COLUMN step_index DROP NOT NULL;
ALTER TABLE flow_log ALTER COLUMN step_type DROP NOT NULL;
```

### 3. 重新建置 db_proxy 套件
```bash
cd /app/db_proxy_ws
colcon build --packages-select db_proxy
source install/setup.bash
```

## 測試結果

執行測試腳本 `/app/test_flow_log_integration.py`：

```
=== 測試 FlowLog 整合 ===

1. 測試 Flow 執行日誌記錄:
   ✅ 成功記錄日誌 - flow_id: FLOW_TEST_20250812132611

2. 測試記錄多個步驟:
   ✅ 成功記錄第二個步驟

3. 測試記錄錯誤狀態:
   ✅ 成功記錄錯誤狀態

4. 查詢記錄的日誌:
   ✅ 找到 3 條日誌記錄
   - Section: initialization, Step: step_1, Function: check_rack_status, Status: success
     執行時間: 0.125s
   - Section: execution, Step: step_2, Function: create_task, Status: success
     執行時間: 0.235s
   - Section: validation, Step: step_3, Function: validate_agv, Status: failed
     錯誤: AGV not found
     執行時間: 0.050s
```

## 影響範圍
- **db_proxy 模型**: FlowLog 模型已更新以支援 flow_wcs 需求
- **資料庫 Schema**: flow_log 表新增 9 個欄位，修改 2 個欄位約束
- **flow_wcs 整合**: `database_unified.py` 的 `log_flow_execution` 方法現在可以正常運作

## 結論
FlowLog 模型已成功修復，flow_wcs 現在可以正確記錄流程執行日誌。所有必要的欄位都已添加，且向後相容性透過保留 legacy 欄位來維護。