# Flow WCS 統一模型修復報告

## 執行日期: 2025-01-12

## 問題描述
Flow WCS 在使用統一模型 (`database_unified.py`) 時遇到以下錯誤：
1. `'Rack' object has no attribute 'to_flow_wcs_dict'`
2. Task 查詢錯誤：缺少 `type` 欄位
3. Flow executor 仍在使用舊的 `database.py` 而非 `database_unified.py`

## 解決方案實施

### 1. 修正 Flow Executor 導入路徑
**檔案**: `/app/flow_wcs_ws/src/flow_wcs/flow_wcs/flow_executor.py`
- 將 `from .database import db_manager` 改為 `from .database_unified import db_manager`

### 2. 為 db_proxy 模型新增缺失欄位
**檔案**: `/app/db_proxy_ws/src/db_proxy/db_proxy/models/agvc_task.py`

#### Work 模型新增欄位：
- `work_code: Optional[str]` - Flow WCS 需要的工作代碼

#### Task 模型新增欄位：
- `task_id: Optional[str]` - Flow WCS 需要的任務 ID
- `type: Optional[str]` - Flow WCS 需要的任務類型
- `location_id: Optional[int]` - 位置 ID
- `rack_id: Optional[int]` - 貨架 ID
- 新增 `generate_task_id()` 方法生成任務 ID

### 3. 資料庫 Schema 更新
執行 SQL 新增欄位：
```sql
ALTER TABLE work ADD COLUMN IF NOT EXISTS work_code VARCHAR(100);
ALTER TABLE task ADD COLUMN IF NOT EXISTS task_id VARCHAR(100);
ALTER TABLE task ADD COLUMN IF NOT EXISTS type VARCHAR(100);
ALTER TABLE task ADD COLUMN IF NOT EXISTS location_id INTEGER;
ALTER TABLE task ADD COLUMN IF NOT EXISTS rack_id INTEGER;
```

### 4. 修正 database_unified.py 查詢方法
**檔案**: `/app/flow_wcs_ws/src/flow_wcs/flow_wcs/database_unified.py`

移除對不存在的 `to_flow_wcs_dict()` 方法的呼叫，改為手動建立字典：

#### query_locations()：
```python
return [
    {
        'id': loc.id,
        'name': loc.name if hasattr(loc, 'name') else f'Location_{loc.id}',
        'type': loc.type if hasattr(loc, 'type') else 'unknown',
        'room_id': loc.room_id if hasattr(loc, 'room_id') else None,
        'x': loc.x if hasattr(loc, 'x') else 0.0,
        'y': loc.y if hasattr(loc, 'y') else 0.0,
        'has_rack': bool(loc.rack_id) if hasattr(loc, 'rack_id') else False,
        'rack_id': loc.rack_id if hasattr(loc, 'rack_id') else None,
        'status': getattr(loc, 'status', 'unknown')
    }
    for loc in locations
]
```

#### query_racks()：
```python
return [
    {
        'id': rack.id,
        'rack_id': getattr(rack, 'rack_id', f'RACK_{rack.id}'),
        'location_id': rack.location_id if hasattr(rack, 'location_id') else None,
        'side_a_status': rack.side_a_status if hasattr(rack, 'side_a_status') else 'empty',
        'side_b_status': rack.side_b_status if hasattr(rack, 'side_b_status') else 'empty',
        'side_a_carrier_id': rack.side_a_carrier_id if hasattr(rack, 'side_a_carrier_id') else None,
        'side_b_carrier_id': rack.side_b_carrier_id if hasattr(rack, 'side_b_carrier_id') else None,
        'rotation_angle': rack.rotation_angle if hasattr(rack, 'rotation_angle') else 0.0,
        'status': getattr(rack, 'status', 'available')
    }
    for rack in racks
]
```

#### query_tasks()：
簡化後的版本（因為 Task 模型現在有了必要的欄位）：
```python
status_map = {
    1: 'pending',
    2: 'assigned',
    3: 'executing',
    4: 'completed',
    5: 'cancelling',
    6: 'failed'
}
status = status_map.get(task.status_id, 'unknown')

result.append({
    'id': task.id,
    'task_id': task.task_id or f'TASK_{task.id}',
    'type': task.type or 'default',
    'work_id': task.work_id,
    'location_id': task.location_id,
    'rack_id': task.rack_id,
    'agv_id': task.agv_id,
    'status': status,
    'priority': task.priority,
    'metadata': task.parameters or {},
    'created_at': task.created_at.isoformat() if task.created_at else None
})
```

### 5. 修正 create_task() 方法
確保設定必要的 `name` 欄位：
```python
task = Task(
    type=type,
    work_id=work.id,
    location_id=location_id,
    rack_id=rack_id,
    priority=priority,
    parameters=metadata or {},
    status_id=1,  # PENDING
    name=f"Task {type} for {work_id}"  # 設定必要的 name 欄位
)
```

## 測試結果

### 測試查詢功能
```
1. 測試查詢 Racks: ✅ 成功查詢到 8 個 racks
2. 測試查詢 Tasks: ✅ 成功查詢到 8 個 tasks
3. 測試查詢 AGVs: ✅ 成功查詢到 6 個 AGVs
4. 測試查詢 Locations: ✅ 成功查詢到 91 個 locations
5. 測試建立任務: ✅ 成功建立任務: TASK_TEST_WORK_001_20250812132143
```

## 重要變更
1. **db_proxy 模型擴展**: 為了相容 flow_wcs，db_proxy 的 Task 和 Work 模型新增了額外欄位
2. **資料庫 Schema 更新**: 資料表新增了 5 個欄位
3. **Flow Executor 更新**: 現在使用統一的 `database_unified.py`
4. **查詢方法改進**: 移除對不存在方法的依賴，改為直接欄位訪問

## 建議後續動作
1. **監控系統執行**: 確保新的統一模型在生產環境中穩定運行
2. **更新其他模組**: 如果其他模組也使用這些模型，需要確保相容性
3. **效能測試**: 驗證新的查詢方法效能符合要求
4. **文檔更新**: 更新相關 API 文檔以反映新的欄位

## 結論
Flow WCS 現已成功使用 db_proxy 統一模型，解決了所有欄位缺失和方法呼叫錯誤。系統可以正常執行流程測試。