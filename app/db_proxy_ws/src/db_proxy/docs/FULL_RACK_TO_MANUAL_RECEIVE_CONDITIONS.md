# 滿料架到人工收料區任務條件說明

## 📋 概述

本文件說明新增的四個條件檢查，用於實現滿料架到人工收料區的任務流程。這些條件支援 OR 邏輯，提供多路徑探索和容錯機制。

## 🔄 任務流程

```
開始 → 檢查人工收料區空位 (ID 2) → 檢查房間2滿料架 (ID 205) → 檢查重複任務 (ID 207) → 結束
                ↓                           ↓
            無空位結束                  檢查cargo任務 (ID 206) ↗
```

## 📝 條件詳細說明

### 條件 1：檢查人工收料區空位（ID: 2）

**功能**：查詢人工收料區是否有空位

**SQL 邏輯**：
```sql
SELECT 
    CASE 
        WHEN COUNT(*) > 0 THEN 'True'
        ELSE 'False'
    END as result,
    CASE 
        WHEN COUNT(*) > 0 THEN '[105,205,305,405,505,605,705,805,905,1005]'
        ELSE NULL
    END as next_id,
    CASE 
        WHEN COUNT(*) > 0 THEN MIN(id)
        ELSE NULL
    END as location
FROM location 
WHERE id = ANY (ARRAY[51,52,53,54,55]) AND location_status_id = 2
```

**回傳欄位**：
- `result`: "True"/"False"
- `next_id`: 字串格式的陣列或 NULL
- `location`: 可用位置的 ID

**流程邏輯**：
- ✅ 有空位 → 跳轉到房間檢查流程
- ❌ 無空位 → 流程結束

---

### 條件 2：檢查房間2出口傳送箱的滿料架（ID: 205）

**功能**：查詢房間2的出口傳送箱位置是否有滿料架

**SQL 邏輯**：
```sql
SELECT 
    CASE 
        WHEN COUNT(*) > 0 THEN 'True'
        ELSE 'False'
    END as result,
    CASE 
        WHEN COUNT(*) > 0 THEN 207
        ELSE 206
    END as next_id,
    CASE 
        WHEN COUNT(*) > 0 THEN MIN(r.id)
        ELSE NULL
    END as rack_id,
    2 as room_id
FROM rack r 
WHERE r.location_id = 20002 AND r.status_id IN (2, 3, 6)
```

**回傳欄位**：
- `result`: "True"/"False"
- `next_id`: 207（有滿料架）或 206（無滿料架）
- `rack_id`: 找到的 rack ID
- `room_id`: 房間 ID (固定為 2)

**滿料架狀態定義**：
- `2`: 滿料架-32
- `3`: 滿料架-16
- `6`: 未滿料-無carrier

**流程邏輯**：
- ✅ 有滿料架 → 跳轉到重複任務檢查 (ID 207)
- ❌ 無滿料架 → 檢查 cargo 任務狀態 (ID 206)

---

### 條件 3：檢查房間2出口傳送箱的 cargo 任務狀態（ID: 206）

**功能**：查詢房間2出口傳送箱是否有已完成的 cargo 任務

**SQL 邏輯**：
```sql
SELECT 
    CASE 
        WHEN COUNT(*) > 0 THEN 'True'
        ELSE 'False'
    END as result,
    207 as next_id,
    2 as room_id,
    2000201 as cargo_work_id
FROM task 
WHERE work_id = 2000201 AND status_id = 4
```

**回傳欄位**：
- `result`: "True"/"False"
- `next_id`: 207（無論結果如何都跳轉）
- `room_id`: 房間 ID (固定為 2)
- `cargo_work_id`: cargo 任務的 work_id

**任務狀態定義**：
- `4`: AGV_FINISH_STATUS（已完成）

**流程邏輯**：
- ✅ 有已完成任務 → 跳轉到重複任務檢查 (ID 207)
- ❌ 無已完成任務 → 跳轉到重複任務檢查 (ID 207)
- 📝 **註**：無論結果如何都會跳轉到下一步

---

### 條件 4：檢查重複任務（ID: 207）

**功能**：檢查是否已存在相同的任務（防止重複建立）

**SQL 邏輯**：
```sql
SELECT 
    CASE 
        WHEN COUNT(*) = 0 THEN 'True'
        ELSE 'False'
    END as result,
    CASE 
        WHEN COUNT(*) = 0 THEN NULL
        ELSE NULL
    END as next_id,
    2 as room_id,
    CASE 
        WHEN COUNT(*) = 0 THEN 'True'
        ELSE 'False'
    END as end
FROM task 
WHERE work_id = 220001 AND node_id = 20002 AND status_id IN (0, 1, 2)
```

**回傳欄位**：
- `result`: "True"/"False"
- `next_id`: NULL（流程結束）
- `room_id`: 房間 ID (固定為 2)
- `end`: "True"/"False"

**任務狀態定義**：
- `0`: OPUI_STATUS（未執行）
- `1`: WCS_STATUS（已選擇）
- `2`: RCS_STATUS（執行中）

**流程邏輯**：
- ✅ 無重複任務 → 可以建立新任務（流程成功結束）
- ❌ 有重複任務 → 不建立任務（流程失敗結束）

## 🔧 技術細節

### 位置 ID 對應

- **人工收料區**: [51, 52, 53, 54, 55]
- **房間2出口傳送箱**: 20002
- **location_status_id = 2**: 無佔用狀態

### Work ID 定義

- **220001**: KUKA 移動貨架的 work_id
- **2000201**: 房間2 CargoAGV 拿出口傳送箱的 work_id

### OR 邏輯支援

這些條件支援新的 OR 邏輯：
- 當 `result = "False"` 但有 `next_id` 時，系統會繼續探索下一個條件
- 提供多路徑探索和容錯機制
- 增加判斷的多重可能性

## 🎯 使用範例

### 成功流程
```
1. 檢查人工收料區 → 有空位 ✅
2. 檢查房間2滿料架 → 有滿料架 ✅
3. 檢查重複任務 → 無重複 ✅
→ 建立新任務
```

### 容錯流程
```
1. 檢查人工收料區 → 有空位 ✅
2. 檢查房間2滿料架 → 無滿料架 ❌
3. 檢查cargo任務 → 有已完成任務 ✅
4. 檢查重複任務 → 無重複 ✅
→ 建立新任務
```

### 失敗流程
```
1. 檢查人工收料區 → 無空位 ❌
→ 流程結束，不建立任務
```

## 📊 測試驗證

所有條件都已通過完整的單元測試，包括：
- SQL 語法驗證
- 邏輯流程驗證
- 條件 ID 唯一性驗證
- OR 邏輯功能驗證

測試檔案：`db_proxy_ws/src/db_proxy/test/test_new_task_conditions.py`
