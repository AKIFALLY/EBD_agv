# 空料架搬運到出口傳送箱任務條件說明

## 📋 概述

本文件說明空料架搬運到出口傳送箱的條件檢查，用於實現空料架從儲存位置搬運到出口傳送箱的任務流程。這些條件確保搬運過程的完整性和效率。

## 🔄 任務流程

```
開始 → 檢查系統空架區 (ID 1) → 檢查出口傳送箱1空間 (ID 201) → 檢查重複任務 (ID 203) → 結束
                ↓                           ↓
            無空料架結束              檢查出口傳送箱2空間 (ID 202) ↗
```

## 📝 條件詳細說明

### 條件 1：檢查系統空架區空料架（ID: 1）

**功能**：查詢系統空架區是否有可用的空料架可以搬運到出口傳送箱

**SQL 邏輯**：
```sql
SELECT 
    CASE 
        WHEN COUNT(*) > 0 THEN 'True'
        ELSE 'False'
    END as result,
    CASE 
        WHEN COUNT(*) > 0 THEN '[101,201,301,401,501,601,701,801,901,1001]'
        ELSE NULL
    END as next_id,
    CASE 
        WHEN COUNT(*) > 0 THEN MIN(id)
        ELSE NULL
    END as location
FROM location 
WHERE id = ANY (ARRAY[31,32,33,34]) AND location_status_id = 3
```

**回傳欄位**：
- `result`: "True"/"False"
- `next_id`: 字串格式的陣列或 NULL
- `location`: 可用空料架位置的 ID

**流程邏輯**：
- ✅ 有空料架 → 跳轉到出口傳送箱檢查流程
- ❌ 無空料架 → 流程結束

---

### 條件 2：檢查出口傳送箱1空間（ID: 201）

**功能**：查詢出口傳送箱1是否有空間接收空料架

**SQL 邏輯**：
```sql
SELECT 
    CASE 
        WHEN COUNT(*) > 0 THEN 'True'
        ELSE 'False'
    END as result,
    CASE 
        WHEN COUNT(*) > 0 THEN 202
        ELSE NULL
    END as next_id,
    CASE 
        WHEN COUNT(*) > 0 THEN 20001
        ELSE NULL
    END as location_id
FROM location 
WHERE id = 20001 AND location_status_id = 1
```

**回傳欄位**：
- `result`: "True"/"False"
- `next_id`: 202（有空間）或 NULL（無空間）
- `location_id`: 出口位置 ID (20001)

**空間狀態定義**：
- `1`: 無佔用狀態（可接收空料架）

**流程邏輯**：
- ✅ 有空間 → 跳轉到出口傳送箱2檢查 (ID 202)
- ❌ 無空間 → 檢查其他出口選項

---

### 條件 3：檢查出口傳送箱2空間（ID: 202）

**功能**：查詢出口傳送箱2是否有空間接收空料架

**SQL 邏輯**：
```sql
SELECT 
    CASE 
        WHEN COUNT(*) > 0 THEN 'True'
        ELSE 'False'
    END as result,
    CASE 
        WHEN COUNT(*) > 0 THEN 203
        ELSE NULL
    END as next_id,
    CASE 
        WHEN COUNT(*) > 0 THEN 20002
        ELSE NULL
    END as location_id
FROM location 
WHERE id = 20002 AND location_status_id = 1
```

**回傳欄位**：
- `result`: "True"/"False"
- `next_id`: 203（有空間）或 NULL（無空間）
- `location_id`: 出口位置 ID (20002)

**空間狀態定義**：
- `1`: 無佔用狀態（可接收空料架）

**流程邏輯**：
- ✅ 有空間 → 跳轉到重複任務檢查 (ID 203)
- ❌ 無空間 → 尋找其他出口選項

---

### 條件 4：檢查重複任務（ID: 203）

**功能**：檢查是否已存在相同的空料架搬運任務（防止重複建立）

**SQL 邏輯**：
```sql
SELECT 
    CASE 
        WHEN COUNT(*) = 0 THEN 'True'
        ELSE 'False'
    END as result,
    NULL as next_id,
    CASE 
        WHEN COUNT(*) = 0 THEN 20002
        ELSE NULL
    END as location_id,
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
- `location_id`: 目標位置 ID (20002)
- `end`: "True"/"False"

**任務狀態定義**：
- `0`: OPUI_STATUS（未執行）
- `1`: WCS_STATUS（已選擇）
- `2`: RCS_STATUS（執行中）

**流程邏輯**：
- ✅ 無重複任務 → 可以建立新的空料架搬運任務（流程成功結束）
- ❌ 有重複任務 → 不建立任務（流程失敗結束）

## 🔧 技術細節

### 位置 ID 對應

- **系統空架區**: [31, 32, 33, 34]
- **出口傳送箱1**: 20001
- **出口傳送箱2**: 20002
- **location_status_id = 1**: 無佔用狀態（可接收空料架）
- **location_status_id = 3**: 有空料架狀態

### Work ID 定義

- **220001**: KUKA 移動貨架的 work_id（空料架搬運到出口傳送箱）

### 條件檢查邏輯

這些條件支援系統化的空料架搬運流程：
- 先確認有可用的空料架
- 再確認出口傳送箱有接收空間
- 最後確認沒有重複任務
- 確保搬運過程的完整性和效率

## 🎯 使用範例

### 成功流程
```
1. 檢查系統空架區 → 有空料架 ✅
2. 檢查出口傳送箱1 → 有空間 ✅
3. 檢查重複任務 → 無重複 ✅
→ 建立空料架搬運任務
```

### 替代流程
```
1. 檢查系統空架區 → 有空料架 ✅
2. 檢查出口傳送箱1 → 無空間 ❌
3. 檢查出口傳送箱2 → 有空間 ✅
4. 檢查重複任務 → 無重複 ✅
→ 建立空料架搬運任務
```

### 失敗流程
```
1. 檢查系統空架區 → 無空料架 ❌
→ 流程結束，不建立任務
```

## 📊 測試驗證

所有條件都已通過完整的測試驗證，包括：
- SQL 語法驗證
- 邏輯流程驗證
- 條件 ID 唯一性驗證
- 空料架搬運流程驗證

測試檔案：`db_proxy_ws/src/db_proxy/test/test_empty_rack_to_boxout_conditions.py`
