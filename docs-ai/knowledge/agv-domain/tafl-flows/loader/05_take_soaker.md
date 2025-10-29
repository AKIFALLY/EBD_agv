# Flow 5: loader_take_soaker.yaml

## 🎯 業務目的
從泡藥機取出浸泡完成的載具（特殊設備：1格處理）

## 📋 基本信息

| 項目 | 值 |
|------|-----|
| 文件名 | `loader_take_soaker.yaml` |
| Flow ID | `loader_take_soaker` |
| 優先級 | 43 |
| 執行間隔 | 12 秒 |
| Work ID | **2040101, 2040201, 2040301, 2040401, 2040501, 2040601**（6個）|

## 🏭 業務場景

### 前置條件
1. Loader AGV 已將載具放入泡藥機（PUT_SOAKER 完成）
2. 泡藥機完成浸泡制程
3. 載具更新為「泡藥完成」（status_id: 403）
4. Loader AGV 處於空閒或有空餘車位

### 觸發條件
- **泡藥機（任一台 A-F）有泡藥完成的載具**（status_id: 403）
- **Loader AGV 車上有空位**（至少1格空位）
- **沒有重複的未完成任務**

### 執行結果
- 創建 Loader AGV 取料任務
- 任務進入待分派隊列（status_id = 1 PENDING）
- 從泡藥機取出載具，進入下一道工序

## 🔧 技術規格

### 泡藥機配置（特殊設備：1 station = 1 port）

**物理結構**：
- Equipment 204（泡藥機 SOAKER）
- 6個 Port（Port 1-6，對應泡藥機 A-F）
- **6個獨立 Station**（**Station 01-06**）

**Station-Port 映射**（**特殊設備**）：
- **Station 01-06**: Port 1-6（**各1格處理**）

**Work ID 對應**（6個 TAKE Work IDs）：
- `2040101`: Station 01 取泡藥機 A（Port 1，**1格**）
- `2040201`: Station 02 取泡藥機 B（Port 2，**1格**）
- `2040301`: Station 03 取泡藥機 C（Port 3，**1格**）
- `2040401`: Station 04 取泡藥機 D（Port 4，**1格**）
- `2040501`: Station 05 取泡藥機 E（Port 5，**1格**）
- `2040601`: Station 06 取泡藥機 F（Port 6，**1格**）

**關鍵特點**：
- ✅ **6個獨立 Work ID**（泡藥機 A-F）
- ✅ **特殊設備配置**（1 station = 1 port）
- ✅ **1格處理**（與標準設備2格不同）
- ✅ **支援多次浸泡**（可重複 PUT → TAKE 流程）

### 批量處理邏輯

**特殊1格處理**（**所有 Station 01-06**）：
- **每個 Station 包含1格**（特殊設備）
- **一次取料操作取出1格**（1格處理）
- **需要檢查泡藥機至少有1個完成載具**（任一台 A-F）
- **需要檢查 AGV 至少有1格空位**
- **不支持批量處理**（每次處理1格）

## 📝 TAFL Flow 設計

### Metadata
```yaml
metadata:
  id: "loader_take_soaker"
  name: "Loader AGV 從泡藥機取料"
  enabled: true
  version: "1.0.0"
  description: "檢查泡藥機浸泡完成載具，創建 Loader AGV 取料任務（特殊設備：1格處理）"
  author: "TAFL System"
  created_at: "2025-10-16"
  tags: ["loader", "soaker", "take", "automation", "special-equipment"]
```

### Variables
```yaml
variables:
  priority: 43
  model: "LOADER"
  soaker_equipment_id: 204
  # 6個 Station 配置（特殊設備，全部1格處理）
  stations:
    - station: 1
      work_id: 2040101
      port: 1
      batch_size: 1
      name: "泡藥機A"
    # ... (Station 02-06 配置相同結構)
```

## 🔄 流程邏輯

### 主要步驟

1. 查詢所有房間
2. 遍歷每個房間
3. 查詢 Loader AGV
4. 檢查 AGV 車上空位（≥ 1格）
5. **遍歷6個 Station**（泡藥機 A-F）
6. 查詢每個泡藥機載具（status_id: 403）
7. 檢查重複任務
8. 創建取料任務（動態選擇有完成載具的泡藥機）

**關鍵查詢**：
```yaml
# 查詢泡藥機載具
- query:
    target: carriers
    where:
      room_id: "${room.id}"
      equipment_id: "${soaker_equipment_id}"  # 204
      port: "${station.port}"                 # 1-6
      status_id: 403                          # 泡藥完成
    as: ready_carrier
```

## ⚠️ 注意事項

### 特殊設備設計重點
- **編碼規則**: work_id 使用 Station 編號（**01-06**），對應 Port 1-6
- **特殊設備映射**: **1 station = 1 port**
- **獨立操作**: 每個泡藥機獨立檢測和取料

### 多次浸泡支援
- **第1次浸泡**: PUT_SOAKER → TAKE_SOAKER → **PUT_PRE_DRYER**（完成）
- **第2次浸泡**: PUT_SOAKER → TAKE_SOAKER → **PUT_SOAKER** → TAKE_SOAKER → PUT_PRE_DRYER
- **靈活制程**: 根據製程需求決定浸泡次數

### 狀態碼映射
- `402`: 泡藥中（PUT_SOAKER 放入後）
- `403`: 泡藥完成（TAKE_SOAKER 觸發條件）

## 🧪 測試要點

### 單元測試
1. ✅ 正確遍歷6個 Station
2. ✅ 正確使用 status_id: 403 查詢完成載具
3. ✅ AGV 空位判斷（>= 1）

### 整合測試
1. ✅ 與 PUT_SOAKER 銜接
2. ✅ 支援多次浸泡流程（循環 PUT → TAKE）
3. ✅ 與後續流程銜接（PUT_PRE_DRYER 或再次 PUT_SOAKER）

### 邊界測試
1. ✅ 6台泡藥機全空時不創建任務
2. ✅ AGV 滿載時不創建任務

## 🔗 相關文檔

- **Loader AGV 代碼**: `/app/agv_ws/src/loader_agv/loader_agv/robot_states/take_soaker/`
- **前置流程**: Flow 4 - `PUT_SOAKER`
- **後續流程**: Flow 6 - `PUT_PRE_DRYER`（或再次 Flow 4 - `PUT_SOAKER` 進行多次浸泡）
