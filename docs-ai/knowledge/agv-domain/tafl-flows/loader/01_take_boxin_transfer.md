# Flow 1: loader_take_boxin_transfer.yaml

## 🎯 業務目的
從入口傳送箱批量取出 Cargo AGV 送來的載具，啟動 Loader AGV 的前段制程流程

## 📋 基本信息

| 項目 | 值 |
|------|-----|
| 文件名 | `loader_take_boxin_transfer.yaml` |
| Flow ID | `loader_take_boxin_transfer` |
| 優先級 | 50 |
| 執行間隔 | 12 秒 |
| Work ID | **2010101, 2010301**（Station-based，2個）|

## 🏭 業務場景

### 前置條件
1. Cargo AGV 已將載具卸載到入口傳送箱（ENTRANCE_UNLOAD 完成）
2. 入口傳送箱（Equipment 201）有待處理載具（status_id: 102）
3. Loader AGV 處於空閒或有空餘車位

### 觸發條件
- **入口傳送箱有待處理的載具**（status_id: 102，至少2個）
- **Loader AGV 車上有空位**（至少2格空位）
- **沒有重複的未完成任務**

### 執行結果
- 創建 Loader AGV 取料任務
- 任務進入待分派隊列（status_id = 1 PENDING）
- RCS 系統分派給空閒的 Loader AGV

## 🔧 技術規格

### 入口傳送箱配置（Station-based）

**物理結構**：
- Equipment 201（入口傳送箱 BOX_IN_TRANSFER）
- 4個 Port（Port 1-4）
- **2個 Station**（**Station 01, 03**）

**Station-Port 映射**（標準設備）：
- **Station 01**: Port 1-2（**2格批量**）
- **Station 03**: Port 3-4（**2格批量**）

**Work ID 對應**（Station-based 編碼）：
- `2010101`: Station 01 取入口箱（Port 1-2，**2格批量**）
- `2010301`: Station 03 取入口箱（Port 3-4，**2格批量**）

**關鍵特點**：
- ✅ **2個 Work ID**（全部2格批量處理）
- ✅ **標準設備配置**（1 station = 2 ports）
- ✅ **與 Cargo AGV 銜接**（接收 Cargo 卸載的載具）

### Loader AGV 車載配置

**S尺寸產品**：
- 4格可用（第1層到第4層）
- 最大容量：4個 Carrier

**L尺寸產品**：
- 2格可用（僅第1層和第3層）
- 最大容量：2個 Carrier

### 批量處理邏輯

**標準2格批量處理**（**Station 01/03**）：
- **每個 Station 包含2格**（標準設備）
- **一次取料操作取出2格**（2格批量處理）
- **需要檢查入口箱至少有2個載具**（Port 1-2 或 3-4）
- **需要檢查 AGV 至少有2格空位**
- **不支持部分取料**（要么取2格，要么不取）

## 📝 TAFL Flow 設計

### Metadata
```yaml
metadata:
  id: "loader_take_boxin_transfer"
  name: "Loader AGV 從入口傳送箱取料"
  enabled: true
  version: "1.0.0"
  description: "檢查入口傳送箱待處理載具，創建 Loader AGV 取料任務"
  author: "TAFL System"
  created_at: "2025-10-16"
  tags: ["loader", "transfer", "take", "automation"]
```

### Settings
```yaml
settings:
  execution_interval: 12  # 每12秒執行一次
```

### Variables
```yaml
variables:
  priority: 50                       # 最高優先級（啟動前段制程）
  model: "LOADER"                    # AGV 型號
  entrance_equipment_id: 201         # 入口傳送箱 Equipment ID
  # 2個 Station 配置（Station-based，全部2格批量）
  stations:
    - station: 1                     # Station 01
      work_id: 2010101
      ports: [1, 2]
      batch_size: 2                  # 批量2格
    - station: 3                     # Station 03
      work_id: 2010301
      ports: [3, 4]
      batch_size: 2                  # 批量2格
```

## 🔄 流程邏輯

### 主要步驟

#### 1. 查詢所有房間
```yaml
- query:
    target: rooms
    where:
      enabled: true
    as: active_rooms
    description: "查詢所有啟用的房間"
```

#### 2. 遍歷每個房間
```yaml
- for:
    in: "${active_rooms}"
    as: room
    do:
      # 處理該房間的入口傳送箱
```

#### 3. 遍歷每個 Station
```yaml
- for:
    in: "${stations}"
    as: station
    do:
      # 檢查該 Station 的載具狀態
```

#### 4. 查詢 Station 的載具
```yaml
- query:
    target: carriers
    where:
      room_id: "${room.id}"
      equipment_id: "${entrance_equipment_id}"  # 201
      port_in: "${station.ports}"               # [1, 2] 或 [3, 4]
      status_id: 102                            # 進入入口傳送箱完成
    as: ready_carriers
    description: "查詢 Station ${station.station} 入口箱載具"
```

#### 5. 檢查載具數量（2格批量）
```yaml
- set:
    carrier_count: "${ready_carriers.length}"
    required_count: 2                           # 固定2格批量
    has_enough_carriers: "${carrier_count >= required_count}"
```

#### 6. 查詢 Loader AGV 狀態
```yaml
- query:
    target: agvs
    where:
      agv_type: "loader"
      room_id: "${room.id}"
      status: "idle"
    as: available_agvs
    description: "查詢空閒的 Loader AGV"
```

#### 7. 檢查 AGV 車上空位（2格批量）
```yaml
- query:
    target: carriers
    where:
      agv_id: "${agv.id}"
    as: agv_carriers
    description: "查詢 AGV 車上載具"

- set:
    agv_carrier_count: "${agv_carriers.length}"
    max_capacity: 4                              # AGV 最大容量4格
    required_space: 2                            # 需要2格空位
    available_space: "${max_capacity - agv_carrier_count}"
    agv_has_space: "${available_space >= required_space}"
```

#### 8. 檢查重複任務
```yaml
- query:
    target: tasks
    where:
      work_id: "${station.work_id}"
      room_id: "${room.id}"
      status_id_in: [0, 1, 2, 3]                # 未完成的狀態
    as: existing_tasks
    description: "檢查是否已存在取料任務"
```

#### 9. 創建取料任務
```yaml
- if:
    condition: "${has_enough_carriers} && ${agv_has_space} && ${existing_tasks.length == 0}"
    then:
      - create:
          target: task
          with:
            type: "loader_take"
            name: "房間${room.id}入口箱 Station${station.station} 取料"
            description: "從入口箱 Station ${station.station} 取出${carrier_count}個載具（2格批量）"
            work_id: "${station.work_id}"
            room_id: "${room.id}"
            priority: "${priority}"
            status_id: 1                         # PENDING
            parameters:
              station: "${station.station}"
              work_id: "${station.work_id}"
              room_id: "${room.id}"
              equipment_id: "${entrance_equipment_id}"
              ports: "${station.ports}"
              batch_size: 2                      # 固定2格
              model: "${model}"
              carrier_count: "${carrier_count}"
              reason: "入口傳送箱有待處理載具，AGV 有空位"
          description: "創建 Loader AGV 取料任務（Station-based）"
```

## 🔍 查詢條件詳解

### Carrier 查詢條件（Station-based）

**必要條件**：
- `room_id`: 特定房間
- `equipment_id: 201` (入口傳送箱)
- `port_in`: Station-based Port 映射（**2格批量**）
  - **Station 01**: [1, 2]（**批量2格**）
  - **Station 03**: [3, 4]（**批量2格**）

**狀態條件**：
- `status_id: 102`（進入入口傳送箱完成）
  - Cargo AGV ENTRANCE_UNLOAD 完成後的狀態
  - 表示載具已從 Rack 卸載到入口箱，等待 Loader AGV 取料

**數量要求**（固定2格批量）：
- **至少2個載具**（一次取2格）
- 不支持部分取料

### AGV 查詢條件

**Loader AGV 條件**：
- `agv_type`: "loader"
- `room_id`: 特定房間
- `status`: "idle"（空閒狀態）

**車載空位計算**（**2格批量處理**）：
- 查詢 AGV 車上現有載具數量
- **需要 ≥ 2格空位**
- 最大容量：4格（S尺寸產品）或2格（L尺寸產品）
- **不支持部分取料**：必須有2格空位才創建任務

### Task 重複檢查

**防止重複創建**：
- 檢查相同 `work_id` 和 `room_id`
- 狀態為未完成（0=創建, 1=待分派, 2=執行中, 3=暫停）
- 如果存在未完成任務，不創建新任務

## ⚠️ 注意事項

### Station-based 設計重點
- **編碼規則**: work_id 使用 Station 編號（**01/03**），非 Port 起始號
- **標準設備映射**: **Station 01/03 全部批量2格**（標準設備配置）
- **設計簡化**: 2個 Station，統一2格批量處理

### 批量處理邏輯
- **所有 Station 01/03**: 必須有 ≥ 2個載具，AGV 需 ≥ 2格空位
- **不支持部分取料**（要么取2格，要么不取）
- **標準批量**: 2格為單位，與泡藥機（1格）不同

### 狀態碼映射
Carrier status_id 定義：
- `102`: 進入入口傳送箱完成（TAKE_BOXIN_TRANSFER 觸發條件）
  - Cargo ENTRANCE_UNLOAD 完成後更新為此狀態
  - 表示載具已從 Rack 卸載，等待 Loader AGV 取料開始前段制程

### 與 Cargo AGV 銜接
- **前置流程**: Cargo AGV ENTRANCE_UNLOAD（從 Rack 卸載到入口箱）
- **狀態轉換**: Cargo 卸載完成 → 載具 status_id 更新為 102
- **觸發條件**: 檢測到 status_id = 102 且數量 ≥ 2

## 🧪 測試要點

### 單元測試
1. ✅ 查詢邏輯正確性
   - 正確查詢入口箱待處理載具
   - 正確過濾房間和端口
   - 正確使用 port_in: [1, 2] 或 [3, 4]

2. ✅ 數量判斷（固定2格批量）
   - 載具數量 >= 2
   - AGV 空位 >= 2

3. ✅ 重複檢查
   - 存在未完成任務時不創建
   - 任務參數正確傳遞

### 整合測試
1. ✅ 與 Cargo AGV 銜接
   - Cargo ENTRANCE_UNLOAD 完成
   - 載具狀態更新為在入口箱
   - Loader TAKE_BOXIN_TRANSFER 檢測到並創建任務

2. ✅ RCS 調度
   - 任務正確創建
   - RCS 分派給 Loader AGV
   - AGV 執行取料

3. ✅ 與後續 Flow 銜接
   - TAKE_BOXIN_TRANSFER 取料完成
   - 載具狀態更新
   - PUT_CLEANER 或其他後續 Flow 檢測到

### 邊界測試
1. ✅ 入口箱只有1個載具
   - 不創建任務（需要至少2個）

2. ✅ AGV 只有1格空位
   - 不創建任務（需要至少2格）

3. ✅ 同時有多個房間入口箱就緒
   - 正確為每個房間創建任務
   - 不產生衝突

## 📊 成功指標

### 功能指標
- ✅ 正確檢測入口箱待處理載具
- ✅ 正確創建取料任務
- ✅ 不產生重複任務
- ✅ 任務參數完整準確

### 性能指標
- 執行間隔：12秒
- 響應時間：< 2秒
- 任務創建延遲：< 30秒（從載具就緒到任務創建）

### 可靠性指標
- 零漏檢：所有就緒載具都能被檢測
- 零誤創建：不創建重複或錯誤任務
- 容錯性：數據庫查詢失敗時優雅降級

## 🔗 相關文檔

- **Loader AGV 代碼**: `/app/agv_ws/src/loader_agv/loader_agv/robot_states/take_transfer/`
- **Work ID 定義**: 資料庫 `agvc.work` 表
- **前置流程**: Cargo AGV - `ENTRANCE_UNLOAD`
- **後續流程**: `PUT_CLEANER`, `PUT_SOAKER`, 或其他前段制程
