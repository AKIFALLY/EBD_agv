# Flow 2: cargo_exit_load.yaml

## 🎯 業務目的
從出口傳送箱批量取出完成制程的載具，裝載回料架（Rack），完成整個制程循環

## 📋 基本信息

| 項目 | 值 |
|------|-----|
| 文件名 | `cargo_exit_load.yaml` |
| Flow ID | `cargo_exit_load` |
| 優先級 | 40 |
| 執行間隔 | 15 秒 |
| Work ID | **2000201**（Cargo AGV 出口裝載）|

## 🏭 業務場景

### 前置條件
1. Unloader AGV 已將載具放入出口傳送箱（PUT_BOXOUT_TRANSFER 完成）
2. 出口傳送箱（Equipment 202）有完成制程的載具（至少4個）
3. 料架（Rack）有空位可以裝載載具
4. KUKA Fleet Manager 服務正常運行
5. Cargo AGV 處於空閒或可接受任務狀態

### 觸發條件
- **出口傳送箱有完成制程的載具**（Equipment 202, status_id: 202，至少4個）
- **料架有4格空位**可以裝載載具
- **沒有重複的未完成任務**

### 執行結果
- 創建 Cargo AGV 出口裝載任務
- 任務進入待分派隊列（status_id = 1 PENDING）
- KUKA Fleet Manager 調度 KUKA AGV 執行
- 載具從出口傳送箱移回料架
- 完成整個制程循環，載具回到儲存狀態

## 🔧 技術規格

### 出口傳送箱配置

**物理結構**：
- Equipment 202（出口傳送箱 BOX_OUT_TRANSFER）
- 4個 Port（Port 1-4）
- **Station 01**: Port 1-2-3-4（**批量4格**）

**UnloaderAGV 自定義映射**：
- **Station 01**: Port 1-2-3-4（**批量4格**，Unloader PUT 專用）

**Work ID 配置**：
- `2000201`: Cargo AGV 出口裝載（出口傳送箱 → Rack）

### 料架（Rack）配置

**物理結構**：
- 料架為移動式儲存單元
- 每個料架可容納多個載具
- 支援 KUKA AGV 舉起搬運
- 需要有足夠空位接收完成制程的載具

**批量處理**：
- ✅ **統一4格批量處理**
- ✅ **與 Unloader 批量一致**
- ✅ **一次裝載4個載具**

### KUKA Fleet API 整合

```python
# Work ID 2000201 參數配置
{
    "id": 2000201,
    "name": "CargoAGV拿出口傳送箱",
    "description": "從出口傳送箱拿carrier到料架放",
    "parameters": {
        "function": "rack_move",        # KUKA Fleet 功能類型
        "api": "submit_mission",        # KUKA API 端點
        "missionType": "RACK_MOVE",     # KUKA 任務類型
        "nodes": []                     # 路徑節點（動態生成）
    }
}
```

**KUKA AGV 執行流程**：
1. 導航到料架位置
2. 舉起料架（Lift Rack）
3. 旋轉料架 180 度（對接傳送箱方向）
4. 移動到出口傳送箱位置
5. 對接傳送箱
6. 裝載4個載具（Port 1-4）
7. 分離料架
8. 旋轉料架 180 度（恢復原方向）
9. 放下料架（Drop Rack）
10. 返回待命位置

### 批量處理邏輯

**統一4格批量處理**：
- **一次裝載4個載具**（統一批量處理）
- **需要檢查出口傳送箱至少有4個完成制程載具**（Port 1-4 全部有載具）
- **需要檢查料架至少有4格空位**
- **不支持部分裝載**（要么裝載4格，要么不裝載）

### Cargo AGV 狀態機流程

**出口裝載狀態機** (`exit/` 目錄）：

```
check_rack_side_state           # 1. 檢查料架側邊
    ↓
select_rack_port_state          # 2. 選擇料架端口（選擇4個Port）
    ↓
transfer_check_empty_state      # 3. 檢查傳送箱載具（確認4個載具）
    ↓
transfer_vision_position_state  # 4. 傳送箱視覺定位
    ↓
take_transfer_state            # 5. 從傳送箱取出載具（批量4個）
    ↓
wait_rotation_state            # 6. 等待料架旋轉（180度）
    ↓
rack_vision_position_state     # 7. 料架視覺定位
    ↓
put_rack_port_state           # 8. 放入料架（批量4個）
```

## 📝 TAFL Flow 設計

### Metadata
```yaml
metadata:
  id: "cargo_exit_load"
  name: "Cargo AGV 出口裝載"
  enabled: true
  version: "1.0.0"
  description: "檢查出口傳送箱完成制程載具和料架空位，創建 Cargo AGV 出口裝載任務"
  author: "TAFL System"
  created_at: "2025-10-16"
  tags: ["cargo", "exit", "load", "automation", "kuka_fleet"]
```

### Settings
```yaml
settings:
  execution_interval: 15  # 每15秒執行一次
```

### Variables
```yaml
variables:
  priority: 40                    # 高優先級（完成制程循環）
  work_id: 2000201                # 固定 Work ID
  exit_equipment_id: 202          # 出口傳送箱 Equipment ID
  ports: [1, 2, 3, 4]            # 出口傳送箱 Port 1-4
  batch_size: 4                   # 批量4格
  carrier_status_id: 202          # 載具狀態：在出口傳送箱
  function: "rack_move"           # KUKA Fleet 功能
  api: "submit_mission"           # KUKA API
  missionType: "RACK_MOVE"        # KUKA 任務類型
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
      # 處理該房間的出口傳送箱和料架
```

#### 3. 查詢出口傳送箱載具
```yaml
- query:
    target: carriers
    where:
      room_id: "${room.id}"
      equipment_id: "${exit_equipment_id}"  # 202
      port_in: "${ports}"                   # [1, 2, 3, 4]
      status_id: 202                        # 進入出口傳送箱完成
    as: exit_carriers
    description: "查詢出口傳送箱完成制程的載具（Port 1-4）"
```

#### 4. 檢查載具數量（批量4格）
```yaml
- set:
    carrier_count: "${exit_carriers.length}"
    required_count: 4                 # 固定4格批量
    has_enough_carriers: "${carrier_count >= required_count}"
```

#### 5. 查詢料架空位
```yaml
- query:
    target: racks
    where:
      room_id: "${room.id}"
      available_space_gte: 4          # 至少4格空位
    as: available_racks
    description: "查詢有足夠空位的料架（≥4格）"
```

#### 6. 檢查料架空位（批量4格）
```yaml
- set:
    rack_count: "${available_racks.length}"
    has_rack_space: "${rack_count > 0}"
```

#### 7. 檢查重複任務
```yaml
- query:
    target: tasks
    where:
      work_id: "${work_id}"           # 2000201
      room_id: "${room.id}"
      status_id_in: [0, 1, 2, 3]     # 未完成的狀態
    as: existing_tasks
    description: "檢查是否已存在出口裝載任務"
```

#### 8. 創建出口裝載任務
```yaml
- if:
    condition: "${has_enough_carriers} && ${has_rack_space} && ${existing_tasks.length == 0}"
    then:
      - create:
          target: task
          with:
            type: "cargo_exit_load"
            name: "房間${room.id} Cargo出口裝載"
            description: "從出口傳送箱裝載${carrier_count}個載具回料架（批量4格）"
            work_id: "${work_id}"      # 2000201
            room_id: "${room.id}"
            priority: "${priority}"
            status_id: 1               # PENDING
            parameters:
              work_id: "${work_id}"
              room_id: "${room.id}"
              equipment_id: "${exit_equipment_id}"
              ports: "${ports}"
              batch_size: "${batch_size}"
              carrier_count: "${carrier_count}"
              rack_id: "${available_racks[0].id}"  # 選擇第一個有空位的料架
              function: "${function}"        # rack_move
              api: "${api}"                  # submit_mission
              missionType: "${missionType}"  # RACK_MOVE
              nodes: []                      # 動態生成
              reason: "出口傳送箱有完成制程載具，料架有空位"
          description: "創建 Cargo AGV 出口裝載任務（KUKA Fleet API）"
```

## 🔍 查詢條件詳解

### Carrier 查詢條件

**出口傳送箱載具條件**：
- `room_id`: 特定房間
- `equipment_id: 202` (出口傳送箱 BOX_OUT_TRANSFER)
- `port_in: [1, 2, 3, 4]` (固定 Port 映射)
- `status_id: 202` (進入出口傳送箱完成)
  - Unloader AGV PUT_BOXOUT_TRANSFER 完成後的狀態
  - 表示載具已完成整個制程，在出口傳送箱等待 Cargo AGV 裝載回料架

**數量要求**（固定4格批量）：
- **至少4個載具**（一次裝載4格）
- Port 1-4 全部有載具才創建任務
- 不支持部分裝載

### Rack 查詢條件

**料架空位條件**：
- `room_id`: 特定房間
- `available_space_gte: 4` (至少4格空位)

**空位要求**（固定4格批量）：
- **至少4格空位**（一次裝載4格）
- 確保料架可以接收完整批量

### Task 重複檢查

**防止重複創建**：
- 檢查相同 `work_id` 和 `room_id`
- 狀態為未完成（0=創建, 1=待分派, 2=執行中, 3=暫停）
- 如果存在未完成任務，不創建新任務

## ⚠️ 注意事項

### KUKA Fleet 整合重點
- **KUKA Fleet Manager 依賴**: 必須確保 KUKA Fleet Manager 服務運行正常
- **rack_move API**: 專門的料架搬運功能，自動處理舉起、旋轉、放下
- **動態路徑**: nodes 參數由系統動態生成，根據傳送箱和料架位置計算
- **料架旋轉**: KUKA AGV 自動處理180度旋轉，對接傳送箱

### 批量處理邏輯
- **固定4格批量**: 必須有 ≥ 4個完成制程載具，料架需 ≥ 4格空位
- **不支持部分裝載**（要么裝載4格，要么不裝載）
- **批量一致性**: 與 Unloader 保持統一4格批量

### 狀態碼映射
Carrier status_id 定義：
- `202`: 進入出口傳送箱完成（Equipment 202）- CARGO_EXIT_LOAD 觸發條件
  - Unloader PUT_BOXOUT_TRANSFER 完成後更新為此狀態
  - 表示載具已完成整個制程，等待 Cargo AGV 裝載回料架
- 裝載回料架後：更新為儲存狀態

### 設備配置
- **固定 equipment_id**: 202（出口傳送箱 BOX_OUT_TRANSFER）
- **固定 Work ID**: 2000201
- **固定 Port**: 1-4（批量4格）

### 與 Unloader AGV 銜接
- **前置流程**: Unloader AGV PUT_BOXOUT_TRANSFER（Flow 4）
- **狀態轉換**: Unloader 放入出口傳送箱 → 載具 status_id 更新為 202
- **觸發條件**: 檢測到 status_id = 202 且數量 ≥ 4

## 🧪 測試要點

### 單元測試
1. ✅ 查詢邏輯正確性
   - 正確查詢出口傳送箱完成制程載具
   - 正確查詢料架空位
   - 正確過濾房間和端口

2. ✅ 數量判斷（固定4格批量）
   - 載具數量 >= 4
   - 料架空位 >= 4

3. ✅ 重複檢查
   - 存在未完成任務時不創建
   - 任務參數正確傳遞

4. ✅ KUKA Fleet 參數
   - 正確使用 function: "rack_move"
   - 正確使用 missionType: "RACK_MOVE"
   - 正確使用 api: "submit_mission"

### 整合測試
1. ✅ 與 Unloader AGV 銜接
   - Unloader PUT_BOXOUT_TRANSFER 完成
   - 載具狀態更新為在出口傳送箱
   - Cargo EXIT_LOAD 檢測到並創建任務

2. ✅ 與 KUKA Fleet Manager 銜接
   - 任務正確創建
   - KUKA Fleet Manager 接收任務
   - KUKA AGV 執行 rack_move

3. ✅ 完整流程
   - Cargo AGV 執行出口裝載
   - 料架旋轉180度對接傳送箱
   - 裝載4個載具回料架
   - 料架恢復原方向並返回
   - 載具狀態更新為儲存
   - 任務狀態更新為完成

### 邊界測試
1. ✅ 出口傳送箱只有3個載具
   - 不創建任務（需要至少4個）

2. ✅ 料架只有3格空位
   - 不創建任務（需要至少4格）

3. ✅ 同時有多個房間出口傳送箱就緒
   - 正確為每個房間創建任務
   - 不產生衝突

## 📊 成功指標

### 功能指標
- ✅ 正確檢測出口傳送箱完成制程載具
- ✅ 正確檢測料架空位
- ✅ 正確創建出口裝載任務
- ✅ 不產生重複任務
- ✅ 任務參數完整準確
- ✅ KUKA Fleet 參數正確
- ✅ 100% 完成制程循環

### 性能指標
- 執行間隔：15秒
- 響應時間：< 2秒
- 任務創建延遲：< 30秒（從載具就緒到任務創建）
- KUKA AGV 執行時間：< 5分鐘（從接收任務到完成裝載）
- 整體制程循環時間：監控從入口卸載到出口裝載的完整時間

### 可靠性指標
- 零漏檢：所有就緒載具都能被檢測
- 零誤創建：不創建重複或錯誤任務
- KUKA Fleet 成功率：> 99%
- 循環完整性：100% 載具回到料架
- 容錯性：KUKA Fleet 失敗時優雅降級

## 🔗 相關文檔

- **Cargo AGV 代碼**: `/app/agv_ws/src/cargo_mover_agv/cargo_mover_agv/robot_states/exit/`
- **KUKA Fleet Manager**: `/app/kuka_fleet_ws/src/kuka_fleet/kuka_fleet/kuka_fleet_manager.py`
- **Work ID 定義**: 資料庫 `agvc.work` 表
- **前置流程**: Unloader AGV - `PUT_BOXOUT_TRANSFER` (Flow 4)
- **相關流程**: Cargo AGV - `ENTRANCE_UNLOAD` (Flow 1)
