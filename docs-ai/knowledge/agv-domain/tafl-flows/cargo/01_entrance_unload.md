# Flow 1: cargo_entrance_unload.yaml

## 🎯 業務目的
從料架（Rack）批量取出載具，卸載到入口傳送箱，啟動整個制程流程

## 📋 基本信息

| 項目 | 值 |
|------|-----|
| 文件名 | `cargo_entrance_unload.yaml` |
| Flow ID | `cargo_entrance_unload` |
| 優先級 | 50 |
| 執行間隔 | 15 秒 |
| Work ID | **2000102**（Cargo AGV 入口卸載）|

## 🏭 業務場景

### 前置條件
1. 料架（Rack）有待處理的載具
2. 入口傳送箱（Equipment 201）有空位（至少4格）
3. KUKA Fleet Manager 服務正常運行
4. Cargo AGV 處於空閒或可接受任務狀態

### 觸發條件
- **料架有待入料的載具**（status: 待入料，至少4個）
- **入口傳送箱有4格空位**（Equipment 201, Port 1-4）
- **沒有重複的未完成任務**

### 執行結果
- 創建 Cargo AGV 入口卸載任務
- 任務進入待分派隊列（status_id = 1 PENDING）
- KUKA Fleet Manager 調度 KUKA AGV 執行
- 載具從料架移至入口傳送箱
- 等待 Loader AGV 取料開始前段制程

## 🔧 技術規格

### 料架（Rack）配置

**物理結構**：
- 料架為移動式儲存單元
- 每個料架可容納多個載具
- 支援 KUKA AGV 舉起搬運

**Work ID 配置**：
- `2000102`: Cargo AGV 入口卸載（Rack → 入口傳送箱）

### 入口傳送箱配置

**物理結構**：
- Equipment 201（入口傳送箱 BOX_IN_TRANSFER）
- 4個 Port（Port 1-4）
- **Station 01**: Port 1-2-3-4（**批量4格**）

**批量處理**：
- ✅ **統一4格批量處理**
- ✅ **與 Loader/Unloader 批量一致**
- ✅ **一次卸載4個載具**

### KUKA Fleet API 整合

```python
# Work ID 2000102 參數配置
{
    "id": 2000102,
    "name": "CargoAGV放入口傳送箱",
    "description": "從料架拿carrier到入口傳送箱放",
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
4. 移動到入口傳送箱位置
5. 對接傳送箱
6. 卸載4個載具（Port 1-4）
7. 分離料架
8. 旋轉料架 180 度（恢復原方向）
9. 放下料架（Drop Rack）
10. 返回待命位置

### 批量處理邏輯

**統一4格批量處理**：
- **一次卸載4個載具**（統一批量處理）
- **需要檢查料架至少有4個待入料載具**
- **需要檢查入口傳送箱至少有4格空位**（Port 1-4 全部空閒）
- **不支持部分卸載**（要么卸載4格，要么不卸載）

### Cargo AGV 狀態機流程

**入口卸載狀態機** (`entrance/` 目錄）：

```
check_rack_side_state           # 1. 檢查料架側邊
    ↓
select_rack_port_state          # 2. 選擇料架端口（選擇4個Port）
    ↓
take_rack_port_state            # 3. 從料架取出載具（批量4個）
    ↓
rack_vision_position_state      # 4. 料架視覺定位
    ↓
wait_rotation_state             # 5. 等待料架旋轉（180度）
    ↓
transfer_check_empty_state      # 6. 檢查傳送箱空位（確認4格空位）
    ↓
transfer_vision_position_state  # 7. 傳送箱視覺定位
    ↓
put_tranfer_state              # 8. 放入傳送箱（批量4個）
```

## 📝 TAFL Flow 設計

### Metadata
```yaml
metadata:
  id: "cargo_entrance_unload"
  name: "Cargo AGV 入口卸載"
  enabled: true
  version: "1.0.0"
  description: "檢查料架待入料載具和入口傳送箱空位，創建 Cargo AGV 入口卸載任務"
  author: "TAFL System"
  created_at: "2025-10-16"
  tags: ["cargo", "entrance", "unload", "automation", "kuka_fleet"]
```

### Settings
```yaml
settings:
  execution_interval: 15  # 每15秒執行一次
```

### Variables
```yaml
variables:
  priority: 50                    # 最高優先級（啟動整體制程）
  work_id: 2000102                # 固定 Work ID
  entrance_equipment_id: 201      # 入口傳送箱 Equipment ID
  ports: [1, 2, 3, 4]            # 入口傳送箱 Port 1-4
  batch_size: 4                   # 批量4格
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
      # 處理該房間的料架和入口傳送箱
```

#### 3. 查詢料架上待入料載具
```yaml
- query:
    target: carriers
    where:
      room_id: "${room.id}"
      location_type: "rack"           # 在料架上
      status: "待入料"                 # 待入料狀態
    as: rack_carriers
    description: "查詢料架上待入料的載具"
```

#### 4. 檢查載具數量（批量4格）
```yaml
- set:
    carrier_count: "${rack_carriers.length}"
    required_count: 4                 # 固定4格批量
    has_enough_carriers: "${carrier_count >= required_count}"
```

#### 5. 查詢入口傳送箱空位
```yaml
- query:
    target: equipment_ports
    where:
      room_id: "${room.id}"
      equipment_id: "${entrance_equipment_id}"  # 201
      port_in: "${ports}"                       # [1, 2, 3, 4]
      status: "empty"
    as: empty_ports
    description: "查詢入口傳送箱空位（Port 1-4）"
```

#### 6. 檢查空位數量（批量4格）
```yaml
- set:
    empty_count: "${empty_ports.length}"
    required_space: 4                # 固定4格批量
    has_space: "${empty_count >= required_space}"
```

#### 7. 檢查重複任務
```yaml
- query:
    target: tasks
    where:
      work_id: "${work_id}"           # 2000102
      room_id: "${room.id}"
      status_id_in: [0, 1, 2, 3]     # 未完成的狀態
    as: existing_tasks
    description: "檢查是否已存在入口卸載任務"
```

#### 8. 創建入口卸載任務
```yaml
- if:
    condition: "${has_enough_carriers} && ${has_space} && ${existing_tasks.length == 0}"
    then:
      - create:
          target: task
          with:
            type: "cargo_entrance_unload"
            name: "房間${room.id} Cargo入口卸載"
            description: "從料架卸載${carrier_count}個載具到入口傳送箱（批量4格）"
            work_id: "${work_id}"      # 2000102
            room_id: "${room.id}"
            priority: "${priority}"
            status_id: 1               # PENDING
            parameters:
              work_id: "${work_id}"
              room_id: "${room.id}"
              equipment_id: "${entrance_equipment_id}"
              ports: "${ports}"
              batch_size: "${batch_size}"
              carrier_count: "${carrier_count}"
              function: "${function}"        # rack_move
              api: "${api}"                  # submit_mission
              missionType: "${missionType}"  # RACK_MOVE
              nodes: []                      # 動態生成
              reason: "料架有待入料載具，入口傳送箱有空位"
          description: "創建 Cargo AGV 入口卸載任務（KUKA Fleet API）"
```

## 🔍 查詢條件詳解

### Carrier 查詢條件

**料架載具條件**：
- `room_id`: 特定房間
- `location_type`: "rack"（在料架上）
- `status`: "待入料"

**數量要求**（固定4格批量）：
- **至少4個載具**（一次卸載4格）
- 不支持部分卸載

### Equipment Port 查詢條件

**入口傳送箱空位**（固定配置）：
- `room_id`: 特定房間
- `equipment_id: 201` (固定的入口傳送箱設備ID)
- `port_in: [1, 2, 3, 4]` (固定 Port 映射)
- `status: "empty"` (空位)

**空位要求**（固定4格批量）：
- **至少4格空位**（一次卸載4格，Port 1-4 全部空閒）

### Task 重複檢查

**防止重複創建**：
- 檢查相同 `work_id` 和 `room_id`
- 狀態為未完成（0=創建, 1=待分派, 2=執行中, 3=暫停）
- 如果存在未完成任務，不創建新任務

## ⚠️ 注意事項

### KUKA Fleet 整合重點
- **KUKA Fleet Manager 依賴**: 必須確保 KUKA Fleet Manager 服務運行正常
- **rack_move API**: 專門的料架搬運功能，自動處理舉起、旋轉、放下
- **動態路徑**: nodes 參數由系統動態生成，根據料架和傳送箱位置計算
- **料架旋轉**: KUKA AGV 自動處理180度旋轉，對接傳送箱

### 批量處理邏輯
- **固定4格批量**: 必須有 ≥ 4個待入料載具，入口傳送箱需 ≥ 4格空位
- **不支持部分卸載**（要么卸載4格，要么不卸載）
- **批量一致性**: 與 Loader/Unloader 保持統一4格批量

### 狀態碼映射
Carrier 狀態定義：
- `待入料`: 在料架上，等待 Cargo AGV 卸載到入口傳送箱
- `201`: 在入口傳送箱（Equipment 201），等待 Loader AGV 取料

### 設備配置
- **固定 equipment_id**: 201（入口傳送箱 BOX_IN_TRANSFER）
- **固定 Work ID**: 2000102
- **固定 Port**: 1-4（批量4格）

## 🧪 測試要點

### 單元測試
1. ✅ 查詢邏輯正確性
   - 正確查詢料架上待入料載具
   - 正確查詢入口傳送箱空位
   - 正確過濾房間和端口

2. ✅ 數量判斷（固定4格批量）
   - 載具數量 >= 4
   - 空位數量 >= 4

3. ✅ 重複檢查
   - 存在未完成任務時不創建
   - 任務參數正確傳遞

4. ✅ KUKA Fleet 參數
   - 正確使用 function: "rack_move"
   - 正確使用 missionType: "RACK_MOVE"
   - 正確使用 api: "submit_mission"

### 整合測試
1. ✅ 與 KUKA Fleet Manager 銜接
   - 任務正確創建
   - KUKA Fleet Manager 接收任務
   - KUKA AGV 執行 rack_move

2. ✅ 與 Loader AGV 銜接
   - Cargo 卸載完成
   - 載具狀態更新為在入口傳送箱
   - Loader AGV TAKE_BOXIN_TRANSFER 檢測到並創建任務

3. ✅ 完整流程
   - Cargo AGV 執行入口卸載
   - 料架旋轉180度對接傳送箱
   - 卸載4個載具到入口傳送箱
   - 料架恢復原方向並返回
   - 載具狀態更新
   - 任務狀態更新為完成

### 邊界測試
1. ✅ 料架只有3個待入料載具
   - 不創建任務（需要至少4個）

2. ✅ 入口傳送箱只有3格空位
   - 不創建任務（需要至少4格）

3. ✅ 同時有多個房間料架就緒
   - 正確為每個房間創建任務
   - 不產生衝突

## 📊 成功指標

### 功能指標
- ✅ 正確檢測料架待入料載具
- ✅ 正確檢測入口傳送箱空位
- ✅ 正確創建入口卸載任務
- ✅ 不產生重複任務
- ✅ 任務參數完整準確
- ✅ KUKA Fleet 參數正確

### 性能指標
- 執行間隔：15秒
- 響應時間：< 2秒
- 任務創建延遲：< 30秒（從載具就緒到任務創建）
- KUKA AGV 執行時間：< 5分鐘（從接收任務到完成卸載）

### 可靠性指標
- 零漏檢：所有就緒載具都能被檢測
- 零誤創建：不創建重複或錯誤任務
- KUKA Fleet 成功率：> 99%
- 容錯性：KUKA Fleet 失敗時優雅降級

## 🔗 相關文檔

- **Cargo AGV 代碼**: `/app/agv_ws/src/cargo_mover_agv/cargo_mover_agv/robot_states/entrance/`
- **KUKA Fleet Manager**: `/app/kuka_fleet_ws/src/kuka_fleet/kuka_fleet/kuka_fleet_manager.py`
- **Work ID 定義**: 資料庫 `agvc.work` 表
- **後續流程**: Loader AGV - `TAKE_BOXIN_TRANSFER`
