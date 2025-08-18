# 架台旋轉邏輯和 Task Parameters 資料流

## 🎯 適用場景
- 理解房間入口架台旋轉的觸發條件
- 追蹤 Task.parameters 的資料來源和更新流程
- KUKA Fleet 回調與 Task 參數的整合機制
- 架台旋轉任務的完整生命週期

## 📋 房間入口架台旋轉條件

### 核心觸發條件
房間入口的架台旋轉需要**同時滿足**以下條件：

1. **A面為空** (`a_side_empty = true`)
   - 檢查 A 面沒有載具（載具都被取走了）
   - 程式碼：`check_type='empty'` 返回 `len(side_carriers) == 0`
   - rack_index 範圍：1-16（S尺寸）或 1-8（L尺寸）

2. **B面有待作業載具** (`b_side_has_work = true`)
   - 檢查 B 面有需要處理的載具
   - 程式碼：`check_type='has_work'` 返回 `any(c.status_id != 8 for c in side_carriers)`
   - rack_index 範圍：17-32（S尺寸）或 9-16（L尺寸）

3. **沒有重複任務** (`!task_exists`)
   - 確保該位置和架台沒有未完成的旋轉任務

### 重要澄清
⚠️ **注意**：`check_type='all_complete'` 的命名可能誤導
- 實際檢查的是「該面是否為空」，而非「所有載具完成作業」
- 建議使用 `check_type='empty'` 以避免誤解

### 業務邏輯說明
**簡單來說**：當前面（A面）的載具都被取完了，但後面（B面）還有載具要處理，就把架台轉180度，讓B面朝前方便取用。

## 🔄 Task Parameters 資料流

### 1. 參數的初始來源（任務創建時）

#### 資料流程
```
Flow 配置檔案 (rack_rotation_room_inlet.yaml)
    ↓
定義 metadata 參數
    ↓
執行 task.create_task (database.py)
    ↓
合併 Work.parameters + metadata
    ↓
存入 Task.parameters
```

#### 初始參數內容
```json
{
  "rack_name": "Rack_101",           // 架台識別名稱
  "location_name": "Room1_Inlet",    // 位置描述名稱
  "room_id": 1,                      // 房間 ID
  "rotation_angle": 180,             // 旋轉角度
  "reason": "A面為空（載具已取走），B面待作業",
  "model": "KUKA400i",               // 指定 AGV 型號
  "nodes": [10001, 10002, 10001]    // [起點, 轉向點, 終點]
}
```

### 2. 參數的更新流程（KUKA 執行時）

#### 更新流程
```
KUKA Fleet Manager 執行任務
    ↓
發送狀態回調到 /interfaces/api/amr/missionStateCallback
    ↓
web_api/routers/kuka.py 接收回調
    ↓
根據 mission_code 查找 Task
    ↓
更新 Task.parameters（保留原有 + 新增 KUKA 狀態）
    ↓
存回資料庫
```

#### KUKA 回調新增的參數
```json
{
  // 保留所有原始參數...
  
  // KUKA 回調新增的參數（kuka_ 前綴）
  "kuka_mission_status": "COMPLETED",      // 任務執行狀態
  "kuka_robot_id": "kuka01",              // 執行的機器人 ID
  "kuka_container_code": "C001",          // 容器代碼
  "kuka_current_position": "10001",       // 當前位置節點
  "kuka_slot_code": "S01",                // 槽位代碼
  "kuka_view_board_type": "rotation",     // 任務類型
  "kuka_message": "Rotation completed",    // 狀態訊息
  "kuka_mission_data": {...},             // 任務自訂資料
  "kuka_last_update": "2025-08-14T10:30:00Z"  // 最後更新時間
}
```

### 3. 參數更新策略

#### 重要特性
1. **累積式更新**：保留原有參數，新增 KUKA 狀態資訊
2. **前綴區分**：KUKA 參數使用 `kuka_` 前綴，避免覆蓋原始配置
3. **狀態分離**：parameters 更新不影響 Task.status_id（由 WCS 統一管理）
4. **歷史追蹤**：累積所有狀態變化，提供完整執行歷史

## 📊 完整生命週期時序

### 任務執行時序圖
```
時間點 1: Flow WCS 創建任務
├─ 檢測到 A 面為空、B 面有待作業載具
├─ 創建 RACK_ROTATION 任務（work_id: 220001）
└─ 初始 parameters：架台資訊、導航節點等

時間點 2: KUKA 開始執行
├─ 接收任務指派
├─ 回調 MOVE_BEGIN 狀態
└─ parameters += {kuka_mission_status: "MOVE_BEGIN", ...}

時間點 3: 到達轉向點
├─ AGV 移動到 node_id + 1
├─ 回調 ARRIVED 狀態
└─ parameters += {kuka_mission_status: "ARRIVED", ...}

時間點 4: 執行旋轉
├─ 架台旋轉 180 度
├─ 回調執行中狀態
└─ parameters 持續更新

時間點 5: 任務完成
├─ 返回起始位置
├─ 回調 COMPLETED 狀態
└─ parameters += {kuka_mission_status: "COMPLETED", ...}
```

## 🔧 實作細節

### database.py 中的參數合併邏輯
```python
def create_task(self, ...metadata: Dict = None):
    # 參數合併策略
    task_parameters = {}
    
    # 1. 載入 Work 的預設參數
    if work.parameters:
        task_parameters.update(work.parameters)
    
    # 2. 覆蓋或新增 metadata 參數
    if metadata:
        task_parameters.update(metadata)
    
    # 3. 創建任務時存入合併後的參數
    task = Task(
        parameters=task_parameters,
        ...
    )
```

### kuka.py 中的參數更新邏輯
```python
def mission_state_callback(data: MissionStateCallbackData):
    # 保留原有參數
    current_params = existing_task.parameters or {}
    
    # 新增 KUKA 狀態資訊
    kuka_status_info = {
        "kuka_mission_status": data.missionStatus,
        "kuka_robot_id": data.robotId,
        # ... 其他 KUKA 狀態
    }
    
    # 合併更新（累積式）
    current_params.update(kuka_status_info)
    existing_task.parameters = dict(current_params)
```

## 💡 設計優勢

### 參數管理的優點
1. **完整性**：保存任務的完整配置和執行歷史
2. **可追溯**：每個階段的狀態都有記錄
3. **靈活性**：不同來源的參數可以共存
4. **擴展性**：易於新增新的參數來源

### 實際應用價值
- **故障排查**：通過 parameters 追蹤任務執行過程
- **效能分析**：分析各階段的時間戳和狀態
- **業務審計**：完整的任務執行記錄
- **系統優化**：基於歷史資料優化流程

## 🔗 相關文檔
- Flow WCS 系統架構：@docs-ai/knowledge/system/flow-wcs-system.md
- KUKA Fleet API：@docs-ai/knowledge/protocols/kuka-fleet-api.md
- KUKA Fleet 回調：@docs-ai/knowledge/protocols/kuka-fleet-callback.md
- Linear Flow 進階功能：@docs-ai/knowledge/system/linear-flow-advanced-features.md
- WCS 資料庫設計：@docs-ai/knowledge/agv-domain/wcs-database-design.md