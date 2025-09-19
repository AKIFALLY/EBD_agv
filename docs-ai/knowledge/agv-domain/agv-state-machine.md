# AGV 狀態機架構與流程

## 🎯 適用場景
- 理解 RosAGV 3層狀態機架構的設計原理和實作細節
- 為 AGV 狀態機開發和除錯提供完整的狀態轉換指導
- 解決 AGV 狀態機相關的故障排除和問題診斷

## 📋 3層狀態機架構概述

RosAGV 採用 3層狀態機架構，提供模組化、可擴展的 AGV 控制系統：

```
Layer 3: Robot Layer (機械臂控制層)
├── Robot 控制狀態
├── PGNO 指令系統
└── 機械臂任務執行

Layer 2: AGV Layer (任務執行層)  
├── MissionSelectState (任務選擇)
├── WritePathState (路徑規劃)
├── RunningState (路徑執行)
└── WaitRobotState (等待機械臂)

Layer 1: Base Layer (基礎控制層)
├── IdleState (閒置狀態)
├── AutoState (自動模式)
├── ManualState (手動模式)
└── ErrorState (錯誤狀態)
```

### 架構設計原理
- **分層責任**: 每層專注於特定控制範疇，降低系統複雜度
- **狀態繼承**: 上層狀態繼承下層狀態的基礎功能
- **事件驅動**: 基於事件的狀態轉換機制，確保狀態同步
- **車型特化**: 每種車型可自定義特定狀態實作

## 🏗️ Base Layer (基礎控制層) 狀態詳解

### IdleState (閒置狀態)
**檔案位置**: `app/agv_ws/src/agv_base/agv_base/states/idle_state.py`

#### 進入條件 (Entry Conditions)
- AGV 系統初始化完成
- 從其他狀態返回到待機狀態
- 緊急停止後恢復正常

#### 核心邏輯
```python
# TAG 坐標處理邏輯
def handle(self, context):
    if self.node.agv_status.AGV_FPGV > 0:
        # 更新當前位置
        if self.node.AGV_id == 1:
            self.agvdbclient.async_update_agv_position(...)
        
        # 檢查模式切換條件
        if context.auto_mode_enabled:
            context.set_state(AutoState(self.node))
```

#### 離開條件 (Exit Conditions)
- `context.auto_mode_enabled = True` → 切換到 AutoState
- `context.manual_mode_enabled = True` → 切換到 ManualState  
- 系統錯誤發生 → 切換到 ErrorState

#### 關鍵參數
- `AGV_FPGV`: 當前 TAG 坐標位置
- `AGV_id`: AGV 識別碼
- `auto_mode_enabled`: 自動模式啟用標誌

### AutoState (自動模式)
**檔案位置**: `app/agv_ws/src/agv_base/agv_base/states/auto_state.py`

#### 進入條件 (Entry Conditions)
- 從 IdleState 收到自動模式啟用信號
- 系統處於正常運行狀態
- 無緊急停止或錯誤狀態

#### 核心邏輯
```python
def handle(self, context):
    # 檢查 AGV 層狀態機啟用
    if context.agv_layer_enabled:
        from agv_base.agv_states.mission_select_state import MissionSelectState
        context.set_state(MissionSelectState(self.node))
```

#### 離開條件 (Exit Conditions)
- `context.agv_layer_enabled = True` → 啟動 AGV Layer 狀態機
- `context.manual_mode_requested = True` → 切換到 ManualState
- 系統錯誤 → 切換到 ErrorState

#### 狀態轉換邏輯
- **向上轉換**: 啟動 AGV Layer 的 MissionSelectState
- **橫向轉換**: 可切換到 ManualState 或 ErrorState
- **向下轉換**: 可返回 IdleState

### ManualState (手動模式)
**檔案位置**: `app/agv_ws/src/agv_base/agv_base/states/manual_state.py`

#### 進入條件 (Entry Conditions)
- 操作員選擇手動控制模式
- 從自動模式切換到手動干預
- 系統維護或調試需求

#### 核心邏輯
```python
def handle(self, context):
    # 手動控制邏輯處理
    if context.manual_command_received:
        self.process_manual_command(context.manual_command)
    
    # 檢查返回自動模式
    if context.auto_mode_requested:
        context.set_state(AutoState(self.node))
```

#### 離開條件 (Exit Conditions)
- `context.auto_mode_requested = True` → 切換到 AutoState
- 手動操作完成 → 返回 IdleState
- 系統錯誤 → 切換到 ErrorState

### ErrorState (錯誤狀態)
**檔案位置**: `app/agv_ws/src/agv_base/agv_base/states/error_state.py`

#### 進入條件 (Entry Conditions)
- 系統偵測到硬體故障
- PLC 通訊中斷或異常
- 感測器資料異常
- 緊急停止觸發

#### 核心邏輯
```python
def handle(self, context):
    # 錯誤診斷和處理
    if self.diagnose_system_error():
        self.log_error_details()
        self.attempt_recovery()
    
    # 檢查恢復條件
    if context.error_cleared:
        context.set_state(IdleState(self.node))
```

#### 離開條件 (Exit Conditions)
- 錯誤修復完成 → 返回 IdleState
- 系統重置 → 返回 IdleState

## 🎯 AGV Layer (任務執行層) 狀態詳解

### MissionSelectState (任務選擇狀態)
**檔案位置**: `app/agv_ws/src/agv_base/agv_base/agv_states/mission_select_state.py`

#### 進入條件 (Entry Conditions)
- Base Layer 的 AutoState 啟動 AGV Layer
- AGV 完成前一個任務，準備接收新任務
- 系統初始化後進入任務待機狀態

#### 核心邏輯
```python
def handle(self, context):
    # 檢查現有路徑
    if self.node.agv_status.AGV_PATH:
        context.set_state(RunningState(self.node))
        return
    
    # 任務選擇邏輯
    if self.highest_priority_task:
        context.set_state(WritePathState(self.node))
    
    # HMI 本地任務檢查
    if self.localMission and not self.node.agv_status.AGV_PATH:
        context.set_state(WritePathState(self.node))
```

#### 任務篩選條件
- 狀態為 `PENDING` 的未執行任務
- `work_id` 範圍在 2000-3000
- `agv_id = 0` (未分配) 且符合 `room_id`
- 或已分配給當前 AGV 的 `READY_TO_EXECUTE` 任務

#### 離開條件 (Exit Conditions)
- 找到高優先級任務 → 切換到 WritePathState
- AGV 已有路徑資料 → 切換到 RunningState  
- HMI 本地任務啟動 → 切換到 WritePathState
- 超過 30 次循環無任務 → 重置計數器

#### 關鍵參數
- `task_list`: 任務清單
- `highest_priority_task`: 最高優先級任務
- `localMission`: HMI 本地任務標誌
- `MAGIC`: HMI 任務觸發值
- `AGV_END_POINT`: 目標端點

### WritePathState (路徑規劃狀態)
**檔案位置**: `app/agv_ws/src/agv_base/agv_base/agv_states/write_path_state.py`

#### 進入條件 (Entry Conditions)
- MissionSelectState 選擇了待執行任務
- HMI 設定了有效的目標點和 MAGIC 值
- 需要重新計算路徑的情況

#### 核心邏輯
```python
def handle(self, context):
    # 檢查既有路徑
    if self.node.agv_status.AGV_PATH:
        context.set_state(RunningState(self.node))
        return
    
    # A* 路徑計算
    if not self.node.agv_status.AGV_PATH and self.step == 0:
        self.As = AStarAlgorithm(self.TagNo, self.node.node_id)
        self.path = self.As.run()
        
        # 路徑資料寫入 PLC
        self.plc_client.async_write_continuous_data(...)
        
        # 更新任務狀態為執行中
        self.node.task.status_id = 3
        self.agvdbclient.async_update_task(...)
```

#### A* 演算法整合
- **起點**: `AGV_FPGV` (當前 TAG 位置)
- **終點**: `node_id` (目標節點)
- **路徑資料**: 包含 TAG 坐標、動作、速度、偏移等
- **PLC 寫入**: 分兩段寫入 DM3000 和 DM4000

#### 離開條件 (Exit Conditions)
- 路徑計算和 PLC 寫入成功 → 切換到 RunningState
- 寫入失敗超過 5 次 → 返回 MissionSelectState 並標記異常
- AGV 已有路徑資料 → 直接切換到 RunningState

#### 關鍵參數
- `path`: A* 計算的路徑陣列
- `dataValue`: 2000 長度的 PLC 資料陣列
- `count`: 寫入嘗試次數計數器
- `step`: 步驟計數器

### RunningState (路徑執行狀態)
**檔案位置**: `app/agv_ws/src/agv_base/agv_base/agv_states/Running_state.py`

#### 進入條件 (Entry Conditions)
- WritePathState 完成路徑規劃和 PLC 寫入
- MissionSelectState 檢測到既有路徑資料
- AGV 準備開始執行移動任務

#### 核心邏輯
```python
def handle(self, context):
    # 檢查路徑執行狀態
    if self.node.agv_status.AGV_PATH:
        # 路徑執行中，監控狀態
        if self.check_path_completion():
            context.set_state(WaitRobotState(self.node))
    else:
        # 路徑執行完成或異常
        self.handle_path_completion(context)
```

#### 路徑監控
- **AGV_PATH**: 路徑執行狀態監控
- **位置回饋**: 實時 TAG 位置更新
- **異常檢測**: 路徑偏離或執行異常
- **完成條件**: 到達目標位置

#### 離開條件 (Exit Conditions)
- 路徑執行完成 → 切換到 WaitRobotState
- 路徑執行異常 → 返回 MissionSelectState 或 ErrorState
- 緊急停止 → 切換到 ErrorState

### WaitRobotState (等待機械臂狀態)
**檔案位置**: `app/agv_ws/src/agv_base/agv_base/agv_states/wait_robot_state.py`

#### 進入條件 (Entry Conditions)
- RunningState 完成路徑執行
- AGV 到達目標位置，準備執行機械臂任務
- Robot Layer 準備開始作業

#### 核心邏輯
```python
def handle(self, context):
    # Robot 層狀態機處理
    if context.robot_layer_enabled:
        # 監控 Robot 任務執行
        if self.check_robot_task_completion():
            # 任務完成，更新狀態並返回
            self.update_task_status_completed()
            context.set_state(MissionSelectState(self.node))
```

#### Robot 層整合
- **Robot 啟動**: 啟動 Robot Layer 狀態機
- **任務監控**: 監控機械臂任務執行狀態
- **完成檢測**: 檢測機械臂任務完成
- **狀態同步**: Robot 層與 AGV 層狀態同步

#### 離開條件 (Exit Conditions)
- Robot 任務完成 → 返回 MissionSelectState
- Robot 任務失敗 → 根據錯誤類型處理
- 超時無回應 → 錯誤處理或重試

## 🔧 狀態轉換流程圖

### 完整狀態轉換流程
```
系統啟動
    ↓
IdleState (Base Layer)
    ↓ (auto_mode_enabled)
AutoState (Base Layer)
    ↓ (agv_layer_enabled)
MissionSelectState (AGV Layer)
    ↓ (task_selected)
WritePathState (AGV Layer)
    ↓ (path_written)
RunningState (AGV Layer)
    ↓ (path_completed)
WaitRobotState (AGV Layer)
    ↓ (robot_task_completed)
MissionSelectState (AGV Layer) ← 循環
```

### 異常狀態轉換
```
任何狀態 → ErrorState (緊急狀止/系統錯誤)
    ↓ (error_cleared)
IdleState → 重新開始流程
```

### 手動干預流程
```
AutoState ↔ ManualState (手動切換)
    ↓ (manual_completed)
IdleState → 返回正常流程
```

## 🛠️ 車型特化實作

### Cargo Mover AGV 特化
- **光通訊整合**: Hokuyo 8bit 光通訊模組
- **架台搬運**: ENTRANCE/EXIT 流程特化
- **PLC 通訊**: 特定的 PGNO 指令集

### Loader AGV 特化
- **多工位支援**: Transfer, Cleaner, Soaker, Pre-dryer
- **端口管理**: 動態端口狀態管理
- **完整測試**: 具備完整的測試套件

### Unloader AGV 特化
- **後段工位**: Pre-dryer, Oven, Boxout Transfer
- **分揀邏輯**: 智能分揀和處理邏輯
- **基礎架構**: 40% 實作，架構完整

## 📊 狀態機效能特性

### 執行週期
- **主循環**: 50ms 週期執行
- **狀態檢查**: 每次循環檢查狀態轉換條件
- **事件響應**: 即時響應外部事件

### 記憶體使用
- **狀態物件**: 輕量級狀態物件設計
- **上下文管理**: 統一的 Context 生命週期管理
- **資源清理**: 狀態切換時自動資源清理

### 錯誤處理
- **異常捕獲**: 完整的異常處理機制
- **狀態恢復**: 自動狀態恢復和重試機制
- **日誌記錄**: 詳細的狀態轉換日誌

## 🔍 除錯和診斷

### 狀態機除錯
```bash
# 檢查當前狀態
ros2 topic echo /agv_status

# 檢查狀態轉換事件
ros2 topic echo /agv_events

# 檢查任務狀態
ros2 service call /get_task_status
```

### 日誌分析
```bash
# 狀態機日誌
tail -f /tmp/agv.log | rg "狀態|State"

# 任務執行日誌
tail -f /tmp/agv.log | rg "任務|Mission"
```

### 常見問題診斷
1. **狀態卡住**: 檢查狀態轉換條件和事件處理
2. **任務選擇失敗**: 檢查任務篩選條件和資料庫連接
3. **路徑規劃失敗**: 檢查 A* 演算法和 PLC 通訊
4. **Robot 任務異常**: 檢查 Robot 層狀態和 PGNO 系統

## 🔗 交叉引用
- AGV 工作空間: docs-ai/context/workspaces/agv-workspaces.md
- 車型特性: docs-ai/knowledge/agv-domain/vehicle-types.md
- PLC 通訊: docs-ai/knowledge/protocols/keyence-plc-protocol.md
- 路徑規劃: `app/path_algorithm/CLAUDE.md`
- Robot 控制: `app/agv_ws/src/agv_base/agv_base/robot.py`