# Cargo State vs Base State 完整流程差異分析

## 📊 整體架構對比

### 基礎狀態機（agv_base）
```
MissionSelectState
    └─> WritePathState
            └─> RunningState
                    └─> WaitRobotState
```

### Cargo 狀態機（cargo_mover_agv）
```
CargoMissionSelectState (繼承 + 攔截)
    └─> CargoWritePathState (繼承 + 攔截)
            └─> CargoRunningState (繼承 + 交管)
                    └─> WaitRobotState (共用基礎版本)
```

---

## 🔍 逐一狀態對比

### 1️⃣ MissionSelectState vs CargoMissionSelectState

#### 基礎 MissionSelectState (agv_base)
**檔案**: `agv_base/agv_states/mission_select_state.py`

**核心邏輯**:
```python
def handle(self, context):
    # 1. 檢查任務列表
    if self.node.latest_tasks:
        for task in self.node.latest_tasks:
            if task.agv_id == self.node.agv_id:
                if task.status_id == 3 and task.work_id != 0:
                    # EXECUTING 狀態任務
                    if MISSION_CANCEL == 1:
                        # 重新規劃路徑
                        context.set_state(WritePathState(self.node))
                    else:
                        # 進入 WaitRobot
                        context.set_state(WaitRobotState(self.node))
                else:
                    # status=1,2 正常寫路徑
                    context.set_state(WritePathState(self.node))

    # 2. Local 模式（MAGIC + AGV_END_POINT）
    elif self.localMission and not AGV_PATH:
        context.set_state(WritePathState(self.node))
```

**特點**:
- ✅ 純粹的任務選擇邏輯
- ✅ 支援 MISSION_CANCEL 重新規劃
- ❌ 沒有交管考量
- ❌ 直接創建基礎 WritePathState

---

#### Cargo MissionSelectState (cargo_mover_agv)
**檔案**: `cargo_mover_agv/states/cargo_mission_select_state.py`

**核心邏輯**:
```python
def handle(self, context):
    # 【新增】檢查是否應在無任務時進入 RunningState
    if self._should_enter_running_without_task():
        # Local 模式 (MAGIC=21) 有路徑但無任務
        context.set_state(CargoRunningState(self.node))
        return

    # 【攔截機制】使用猴子補丁攔截 set_state
    original_set_state = context.set_state

    def patched_set_state(new_state):
        # 攔截 WritePathState → CargoWritePathState
        if isinstance(new_state, WritePathState):
            日誌：[Cargo] 🔄 攔截狀態轉換：WritePathState → CargoWritePathState
            original_set_state(CargoWritePathState(self.node))
        # 攔截 RunningState → CargoRunningState
        elif isinstance(new_state, RunningState):
            日誌：[Cargo] 🔄 攔截狀態轉換：RunningState → CargoRunningState
            original_set_state(CargoRunningState(self.node))
        else:
            original_set_state(new_state)

    context.set_state = patched_set_state

    try:
        # 呼叫父類邏輯
        super().handle(context)
    finally:
        context.set_state = original_set_state

def _should_enter_running_without_task(self) -> bool:
    # Local 模式特殊處理：有路徑 + LOCAL=ON + MAGIC=21 + 無任務
    return (AGV_PATH and AGV_LOCAL and MAGIC==21 and no_valid_task)
```

**差異總結**:

| 功能 | Base | Cargo | 說明 |
|------|------|-------|------|
| 任務選擇邏輯 | ✅ | ✅ | 繼承基礎邏輯 |
| MISSION_CANCEL | ✅ | ✅ | 繼承支援 |
| Local 模式 | ✅ | ✅ 強化 | Cargo 支援無任務進 Running |
| 狀態攔截機制 | ❌ | ✅ | 猴子補丁攔截 |
| 交管考量 | ❌ | ✅ 間接 | 透過 CargoRunningState |

---

### 2️⃣ WritePathState vs CargoWritePathState

#### 基礎 WritePathState (agv_base)
**檔案**: `agv_base/agv_states/write_path_state.py`

**核心邏輯**:
```python
def handle(self, context):
    # 【檢查1】是否已有路徑
    if self.node.agv_status.AGV_PATH:
        日誌："AGV 已有路徑資料，離開 WritePathState-->RunningState"
        context.set_state(RunningState(self.node))  # ❌ 創建基礎版本
        return

    # 【檢查2】寫入次數
    if self.count > 5:
        日誌："路徑資料寫入失敗過多"
        plc_client.async_force_on('MR', '3204')  # 寫入異常
        context.set_state(MissionSelectState(self.node))

    # 【主流程】無路徑時計算並寫入
    if not AGV_PATH and step == 0 and not path_calculated:
        # 1. A* 演算法計算路徑
        self.As = AStarAlgorithm(TagNo, node_id)
        self.path = self.As.run()

        # 2. 組裝 dataValue[] (2000 筆資料)
        for i in range(len(self.path)):
            dataValue[i*20] = TagNo
            dataValue[i*20+1] = PGV
            dataValue[i*20+2] = ACT/Station
            # ... (完整 20 個參數)

        # 3. 更新任務狀態
        if MAGIC != 21:
            task.status_id = 3  # EXECUTING
            async_update_task(task, callback)

        # 4. 寫入 PLC
        async_write_continuous_data('DM', '3000', dataValue[0:1000])
        async_write_continuous_data('DM', '4000', dataValue[1000:2000])
```

**特點**:
- ✅ 完整的路徑計算邏輯
- ✅ PLC 資料寫入（DM3000/DM4000）
- ✅ 任務狀態更新（status_id=3）
- ✅ MAGIC=21 特殊處理
- ❌ 直接創建 RunningState（無交管）

---

#### Cargo WritePathState (cargo_mover_agv) - 修改後
**檔案**: `cargo_mover_agv/states/cargo_write_path_state.py`

**核心邏輯**:
```python
def handle(self, context):
    # 【新增】優先檢查：直接創建 CargoRunningState
    if self.node.agv_status.AGV_PATH:
        日誌："[Cargo] ✅ AGV 已有路徑資料，離開 WritePathState → CargoRunningState"
        context.set_state(CargoRunningState(self.node))  # ✅ 創建 Cargo 版本
        return  # 提早返回

    # 【備用機制】猴子補丁攔截
    original_set_state = context.set_state

    def patched_set_state(new_state):
        if isinstance(new_state, RunningState) and not isinstance(new_state, CargoRunningState):
            日誌："[Cargo] 🔄 攔截狀態轉換：RunningState → CargoRunningState"
            original_set_state(CargoRunningState(self.node))
        else:
            original_set_state(new_state)

    context.set_state = patched_set_state

    try:
        # 呼叫父類邏輯（路徑計算、PLC 寫入等）
        super().handle(context)
    finally:
        context.set_state = original_set_state
```

**差異總結**:

| 功能 | Base | Cargo (修改前) | Cargo (修改後) |
|------|------|----------------|----------------|
| 路徑計算 | ✅ | ✅ 繼承 | ✅ 繼承 |
| PLC 寫入 | ✅ | ✅ 繼承 | ✅ 繼承 |
| 任務更新 | ✅ | ✅ 繼承 | ✅ 繼承 |
| 有路徑時處理 | 創建 RunningState | 猴子補丁攔截 | **直接創建 CargoRunningState** |
| 無路徑時處理 | 創建 RunningState | 猴子補丁攔截 | 猴子補丁攔截（備用） |
| 可靠性 | N/A | ❌ 低（失效） | ✅ 高（雙重保險） |

---

### 3️⃣ RunningState vs CargoRunningState ⭐ 核心差異

#### 基礎 RunningState (agv_base)
**檔案**: `agv_base/agv_states/Running_state.py`

**核心邏輯**:
```python
class RunningState(State):
    def __init__(self, node: Node):
        super().__init__(node)
        self.agvdbclient = AGVCDatabaseClient(node)
        self.count = 0
        self.next_node = 0
        self.ask_traffic_area = []       # ⚠️ 定義但未使用
        self.traffic_area_registed = []  # ⚠️ 定義但未使用

    def enter(self):
        日誌："🏃 AGV 進入: Running 狀態"
        日誌："📡 使用全局 /agvc/tasks 訂閱"
        # ❌ 無任何交管邏輯

    def leave(self):
        日誌："🚪 AGV 離開 Running 狀態"
        # 離開前抓取當前任務
        if latest_tasks:
            for task in latest_tasks:
                if task.agv_id == self.node.agv_id and task.status_id in [2, 3]:
                    self.node.task = task
                    break

    def handle(self, context):
        # 【檢查1】沒有路徑 → WaitRobot
        if not AGV_PATH:
            日誌："⚠️ AGV 沒有路徑資料，進入 WaitRobot 狀態"
            context.set_state(WaitRobotState(self.node))

        # 【檢查2】有路徑 → 持續運行
        if AGV_PATH:
            if self.count > 100:
                日誌："🏃 AGV RunningState..."
            self.count += 1

        # 【檢查3】到達目標 → WaitRobot
        if AGV_2POSITION:
            日誌："✅ AGV 到達目標位置"
            # 抓取當前任務
            if task_found and task.id != 0:
                context.set_state(WaitRobotState(self.node))
            else:
                context.set_state(MissionSelectState(self.node))

        # 【狀態一致性檢查】每 5 秒
        if status_check_counter > 100:
            self._check_status_consistency()  # 檢查 status_id 是否為 3
```

**特點**:
- ✅ 路徑監控
- ✅ 到達檢測
- ✅ 狀態一致性檢查
- ⚠️ 有交管變數定義但未實作
- ❌ **無任何交管邏輯**

---

#### Cargo RunningState (cargo_mover_agv)
**檔案**: `cargo_mover_agv/states/cargo_running_state.py`

**核心邏輯**:
```python
class CargoRunningState(RunningState):  # 繼承基礎版本
    def __init__(self, node: Node):
        super().__init__(node)

        # 【新增】交管相關屬性
        self.traffic_client = TrafficClient(node)
        self.traffic_zone_id = 1  # room2 交管區
        self.plc_stop_triggered = False
        self.plc_client = node.plc_client

        # 【新增】交管流程控制
        self.traffic_init_step = 0  # 0:初始, 1:等待PLC, 2:等待交管, 3:完成
        self.plc_force_on_result = None
        self.traffic_request_result = None

        # 【新增】定時重試機制（重要！）
        self.traffic_retry_timer = None      # 重試計時器
        self.traffic_retry_count = 0         # 重試次數
        self.traffic_allowed = False         # 是否已獲得通行許可
        self.traffic_retry_interval = 3.0    # 重試間隔（3秒）

    def enter(self):
        """進入 RunningState 時的交管處理"""
        super().enter()  # 呼叫父類

        # ✅ 階段 1：先觸發 PLC MR7005 停止（安全優先）
        self._initial_plc_stop()

    def _initial_plc_stop(self):
        """階段 1：初始化時先停止 AGV"""
        日誌："[交管] 🚨 進入 RunningState，先觸發 PLC MR7005 停止"
        try:
            plc_client.async_force_on('MR', '7005', callback)
            self.traffic_init_step = 1  # 進入等待 PLC 狀態
            self.plc_stop_triggered = True
        except Exception as e:
            日誌：f"[交管] ❌ 初始 PLC 停止失敗: {e}"
            self._request_traffic_control()  # 失敗也繼續

    def _initial_plc_stop_callback(self, response):
        """階段 2：PLC 回應處理"""
        if response and response.success:
            日誌："[交管] ✅ 初始 PLC MR7005 停止成功"
            self.plc_force_on_result = True
        else:
            日誌："[交管] ❌ 初始 PLC 停止失敗"
            self.plc_force_on_result = False

        # 進入下一階段：請求交管區
        self.traffic_init_step = 2
        self._request_traffic_control()

    def _request_traffic_control(self):
        """階段 3：請求交管區控制權"""
        # 日誌顯示重試次數（如果有）
        retry_info = f" (第 {self.traffic_retry_count + 1} 次嘗試)" if self.traffic_retry_count > 0 else ""
        日誌：f"[交管] 請求交管區 {self.traffic_zone_id}...{retry_info}"

        # 發送 HTTP 請求到 web_api
        result = traffic_client.acquire_traffic_zone(
            traffic_id=self.traffic_zone_id,
            agv_id=self.node.agv_id
        )

        # 階段 4：根據回應處理
        if result.get("isAllow", False):
            # ✅ 允許通行：取消重試計時器 + 關閉 PLC MR7005
            if self.traffic_retry_count > 0:
                日誌：f"[交管] ✅ 允許通行（重試 {self.traffic_retry_count} 次後成功），關閉 PLC MR7005"
            else:
                日誌："[交管] ✅ 允許通行，關閉 PLC MR7005"

            self.traffic_allowed = True
            self._cancel_retry_timer()  # 取消重試計時器
            self._clear_plc_stop()
            self.traffic_init_step = 3  # 完成
        else:
            # ⛔ 拒絕通行：啟動 3 秒定時器重試（重要！）
            owner_id = result.get("owner_agv_id", "未知")
            日誌：f"[交管] ⛔ 拒絕通行：交管區 {self.traffic_zone_id} 被 AGV {owner_id} 佔用"

            # 啟動重試計時器
            self._start_retry_timer()
            self.traffic_init_step = 3  # 完成初始流程（但會持續重試）

    def _start_retry_timer(self):
        """啟動重試計時器（3秒後重新請求交管）"""
        self._cancel_retry_timer()  # 先取消現有計時器
        self.traffic_retry_timer = self.node.create_timer(
            self.traffic_retry_interval,
            self._retry_traffic_request
        )
        日誌：f"[交管] ⏱️ 啟動重試計時器，{self.traffic_retry_interval} 秒後重新請求"

    def _cancel_retry_timer(self):
        """取消重試計時器"""
        if self.traffic_retry_timer is not None:
            self.traffic_retry_timer.cancel()
            self.traffic_retry_timer = None
            日誌："[交管] ⏹️ 取消重試計時器"

    def _retry_traffic_request(self):
        """定時器觸發：重新請求交管區"""
        if self.traffic_retry_timer is not None:
            self.traffic_retry_timer.cancel()
            self.traffic_retry_timer = None
        self.traffic_retry_count += 1
        self._request_traffic_control()  # 重新請求

    def _clear_plc_stop(self):
        """關閉 PLC MR7005 停止信號"""
        try:
            plc_client.async_force_off('MR', '7005', callback)
            self.plc_stop_triggered = False
            日誌："[交管] ✅ 已關閉 PLC MR7005 停止信號"
        except Exception as e:
            日誌：f"[交管] ❌ 關閉 PLC 停止失敗: {e}"

    def leave(self):
        """離開 RunningState 時釋放交管區"""
        # 【重要】先取消重試計時器
        self._cancel_retry_timer()

        日誌：f"[交管] 離開 RunningState，釋放交管區 {self.traffic_zone_id}..."

        # 釋放交管區
        success = traffic_client.release_traffic_zone(
            traffic_id=self.traffic_zone_id,
            agv_id=self.node.agv_id
        )

        # 無論成功或失敗，都關閉 PLC MR7005
        if self.plc_stop_triggered:
            self._clear_plc_stop()

        super().leave()  # 呼叫父類
```

**差異總結**:

| 功能 | Base RunningState | Cargo RunningState |
|------|-------------------|-------------------|
| **基礎功能** | | |
| 路徑監控 | ✅ | ✅ 繼承 |
| 到達檢測 | ✅ | ✅ 繼承 |
| 狀態一致性 | ✅ | ✅ 繼承 |
| **交管功能** | | |
| 交管客戶端 | ❌ | ✅ TrafficClient (HTTP) |
| PLC 停止控制 | ❌ | ✅ MR7005 強制開關 |
| 交管區請求 | ❌ | ✅ acquire_traffic_zone |
| 交管區釋放 | ❌ | ✅ release_traffic_zone |
| 條件通行邏輯 | ❌ | ✅ 允許/拒絕處理 |
| **定時重試機制** | ❌ | ✅ **3秒自動重試** ⭐ |
| **流程控制** | | |
| enter() | 簡單日誌 | 4 階段交管流程 |
| leave() | 任務抓取 | 取消計時器 + 交管釋放 + PLC 關閉 |
| handle() | 路徑/到達檢查 | 繼承基礎邏輯 |

---

## 🔄 完整流程對比

### 場景：AGV 選擇任務並進入 Running 狀態

#### 基礎流程 (Base)
```
1. MissionSelectState
   └─ 選擇任務 (task.id=123, status_id=1)
   └─ context.set_state(WritePathState(node))

2. WritePathState
   ├─ 檢查：AGV_PATH = False
   ├─ 計算路徑：A* 演算法
   ├─ 組裝資料：dataValue[2000]
   ├─ 更新任務：task.status_id = 3
   ├─ 寫入 PLC：DM3000/DM4000
   └─ context.set_state(RunningState(node))  ❌

3. RunningState
   ├─ enter()：日誌輸出
   ├─ handle()：監控路徑
   └─ AGV 開始移動  ❌ 無交管控制
```

#### Cargo 流程 (修改後)
```
1. CargoMissionSelectState
   ├─ 猴子補丁設置
   ├─ super().handle() → 選擇任務
   └─ 攔截：WritePathState → CargoWritePathState ✅

2. CargoWritePathState
   ├─ 檢查：AGV_PATH = False
   ├─ 猴子補丁設置（備用）
   ├─ super().handle()
   │   ├─ 計算路徑：A* 演算法
   │   ├─ 組裝資料：dataValue[2000]
   │   ├─ 更新任務：task.status_id = 3
   │   └─ 寫入 PLC：DM3000/DM4000
   └─ 攔截：RunningState → CargoRunningState ✅

3. CargoRunningState
   ├─ enter()：交管 4 階段流程
   │   ├─ 階段 1：PLC MR7005 停止 (安全優先)
   │   │   日誌："[交管] 🚨 進入 RunningState，先觸發 PLC MR7005 停止"
   │   ├─ 階段 2：等待 PLC 回應
   │   │   日誌："[交管] ✅ 初始 PLC MR7005 停止成功"
   │   ├─ 階段 3：請求交管區
   │   │   日誌："[交管] 請求交管區 1..."
   │   │   HTTP POST → web_api /traffic/acquire
   │   └─ 階段 4：處理交管回應
   │       ├─ ✅ 允許：取消計時器 + 關閉 MR7005 → AGV 可移動
   │       │   日誌："[交管] ✅ 允許通行，關閉 PLC MR7005"
   │       └─ ⛔ 拒絕：啟動 3 秒計時器 → 保持 MR7005 → 自動重試
   │           日誌："[交管] ⛔ 拒絕通行：交管區 1 被 AGV X 佔用"
   │           日誌："[交管] ⏱️ 啟動重試計時器，3.0 秒後重新請求"
   │           ↓
   │           （等待 3 秒）
   │           ↓
   │           日誌："[交管] 請求交管區 1... (第 2 次嘗試)"
   │           ↓
   │           （持續重試直到允許或離開狀態）
   ├─ handle()：監控路徑（繼承基礎邏輯）
   └─ leave()：取消重試計時器 + 釋放交管區 + 關閉 MR7005
```

---

## 🎯 關鍵差異總結

### 1. 架構差異
| 層面 | Base | Cargo |
|------|------|-------|
| 設計模式 | 純繼承 | 繼承 + 猴子補丁攔截 |
| 狀態替換 | 無 | 3 層攔截機制 |
| 交管整合 | 無 | HTTP 客戶端 + PLC 控制 |

### 2. 功能差異
| 功能 | Base | Cargo |
|------|------|-------|
| 任務選擇 | ✅ | ✅ + Local 模式增強 |
| 路徑計算 | ✅ | ✅ 繼承 |
| PLC 寫入 | ✅ | ✅ 繼承 |
| **交通管制** | ❌ | ✅ **完整實作** |
| **安全控制** | ❌ | ✅ **PLC MR7005** |
| **區域衝突** | ❌ | ✅ **HTTP 協調** |

### 3. 執行流程差異
| 階段 | Base | Cargo |
|------|------|-------|
| 進入 Running | 直接開始移動 | 先停止 → 請求交管 → 條件允許 |
| 運行中 | 無衝突檢測 | 交管區保護 + **3秒自動重試** ⭐ |
| 被拒絕時 | N/A（無交管） | **啟動計時器 → 3秒後重試 → 持續直到允許** |
| 離開 Running | 單純任務抓取 | **取消計時器** + 釋放交管區 + PLC 復位 |

---

## 📌 結論

**Cargo 狀態機 = 基礎狀態機 + 交通管制層**

1. **繼承基礎邏輯**：路徑計算、PLC 寫入、任務管理
2. **增加交管層**：安全控制（PLC MR7005）+ 區域協調（HTTP API）
3. **攔截機制**：確保 Cargo AGV 使用專屬狀態類別
4. **完全向後兼容**：不影響基礎功能，純增強

**本次修改重點**：

#### 第一階段修改（CargoWritePathState）
- 修改前：攔截機制不穩定，可能使用基礎 RunningState（無交管）
- 修改後：雙重保險（直接檢查 + 猴子補丁），確保使用 CargoRunningState（有交管）

#### 第二階段修改（CargoRunningState）⭐ 重要
- **問題**：交管被拒絕後 AGV 永久卡住，無法自動恢復
- **解決**：加入 3 秒定時重試機制
  - 拒絕時啟動計時器（不是直接卡住）
  - 3 秒後自動重新請求交管
  - 持續重試直到允許通行或離開狀態
  - 離開狀態時自動取消計時器（防止資源洩漏）

**完整流程範例**：
```
進入 RunningState
→ PLC MR7005 停止（安全優先）
→ 請求交管區 1
→ ⛔ 被拒絕（AGV 2 佔用）
→ 啟動 3 秒計時器 ⏱️
→ （等待 3 秒，AGV 保持停止）
→ 重新請求交管區 1 (第 2 次)
→ ⛔ 仍被拒絕
→ 啟動 3 秒計時器 ⏱️
→ （等待 3 秒）
→ 重新請求交管區 1 (第 3 次)
→ ✅ 允許通行（AGV 2 已離開）
→ 取消計時器 + 關閉 PLC MR7005
→ AGV 開始移動 🚀
```
