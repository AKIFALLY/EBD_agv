# Cargo Mover AGV 完整架構分析報告

## 1. 系統概覽

### 1.1 定位
Cargo Mover AGV 是眼鏡生產系統中的**貨物搬運車**，負責房間門口的 Rack ↔ 傳送箱轉移作業，配備：
- Hokuyo 8bit 光通訊模組（左右側）
- KUKA 機械臂
- 麥克納姆輪（全向移動）
- 3D 視覺定位 + OCR 識別系統

### 1.2 核心職責
1. **3D 視覺定位**：識別 Rack 上的 2D Mark，進行基座座標更新
2. **OCR 掃描**：逐一掃描每個 Carrier 的產品編號，判斷是否符合房間製程需求
3. **製程匹配檢查**：判斷產品製程適配性（泡藥1次 vs 2次）
4. **Rack ↔ 傳送箱轉移**：根據製程需求進行轉移操作
5. **A/B 雙面處理**：完成一面後，等待 Rack 轉向再處理另一面

---

## 2. 架構層級分析

### 2.1 三層狀態機設計
```
BASE層 (通用邏輯)
├─ IDLE STATE       (閒置)
├─ MANUAL STATE     (手動模式)
├─ AUTO STATE       (自動模式) ────┐
└─ ERROR STATE      (錯誤狀態)     │
                                   │
AGV層 (Cargo特定)                  │
├─ MISSION_SELECT   (任務選擇) ◄───┘
├─ WRITE_PATH       (路徑計算)
├─ RUNNING          (路徑執行)
└─ WAIT_ROBOT       (等待機械臂)
       │
       └──→ ROBOT層 (機械臂操作)
           ├─ IDLE_STATE (閒置)
           ├─ ENTRANCE層 (進入操作)
           │  ├─ CHECK_RACK_SIDE
           │  ├─ SELECT_RACK_PORT
           │  ├─ TRANSFER_VISION_POSITION
           │  ├─ TRANSFER_CHECK_EMPTY
           │  ├─ TAKE_RACK_PORT
           │  ├─ PUT_TRANFER
           │  └─ WAIT_ROTATION
           │
           ├─ EXIT層 (退出操作)
           │  ├─ CHECK_RACK_SIDE
           │  ├─ SELECT_RACK_PORT
           │  ├─ TRANSFER_VISION_POSITION
           │  ├─ TRANSFER_CHECK_HAVE
           │  ├─ PUT_RACK_PORT
           │  └─ WAIT_ROTATION
           │
           └─ COMPLETE_STATE (完成)
```

### 2.2 任務資料流

```
【任務發佈源】
PostgreSQL (agvc DB)
    ↓
agvc_database_publish_node
    ↓
ROS2 Topic: /agvc/tasks (Tasks 訊息)
    ↓ 
【任務訂閱點】
├─ MissionSelectState.tasks_callback()    [AGV層入口]
└─ RunningState.tasks_callback()          [任務監控]
    ↓
self.latest_tasks[]   (最新任務列表)
    ↓
_process_tasks()      (任務篩選邏輯)
    ↓
self.node.task        (當前執行任務)
```

---

## 3. 核心文件解析

### 3.1 啟動配置 (launch/launch.py)

**文件位置**: `/home/ct/EBD_agv/app/agv_ws/src/cargo_mover_agv/launch/launch.py`

```python
# 動態讀取 AGV 配置
agv_id = os.environ.get('AGV_ID', 'cargo01')  # 預設 cargo01
room_id = int(agv_id[-2:])                    # 從 ID 提取房間號

# 啟動四個節點
1. plc_service         - PLC 通訊服務
2. joy_linux_node      - 搖桿控制
3. cargo_agv_node      - 核心 AGV 節點
```

**參數說明**:
- `room_id`: 自動從 AGV ID 提取（cargo01 → room_id=1）
- `agv_command_file`: `/app/agv_cmd_service_ws/src/agv_cmd_service/config/agv_cmd_service.yaml`
- `device_config_file`: `/app/config/agv/{agv_id}_config.yaml`

### 3.2 AGV 核心節點 (agv_core_node.py)

**文件位置**: `/home/ct/EBD_agv/app/agv_ws/src/cargo_mover_agv/cargo_mover_agv/agv_core_node.py`

#### 初始化流程
```python
class AgvCoreNode(AgvNodebase):
    def __init__(self):
        # 1. 設置通用參數和訂閱
        self.setup_common_parameters()      # 從 AgvNodebase 繼承
        self.setup_agv_subscription()       # 訂閱任務資料

        # 2. 初始化硬體
        self.robot = Robot(...)            # 機械臂控制
        self.hokuyo_dms_8bit_1 = ...       # 左側光通訊
        self.hokuyo_dms_8bit_2 = ...       # 右側光通訊

        # 3. 初始化三層狀態機
        self.base_context = BaseContext(IdleState(self))
        self.cargo_context = CargoContext(MissionSelectState(self))
        self.robot_context = RobotContext(IdleState(self))

        # 4. 綁定事件回調
        self.base_context.on_state_changed += self.state_changed
        self.cargo_context.on_state_changed += self.state_changed
        self.robot_context.on_state_changed += self.state_changed

        # 5. 初始化 JSON 狀態記錄器
        self.json_recorder = CargoAgvStatusJsonRecorder()
        self.json_save_timer = create_timer(1.0, self._update_json_status_file)
```

#### 狀態機執行邏輯
```python
def base_after_handle(self, state):
    """Base層狀態後處理"""
    if isinstance(state, AutoState):
        # AUTO 模式下才執行 AGV 狀態機
        self.cargo_context.handle()
    else:
        # 非 AUTO 狀態下進行特殊處理
        self._handle_non_auto_state(...)

def _handle_non_auto_state(self, base_state_name):
    """處理非 AutoState 時的邏輯"""
    # 1. 轉換 Robot 層到 IdleState
    if not isinstance(self.robot_context.state, IdleState):
        self.robot_context.set_state(IdleState(self))
    
    # 2. 檢查 AGV 層是否在 WaitRobotState，若是則轉換到 MissionSelectState
    if isinstance(self.cargo_context.state, WaitRobotState):
        self.cargo_context.set_state(MissionSelectState(self))
    
    # 3. 繼續執行一次 AGV 層狀態機
    self.cargo_context.handle()
```

### 3.3 任務選擇狀態 (mission_select_state.py)

**文件位置**: `/home/ct/EBD_agv/app/agv_ws/src/agv_base/agv_base/agv_states/mission_select_state.py`

#### 最關鍵部分：latest_tasks 的使用

```python
class MissionSelectState(State):
    def __init__(self, node):
        self.latest_tasks = []  # 【關鍵】儲存最新的任務資料

    def enter(self):
        # 訂閱任務資料
        subscription = self.node.create_subscription(
            Tasks,
            '/agvc/tasks',
            self.tasks_callback,
            qos_profile
        )

    def tasks_callback(self, msg: Tasks):
        """【任務接收入口】當 /agvc/tasks topic 有新資料時觸發"""
        tasks = msg.datas
        self.latest_tasks = tasks  # 【重要】直接保存最新任務列表
        self._process_tasks(tasks)  # 立即處理任務篩選

    def _process_tasks(self, tasks):
        """【任務篩選邏輯】篩選該 AGV 的待執行任務"""
        running_tasks = [
            t for t in tasks
            if (t.status_id in [READY_TO_EXECUTE, EXECUTING, PENDING] and
                t.agv_id == self.node.agv_id)
        ]
        
        if len(running_tasks) > 0:
            self.node.mission_id = running_tasks[0].id
            self.node.task = running_tasks[0]  # 【關鍵】設置當前執行任務
            return True
        return False

    def handle(self, context):
        """【狀態循環邏輯】每 50ms 檢查一次"""
        if self.count > 30:  # 即 1.5秒 (30 * 50ms)
            self.count = 0
            
            # 1. 確保從 latest_tasks 中搜尋該 AGV 的任務資料
            if self.latest_tasks and len(self.latest_tasks) > 0:
                has_task = self._process_tasks(self.latest_tasks)
            
            # 2. 檢查是否已經有路徑資料
            if self.node.agv_status.AGV_PATH:
                if hasattr(self.node, 'task') and self.node.task:
                    # 【狀態轉換】離開 MissionSelect，進入 Running
                    context.set_state(RunningState(self.node))
            
            # 3. 或者已有高優先度任務
            elif self.highest_priority_task:
                context.set_state(WritePathState(self.node))  # 計算路徑
            
            # 4. 或者有本地 HMI 任務
            elif self.localMission and not self.node.agv_status.AGV_PATH:
                context.set_state(WritePathState(self.node))
        
        self.count += 1
```

#### latest_tasks 的四個使用場景

| 場景 | 觸發點 | 操作 | 目的 |
|------|--------|------|------|
| **1. 任務到達時** | `tasks_callback()` | 直接保存 `latest_tasks = tasks` | 保持最新任務列表 |
| **2. 定時檢查** | `handle()` 每1.5秒 | `_process_tasks(latest_tasks)` | 篩選該AGV的任務 |
| **3. 防止任務丟失** | 路徑計算中 | 檢查 `latest_tasks` 中是否有新任務 | 避免同時有路徑和任務資料丟失 |
| **4. 狀態轉換時** | 離開 MissionSelect 前 | 驗證 `latest_tasks` 是否有該AGV的任務 | 確保任務資料完整性 |

### 3.4 運行狀態 (Running_state.py)

**文件位置**: `/home/ct/EBD_agv/app/agv_ws/src/agv_base/agv_base/agv_states/Running_state.py`

```python
class RunningState(State):
    def __init__(self, node):
        self.latest_tasks = []  # 【同樣的機制】

    def enter(self):
        # 【重複訂閱】Running 狀態也訂閱任務
        subscription = self.node.create_subscription(
            Tasks, '/agvc/tasks', self.tasks_callback, qos_profile
        )

    def handle(self, context):
        # 路徑執行中的任務監控邏輯
        if not self.node.agv_status.AGV_PATH:
            # 沒有路徑，回到任務選擇
            # 【在轉換前】先從 latest_tasks 中抓取任務
            for task in self.latest_tasks:
                if task.agv_id == self.node.agv_id and \
                   task.status_id in [READY_TO_EXECUTE, EXECUTING]:
                    self.node.task = task  # 【確保不丟失任務】
                    break
            context.set_state(MissionSelectState(self.node))

        if self.node.agv_status.AGV_2POSITION:
            # 到達目標位置，進入機械臂操作
            # 【再次檢查】確保有有效任務
            for task in self.latest_tasks:
                if task.agv_id == self.node.agv_id and \
                   task.status_id in [READY_TO_EXECUTE, EXECUTING]:
                    self.node.task = task
                    if task.id != 0:
                        context.set_state(WaitRobotState(self.node))
                    break
```

### 3.5 Cargo 上下文 (cargo_context.py)

**文件位置**: `/home/ct/EBD_agv/app/agv_ws/src/cargo_mover_agv/cargo_mover_agv/cargo_context.py`

```python
class CargoContext(BaseContext):
    """管理 cargo agv 狀態機"""
    
    def __init__(self, initial_state: State):
        super().__init__(initial_state)
        
        # AGV 層跨狀態共享的參數
        self.rack_rotation = False  # Rack 轉向標誌
        self.completed = False      # 完成標誌
```

---

## 4. 任務資料流詳細分析

### 4.1 任務資料源頭

```
PostgreSQL agvc Database
├─ TABLE: task (任務表)
│  ├─ id (唯一識別)
│  ├─ agv_id (指定 AGV)
│  ├─ status_id (任務狀態)
│  ├─ node_id (目標節點)
│  ├─ work_id (工作 ID)
│  ├─ priority (優先級)
│  └─ parameters (JSON 參數)
│
└─ Publisher: agvc_database_publish_node (1.5秒發佈一次)
   └─ Topic: /agvc/tasks (Tasks 訊息型態)
      └─ msg.datas[] (任務陣列)
```

**推送策略**:
- **發佈頻率**: 1.5 秒
- **QoS 設定**: RELIABLE + VOLATILE + depth=10
- **訂閱方**: MissionSelectState 和 RunningState

### 4.2 任務狀態流轉

```
0 - REQUESTING       (UI 請求)
  ↓
1 - PENDING          (WCS 已接受)
  ↓
2 - READY_TO_EXECUTE (RCS 派發給 AGV)
  ↓
3 - EXECUTING        (AGV 正在執行)
  ↓
4 - COMPLETED        (AGV 完成)

AGV 篩選條件: status_id in [2, 3, 1] 且 agv_id 相符
```

### 4.3 與其他 AGV 的對比

| 功能點 | Cargo Mover | Loader | Unloader | 說明 |
|--------|------------|--------|---------|------|
| **MissionSelectState** | ✅ | ✅ | ✅ | 都使用相同基類 |
| **latest_tasks** | ✅ | ✅ | ✅ | 三個 AGV 都有 |
| **任務訂閱位置** | MissionSelectState + RunningState | 相同 | 相同 | 完全相同實現 |
| **任務篩選邏輯** | `_process_tasks()` | 相同 | 相同 | 完全相同邏輯 |
| **Robot 層狀態** | entrance/exit | 複雜多層 | pre_dryer/oven/boxout | 各自特定業務 |
| **Hokuyo 配置** | 左右2個 | 前方1個 | 前方1個 | 根據任務需求 |

---

## 5. 可能的問題點分析

### 5.1 latest_tasks 的使用陷阱

**問題1**: 雙重訂閱導致的狀態不同步
```python
# MissionSelectState 訂閱 1 次
subscription1 = create_subscription(Tasks, '/agvc/tasks', callback1)

# RunningState 再訂閱 1 次  
subscription2 = create_subscription(Tasks, '/agvc/tasks', callback2)

# 【問題】兩個 latest_tasks[] 可能收到不同版本的任務資料
# 原因：callback 執行時間差異、state 切換時機
```

**解決方案**: 
- 在 AgvNodebase 層級實現唯一的任務訂閱
- 所有狀態通過 `self.node.latest_tasks` 訪問共享任務列表

**問題2**: 狀態轉換時的任務遺失
```python
# RunningState.handle()
if self.node.agv_status.AGV_2POSITION:
    # 【危險】此時如果 latest_tasks 正在被回調更新
    # 可能導致任務資料不一致
    for task in self.latest_tasks:  # ←【競態條件】
        self.node.task = task
```

**解決方案**:
- 使用 `threading.Lock()` 保護 `latest_tasks`
- 或者使用消息隊列確保任務資料順序性

**問題3**: 任務更新頻率不足
```python
# agvc_database_publish_node 每 1.5 秒發佈一次
# MissionSelectState.handle() 每 1.5 秒檢查一次 (count > 30)
# 【問題】如果任務在檢查之間變更，可能延遲 3 秒才發現
```

**解決方案**:
- 增加發佈頻率（改為 1 秒或 500ms）
- 或在 tasks_callback() 中立即調用 `_process_tasks()`

### 5.2 狀態轉換的邊界條件

**問題4**: 同時有多個狀態轉換觸發點
```python
def handle(self, context):
    # 1. 路徑存在 → RunningState
    if self.node.agv_status.AGV_PATH:
        context.set_state(RunningState(...))
    
    # 2. 有高優先度任務 → WritePathState
    elif self.highest_priority_task:
        context.set_state(WritePathState(...))
    
    # 3. 有本地 HMI 任務 → WritePathState
    elif self.localMission:
        context.set_state(WritePathState(...))
    
    # 【問題】優先級不明確，如果同時滿足多個條件？
```

**解決方案**:
- 明確定義轉換優先級
- 添加日誌記錄為什麼選擇某個轉換

### 5.3 CargoContext 的簡化性

**問題5**: CargoContext 只有兩個欄位
```python
class CargoContext(BaseContext):
    def __init__(self, initial_state):
        self.rack_rotation = False  # 只用於 A/B 面
        self.completed = False      # 簡單標誌
    
    # 【問題】缺少：
    # - 當前處理的 Carrier 索引
    # - A/B 面狀態追蹤
    # - OCR 結果緩存
    # - 製程匹配檢查結果
```

**改進建議**:
```python
class CargoContext(BaseContext):
    def __init__(self, initial_state):
        self.rack_rotation = False
        self.completed = False
        
        # 新增欄位
        self.current_side = 'A'              # A/B 面
        self.current_carrier_index = 0       # 當前 Carrier
        self.ocr_results = {}                # OCR 快取
        self.process_check_results = {}      # 製程匹配結果
        self.failed_carriers = []            # 失敗的 Carrier
```

---

## 6. 啟動配置詳解

### 6.1 環境變數控制

```bash
# 啟動 cargo01
AGV_ID=cargo01 ROS_NAMESPACE=/cargo01 \
  ros2 launch cargo_mover_agv launch.py

# 啟動 cargo02
AGV_ID=cargo02 ROS_NAMESPACE=/cargo02 \
  ros2 launch cargo_mover_agv launch.py
```

### 6.2 配置文件位置

```
/app/config/agv/
├─ base_config.yaml          # 基礎配置
├─ cargo01_config.yaml       # Cargo01 特定配置
├─ cargo02_config.yaml       # Cargo02 特定配置
├─ loader01_config.yaml
├─ loader02_config.yaml
├─ unloader01_config.yaml
└─ unloader02_config.yaml
```

**配置內容示例** (cargo01_config.yaml):
```yaml
cargo01:
  device_config_file: "/app/config/hokuyo_dms_config.yaml"
  hokuyo_devices:
    hokuyo_left:
      ip: "192.168.1.101"
      port: 8000
    hokuyo_right:
      ip: "192.168.1.102"
      port: 8000
  plc:
    ip: "192.168.100.1"
    port: 502
```

### 6.3 參數流轉

```
launch.py 中讀取環境變數
  ↓ AGV_ID=cargo01
  ↓
├─ agv_id = "cargo01"
├─ room_id = 1                    (從 cargo01 提取)
├─ ros_namespace = "/cargo01"
└─ device_config_file = "/app/config/agv/cargo01_config.yaml"
  ↓
傳遞給 cargo_agv_node
  ↓
  ├─ plc_service       (namespace=/cargo01)
  ├─ joy_linux_node    (namespace=/cargo01)
  └─ cargo_agv_node    (namespace=/cargo01, room_id=1)
```

---

## 7. 與 Loader/Unloader 的關鍵差異

### 7.1 Hardware 配置

| 元件 | Cargo Mover | Loader | Unloader |
|------|------------|--------|---------|
| **Hokuyo 位置** | 左右側 | 前方 | 前方 |
| **Hokuyo 數量** | 2 個 | 1 個 | 1 個 |
| **用途** | 傳送箱門控制 | 機械臂輔助 | 機械臂輔助 |
| **KUKA 臂** | 單臂 | 單臂 | 單臂 |
| **視覺系統** | 3D + OCR | 視覺定位 | 視覺定位 |

### 7.2 Business Logic 差異

| 業務邏輯 | Cargo Mover | Loader | Unloader |
|---------|------------|--------|---------|
| **任務類型** | TAKE/PUT | 7 種 work_id | 4 種 work_id |
| **批量處理** | 逐一 Carrier | 一次 1-2 格 | 一次 2 格 |
| **製程檢查** | OCR + 製程匹配 | 站點自動路由 | 站點自動路由 |
| **Rack 操作** | A/B 面轉向等待 | 不涉及 | 不涉及 |

### 7.3 Robot 層狀態機

**Cargo Mover** (簡單):
```
idle_state ↔ entrance ↔ exit ↔ complete_state
```

**Loader** (複雜):
```
多層嵌套狀態機
├─ TRANSFER (傳送箱取料)
├─ CLEANER (清潔機)
├─ SOAKER (浸潤機)
└─ PRE_DRYER (預乾燥機)
```

**Unloader** (複雜):
```
多層嵌套狀態機
├─ PRE_DRYER (預烘機取料)
├─ OVEN (烘箱操作)
└─ BOXOUT_TRANSFER (出口傳送箱)
```

---

## 8. 完整數據流時序圖

```
時間軸
├─ T=0s: WCS 創建任務 → Task(id=1, agv_id=1, status=1[PENDING])
│
├─ T=0.5s: RCS 派發任務 → Task(status=2[READY_TO_EXECUTE])
│
├─ T=1.5s: agvc_database_publish_node 發佈
│         Topic: /agvc/tasks
│         Data: [Task(..., status=2)]
│
├─ T=1.5s: MissionSelectState.tasks_callback()
│         latest_tasks = [Task(...)]
│         _process_tasks([Task(...)])
│         node.task = Task(...)
│
├─ T=1.5s: MissionSelectState.handle() (count=30 時)
│         檢查 node.agv_status.AGV_PATH
│         ├─ 無路徑 → WritePathState (計算路徑)
│         └─ 有路徑 → RunningState (執行路徑)
│
├─ T=2s~3s: WritePathState 計算 A* 路徑
│          node.agv_status.AGV_PATH = path
│
├─ T=3s: RunningState 執行路徑
│       node.agv_status.AGV_2POSITION = True
│
└─ T=3.5s: WaitRobotState
          執行機械臂操作
          Task(status=3[EXECUTING]) → Task(status=4[COMPLETED])
```

---

## 9. 關鍵檔案總結表

| 檔案 | 位置 | 功能 | 重要程度 |
|------|------|------|----------|
| `agv_core_node.py` | cargo_mover_agv/ | 核心節點，三層狀態機初始化 | ⭐⭐⭐⭐⭐ |
| `cargo_context.py` | cargo_mover_agv/ | AGV層上下文 | ⭐⭐⭐⭐ |
| `mission_select_state.py` | agv_base/ | **任務選擇，latest_tasks 入口** | ⭐⭐⭐⭐⭐ |
| `Running_state.py` | agv_base/ | 路徑執行，任務監控 | ⭐⭐⭐⭐ |
| `write_path_state.py` | agv_base/ | 路徑計算狀態 | ⭐⭐⭐⭐ |
| `launch.py` | cargo_mover_agv/launch/ | 啟動配置 | ⭐⭐⭐⭐ |
| `agvc_database_publish_node.py` | db_proxy_ws/ | **任務發佈源** | ⭐⭐⭐⭐⭐ |

---

## 10. 開發建議

### 10.1 改進方向

1. **統一任務訂閱**
   - 在 `AgvNodeBase` 層實現單一訂閱
   - 所有狀態共享 `node.latest_tasks`

2. **線程安全**
   - 保護 `latest_tasks` 訪問
   - 使用 `threading.Lock()` 或 atomic operations

3. **任務追蹤**
   - 增強 `CargoContext` 的任務狀態追蹤
   - 記錄每個 Carrier 的處理結果

4. **錯誤恢復**
   - OCR 失敗時的 Alarm 機制
   - 製程不匹配時的人工介入流程

5. **監控和診斷**
   - 完整的 JSON 狀態記錄器
   - 任務流轉的詳細日誌

### 10.2 測試策略

```bash
# 單元測試
python3 -m pytest test/test_mission_select.py -v

# 集成測試
python3 -m pytest test/test_cargo_agv_integration.py -v

# 性能測試
python3 -m pytest test/test_cargo_agv_performance.py -v
```

---

## 11. 常見問題排查

### Q1: 為什麼 latest_tasks 有時為空？
**A**: 檢查 `/agvc/tasks` topic 是否有發佈
```bash
ros2 topic echo /agvc/tasks
```

### Q2: 任務為什麼沒有被執行？
**A**: 檢查三個條件
1. `task.agv_id == node.agv_id`
2. `task.status_id` 在 [1, 2, 3] 之間
3. `node.agv_status.AGV_PATH` 是否已設置

### Q3: 狀態機為什麼卡在 MissionSelectState？
**A**: 可能原因
1. 沒有符合條件的任務
2. `AGV_PATH` 和 `highest_priority_task` 都為空
3. `localMission` 標誌未設置

