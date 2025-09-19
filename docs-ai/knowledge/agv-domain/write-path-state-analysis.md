# WritePathState 功能分析

## 🎯 適用場景
- 理解 WritePathState 在 AGV 狀態機中的核心功能和實作細節
- 為路徑規劃和 PLC 資料寫入相關問題提供深度技術分析
- 解決 A* 演算法整合、PLC 通訊、資料庫更新等複雜功能的故障排除

## 📋 WritePathState 核心功能概述

WritePathState 是 AGV Layer 中的關鍵狀態，負責 **路徑規劃、資料轉換、PLC 寫入和任務狀態更新** 的完整流程。

**檔案位置**: `app/agv_ws/src/agv_base/agv_base/agv_states/write_path_state.py`

### 主要職責
1. **A* 路徑規劃**: 使用 A* 演算法計算最佳路徑
2. **路徑資料轉換**: 將路徑轉換為 PLC 可讀格式
3. **PLC 資料寫入**: 分段寫入路徑資料到 PLC 記憶體
4. **任務狀態更新**: 更新資料庫中的任務執行狀態
5. **錯誤處理機制**: 完整的重試和異常處理邏輯

## 🏗️ 類別架構和初始化

### 核心屬性
```python
class WritePathState(State):
    def __init__(self, node: Node):
        super().__init__(node)
        # 路徑規劃相關
        self.StationID = None           # 站點ID
        self.TagNo = None               # 當前TAG位置
        self.path = []                  # A*計算的路徑陣列
        self.source_data = None         # A*演算法的來源資料
        
        # PLC通訊客戶端
        self.plc_client = PlcClient(node)
        self.agvdbclient = AGVCDatabaseClient(node)
        
        # 路徑資料解析
        self.cantomove_tag = None       # 可移動標籤
        self.act = []                   # 動作序列
        self.pgv = 0                    # PGV值
        self.speed = []                 # 速度設定
        self.shift = []                 # 偏移設定
        self.inposition = []            # 進位設定
        self.safe_sensor_setting = []  # 安全感測器設定
        
        # PLC資料陣列 (2000個16位元資料)
        self.dataValue = [0] * 2000
        
        # 控制計數器
        self.count = 0                  # 寫入嘗試次數
        self.step = 0                   # 步驟計數器
```

### 依賴模組整合
- **A* 演算法**: `astar_algorithm.astar_algorithm.AStarAlgorithm`
- **資料庫客戶端**: `db_proxy.agvc_database_client.AGVCDatabaseClient`
- **PLC 客戶端**: `plc_proxy.plc_client.PlcClient`

## 🔄 狀態處理主流程

### handle() 方法核心邏輯
```python
def handle(self, context):
    # 1. 錯誤檢查：寫入失敗過多
    if self.count > 5:
        self.node.get_logger().error("❌ 路徑資料寫入失敗過多，寫入異常到PLC")
        self.plc_client.async_force_on('MR', '3204', self.force_callback)
        from agv_base.agv_states.mission_select_state import MissionSelectState
        context.set_state(MissionSelectState(self.node))
        return
    
    # 2. 既有路徑檢查：直接跳轉到執行狀態
    if self.node.agv_status.AGV_PATH:
        self.node.get_logger().info("AGV 已有路徑資料，離開 WritePathState-->RunningState")
        from agv_base.agv_states.Running_state import RunningState
        context.set_state(RunningState(self.node))
        return
    
    # 3. 步驟控制邏輯
    if self.step >= 3:
        self.step += 1
        if self.step >= 100:
            self.step = 0  # 重置避免無限循環
    
    # 4. 路徑規劃和PLC寫入 (step == 0時執行)
    if not self.node.agv_status.AGV_PATH and self.step == 0:
        self.execute_path_planning_and_plc_write()
```

## 🗺️ A* 路徑規劃整合

### 路徑規劃初始化
```python
# 設定起點和終點
self.StationID = "Washing"  # 固定站點ID
self.TagNo = self.node.agv_status.AGV_FPGV  # 當前TAG位置

try:
    # 初始化A*演算法
    self.As = AStarAlgorithm(self.TagNo, self.node.node_id)
    self.node.get_logger().info(
        f"✅ A*演算法初始化成功, 現在位置: {self.TagNo}, 目標節點: {self.node.node_id}")
    
    # 執行路徑計算
    self.path = self.As.run()
    self.node.pathdata = self.path  # 儲存到節點屬性
    self.node.get_logger().info(f"✅ 計算路徑成功: {self.path}")
    
except Exception as e:
    self.node.get_logger().error(
        f"❌ 計算路徑失敗- 現在位置: {self.TagNo}, 目標節點: {self.node.node_id}")
    self.count += 1  # 增加失敗計數
    return

# 取得來源資料用於後續處理
self.source_data = self.As.source_data
```

### A* 演算法技術細節
- **演算法實作**: 基於 NetworkX 的 `nx.astar_path()` 高效實作
- **啟發函數**: 歐式距離 `math.hypot(ux - vx, uy - vy)`
- **圖形結構**: 有向圖，支援單向通行限制
- **權重計算**: 節點間實際距離作為邊權重

## 📊 PLC 資料格式轉換

### 資料結構映射
WritePathState 將 A* 路徑轉換為 PLC 可讀的結構化資料：

```python
# 每個路徑點使用20個16位元資料槽
for i in range(len(self.path)):
    # 確定當前點和下一個點
    x = 0
    y = False  # 是否為最後一個點
    if len(self.path)-1 == i:
        x = self.path[i]      # 最後一個點
        y = True
    else:
        x = self.path[i+1]    # 下一個點
        y = False
    
    # 從source_data中查找對應TAG的詳細資訊
    for tag in self.source_data:
        if tag.get('TagNo') == x:
            # 解析CanToMoveSet中的移動配置
            cantomove_tag = tag.get('CanToMoveSet')
            for j in range(len(cantomove_tag)):
                if cantomove_tag[j].get('CanToMoveTag') == self.path[i]:
                    # 提取移動參數
                    self.cantomove_tag = cantomove_tag[j].get('CanToMoveTag')
                    self.pgv = cantomove_tag[j].get('PGV')
                    self.act = cantomove_tag[j].get('Act')
                    self.speed = cantomove_tag[j].get('Speed')
                    self.shift = cantomove_tag[j].get('SHIFT')
                    self.inposition = cantomove_tag[j].get('Inposition')
                    self.safe_sensor_setting = cantomove_tag[j].get('SafeSensorSetting')
```

### PLC 資料陣列格式 (每個路徑點20個資料槽)
```python
# 基礎位置資訊 (索引 i*20 + offset)
dataValue[i*20+0] = TAG_NUMBER        # TAG編號
dataValue[i*20+1] = PGV_VALUE         # PGV值
dataValue[i*20+2] = ACTION_OR_STATION # 動作或站點+20

# 座標資訊 (32位元拆分為兩個16位元)
dataValue[i*20+4], dataValue[i*20+5] = split_32_to_16(Tag_X)   # X座標
dataValue[i*20+9], dataValue[i*20+10] = split_32_to_16(Tag_Y)  # Y座標

# 安全感測器設定
dataValue[i*20+6] = safe_sensor_setting[0]   # 感測器1
dataValue[i*20+11] = safe_sensor_setting[1]  # 感測器2
dataValue[i*20+16] = safe_sensor_setting[2]  # 感測器3

# 速度設定
dataValue[i*20+3] = speed[0]    # 速度1
dataValue[i*20+8] = speed[1]    # 速度2
dataValue[i*20+13] = speed[2]   # 速度3

# 固定動作值
dataValue[i*20+7] = 12          # 固定動作值
dataValue[i*20+12] = 12         # 固定動作值

# 旋轉角度 (32位元拆分)
dataValue[i*20+14], dataValue[i*20+15] = split_32_to_16(shift[2])
```

### 特殊處理邏輯
- **MAGIC = 21 特殊模式**: 僅影響最後一個點，設定 `dataValue[i*20+2] = 21`
- **最後一個點 (正常)**: 使用站點ID (`tag.get('Station')+20`)
- **最後一個點 (MAGIC=21)**: 直接設定為 21，不使用站點+20
- **中間點**: 使用動作值 (`act[0]`)，不受 MAGIC 影響
- **TAG號選擇**: 最後一個點用實際TAG，其他用`cantomove_tag`

## 🔌 PLC 通訊機制

### 分段寫入策略
```python
# 轉換為字串陣列
string_values = [str(v) for v in self.dataValue]
string_values_1 = string_values[:1000]     # 前1000筆
string_values_2 = string_values[1000:2000] # 後1000筆

# 分兩次寫入PLC記憶體
self.plc_client.async_write_continuous_data(
    'DM', '3000', string_values_1, self.write_path_callback)
self.plc_client.async_write_continuous_data(
    'DM', '4000', string_values_2, self.write_path_callback)
```

### PLC 記憶體配置
- **DM3000-DM3999**: 路徑資料前半段 (1000個16位元資料)
- **DM4000-DM4999**: 路徑資料後半段 (1000個16位元資料)
- **總容量**: 2000個16位元資料 = 100個路徑點 (每點20個資料)

### 非同步回調處理
```python
def write_path_callback(self, response):
    if response.success:
        self.node.get_logger().info("✅ PLC 路徑資料寫入成功")
        self.step += 1  # 增加步驟計數器
    else:
        self.node.get_logger().warn("⚠️ PLC 路徑資料寫入失敗")

def force_callback(self, response):
    """異常狀態寫入PLC回調"""
    if response.success:
        self.node.get_logger().info("✅ PLC force寫入成功")
    else:
        self.node.get_logger().warn("⚠️ PLC force寫入失敗")
```

## 💾 資料庫狀態更新

### 任務狀態更新
```python
# 更新任務為執行中狀態
self.node.task.status_id = 3              # 狀態ID: 3 = 執行中
self.node.task.agv_id = self.node.AGV_id  # 綁定到當前AGV

# 非同步更新資料庫
self.agvdbclient.async_update_task(
    self.node.task, self.task_update_callback)
```

### 任務狀態對照表
```
ID  名稱    描述
0   請求中  UI-請求執行任務
1   待處理  WCS-任務已接受，待處理
2   待執行  RCS-任務已派發，待執行
3   執行中  AGV-任務正在執行        ← WritePathState設定
4   已完成  AGV-任務已完成
5   取消中  任務取消中
6   錯誤    錯誤
```

### 資料庫更新回調
```python
def task_update_callback(self, response):
    if response is None:
        print("❌ 未收到任務更新的回應（可能逾時或錯誤）")
        return
    
    if response.success:
        print(f"✅ 任務更新成功，訊息: {response.message}")
    else:
        print(f"⚠️ 任務更新失敗，訊息: {response.message}")
```

## 🔧 輔助工具函數

### 32位元整數分割
```python
def split_32_to_16(self, value):
    """將32位元整數分割成兩個16位元整數"""
    value &= 0xFFFFFFFF           # 確保無符號32位元
    low = value & 0xFFFF          # 取低16位元
    high = (value >> 16) & 0xFFFF # 取高16位元
    return low, high
```

**使用場景**:
- X, Y 座標轉換 (通常為mm單位的大數值)
- 旋轉角度資料轉換
- PLC 16位元記憶體限制的資料拆分

## ⚡ 效能和最佳化

### 步驟控制機制
```python
# 避免重複執行的步驟控制
if self.step >= 3:
    self.step += 1
    if self.step >= 100:
        self.step = 0  # 防止無限循環
```

### 重試機制
- **最大重試次數**: 5次 (`self.count > 5`)
- **失敗處理**: 超過限制後寫入PLC異常標誌 (`MR3204`)
- **狀態回退**: 失敗後返回 `MissionSelectState`

### 快速路徑檢查
```python
# 優先檢查既有路徑，避免重複計算
if self.node.agv_status.AGV_PATH:
    context.set_state(RunningState(self.node))
    return
```

## 🚨 錯誤處理和故障排除

### 常見錯誤類型

#### 1. A* 路徑規劃失敗
**原因**:
- 起點或終點TAG不存在
- 圖形連通性問題
- 配置檔案載入失敗

**診斷方法**:
```bash
# 檢查A*演算法配置
cat /app/config/path.yaml
cat /app/config/stationID.yaml

# 測試A*演算法
cd /app/path_algorithm/src/astar_algorithm/astar_algorithm
python3 astar_algorithm.py
```

#### 2. PLC 通訊失敗
**原因**:
- PLC 連接中斷
- PLC 記憶體寫入權限問題
- 資料格式錯誤

**診斷方法**:
```bash
# 檢查PLC連接
ros2 service call /plc_read plc_interfaces/PLCRead "address: 'DM3000'"

# 檢查PLC服務狀態
ros2 service list | rg plc
```

#### 3. 資料庫更新失敗
**原因**:
- 資料庫連接問題
- 任務ID不存在
- 權限或約束違反

**診斷方法**:
```bash
# 檢查資料庫連接
ros2 service call /db_query db_proxy_interfaces/GenericQuery "query: 'SELECT * FROM tasks LIMIT 1'"
```

### 日誌分析
```bash
# WritePathState相關日誌
tail -f /tmp/agv.log | rg "WritePathState|路徑|A\*"

# PLC寫入日誌
tail -f /tmp/agv.log | rg "PLC.*寫入"

# 任務更新日誌
tail -f /tmp/agv.log | rg "任務.*更新"
```

## 📋 狀態轉換條件

### 進入條件
- `MissionSelectState` 選定任務後轉入
- HMI 設定有效目標點且 `MAGIC > 0`
- 需要重新計算路徑的情況

### 離開條件
- **成功完成**: 路徑計算和PLC寫入成功 → `RunningState`
- **既有路徑**: 檢測到 `AGV_PATH` 已存在 → `RunningState`
- **失敗超限**: 寫入失敗超過5次 → `MissionSelectState`

### 狀態持續條件
- 正在進行A*計算 (`step == 0`)
- 等待PLC寫入回應 (`step == 1, 2`)
- 步驟計數控制 (`step >= 3`)

## 🔍 監控和除錯

### 關鍵監控點
```python
# 路徑規劃狀態
self.node.get_logger().info(f"✅ 準備計算路徑, 執行次數: {self.count}, 當前步驟: {self.step}")

# A*演算法狀態
self.node.get_logger().info(f"✅ A*演算法初始化成功, 現在位置: {self.TagNo}, 目標節點: {self.node.node_id}")

# 路徑計算結果
self.node.get_logger().info(f"✅ 計算路徑成功: {self.path}")

# PLC寫入狀態
self.node.get_logger().info(f"✅ PLC 路徑資料寫入, 執行次數: {self.count}")
```

### 除錯工具
```bash
# 檢查AGV狀態
ros2 topic echo /agv_status

# 檢查路徑資料
ros2 param get /agv_node pathdata

# 檢查任務狀態
ros2 service call /get_task_status
```

## 🔗 系統整合關係

### 前置狀態依賴
- `MissionSelectState`: 提供任務和目標節點資訊
- `AGVCDatabaseClient`: 提供任務物件和資料庫連接
- `PlcClient`: 提供PLC通訊介面

### 後續狀態交接
- `RunningState`: 接收路徑資料，執行實際移動
- `MissionSelectState`: 失敗時的回退狀態

### 資料流傳遞
```
MissionSelectState
    ↓ (task, node_id)
WritePathState
    ↓ (AGV_PATH, pathdata)
RunningState
```

## 💡 最佳實踐和優化建議

### 效能優化
1. **快取路徑資料**: 避免重複計算相同路徑
2. **分批PLC寫入**: 已實作分段寫入，減少單次傳輸負擔
3. **異步處理**: 使用非同步回調避免阻塞

### 穩定性改進
1. **重試機制**: 已實作5次重試限制
2. **狀態檢查**: 優先檢查既有路徑避免重複工作
3. **異常處理**: 完整的try-catch和回調錯誤處理

### 維護建議
1. **定期檢查配置檔案**: 確保路徑資料和站點映射正確
2. **監控PLC記憶體**: 確保DM3000-4999區域可用
3. **資料庫維護**: 定期檢查任務表的完整性

## 🎯 MAGIC 值和 HMI 本地任務觸發機制

### MAGIC 在 MissionSelectState 中的作用
WritePathState 經常由 `MissionSelectState` 的 HMI 本地任務觸發機制啟動：

```python
# 在 MissionSelectState.local_mission() 方法中
if self.node.agv_status.MAGIC > 0:
    if self.node.agv_status.AGV_END_POINT > 0:
        self.node.node_id = self.node.agv_status.AGV_END_POINT
        self.localMission = True
```

### MAGIC 觸發的 WritePathState 執行流程
1. **MAGIC 檢測**: `MissionSelectState.local_mission()` 每秒檢查 MAGIC 值
2. **條件滿足**: MAGIC > 0 且 AGV_END_POINT > 0
3. **狀態設定**: 設定目標節點並啟用本地任務標誌
4. **狀態轉換**: 轉換到 WritePathState 進行路徑規劃
5. **路徑規劃**: WritePathState 使用 AGV_END_POINT 作為目標執行 A* 計算
6. **特殊處理**: 如果 MAGIC = 21，則在 PLC 資料中強制設定動作值為 1

### MAGIC = 21 特殊模式處理
```python
# 在 WritePathState 路徑資料處理中 - 僅影響最後一個點
if y:  # 最後一個點
    if self.node.agv_status.MAGIC == 21:
        self.dataValue[i*20+2] = 21  # MAGIC=21 特殊處理：最後一個點直接給21
    else:
        self.dataValue[i*20+2] = tag.get('Station')+20  # 正常情況：站點+20
    break
else:  # 中間點 (不受 MAGIC 影響)
    self.dataValue[i*20+2] = self.act[0]
```

### HMI 本地任務的路徑規劃特點
- **固定起點**: 使用 "Washing" 站點ID (StationID = 3)
- **動態終點**: 來自 PLC 的 AGV_END_POINT 值
- **即時觸發**: 不需等待 WCS 任務派發，直接執行
- **優先處理**: 在 MissionSelectState 中優先檢查本地任務

## 🔗 交叉引用
- AGV狀態機架構: docs-ai/knowledge/agv-domain/agv-state-machine.md
- A*演算法模組: `app/path_algorithm/CLAUDE.md`
- PLC通訊協議: docs-ai/knowledge/protocols/keyence-plc-protocol.md
- PLC通訊開發: docs-ai/operations/development/ros2/plc-communication.md
- 資料庫操作: docs-ai/operations/development/database-operations.md
- 系統診斷: docs-ai/operations/guides/system-diagnostics.md
- MAGIC值分析: docs-ai/knowledge/agv-domain/magic-value-analysis.md