# MAGIC 值系統分析

## 🎯 適用場景
- 理解 MAGIC 值在 AGV 狀態機中的作用和工作機制
- 解決 HMI 本地任務觸發相關問題
- 分析 MAGIC 值的生命週期和狀態管理

## 📋 MAGIC 值概述

MAGIC 是 AGV 系統中的一個重要控制信號，用於觸發 **HMI 本地任務**，實現手動操作介面與 AGV 狀態機的整合。

**定義位置**: `app/agv_ws/src/agv_base/agv_base/agv_status.py:31`
**PLC 記憶體地址**: DM7660 (2個16位元資料)
**資料類型**: 32位元整數 (透過 `get_int(7660, 2)` 讀取)

## 🔄 MAGIC 值生命週期

### 1. 初始化階段
```python
# 在 agv_status.py 第31行
self.MAGIC = None
```

### 2. PLC 讀取階段  
```python
# 在 agv_status.py 第360行
self.MAGIC = dMMemory.get_int(7660, 2)
```

### 3. 狀態發布階段
```python
# 在 agv_node_base.py 第189行
msg.magic = self.agv_status.MAGIC or 0
```

### 4. 本地任務觸發階段
```python
# 在 mission_select_state.py 第122行
if self.node.agv_status.MAGIC > 0:
    if self.node.agv_status.AGV_END_POINT > 0:
        self.node.node_id = self.node.agv_status.AGV_END_POINT
        self.localMission = True
```

## 🎛️ HMI 本地任務機制

### MissionSelectState 中的 MAGIC 處理
`MissionSelectState` 使用定時器每秒檢查 MAGIC 值：

```python
# 進入狀態時啟動定時器
def enter(self):
    self.locamissiontimer = self.node.create_timer(1.0, self.local_mission)

# 每秒執行的檢查邏輯
def local_mission(self):
    if self.node.agv_status.MAGIC > 0:
        if self.node.agv_status.AGV_END_POINT > 0:
            self.node.node_id = self.node.agv_status.AGV_END_POINT
            self.localMission = True
```

### 本地任務觸發條件
1. **MAGIC > 0**: HMI 設定有效的魔術數值
2. **AGV_END_POINT > 0**: HMI 設定有效的目標節點
3. **同時滿足**: 兩個條件必須同時滿足才會觸發

### 特殊 MAGIC 值功能
- **MAGIC = 21**: 特殊模式，在 WritePathState 中僅影響最後一個點，將最後一個點的 `dataValue[i*20+2]` 設為 21 (而非正常的站點+20)

### 觸發後的狀態轉換
```python
# 在 MissionSelectState.handle() 方法中
if self.localMission and not self.node.agv_status.AGV_PATH:
    self.node.get_logger().info(
        f"✅ HMI任務下達---  Magic:{self.node.agv_status.MAGIC}  Dest.:{self.node.agv_status.AGV_END_POINT}")
    context.set_state(WritePathState(self.node))  # 切換到路徑規劃狀態
```

## 📊 MAGIC 值的業務邏輯

### 與 WCS 任務系統的區別
| 特性 | WCS 任務 | HMI 本地任務 (MAGIC) |
|------|----------|---------------------|
| **觸發方式** | 資料庫任務派發 | PLC MAGIC 值 |
| **任務來源** | AGVC 管理系統 | 現場 HMI 操作 |
| **優先級** | 任務優先級排程 | 即時觸發執行 |
| **資料儲存** | 資料庫任務表 | PLC 記憶體暫存 |
| **狀態追蹤** | 完整任務生命週期 | 簡化本地執行 |

### 本地任務的優勢
- **即時回應**: 無需等待 WCS 任務派發
- **簡化操作**: 直接在現場 HMI 設定目標
- **緊急使用**: 適合測試、調試、緊急操作

## 🔍 MAGIC 值監控和除錯

### 監控 MAGIC 值狀態
```bash
# 檢查 AGV 狀態中的 MAGIC 值
ros2 topic echo /agv_status

# 檢查 PLC 記憶體中的原始值
ros2 service call /plc_read plc_interfaces/PLCRead "address: 'DM7660'"
```

### 日誌分析
```bash
# 監控 MAGIC 相關日誌
tail -f /tmp/agv.log | rg "Magic|MAGIC|HMI任務"

# 監控本地任務觸發日誌
tail -f /tmp/agv.log | rg "HMI任務下達"
```

### 除錯檢查點
```python
# 在 local_mission() 方法中的除錯日誌
self.node.get_logger().info(f"(magic={self.node.agv_status.MAGIC}) dest.={self.node.agv_status.AGV_END_POINT}")
```

## ⚠️ 常見問題和解決方案

### 問題1: MAGIC 值無法觸發本地任務
**可能原因**:
- MAGIC 值為 0 或負數
- AGV_END_POINT 值為 0 或無效
- AGV 不在 MissionSelectState 狀態
- 定時器未正確啟動

**診斷方法**:
```bash
# 檢查當前 AGV 狀態
ros2 topic echo /agv_status | head -20

# 檢查 PLC 連接
ros2 service call /plc_read plc_interfaces/PLCRead "address: 'DM7660'"
ros2 service call /plc_read plc_interfaces/PLCRead "address: 'DM7670'"
```

### 問題2: 本地任務設定後無回應
**可能原因**:
- AGV 已有現有路徑 (AGV_PATH 不為空)
- WritePathState 路徑規劃失敗
- PLC 寫入失敗

**診斷方法**:
```bash
# 檢查路徑狀態
ros2 param get /agv_node pathdata

# 檢查狀態機當前狀態
ros2 topic echo /agv_state
```

### 問題3: MAGIC 值讀取異常
**可能原因**:
- PLC 連接中斷
- PLC 記憶體地址配置錯誤
- 資料格式轉換問題

**解決方案**:
```bash
# 重新啟動 PLC 相關服務
ros2 lifecycle set /plc_node shutdown
ros2 run keyence_plc plc_node

# 檢查 PLC 連接狀態
ping <PLC_IP>
telnet <PLC_IP> 8501
```

## 🔧 MAGIC 值最佳實踐

### HMI 操作建議
1. **設定 MAGIC**: 在 HMI 中設定大於 0 的 MAGIC 值
2. **設定目標**: 確保 AGV_END_POINT 對應有效的節點ID
3. **狀態確認**: 確保 AGV 在 MissionSelectState 狀態
4. **清除設定**: 任務完成後清除 MAGIC 和 AGV_END_POINT

### 開發測試建議
```python
# 在測試中模擬 MAGIC 觸發
def test_magic_trigger():
    # 設定 MAGIC 值
    agv_status.MAGIC = 100
    agv_status.AGV_END_POINT = 5
    
    # 執行本地任務檢查
    mission_state.local_mission()
    
    # 驗證本地任務標誌
    assert mission_state.localMission == True
    assert node.node_id == 5
```

### 監控和維護
1. **定期檢查**: 監控 MAGIC 值的變化趨勢
2. **日誌輪轉**: 確保 MAGIC 相關日誌不會過量
3. **PLC 維護**: 定期檢查 PLC 記憶體區域的完整性

## 📋 MAGIC 值技術規格

### PLC 記憶體配置
- **位址**: DM7660-DM7661 (2個16位元暫存器)
- **資料類型**: 32位元無號整數
- **讀取方式**: `dMMemory.get_int(7660, 2)`
- **有效範圍**: 1 到 4,294,967,295 (0 表示無效)

### 相關 PLC 變數
- **MAGIC**: DM7660-7661 (HMI 本地任務觸發信號)
- **AGV_END_POINT**: DM7670-7671 (目標節點ID)
- **AGV_FPGV**: 當前位置TAG (WritePathState 起點)

### 狀態機整合點
```
MissionSelectState (檢查 MAGIC)
    ↓ (MAGIC > 0 && AGV_END_POINT > 0)
WritePathState (執行路徑規劃)
    ↓ (路徑規劃完成)
RunningState (執行移動)
```

## 🔗 交叉引用
- AGV狀態機架構: docs-ai/knowledge/agv-domain/agv-state-machine.md
- WritePathState分析: docs-ai/knowledge/agv-domain/write-path-state-analysis.md
- MissionSelectState: `app/agv_ws/src/agv_base/agv_base/agv_states/mission_select_state.py`
- PLC通訊協議: docs-ai/knowledge/protocols/keyence-plc-protocol.md
- AGV狀態管理: `app/agv_ws/src/agv_base/agv_base/agv_status.py`
- 系統診斷: docs-ai/operations/guides/system-diagnostics.md