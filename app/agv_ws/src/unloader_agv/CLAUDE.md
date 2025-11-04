# unloader_agv - 卸載車AGV控制系統

## 📚 Context Loading
../../../../CLAUDE.md  # 引用根目錄系統文檔
../../CLAUDE.md  # 引用上層 agv_ws 工作空間文檔

# Unloader 專屬知識（套件特有）
@docs-ai/knowledge/agv-domain/write-path-state-analysis.md  # 路徑狀態分析

## 📋 專案概述
unloader_agv 實現 Unloader AGV 的完整控制邏輯，支援從烘箱取料、預烘機操作、出料傳送帶操作、數量計算等複雜卸載流程。負責眼鏡生產房間內後段製程的批量卸載操作，包含 3 種設備類型共 4 種業務流程，採用一次2格的高效批量處理方式。

## 🏭 核心業務流程 (基於眼鏡生產系統)

### 主要職責
- **工作區域**: 眼鏡房間內部
- **核心任務**: 後段製程的批量卸載操作
- **關鍵設備**: Hokuyo 8bit 光通訊模組 (前方配置)
- **工作模式**: 一次2格批量操作，提高後段製程效率

### Work ID 自動路由系統
Unloader AGV 採用自動 work_id 路由系統，格式：`room_id + equipment_type + station_number + action_type`

**設備類型編碼**：
- `02` = BOX_OUT_TRANSFER (出口傳送箱)
- `05` = PRE_DRYER (預烘機)
- `06` = OVEN (烘箱)

**動作類型編碼**：
- `01` = TAKE (取料)
- `02` = PUT (放料)

### 4種核心業務流程

#### 製程設備取料
**1. TAKE_PRE_DRYER - 從預烘機取料**
- Work ID: `room_id + "05" + "01-04" + "01"`
- 功能: 從預烘機取出完成預乾燥的載具
- 工作站數量: 4個工作站 (每站包含2個 PORT，總計8格)
- 取料方式: **一次取2格** (批量處理方式)
- 計算邏輯: Port 1-4 → row=1, Port 5-8 → row=2, column=0

**2. TAKE_OVEN - 從烘箱下排取料**
- Work ID: `room_id + "06" + "01-02" + "01"`
- 功能: 從烘箱下排取出完成烘乾的載具
- 烘箱配置: 總共8格 (上排4格 + 下排4格)
- 取料位置: **下排4格** (烤完後的出料位置)
- 取料方式: **一次取2格** (批量處理方式)
- 工作站: 2個工作站 (一次取2格的邏輯)

#### 製程設備放料
**3. PUT_OVEN - 放料到烘箱上排**
- Work ID: `room_id + "06" + "01-02" + "02"`
- 功能: 將載具放入烘箱上排進行烘乾製程
- 烘箱配置: 總共8格 (上排4格 + 下排4格)
- 放料位置: **上排4格** (烘乾製程的進料位置)
- 放料方式: **一次放2格** (批量處理方式)
- 工作站: 2個工作站 (一次放2格的邏輯)

#### 出口轉移
**4. PUT_BOXOUT_TRANSFER - 放料到出口傳送箱**
- Work ID: `room_id + "02" + "01-02" + "02"`
- 功能: 將處理完成的載具放入出口傳送箱
- 出口傳送箱配置: 總共4格 (與入口傳送箱相同結構)
- 放料方式: **一次放2格** (可選擇上2格或下2格)
- 工作站: 2個工作站 (一次放2格的邏輯)

### 製程流程角色分工
- **預烘機**: Loader AGV 負責 PUT_PRE_DRYER，Unloader AGV 負責 TAKE_PRE_DRYER
- **烘箱雙向操作**: Unloader AGV 負責 PUT_OVEN 和 TAKE_OVEN
- **最終出料**: Unloader AGV 負責 PUT_BOXOUT_TRANSFER，等待 Cargo AGV 收集

### 房間內設備物理配置詳解

**預烘機 (PRE_DRYER)**：
- 物理結構: 8格配置 (上排4格 + 下排4格)
- Loader AGV: 負責 PUT_PRE_DRYER (放入)
- Unloader AGV: 負責 TAKE_PRE_DRYER (一次取2格)

**烘箱 (OVEN)**：
- 物理結構: 8格配置 (上排4格 + 下排4格)
- 上排4格: **進料位置** - Unloader PUT_OVEN (一次放2格)
- 下排4格: **出料位置** - Unloader TAKE_OVEN (一次取2格)
- 烘乾流程: 上排進料 → 烘乾製程 → 下排出料

**出口傳送箱 (BOX_OUT_TRANSFER)**：
- 物理結構: 4格配置 (與入口傳送箱相同)
- Unloader AGV: 一次放2格 (可選上2格或下2格)
- Cargo AGV: 取出 Carrier 放回 Rack

### 車載料架配置
- **物理結構**: 4格料架，上排2格 + 下排2格
- **S尺寸產品配置**: 
  - 使用上排2格 + 下排2格 (全部4格可用)
  - 一次處理2格的批量操作
  - 車載容量: 最大4個 Carrier
- **L尺寸產品配置**: 
  - 只使用上排2格 (下排掛勾由人工解下)
  - 一次處理2格 (僅上排操作)
  - 車載容量: 最大2個 Carrier

### 技術實作特點
- **批量處理邏輯**: 所有操作都採用 **一次2格** 的高效方式
- **任務與執行動作的對應關係** ⚠️ 重要設計邏輯：
  - **TAFL 任務定義**: 一次任務的 `batch_size = 4` (處理4個 Carrier)
  - **機械臂能力**: 一次取放動作只能處理 **2格** Carrier
  - **執行邏輯**: **一次任務需要執行 2次取放動作**
    ```
    TAFL 任務: batch_size=4, ports=[2051, 2052, 2055, 2056]
      ↓
    第1次動作: 從 port 2051, 2052 取2格 → 放到 AGV 車上排（或下排）
      ↓
    第2次動作: 從 port 2055, 2056 取2格 → 放到 AGV 車另一排
      ↓
    任務完成: 總共處理 4格 Carrier
    ```
  - **與 Loader AGV 對比**:
    - Loader AGV: 一次任務 = 一次動作 (batch_size=1, 處理1格)
    - Unloader AGV: 一次任務 = 兩次動作 (batch_size=4, 處理4格)
  - **設計優勢**:
    - 平衡了機械臂硬體限制（一次2格）與任務效率（批量4格）
    - 減少 TAFL WCS 任務調度頻率，提升系統整體效率
    - 充分利用 AGV 4格料架容量（上2格 + 下2格）
- **效率提升**: 減少機械臂動作次數，提高整體製程效率
- **參數化控制**: UnloaderRobotParameter 支援多工位參數管理和批量計算
- **車載料架管理**: 上下排分離設計，最佳化批量處理效率
- **完整測試覆蓋**: 具備預烘機計算邏輯和取料數量的專項測試

## 📂 完整檔案結構
## 📂 完整檔案結構

### 核心控制
```
unloader_agv/
├─ agv_core_node.py          # Unloader AGV 核心控制節點
├─ unloader_context.py       # Unloader 狀態管理上下文 (AGV層)
├─ robot_context.py          # 機器人狀態控制 (Robot層)
├─ status_json_recorder.py   # 狀態 JSON 記錄器
├─ launch/launch.py          # ROS 2 啟動配置
└─ robot_states/
    ├─ __init__.py           # 模組初始化檔案
    ├─ base_robot_state.py   # Robot 狀態基類
    ├─ complete_state.py     # 完成狀態
    ├─ idle_state.py         # 閒置狀態
    ├─ unloader_robot_parameter.py  # Unloader 參數計算
    ├─ take_pre_dryer/       # 從預烘機取料（5個狀態檔案）
    ├─ take_oven/            # 從烘箱取料（5個狀態檔案）
    ├─ put_oven/             # 放料到烘箱（5個狀態檔案）
    └─ put_boxout_transfer/  # 放料到出口傳送箱（5個狀態檔案）
```

### 測試說明
⚠️ **Unloader AGV 本身無任何測試**

**測試歷史**：
- **單元測試**：曾於 2025-07-22 實作，於 2025-08-18 被刪除
  - 刪除檔案：`test_pre_dryer_calculation.py`, `test_take_quantity.py`, `README.md`
  - 測試內容：Port 計算邏輯、取料數量計算、邊界條件處理
  - 刪除原因：系統性測試策略調整（提交 672d2719）

- **狀態機測試**：從未實作
  - `robot_states/` 中的所有狀態類別無測試覆蓋
  - `UnloaderRobotParameter` 計算邏輯無測試驗證

**相關測試（WCS 層級）**：
- **位置**：`/app/tafl_wcs_ws/src/tafl_wcs/test/test_unloader_flows.py`
- **測試範圍**：TAFL 流程能否為 Unloader AGV 創建正確的 Task
- **測試內容**：4 個業務流程的任務創建驗證
- **⚠️ 重要限制**：僅測試 WCS 任務調度層，**不測試 AGV 控制邏輯和狀態機**

## 🚀 測試與驗證

### WCS 層級測試（僅測試任務調度）
```bash
# ⚠️ 注意：此測試不驗證 Unloader AGV 本身的控制邏輯
cd /app/tafl_wcs_ws/src/tafl_wcs
python3 test/test_unloader_flows.py

# 測試範圍：
# - ✅ TAFL 流程執行正確性
# - ✅ Task 創建與參數正確性（work_id, batch_size, ports 等）
# - ✅ 資料庫操作正確性
# - ❌ 不測試 AGV 狀態機邏輯
# - ❌ 不測試 robot_states/ 狀態轉換
# - ❌ 不測試 Port 計算和數量計算
```

### AGV 功能驗證建議
**如需驗證 Unloader AGV 本身的功能，建議：**
1. **重新實作單元測試**（參考 Loader AGV 測試結構）
   - 測試 `UnloaderRobotParameter` 計算邏輯
   - 測試各 robot_states 的狀態轉換
   - 測試批量處理邏輯（一次2格）

2. **實際系統運行驗證**
   - 在測試環境執行完整業務流程
   - 驗證 Port 計算：Port 1-4 → row=1, Port 5-8 → row=2
   - 驗證 Take Quantity 邏輯和參數轉換
   - 驗證邊界條件處理

3. **集成測試**（需實作）
   - AGV 狀態機與 PLC 整合測試
   - 視覺定位系統整合測試
   - 完整取放料流程測試

## 📊 配置設定

### AGV 配置檔案
- `/app/config/agv/unloader01_config.yaml` - Unloader01 配置
- `/app/config/agv/unloader02_config.yaml` - Unloader02 配置

### 關鍵配置參數
```yaml
agv_id: "unloader01"
agv_type: "unloader"
robot_arm_enabled: true
vision_system_enabled: true

# 基礎工位配置
stations:
  oven: {approach_angle: 0.0}
  pre_dryer: {approach_angle: 1.57}
  boxout_transfer: {approach_angle: 3.14}

# 測試參數
test_params:
  pre_dryer_port_calculation: true   # Pre Dryer Port 計算邏輯
  take_quantity_logic: true          # Take Quantity 功能邏輯
```

## 🔗 系統整合

### ROS 2 通訊
```bash
# 發布話題
/<agv_id>/status           # Unloader AGV 狀態
/<agv_id>/robot_state      # 機器人狀態

# 訂閱話題
/<agv_id>/cmd              # 任務指令
/system/station_status     # 工位狀態
```

### 外部整合
- **agv_base**: 繼承 3層狀態機架構 (Base → AGV → Robot)
- **plc_proxy_ws**: 工位設備 PLC 控制
- **sensorpart_ws**: 視覺定位和感測器整合

## 🚨 Unloader AGV 專項故障排除

**⚠️ 通用故障排除請參考**: ../CLAUDE.md 故障排除章節

### Unloader AGV 特有問題
- **批量處理邏輯異常**: 檢查一次2格的批量操作邏輯和計算
- **預烘機端口計算錯誤**: 驗證 Port 1-8 的 row 計算邏輯
- **後段工位整合問題**: 檢查 Pre-dryer、Oven、Boxout Transfer 整合
- **批量數量計算異常**: 驗證 Take Quantity 邏輯和參數轉換