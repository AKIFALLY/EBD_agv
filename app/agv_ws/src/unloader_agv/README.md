# Unloader AGV 工作空間

## 概述

`unloader_agv` 是 RosAGV 系統中專門負責卸載作業的 AGV 控制套件。此套件實現了 Unloader AGV 的核心控制邏輯，包括預乾燥機操作、AGV 端口管理、機器人狀態控制等功能。

## 功能特點

### 核心功能
- **預乾燥機操作** (Take Pre Dryer): 從預乾燥機取出載具的完整流程
- **AGV 端口管理**: 動態計算和管理 AGV 端口位址
- **載具轉移**: 將載具從預乾燥機轉移到 AGV 的完整流程
- **狀態機控制**: 基於狀態機的機器人行為管理

### 支援的作業流程
1. **Take Pre Dryer 流程**:
   - 預乾燥機狀態檢查 (`PreDryerCheckHaveState`)
   - AGV 端口空位檢查 (`AgvPortCheckEmptyState`)
   - 預乾燥機取料動作 (`TakePreDryerState`)
   - AGV 放置動作 (`PutAgvState`)

2. **Put Boxout Transfer 流程**:
   - AGV 端口載具檢查 (`AgvPortCheckHaveState`)
   - 載具轉移操作

## 目錄結構

```
unloader_agv/
├── README.md                    # 本文檔
├── package.xml                  # ROS 2 套件配置
├── setup.py                     # Python 套件設定
├── setup.cfg                    # 套件配置
├── launch/                      # 啟動檔案
│   └── launch.py               # 主要啟動檔案
├── resource/                    # 資源檔案
├── test/                        # 測試檔案 ⭐ 新增
│   ├── README.md               # 測試說明文檔
│   ├── __init__.py             # 測試套件初始化
│   ├── test_pre_dryer_calculation.py  # Pre Dryer 計算邏輯測試
│   └── test_take_quantity.py   # Take Quantity 功能邏輯測試
├── unloader_agv/               # 主要程式碼
│   ├── __init__.py
│   ├── robot_context.py        # 機器人上下文
│   ├── unloader_context.py     # Unloader 特定上下文
│   ├── test_agv_core_node.py   # 核心節點測試
│   └── robot_states/           # 機器人狀態實現
│       ├── take_pre_dryer/     # 預乾燥機取料狀態
│       └── put_boxout_transfer/ # 載具轉移狀態
```

## 安裝和建置

### 前置需求
- ROS 2 Jazzy
- Python 3.12+
- 相關依賴套件（見 `package.xml`）

### 建置指令
```bash
# 在 RosAGV 容器環境中
cd /app
source setup.bash
build_ws agv_ws

# 或使用統一建置指令
build_all
```

## 使用方法

### 啟動 Unloader AGV
```bash
# 設定環境變數
export AGV_ID="unloader01"
export ROS_NAMESPACE="/unloader01"

# 啟動 Unloader AGV 節點
ros2 launch unloader_agv launch.py
```

### 環境變數配置
- `AGV_ID`: AGV 識別碼（如 "unloader01", "unloader02"）
- `ROS_NAMESPACE`: ROS 命名空間（通常與 AGV_ID 相同）
- `DEVICE_CONFIG_FILE`: 設備配置檔案路徑

## 測試

### 運行測試
```bash
# 進入測試目錄
cd agv_ws/src/unloader_agv/test

# 運行 Pre Dryer 計算邏輯測試
python3 test_pre_dryer_calculation.py

# 運行 unittest 版本
python3 test_pre_dryer_calculation.py --unittest

# 運行所有測試
python3 -m unittest discover -v
```

### 測試覆蓋範圍
- ✅ **Pre Dryer Port 計算邏輯測試** (`test_pre_dryer_calculation.py`)
  - Port 1-8 的 row 和 column 計算驗證
  - 邊界條件測試
- ✅ **Take Quantity 功能邏輯測試** (`test_take_quantity.py`)
  - Carrier 查詢結果處理邏輯
  - 參數類型轉換和整合
  - 邊界條件和錯誤處理
- 🔄 更多測試正在開發中...

詳細的測試說明請參考 [`test/README.md`](test/README.md)。

## 技術細節

### 動態參數計算
Unloader AGV 使用 `room_id` 動態計算各種參數：

```python
# 預乾燥機參數
port_address = room_id * 1000 + 50    # PRE_DRYER port address
eqp_id = room_id * 100 + 5            # PRE_DRYER eqp_id

# AGV 端口參數
port_address = room_id * 1000 + 110   # AGV port address
eqp_id = room_id * 100 + 11           # AGV eqp_id
```

### Pre Dryer Port 計算邏輯
```python
# Port 1-8 的 row 和 column 計算
pre_dryer_row = 1 if port <= 4 else 2
pre_dryer_column = 0
```

### 狀態機架構
基於 `agv_base.states.state.State` 的狀態機實現，支援：
- 狀態進入/離開處理
- 狀態轉換邏輯
- 錯誤處理和恢復
- 並行操作支援

## 依賴關係

### ROS 2 依賴
- `agv_base`: AGV 基礎功能
- `db_proxy`: 資料庫代理服務
- `plc_proxy`: PLC 通訊代理
- `std_msgs`: 標準訊息類型

### Python 依賴
- `rclpy`: ROS 2 Python 客戶端
- `unittest`: 測試框架

## 開發指南

### 新增狀態
1. 在 `robot_states/` 目錄下建立新的狀態類
2. 繼承適當的基礎狀態類
3. 實現 `enter()`, `handle()`, `leave()` 方法
4. 更新狀態轉換邏輯

### 新增測試
1. 在 `test/` 目錄下建立測試檔案
2. 使用 `test_` 前綴命名
3. 繼承 `unittest.TestCase`
4. 更新 `test/README.md` 文檔

## 故障排除

### 常見問題
1. **環境變數未設定**: 確保 `AGV_ID` 和 `ROS_NAMESPACE` 正確設定
2. **依賴套件缺失**: 檢查 `package.xml` 中的依賴是否已安裝
3. **PLC 通訊問題**: 確認 PLC 代理服務正常運行

### 日誌檢查
```bash
# 檢查 ROS 2 日誌
ros2 node list
ros2 topic list
ros2 service list

# 檢查節點狀態
ros2 node info /unloader01/unloader_agv_node
```

## 相關文檔

- [AGV 基礎套件文檔](../agv_base/README.md)
- [Loader AGV 文檔](../loader_agv/README.md)
- [RosAGV 系統總覽](/app/README.md)
- [測試指南](test/README.md)

## 版本歷史

- **v1.0.0**: 初始版本，支援基本的 Unloader AGV 功能
- **v1.1.0**: 新增測試套件和文檔
