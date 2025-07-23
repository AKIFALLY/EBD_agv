# Loader AGV 工作空間

## 概述

`loader_agv` 是 RosAGV 系統中專門負責裝載作業的 AGV 控制套件。此套件實現了 Loader AGV 的核心控制邏輯，包括傳送箱取料、AGV 端口管理、視覺定位、機器人狀態控制等功能。

## 功能特點

### 核心功能
- **Take Transfer 流程**: 從傳送箱取出載具的完整流程
- **AGV 端口管理**: 動態計算和管理 AGV 端口位址
- **視覺定位**: 整合視覺系統進行精確定位
- **狀態機控制**: 基於狀態機的機器人行為管理

### 支援的作業流程
1. **Take Transfer 流程**:
   - AGV 端口空位檢查 (`AgvPortCheckEmptyState`)
   - 傳送箱載具檢查 (`TransferCheckHaveState`)
   - 視覺定位 (`TransferVisionPositionState`)
   - 傳送箱取料動作 (`TakeTransferState`)
   - AGV 放置動作 (`PutAgvState`)

2. **Put Cleaner 流程**:
   - AGV 端口載具檢查 (`AgvPortCheckHaveState`)
   - 清潔機放置操作

## 目錄結構

```
loader_agv/
├── README.md                    # 本文檔
├── package.xml                  # ROS 2 套件配置
├── setup.py                     # Python 套件設定
├── setup.cfg                    # 套件配置
├── launch/                      # 啟動檔案
│   └── launch.py               # 主要啟動檔案
├── resource/                    # 資源檔案
├── test/                        # 測試檔案 ⭐ 完整測試套件
│   ├── README.md               # 測試說明文檔
│   ├── TEST_REPORT.md          # 詳細測試報告
│   ├── __init__.py             # 測試套件初始化
│   ├── conftest.py             # 測試配置和 fixtures
│   ├── run_tests.py            # 測試運行器
│   ├── simple_test_runner.py   # 簡化測試運行器
│   ├── test_demo.py            # Demo 測試（可運行）
│   ├── test_agv_port_check_empty_state.py      # AGV 端口檢查測試
│   ├── test_transfer_check_have_state.py       # 傳送箱檢查測試
│   ├── test_take_transfer_state.py             # 取傳送箱動作測試
│   ├── test_put_agv_state.py                   # 放置 AGV 動作測試
│   ├── test_transfer_vision_position_state.py  # 視覺定位測試
│   └── test_take_transfer_integration.py       # 整合測試
├── loader_agv/                 # 主要程式碼
│   ├── __init__.py
│   ├── agv_core_node.py        # AGV 核心節點
│   ├── robot_context.py        # 機器人上下文
│   ├── loader_context.py       # Loader 特定上下文
│   ├── test_agv_core_node.py   # 核心節點測試（整合測試）
│   ├── robot_states/           # 機器人狀態實現
│   └── loader_states/          # Loader 特定狀態
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

### 啟動 Loader AGV
```bash
# 設定環境變數
export AGV_ID="loader01"
export ROS_NAMESPACE="/loader01"

# 啟動 Loader AGV 節點
ros2 launch loader_agv launch.py
```

### 環境變數配置
- `AGV_ID`: AGV 識別碼（如 "loader01", "loader02"）
- `ROS_NAMESPACE`: ROS 命名空間（通常與 AGV_ID 相同）
- `DEVICE_CONFIG_FILE`: 設備配置檔案路徑

## 測試

### 環境準備
```bash
# 載入完整環境（必須）
cd /app
source setup.bash
source plc_proxy_ws/install/setup.bash
source agv_ws/install/setup.bash

# 進入測試目錄
cd agv_ws/src/loader_agv/test
```

### 運行測試

#### 🚀 **推薦：使用 pytest**（更好的報告和功能）
```bash
# 運行 Demo 測試（推薦入門）
pytest test_demo.py -v

# 運行所有測試
pytest -v

# 運行特定測試類
pytest test_demo.py::TestTakeTransferDemo -v

# 運行特定測試方法
pytest test_demo.py::TestTakeTransferDemo::test_transfer_continuation_logic -v

# 顯示測試覆蓋率（如果安裝了 pytest-cov）
pytest --cov=loader_agv test/ -v

# 並行運行測試（如果安裝了 pytest-xdist）
pytest -n auto -v

# 只運行失敗的測試
pytest --lf -v

# 詳細輸出和即時顯示
pytest -v -s
```

#### 🔧 **傳統：使用 unittest**（向後相容）
```bash
# 運行 Demo 測試
python3 test_demo.py

# 運行所有測試
python3 -m unittest discover -v

# 運行特定測試檔案
python3 -m unittest test_demo -v

# 運行特定測試類
python3 -m unittest test_demo.TestTakeTransferDemo -v

# 運行特定測試方法
python3 -m unittest test_demo.TestTakeTransferDemo.test_transfer_continuation_logic -v

# 使用自定義測試運行器
python3 run_tests.py
python3 simple_test_runner.py
```

#### 📊 **測試框架比較**
| 特性 | pytest 指令 | unittest 指令 | 推薦 |
|------|-------------|---------------|------|
| 基本運行 | `pytest -v` | `python3 -m unittest discover -v` | pytest |
| 特定檔案 | `pytest test_demo.py -v` | `python3 -m unittest test_demo -v` | 平手 |
| 失敗重試 | `pytest --lf -v` | 不支援 | pytest |
| 並行執行 | `pytest -n auto -v` | 不支援 | pytest |
| 覆蓋率報告 | `pytest --cov -v` | 需要額外工具 | pytest |
| 詳細報告 | ⭐⭐⭐⭐⭐ 彩色、詳細 | ⭐⭐⭐ 基本 | pytest |

### 測試覆蓋範圍
- ✅ **AGV Port 檢查邏輯** (`test_agv_port_check_empty_state.py`)
  - 動態參數計算 (port_address, eqp_id)
  - Port 選擇算法 (15 種狀態組合)
  - EQP 信號查詢和 Carrier 查詢

- ✅ **Transfer Continuation Logic** (`test_demo.py`) ⭐ **核心邏輯**
  - 繼續條件：`select_boxin_port=1 AND boxin_port2=True → continue=True`
  - 繼續條件：`select_boxin_port=3 AND boxin_port4=True → continue=True`
  - 其他情況：`continue=False`

- ✅ **機器人動作控制**
  - 取傳送箱動作 (`test_take_transfer_state.py`)
  - 放置 AGV 動作 (`test_put_agv_state.py`)
  - 視覺定位 (`test_transfer_vision_position_state.py`)

- ✅ **整合流程** (`test_take_transfer_integration.py`)
  - 完整的狀態轉換流程
  - 數據流驗證
  - 錯誤處理

詳細的測試說明請參考 [`test/README.md`](test/README.md) 和 [`test/TEST_REPORT.md`](test/TEST_REPORT.md)。

## 技術細節

### 動態參數計算
Loader AGV 使用 `room_id` 動態計算各種參數：

```python
# AGV 端口參數
port_address = room_id * 1000 + 100    # AGV port address
eqp_id = room_id * 100 + 10            # AGV eqp_id

# 傳送箱參數
transfer_port_address = room_id * 1000 + 200  # Transfer port address
transfer_eqp_id = room_id * 100 + 20          # Transfer eqp_id
```

### Transfer Continuation Logic
```python
# 核心繼續邏輯
continue_condition = (
    (select_boxin_port == 1 and boxin_port2) or
    (select_boxin_port == 3 and boxin_port4)
)
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
1. 在 `robot_states/` 或 `loader_states/` 目錄下建立新的狀態類
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
4. **視覺系統問題**: 檢查視覺定位服務狀態

### 日誌檢查
```bash
# 檢查 ROS 2 日誌
ros2 node list
ros2 topic list
ros2 service list

# 檢查節點狀態
ros2 node info /loader01/loader_agv_node
```

## 相關文檔

- [AGV 基礎套件文檔](../agv_base/README.md)
- [Unloader AGV 文檔](../unloader_agv/README.md)
- [Cargo Mover AGV 文檔](../cargo_mover_agv/README.md)
- [RosAGV 系統總覽](/app/README.md)
- [測試指南](test/README.md)
- [詳細測試報告](test/TEST_REPORT.md)

## 版本歷史

- **v1.0.0**: 初始版本，支援基本的 Loader AGV 功能
- **v1.1.0**: 新增完整的測試套件
- **v1.2.0**: 完善視覺定位整合和文檔
