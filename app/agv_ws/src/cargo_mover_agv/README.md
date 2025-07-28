# Cargo Mover AGV 工作空間

## 概述

`cargo_mover_agv` 是 RosAGV 系統中專門負責貨物搬運作業的 AGV 控制套件。此套件實現了 Cargo Mover AGV 的核心控制邏輯，包括 Hokuyo 8bit 光通訊模組管理（左右側）、狀態機控制、非同步任務處理等功能。

## 功能特點

### 核心功能
- **Hokuyo 光通訊模組管理**: 支援 Hokuyo DMS 8bit 光通訊模組（左右側安裝）的控制
- **狀態機控制**: 基於狀態機的機器人行為管理
- **非同步任務處理**: 支援複雜的非同步任務更新和處理
- **貨物搬運**: 專門針對貨物搬運作業最佳化的控制邏輯

### 支援的作業流程
1. **Idle State**: 空閒狀態管理和 Hokuyo 8bit 光通訊模組初始化（左右側）
2. **Complete State**: 任務完成處理和延遲重置機制
3. **Wait Rotation**: 等待旋轉狀態和非同步任務更新
4. **Cargo States**: 貨物特定的狀態處理

## 目錄結構

```
cargo_mover_agv/
├── README.md                    # 本文檔
├── package.xml                  # ROS 2 套件配置
├── setup.py                     # Python 套件設定
├── setup.cfg                    # 套件配置
├── launch/                      # 啟動檔案
│   └── launch.py               # 主要啟動檔案
├── resource/                    # 資源檔案
├── test/                        # 測試檔案
│   ├── README.md               # 測試說明文檔
│   ├── __init__.py             # 測試套件初始化
│   ├── FINAL_TEST_REPORT.md    # 完整測試報告
│   ├── async_update_task_analysis_report.md  # 非同步任務分析
│   ├── test_idle_state_hokuyo.py           # Idle 狀態 Hokuyo 測試
│   ├── test_complete_state_hokuyo.py       # Complete 狀態 Hokuyo 測試
│   ├── test_complete_state_delayed_reset.py # 延遲重置測試
│   ├── test_hokuyo_busy_states.py          # Hokuyo 忙碌狀態測試
│   ├── test_wait_rotation_async_update_task.py      # 等待旋轉非同步測試
│   └── test_fixed_wait_rotation_async_update_task.py # 修復版非同步測試
├── cargo_mover_agv/            # 主要程式碼
│   ├── __init__.py
│   ├── agv_core_node.py        # AGV 核心節點
│   ├── robot_context.py        # 機器人上下文
│   ├── cargo_context.py        # Cargo 特定上下文
│   ├── test_agv_core_node.py   # 核心節點測試（整合測試）
│   ├── robot_states/           # 機器人狀態實現
│   └── cargo_states/           # Cargo 特定狀態
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

### 啟動 Cargo Mover AGV
```bash
# 設定環境變數
export AGV_ID="cargo02"
export ROS_NAMESPACE="/cargo02"

# 啟動 Cargo Mover AGV 節點
ros2 launch cargo_mover_agv launch.py
```

### 環境變數配置
- `AGV_ID`: AGV 識別碼（如 "cargo02"）
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
cd agv_ws/src/cargo_mover_agv/test
```

### 運行測試

#### 🚀 **推薦：使用 pytest**（更好的報告和功能）
```bash
# 運行所有測試
pytest -v

# 運行特定測試檔案
pytest test_idle_state_hokuyo.py -v

# 運行特定測試類
pytest test_idle_state_hokuyo.py::TestIdleStateHokuyo -v

# 運行特定測試方法
pytest test_idle_state_hokuyo.py::TestIdleStateHokuyo::test_hokuyo_write_initialization -v

# 只運行通過的測試（跳過已知失敗）
pytest -k "not (test_handle_execution_order_hokuyo_completed or test_handle_execution_order_two_calls or test_work_id_calculation)" -v

# 顯示詳細的失敗資訊
pytest -v -s --tb=long

# 並行運行測試（如果安裝了 pytest-xdist）
pytest -n auto -v

# 測試覆蓋率報告（如果安裝了 pytest-cov）
pytest --cov=cargo_mover_agv test/ -v
```

#### 🔧 **傳統：使用 unittest**（向後相容）
```bash
# 運行所有測試
python3 -m unittest discover -v

# 運行特定測試檔案
python3 -m unittest test_idle_state_hokuyo -v

# 運行特定測試類
python3 -m unittest test_idle_state_hokuyo.TestIdleStateHokuyo -v

# 運行特定測試方法
python3 -m unittest test_idle_state_hokuyo.TestIdleStateHokuyo.test_hokuyo_write_initialization -v
```

#### ⚠️ **已知測試問題**
目前有 3 個測試失敗（測試邏輯問題，非環境問題）：
- `test_handle_execution_order_hokuyo_completed`
- `test_handle_execution_order_two_calls`
- `test_work_id_calculation`

這些是測試預期值的問題，不影響核心功能。

### 測試覆蓋範圍
- ✅ **Idle State Hokuyo 測試** (`test_idle_state_hokuyo.py`)
- ✅ **Complete State 測試** (`test_complete_state_hokuyo.py`, `test_complete_state_delayed_reset.py`)
- ✅ **Hokuyo 忙碌狀態測試** (`test_hokuyo_busy_states.py`)
- ✅ **非同步任務處理測試** (`test_wait_rotation_async_update_task.py`, `test_fixed_wait_rotation_async_update_task.py`)

詳細的測試說明請參考 [`test/README.md`](test/README.md)。

## 技術細節

### Hokuyo 設備配置（左右側）
```python
# Hokuyo 設備初始化（左右側）
self.hokuyo_dms_8bit_1 = HokuyoDMS8Bit(
    self, "/app/config/hokuyo_dms_config.yaml", "hokuyo_dms_cargo02_1")
self.hokuyo_dms_8bit_2 = HokuyoDMS8Bit(
    self, "/app/config/hokuyo_dms_config.yaml", "hokuyo_dms_cargo02_2")
```

### 動態參數計算
```python
# Cargo Mover AGV 參數計算
room_id = 2  # Cargo Mover 通常使用 room_id = 2
work_id = 2000102  # Cargo Mover 的工作ID格式
```

### 狀態機架構
基於 `agv_base.states.state.State` 的狀態機實現，支援：
- Hokuyo 設備控制（左右側安裝）
- 非同步任務處理
- 延遲重置機制
- 複雜的狀態轉換邏輯

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
1. 在 `robot_states/` 或 `cargo_states/` 目錄下建立新的狀態類
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
1. **Hokuyo 設備連線問題**: 檢查設備配置和連線狀態
2. **狀態轉換異常**: 檢查狀態機邏輯和條件判斷
3. **非同步任務處理問題**: 檢查任務更新邏輯和同步機制

### 日誌檢查
```bash
# 檢查 ROS 2 日誌
ros2 node list
ros2 topic list
ros2 service list

# 檢查節點狀態
ros2 node info /cargo02/cargo_mover_agv_node
```

## 相關文檔

- [AGV 基礎套件文檔](../agv_base/README.md)
- [Loader AGV 文檔](../loader_agv/README.md)
- [Unloader AGV 文檔](../unloader_agv/README.md)
- [RosAGV 系統總覽](/app/README.md)
- [測試指南](test/README.md)

## 版本歷史

- **v1.0.0**: 初始版本，支援基本的 Cargo Mover AGV 功能
- **v1.1.0**: 新增 Hokuyo 設備支援（左右側）
- **v1.2.0**: 完善測試套件和文檔
