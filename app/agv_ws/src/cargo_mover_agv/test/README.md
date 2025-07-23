# Cargo Mover AGV 測試套件

## 概述

此目錄包含 `cargo_mover_agv` 工作空間的所有測試檔案，用於驗證 Cargo Mover AGV 的各種功能，包括狀態機邏輯、Hokuyo 設備整合、非同步任務處理等核心功能。

## 測試檔案

### test_idle_state_hokuyo.py
**功能**: 測試 IdleState 中的 Hokuyo 寫入功能
**描述**: 驗證 Idle 狀態下 Hokuyo 設備的初始化和寫入操作

### test_complete_state_hokuyo.py
**功能**: 測試 CompleteState 中的 Hokuyo 功能
**描述**: 驗證任務完成狀態下的 Hokuyo 設備操作

### test_complete_state_delayed_reset.py
**功能**: 測試 CompleteState 的延遲重置功能
**描述**: 驗證任務完成後的延遲重置邏輯

### test_hokuyo_busy_states.py
**功能**: 測試 Hokuyo 忙碌狀態處理
**描述**: 驗證 Hokuyo 設備在忙碌狀態下的行為和狀態轉換

### test_wait_rotation_async_update_task.py
**功能**: 測試等待旋轉時的非同步任務更新
**描述**: 驗證旋轉等待狀態下的非同步任務處理邏輯

### test_fixed_wait_rotation_async_update_task.py
**功能**: 測試修復後的等待旋轉非同步任務更新
**描述**: 驗證修復後的旋轉等待狀態非同步處理邏輯

## 運行測試

### 環境準備（必須）
```bash
# 載入完整環境
cd /app
source setup.bash
source plc_proxy_ws/install/setup.bash
source agv_ws/install/setup.bash

# 進入測試目錄
cd agv_ws/src/cargo_mover_agv/test
```

### 🚀 **推薦：使用 pytest**

#### 運行所有測試
```bash
# 運行所有測試（推薦）
pytest -v

# 運行所有測試並顯示詳細輸出
pytest -v -s

# 跳過已知失敗的測試
pytest -k "not (test_handle_execution_order_hokuyo_completed or test_handle_execution_order_two_calls or test_work_id_calculation)" -v
```

#### 運行特定測試
```bash
# 運行特定測試檔案
pytest test_idle_state_hokuyo.py -v
pytest test_complete_state_hokuyo.py -v
pytest test_hokuyo_busy_states.py -v

# 運行特定測試類
pytest test_idle_state_hokuyo.py::TestIdleStateHokuyo -v

# 運行特定測試方法
pytest test_idle_state_hokuyo.py::TestIdleStateHokuyo::test_hokuyo_write_initialization -v
```

#### 高級 pytest 功能
```bash
# 只運行失敗的測試
pytest --lf -v

# 並行運行測試（需要 pytest-xdist）
pytest -n auto -v

# 測試覆蓋率報告（需要 pytest-cov）
pytest --cov=cargo_mover_agv --cov-report=html -v

# 詳細的失敗報告
pytest --tb=long -v
```

### 🔧 **傳統：使用 unittest**

#### 運行所有測試
```bash
# 運行所有測試
python3 -m unittest discover -v
```

#### 運行特定測試
```bash
# 運行特定測試檔案
python3 -m unittest test_idle_state_hokuyo -v
python3 -m unittest test_complete_state_hokuyo -v
python3 -m unittest test_hokuyo_busy_states -v

# 直接執行測試檔案
python3 test_idle_state_hokuyo.py

# 運行特定測試方法
python3 -m unittest test_idle_state_hokuyo.TestIdleStateHokuyo.test_hokuyo_write_initialization -v
```

## 測試架構

### 測試框架
- **主要框架**: Python unittest
- **Mock 策略**: unittest.mock
- **ROS 2 整合**: 模擬 ROS 2 節點和服務

### 測試覆蓋範圍
- ✅ **Idle State 測試**
  - Hokuyo 設備初始化
  - 寫入操作驗證
  - 狀態轉換邏輯

- ✅ **Complete State 測試**
  - 任務完成處理
  - 延遲重置機制
  - Hokuyo 設備操作

- ✅ **Hokuyo 設備測試**
  - 忙碌狀態處理
  - 設備通訊驗證
  - 錯誤處理機制

- ✅ **非同步任務處理**
  - 等待旋轉邏輯
  - 任務更新機制
  - 並發處理驗證

## 測試報告

### 詳細測試報告
- [`FINAL_TEST_REPORT.md`](FINAL_TEST_REPORT.md) - 完整的測試執行報告
- [`async_update_task_analysis_report.md`](async_update_task_analysis_report.md) - 非同步任務分析報告

## 開發指南

### 新增測試檔案
1. 在此目錄下建立新的測試檔案，使用 `test_` 前綴
2. 繼承 `unittest.TestCase` 類
3. 使用描述性的測試方法名稱
4. 更新此 README.md 檔案

### 測試命名慣例
- 測試檔案: `test_<功能名稱>.py`
- 測試類: `Test<功能名稱>`
- 測試方法: `test_<具體測試內容>`

### Mock 策略
- 使用 `unittest.mock.Mock` 模擬 ROS 2 節點
- 模擬 Hokuyo 設備和 PLC 通訊
- 模擬資料庫操作和外部服務

## 相關工作空間

此測試套件屬於 `agv_ws` 工作空間中的 `cargo_mover_agv` 套件。相關的程式碼位於：
- `agv_ws/src/cargo_mover_agv/cargo_mover_agv/` - 主要程式碼
- `agv_ws/src/cargo_mover_agv/cargo_mover_agv/robot_states/` - 機器人狀態實現
- `agv_ws/src/cargo_mover_agv/cargo_mover_agv/cargo_states/` - Cargo 特定狀態

## 注意事項

- 測試檔案應該獨立運行，不依賴外部服務
- 使用適當的 mock 策略模擬 ROS 2 環境
- 測試應該快速執行，避免長時間等待
- 確保測試具有良好的錯誤訊息和描述
- 測試前需要初始化 ROS 2 環境（如果需要）
