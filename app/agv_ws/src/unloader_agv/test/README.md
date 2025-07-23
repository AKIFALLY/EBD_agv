# Unloader AGV 測試套件

## 概述

此目錄包含 `unloader_agv` 工作空間的所有測試檔案，用於驗證 Unloader AGV 的各種功能和計算邏輯。

## 測試檔案

### test_pre_dryer_calculation.py
**功能**: 測試 pre_dryer_port 的計算邏輯
**描述**: 驗證 unloader_agv 中 pre_dryer_port 的 row 和 column 計算是否正確

**測試要求**:
- port 1-4 → row=1, column=0
- port 5-8 → row=2, column=0

**運行方式**:
```bash
# 方法 1: 直接運行（函數版本測試）
cd agv_ws/src/unloader_agv/test
python3 test_pre_dryer_calculation.py

# 方法 2: 運行 unittest 版本
python3 test_pre_dryer_calculation.py --unittest

# 方法 3: 使用 unittest 模組運行
python3 -m unittest test_pre_dryer_calculation -v

# 方法 4: 運行特定測試方法
python3 -m unittest test_pre_dryer_calculation.TestPreDryerCalculation.test_port_1_to_4_calculation -v
```

### test_take_quantity.py
**功能**: 測試 take_quantity 功能邏輯
**描述**: 驗證 unloader_agv 中 take_quantity 的計算邏輯和參數整合處理

**測試內容**:
- Carrier 查詢結果處理邏輯
- Take quantity 計算邏輯
- 參數類型轉換和整合
- 邊界條件和錯誤處理

**運行方式**:
```bash
# 方法 1: 直接運行（函數版本測試）
cd agv_ws/src/unloader_agv/test
python3 test_take_quantity.py

# 方法 2: 運行 unittest 版本
python3 test_take_quantity.py --unittest

# 方法 3: 使用 unittest 模組運行
python3 -m unittest test_take_quantity -v

# 方法 4: 運行特定測試類
python3 -m unittest test_take_quantity.TestTakeQuantityLogic -v
python3 -m unittest test_take_quantity.TestParameterIntegration -v
```

## 測試架構

### 測試框架
- **主要框架**: Python unittest
- **測試類型**: 單元測試
- **相容性**: 提供函數版本和 unittest 版本

### 測試覆蓋範圍
- ✅ **Pre Dryer Port 計算邏輯**
  - Port 1-4 計算邏輯
  - Port 5-8 計算邏輯
  - 邊界條件測試
  - 完整範圍測試 (port 1-8)
- ✅ **Take Quantity 功能邏輯**
  - Carrier 查詢結果處理
  - Take quantity 計算邏輯
  - 參數類型轉換
  - 邊界條件和錯誤處理

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

### 運行所有測試
```bash
# 在 unloader_agv 測試目錄中運行所有測試
cd agv_ws/src/unloader_agv/test
python3 -m unittest discover -v

# 或使用 pytest (如果已安裝)
pytest -v
```

## 相關工作空間

此測試套件屬於 `agv_ws` 工作空間中的 `unloader_agv` 套件。相關的程式碼位於：
- `agv_ws/src/unloader_agv/unloader_agv/` - 主要程式碼
- `agv_ws/src/unloader_agv/unloader_agv/robot_states/take_pre_dryer/` - Pre Dryer 相關狀態

## 注意事項

- 測試檔案應該獨立運行，不依賴外部服務
- 如需模擬 ROS 2 環境，請使用適當的 mock 策略
- 測試應該快速執行，避免長時間等待
- 確保測試具有良好的錯誤訊息和描述
