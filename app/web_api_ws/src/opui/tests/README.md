# OPUI 測試文檔

## 概述

本目錄包含 OPUI 專案的完整測試套件，涵蓋單元測試、整合測試、效能測試和壓力測試。

## 測試結構

```
test/
├── __init__.py                 # 測試模組初始化
├── conftest.py                 # pytest 配置和 fixtures
├── requirements-test.txt       # 測試依賴套件
├── run_tests.py               # 通用測試運行腳本
├── ros2_test.py               # ROS2 專用測試腳本
├── README.md                  # 本文檔
├── TESTING_SUMMARY.md         # 測試摘要文檔
├── test_db.py                 # 資料庫操作測試
├── test_op_ui_socket.py       # Socket.IO 事件處理測試
├── test_op_ui_server.py       # 伺服器模組測試
├── test_routers.py            # API 路由測試
├── test_integration.py        # 整合測試
└── test_performance.py        # 效能和壓力測試

# pytest 配置在 ../setup.cfg 中的 [tool:pytest] 區段
```

## 測試類型

### 1. 單元測試 (Unit Tests)

測試個別模組和函數的功能：

- **test_db.py**: 資料庫操作函數測試
  - 客戶端 CRUD 操作
  - 產品管理功能
  - 任務創建和管理
  - 工作類型和狀態管理

- **test_op_ui_socket.py**: Socket.IO 事件處理測試
  - 連線和斷線處理
  - 使用者登入和會話管理
  - 任務操作（叫空車、派滿車、取消任務）
  - 料架管理
  - 通知功能

- **test_op_ui_server.py**: 伺服器模組測試
  - 伺服器初始化和配置
  - 路由註冊和處理
  - 錯誤處理
  - 中介軟體配置

- **test_routers.py**: API 路由測試
  - 產品 API 端點
  - 製程設定 API 端點
  - 請求驗證和錯誤處理

### 2. 整合測試 (Integration Tests)

測試模組間的整合功能：

- **test_integration.py**: 系統整合測試
  - 伺服器和 Socket 整合
  - 資料庫和 Socket 整合
  - API 和 Socket 整合
  - 完整使用者工作流程
  - 多使用者並發處理

### 3. 效能測試 (Performance Tests)

測試系統效能和可擴展性：

- **test_performance.py**: 效能和壓力測試
  - Socket 連線效能
  - 登入和任務創建效能
  - API 回應時間
  - 記憶體使用穩定性
  - 並發操作處理
  - 高頻率請求處理

## 快速開始

### 1. 安裝測試依賴

```bash
cd /app/web_api_ws/src/opui/test
pip install -r requirements-test.txt
```

### 2. 檢查測試環境

```bash
# 使用 ROS2 專用腳本檢查
python ros2_test.py check

# 或使用通用腳本
python run_tests.py check
```

### 3. 運行所有測試

```bash
# ROS2 標準測試流程（推薦）
cd /app/web_api_ws
all_source  # 或 source install/setup.bash
colcon test --packages-select opui --event-handlers console_direct+

# 查看測試結果
colcon test-result --all --verbose

# 使用 ROS2 專用腳本（推薦）
cd /app/web_api_ws/src/opui/test
python ros2_test.py colcon

# 或直接使用 pytest（需要先設定環境）
python run_tests.py all
```

## 詳細測試指令

### ROS2 專用測試腳本

```bash
# 檢查 ROS2 環境和依賴
python ros2_test.py check

# 使用 colcon 運行所有測試（推薦）
python ros2_test.py colcon

# 直接使用 pytest 運行特定類型測試
python ros2_test.py pytest unit
python ros2_test.py pytest integration
python ros2_test.py pytest performance

# 顯示測試摘要
python ros2_test.py summary
```

### 單元測試

```bash
# 運行所有單元測試
python run_tests.py unit

# 運行特定測試檔案
python run_tests.py specific --file test_db.py

# 運行特定測試函數
python run_tests.py specific --file test_db.py --function test_get_client_existing
```

### 整合測試

```bash
# 運行整合測試
python run_tests.py integration
```

### 效能測試

```bash
# 運行效能測試（排除壓力測試）
python run_tests.py performance

# 運行壓力測試
python run_tests.py stress
```

### 測試覆蓋率

```bash
# 生成測試覆蓋率報告
python run_tests.py coverage

# 查看 HTML 報告
open htmlcov/index.html
```

### 測試報告

```bash
# 生成詳細測試報告
python run_tests.py report

# 查看 HTML 報告
open test_report.html
```

## 測試配置

### setup.cfg [tool:pytest]

主要配置選項（位於 `../setup.cfg`）：

- `testpaths`: 測試目錄
- `python_files`: 測試檔案模式
- `markers`: 測試標記
- `addopts`: 預設選項
- `asyncio_mode`: 異步測試支援
- `filterwarnings`: 警告過濾

### conftest.py

提供的 fixtures：

- `mock_db_session`: 模擬資料庫會話
- `mock_connection_pool`: 模擬連線池
- `sample_*_data`: 範例測試資料
- `mock_sio`: 模擬 Socket.IO 伺服器
- `op_ui_socket`: OpUiSocket 測試實例
- `test_client`: FastAPI 測試客戶端

## 測試最佳實踐

### 1. 測試隔離

- 每個測試都是獨立的，不依賴其他測試的結果
- 使用 mock 隔離外部依賴（資料庫、網路等）
- 測試後自動清理資源

### 2. 測試資料

- 使用 fixtures 提供一致的測試資料
- 避免硬編碼測試值
- 使用工廠函數創建測試物件

### 3. 異步測試

- 使用 `@pytest.mark.asyncio` 標記異步測試
- 正確處理 `await` 和異步上下文

### 4. 錯誤測試

- 測試正常情況和異常情況
- 驗證錯誤處理和恢復機制
- 測試邊界條件

## 常見問題

### 1. 測試運行失敗

**問題**: 測試運行時出現模組導入錯誤

**解決方案**:
```bash
# 確保在正確的環境中運行
cd /app/web_api_ws
all_source  # 或 source install/setup.bash

# 使用 colcon 測試（推薦）
colcon test --packages-select opui --event-handlers console_direct+

# 如果直接使用 pytest，需要設定 Python 路徑
export PYTHONPATH=/app/web_api_ws/src:$PYTHONPATH
```

### 2. 資料庫連線錯誤

**問題**: 測試中出現資料庫連線錯誤

**解決方案**:
- 測試使用 mock 資料庫，不需要真實資料庫連線
- 檢查 `conftest.py` 中的 mock 配置
- 確保測試使用 `mock_connection_pool` fixture

### 3. Socket.IO 測試問題

**問題**: Socket.IO 相關測試失敗

**解決方案**:
- 確保使用 `mock_sio` fixture
- 檢查異步測試標記 `@pytest.mark.asyncio`
- 驗證 mock 設定正確

### 4. 效能測試超時

**問題**: 效能測試因為超時而失敗

**解決方案**:
- 調整測試中的時間限制
- 檢查系統資源使用情況
- 考慮在較快的機器上運行

## 持續整合

### GitHub Actions 配置範例

```yaml
name: OPUI Tests

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-latest

    steps:
    - uses: actions/checkout@v3

    - name: Set up Python
      uses: actions/setup-python@v4
      with:
        python-version: '3.8'

    - name: Install ROS2 and dependencies
      run: |
        # 安裝 ROS2
        sudo apt update
        sudo apt install -y ros-humble-desktop

        # 安裝測試依賴
        pip install -r web_api_ws/src/opui/test/requirements-test.txt

    - name: Build and test
      run: |
        cd web_api_ws
        source /opt/ros/humble/setup.bash
        colcon build --packages-select opui
        source install/setup.bash
        colcon test --packages-select opui --event-handlers console_direct+

    - name: Show test results
      run: |
        cd web_api_ws
        colcon test-result --all --verbose
```

## 貢獻指南

### 新增測試

1. 在適當的測試檔案中新增測試函數
2. 使用描述性的測試名稱
3. 包含文檔字串說明測試目的
4. 遵循現有的測試模式

### 測試命名規範

- 測試檔案: `test_*.py`
- 測試類別: `Test*`
- 測試函數: `test_*`
- 使用描述性名稱，如 `test_login_success`、`test_create_task_validation_error`

### 程式碼覆蓋率

- 目標覆蓋率: 80% 以上
- 重點覆蓋核心業務邏輯
- 包含錯誤處理路徑的測試

## 參考資源

- [pytest 官方文檔](https://docs.pytest.org/)
- [pytest-asyncio 文檔](https://pytest-asyncio.readthedocs.io/)
- [FastAPI 測試指南](https://fastapi.tiangolo.com/tutorial/testing/)
- [SQLModel 測試](https://sqlmodel.tiangolo.com/tutorial/testing/)
- [Socket.IO 測試](https://python-socketio.readthedocs.io/en/latest/testing.html)
