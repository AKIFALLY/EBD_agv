# KUKA 測試套件

完整的 KUKA 相關模組測試套件，包含單元測試、整合測試和模擬環境。

## 📁 檔案結構

```
test/
├── __init__.py                    # 測試包初始化
├── conftest.py                    # pytest 配置和共用 fixtures
├── pytest.ini                    # pytest 設定檔
├── requirements-test.txt          # 測試相依套件
├── README.md                      # 本檔案
│
├── # 單元測試
├── test_kuka_robot.py            # KukaRobot 單元測試
├── test_kuka_container.py        # KukaContainer 單元測試
├── test_kuka_manager.py          # KukaManager 單元測試
│
├── # 整合測試
├── test_kuka_integration.py      # KUKA Fleet API 整合測試
│
├── # 測試工具
├── mock_environment.py           # 模擬測試環境
├── offline_test_server.py        # 離線測試服務器
├── test_environment_manager.py   # 測試環境管理器
│
└── # 執行腳本
├── run_kuka_tests.py             # 單元測試執行腳本
└── run_integration_tests.py      # 整合測試執行腳本
```

## 🚀 快速開始

### 1. 安裝測試相依套件

```bash
# 在 AGVC 容器內執行
cd /app/rcs_ws/src/rcs/test
pip install -r requirements-test.txt
```

### 2. 運行所有測試

```bash
# 基本測試
python run_integration_tests.py

# 詳細輸出 + 覆蓋率報告
python run_integration_tests.py -v --coverage
```

### 3. 運行特定測試

```bash
# 只運行單元測試
python run_kuka_tests.py -v

# 只運行 KukaRobot 測試
python run_kuka_tests.py --robot-only

# 只運行整合測試
python run_integration_tests.py --integration-only
```

## 📋 測試類型

### 單元測試

測試單個類別和方法的功能：

- **test_kuka_robot.py**: 測試 KukaRobot 類別
  - 機器人資料驗證
  - 座標和角度轉換
  - 狀態更新邏輯
  - 電池狀態分析

- **test_kuka_container.py**: 測試 KukaContainer 類別
  - 容器資料驗證
  - Rack 狀態管理
  - 資料庫 CRUD 操作

- **test_kuka_manager.py**: 測試 KukaManager 類別
  - 任務派發邏輯
  - WCS 任務處理
  - API 重試機制
  - 系統狀態管理

### 整合測試

測試模組間的整合和完整流程：

- **test_kuka_integration.py**: 測試 KUKA Fleet API 整合
  - KukaManager 與 Fleet Adapter 整合
  - 端到端任務執行流程
  - 錯誤處理和恢復機制
  - 高負載場景測試

## 🛠️ 測試工具

### Mock Environment

`mock_environment.py` 提供完整的測試環境模擬：

```python
from mock_environment import MockTestEnvironment

# 創建測試環境
env = MockTestEnvironment()

# 取得系統狀態
state = env.get_system_state()

# 模擬網路問題
env.simulate_network_issues(duration=5.0)

# 模擬高延遲
env.simulate_high_latency(delay=2.0)
```

### 離線測試服務器

`offline_test_server.py` 提供獨立的 KUKA Fleet API 模擬服務器：

```bash
# 啟動服務器
python offline_test_server.py --host localhost --port 8080

# 可用 API 端點
# GET  /api/robots - 取得機器人狀態
# GET  /api/containers - 取得容器狀態
# POST /api/missions/move - 創建移動任務
# POST /api/missions/rack_move - 創建貨架移動任務
# GET  /ws - WebSocket 即時更新
```

### 測試環境管理器

`test_environment_manager.py` 提供測試環境配置和管理：

```bash
# 創建測試場景
python test_environment_manager.py create basic_test --description "基本測試場景"

# 列出所有場景
python test_environment_manager.py list

# 載入場景並運行測試
python test_environment_manager.py test basic_test -v

# 啟動帶場景的模擬服務器
python test_environment_manager.py server --scenario basic_test --port 8080
```

### 測試場景

支援多種預定義測試場景：

- **basic**: 基本功能測試
- **high_load**: 高負載測試
- **error_recovery**: 錯誤恢復測試

## 📊 測試報告

測試完成後會生成詳細報告：

- **Markdown 報告**: `kuka_test_report.md`
- **JSON 報告**: `kuka_test_report.json`
- **覆蓋率報告**: `htmlcov/index.html`

### 報告內容

- 測試概要統計
- 單元測試結果
- 整合測試結果
- 錯誤記錄詳情
- 覆蓋率分析

## 🎯 測試場景

### 基本功能測試

```bash
python run_integration_tests.py --scenarios basic
```

測試內容：
- 基本 AGV 狀態更新
- 簡單任務派發
- 容器狀態同步

### 高負載測試

```bash
python run_integration_tests.py --scenarios high_load
```

測試內容：
- 多 AGV 並行派發
- 大量任務處理
- 系統負載均衡

## 🔧 自定義測試

### 添加新的單元測試

1. 在對應的測試檔案中添加測試方法
2. 使用 `conftest.py` 中的 fixtures
3. 遵循命名約定：`test_功能描述`

```python
def test_new_feature(self, mock_rcs_core):
    """測試新功能"""
    # 測試實現
    pass
```

### 添加新的整合測試

1. 在 `test_kuka_integration.py` 中添加測試類別
2. 使用 `MockTestEnvironment` 設置測試環境
3. 測試完整的業務流程

```python
@patch('rcs.kuka_manager.KukaFleetAdapter')
def test_new_integration_scenario(self, mock_adapter, mock_env):
    """測試新的整合場景"""
    # 整合測試實現
    pass
```

### 添加新的測試場景

1. 在 `mock_environment.py` 中定義場景數據
2. 在測試運行器中註冊場景
3. 創建對應的測試用例

## 🐛 故障排除

### 常見問題

1. **ImportError: No module named 'rcs'**
   ```bash
   export PYTHONPATH="/app/rcs_ws/src:$PYTHONPATH"
   ```

2. **Database connection errors**
   - 確保在正確的容器環境中運行
   - 檢查模擬資料庫設置

3. **Test timeout**
   - 調整測試超時設置
   - 檢查模擬環境響應延遲

### 除錯技巧

1. **使用 pytest 除錯選項**
   ```bash
   python -m pytest -v -s --tb=long test_file.py::test_method
   ```

2. **啟用詳細日誌**
   ```bash
   python run_integration_tests.py -v --log-level=DEBUG
   ```

3. **檢查測試覆蓋率**
   ```bash
   python run_integration_tests.py --coverage
   # 查看 htmlcov/index.html
   ```

## 🏃 離線開發工作流程

### 1. 啟動離線測試環境

```bash
# 啟動模擬服務器（在背景執行）
python offline_test_server.py --port 8080 &

# 或使用指定場景
python test_environment_manager.py server --scenario high_load --port 8080 &
```

### 2. 運行開發測試

```bash
# 快速單元測試
python run_kuka_tests.py -v

# 完整整合測試
python run_integration_tests.py --coverage

# 使用特定場景測試
python test_environment_manager.py test basic_test -v
```

### 3. 除錯和分析

```bash
# 查看測試環境狀態
curl http://localhost:8080/test/environment

# 模擬網路問題
curl -X POST http://localhost:8080/test/simulate/network_issues -d '{"duration": 10}'

# 重置環境
curl -X POST http://localhost:8080/test/environment/reset
```

## 🎪 持續整合

### 自動化測試

在開發流程中整合自動化測試：

```bash
#!/bin/bash
# ci_test.sh
set -e

echo "執行 KUKA 測試套件..."
python /app/rcs_ws/src/rcs/test/run_integration_tests.py --coverage

echo "檢查測試結果..."
if [ $? -eq 0 ]; then
    echo "✅ 所有測試通過"
else
    echo "❌ 測試失敗"
    exit 1
fi
```

### Docker 化測試環境

```dockerfile
# Dockerfile.test
FROM python:3.12-slim

WORKDIR /app
COPY requirements-test.txt .
RUN pip install -r requirements-test.txt

COPY . .
CMD ["python", "run_integration_tests.py", "--coverage"]
```

### 測試指標

監控以下測試指標：

- **測試覆蓋率**: 目標 > 80%
- **測試執行時間**: 單元測試 < 2 分鐘，整合測試 < 10 分鐘
- **測試穩定性**: 通過率 > 95%

## 📚 參考資料

- [pytest 官方文檔](https://docs.pytest.org/)
- [unittest.mock 使用指南](https://docs.python.org/3/library/unittest.mock.html)
- [RCS 架構文檔](../CLAUDE.md)
- [KUKA Fleet API 文檔](../../kuka_fleet_ws/CLAUDE.md)