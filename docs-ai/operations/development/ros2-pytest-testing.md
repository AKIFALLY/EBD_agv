# ROS 2 Pytest 自動化測試

## 🎯 適用場景
- ROS 2 套件的單元測試和整合測試
- 使用 pytest 框架進行自動化測試
- colcon test 整合測試執行

## 📋 快速設定

### 1. 配置 setup.cfg
```ini
# 在套件根目錄的 setup.cfg 中添加
[tool:pytest]
python_files = test_*.py
testpaths = test
```

### 2. 測試檔案位置
```
package_name/
├── package_name/       # 源代碼
├── test/              # 測試代碼目錄
│   ├── test_*.py      # 測試檔案 (必須 test_ 開頭)
│   └── conftest.py    # pytest 配置 (可選)
├── setup.cfg          # 包含 pytest 配置
└── setup.py
```

### 3. 移除程式碼規範測試檔案
```bash
# RosAGV 專案政策：不使用程式碼規範測試
# 已移除所有工作空間的以下檔案：
rm test/test_copyright.py
rm test/test_flake8.py
rm test/test_pep257.py

# 這些測試已從所有 RosAGV 工作空間中移除
# 我們只專注於功能測試和業務邏輯測試
```

## 🚀 執行測試指令

### 基本測試執行
```bash
# 切換到工作空間目錄
cd /app/package_ws

# 執行單一套件測試
colcon test --packages-select package_name

# 查看測試結果
colcon test-result --verbose
```

### 即時輸出測試結果
```bash
# 執行測試並即時顯示輸出 (推薦)
colcon test --packages-select package_name --event-handlers console_direct+
```

### 實際範例
```bash
# 測試 db_proxy 套件
colcon test --packages-select db_proxy --event-handlers console_direct+

# 測試 traffic_manager 套件
colcon test --packages-select traffic_manager --event-handlers console_direct+

# 測試多個套件
colcon test --packages-select db_proxy traffic_manager --event-handlers console_direct+
```

## 📝 測試檔案範例

### 基本測試結構
```python
# test/test_example.py
import pytest
import unittest
from package_name.module import YourClass

class TestYourClass(unittest.TestCase):
    def setUp(self):
        self.instance = YourClass()
    
    def test_basic_functionality(self):
        result = self.instance.method()
        self.assertEqual(result, expected_value)
    
    def test_error_handling(self):
        with self.assertRaises(ValueError):
            self.instance.invalid_operation()

if __name__ == '__main__':
    unittest.main()
```

### 使用 pytest 風格
```python
# test/test_pytest_style.py
import pytest
from package_name.module import function_to_test

def test_function_returns_correct_value():
    assert function_to_test(input) == expected_output

def test_function_raises_exception():
    with pytest.raises(ValueError):
        function_to_test(invalid_input)

@pytest.fixture
def sample_data():
    return {"key": "value"}

def test_with_fixture(sample_data):
    assert sample_data["key"] == "value"
```

## ⚠️ 注意事項

1. **工作空間路徑**: 必須在工作空間目錄執行 `colcon test`
2. **測試檔案命名**: 必須以 `test_` 開頭才會被掃描
3. **建置後測試**: 執行測試前先 `colcon build --packages-select package_name`
4. **測試隔離**: 每個測試應該獨立，不依賴其他測試的執行順序

## 🔧 進階選項

```bash
# 只執行特定測試檔案
colcon test --packages-select package_name --pytest-args test/test_specific.py

# 執行測試並產生覆蓋率報告
colcon test --packages-select package_name --pytest-args --cov=package_name

# 平行執行測試
colcon test --packages-select package_name --pytest-args -n auto

# 顯示詳細測試資訊
colcon test --packages-select package_name --pytest-args -v
```

## 💡 最佳實踐

1. **保持測試簡單**: 每個測試只測試一個功能
2. **使用描述性名稱**: `test_create_task_with_valid_data()`
3. **適當的 setUp/tearDown**: 確保測試環境乾淨
4. **Mock 外部依賴**: 使用 `unittest.mock` 隔離測試

## 🔗 交叉引用
- 測試檔案管理: @docs-ai/operations/development/test-file-management.md
- ROS 2 開發: @docs-ai/operations/development/ros2-development.md
- Docker 開發: @docs-ai/operations/development/docker-development.md