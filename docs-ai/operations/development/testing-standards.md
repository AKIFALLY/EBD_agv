# RosAGV 測試標準規範

## 🎯 適用場景
- 為所有 RosAGV 模組提供統一的測試框架標準
- 指導 AI Agent 進行測試相關開發
- 確保測試程式碼的一致性和可維護性

## 📋 測試框架政策

**⚠️ 重要決策：RosAGV 專案統一使用 pytest 測試框架**

自 2025-08-11 起，所有新的測試檔案必須使用 **pytest** 框架，不再使用 unittest。

### 政策範圍
- **AI WCS**: 已實施 pytest 專用標準
- **其他模組**: 建議逐步採用
- **新模組**: 必須使用 pytest

## 📋 pytest 標準規範

### 1. 測試檔案命名
```
test_*.py           # 標準 pytest 檔案命名
*_test.py           # 替代命名方式（不推薦）
```

### 2. 測試函數命名
```python
def test_function_name():     # ✅ 正確
def test_class_method():      # ✅ 正確
def TestClassName():          # ❌ 錯誤（類別命名）
```

### 3. 斷言方式
```python
# ✅ 使用 Python 原生 assert
def test_business_priority():
    priority = BusinessFlowPriority.AGV_ROTATION
    assert priority.value == 100
    assert priority.name == 'AGV_ROTATION'
    assert isinstance(priority, BusinessFlowPriority)

# ❌ 禁止使用 unittest 斷言
def test_wrong_assertion():
    # 不要使用這些
    self.assertEqual(value, expected)      # unittest 風格
    self.assertTrue(condition)             # unittest 風格  
    self.assertIn(item, container)         # unittest 風格
```

### 4. Fixture 使用
```python
import pytest

@pytest.fixture
def sample_decision():
    """創建範例任務決策"""
    return TaskDecision(
        work_id='220001',
        task_type='agv_rotation',
        priority=BusinessFlowPriority.AGV_ROTATION,
        source_location=10001,
        target_location=10001,
        reason='測試用途'
    )

def test_decision_creation(sample_decision):
    """使用 fixture 的測試"""
    assert sample_decision.work_id == '220001'
    assert sample_decision.priority.value == 100
```

### 5. 標記（Marks）使用
```python
import pytest

@pytest.mark.unit
def test_priority_values():
    """單元測試標記"""
    pass

@pytest.mark.integration  
def test_database_integration():
    """整合測試標記"""
    pass

@pytest.mark.slow
def test_performance():
    """慢速測試標記"""
    pass

@pytest.mark.database
def test_db_operations():
    """需要資料庫的測試標記"""
    pass
```

### 6. 參數化測試
```python
@pytest.mark.parametrize("work_id,expected_type", [
    ('220001', 'kuka-移動貨架'),
    ('230001', 'kuka-流程觸發'),
    ('100001', 'opui-call-empty'),
    ('100002', 'opui-dispatch-full'),
])
def test_work_id_mapping(work_id, expected_type):
    manager = WorkIDParameterManager()
    result = manager.get_work_id_info(work_id)
    assert result['type'] == expected_type
```

## 🔧 測試檔案結構標準

### 推薦的測試檔案結構
```python
#!/usr/bin/env python3
"""
模組功能 pytest 測試
簡要描述測試的目標和範圍
"""

import pytest
import sys
import os

# 路徑設定
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# 導入被測試模組
from module_name import ClassName, function_name


# Fixtures（如果需要）
@pytest.fixture
def sample_data():
    """範例資料 fixture"""
    return {"key": "value"}


# 測試函數
def test_basic_functionality():
    """測試基本功能"""
    # 準備
    input_data = "test_input"
    
    # 執行
    result = function_name(input_data)
    
    # 驗證
    assert result is not None
    assert isinstance(result, str)
    assert len(result) > 0


@pytest.mark.integration
def test_integration_scenario():
    """整合測試範例"""
    # 整合測試程式碼
    pass


if __name__ == '__main__':
    pytest.main([__file__])
```

## 🚫 廢棄 unittest 使用

### 現有 unittest 檔案處理
- **保持現狀**：已存在的 unittest 檔案繼續運行，但不再維護
- **新增功能**：所有新測試必須使用 pytest
- **重構時機**：當需要大幅修改舊測試時，同時轉換為 pytest

### 不再使用的模式
```python
# ❌ 禁止新建這種測試類別
import unittest

class TestSomething(unittest.TestCase):
    def setUp(self):
        pass
        
    def test_something(self):
        self.assertEqual(a, b)
        
    def tearDown(self):
        pass
```

## 📊 執行測試指令

### ROS 2 環境測試
```bash
# 進入對應容器環境
agvc_enter && all_source    # AGVC 模組
agv_enter && all_source     # AGV 模組

# ROS 2 標準方式（推薦）
colcon test --packages-select module_name
colcon test-result --verbose

# 直接使用 pytest（開發調試）
python3 -m pytest test/test_specific_file.py -v
python3 -m pytest test/ -v --tb=short

# 執行特定標記的測試
python3 -m pytest test/ -m unit
python3 -m pytest test/ -m integration
python3 -m pytest test/ -m "not slow"
```

### 測試覆蓋率檢查
```bash
# 生成測試覆蓋率報告
python3 -m pytest test/ --cov=module_name --cov-report=html
python3 -m pytest test/ --cov=module_name --cov-report=term-missing
```

## 🔍 程式碼審查檢查項目

### 新 Pull Request 必須檢查：
- [ ] 所有新測試檔案使用 pytest 框架
- [ ] 測試函數使用 `test_` 前綴命名
- [ ] 使用 Python 原生 `assert` 語句
- [ ] 適當使用 pytest fixtures
- [ ] 正確標記測試類型（unit/integration/slow/database）
- [ ] 測試檔案包含適當的文檔字串
- [ ] 所有測試通過且覆蓋率足夠

### 禁止的程式碼模式：
- [ ] 新檔案中使用 `unittest.TestCase`
- [ ] 新檔案中使用 `self.assertEqual()` 等 unittest 斷言
- [ ] 新檔案中使用 `setUp()` 和 `tearDown()` 方法

## 📋 unittest 到 pytest 遷移指南

### 轉換範例

**原始 unittest 程式碼**：
```python
import unittest

class TestBusinessFlow(unittest.TestCase):
    def setUp(self):
        self.manager = WorkIDParameterManager()
    
    def test_priority_values(self):
        priority = BusinessFlowPriority.AGV_ROTATION
        self.assertEqual(priority.value, 100)
        self.assertGreater(priority.value, 90)
    
    def tearDown(self):
        del self.manager
```

**轉換為 pytest**：
```python
import pytest

@pytest.fixture
def manager():
    """WorkIDParameterManager fixture"""
    return WorkIDParameterManager()

def test_priority_values():
    """測試優先級數值"""
    priority = BusinessFlowPriority.AGV_ROTATION
    assert priority.value == 100
    assert priority.value > 90

def test_manager_functionality(manager):
    """測試管理器功能"""
    assert manager is not None
    assert hasattr(manager, 'WORK_ID_MAPPINGS')
```

### 常見轉換模式

| unittest | pytest |
|----------|--------|
| `self.assertEqual(a, b)` | `assert a == b` |
| `self.assertTrue(x)` | `assert x` |
| `self.assertFalse(x)` | `assert not x` |
| `self.assertIn(a, b)` | `assert a in b` |
| `self.assertIsNone(x)` | `assert x is None` |
| `self.assertIsNotNone(x)` | `assert x is not None` |
| `self.assertRaises(Exception)` | `pytest.raises(Exception)` |

## 🎯 模組專用 pytest 配置

### pytest.ini 設定範例
```ini
[tool:pytest]
testpaths = src/module_name/test
python_files = test_*.py
python_classes = Test*
python_functions = test_*
addopts = -v --tb=short --strict-markers
markers =
    unit: 單元測試標記
    integration: 整合測試標記
    functional: 功能測試標記
    slow: 慢速測試標記
    database: 需要資料庫的測試標記

# RosAGV 測試框架政策
# 注意：自 2025-08-11 起，所有新測試必須使用 pytest 框架
# 詳細規範請參考: @docs-ai/operations/development/testing-standards.md
```

## 🔧 AI Agent 開發指導

### 為新模組建立測試時
1. **優先使用 pytest**: 所有新測試檔案使用 pytest 框架
2. **參考 AI WCS**: 以 `app/ai_wcs_ws/src/ai_wcs/test/test_ai_wcs_pytest.py` 為範本
3. **遵循規範**: 嚴格遵循本文檔標準
4. **適當標記**: 使用 `@pytest.mark.unit` 等標記

### 修改現有測試時
1. **保持相容**: 現有 unittest 測試繼續運行
2. **新增使用 pytest**: 新增的測試函數使用 pytest 風格
3. **重構時轉換**: 大幅修改時同時轉換為 pytest
4. **更新引用**: 更新 CLAUDE.md 中的測試標準引用

## 🎯 執行標準

**自 2025-08-11 起，所有 RosAGV 相關的新測試開發都必須遵循此 pytest 標準。**

違反此標準的程式碼將不被接受，直到修正為符合 pytest 規範為止。

### 適用範圍
- **AI WCS**: 已全面實施
- **新模組**: 必須遵循
- **現有模組**: 建議逐步採用

## 🔗 交叉引用
- ROS 2 工作空間測試結構: @docs-ai/operations/development/ros2-workspace-test-structure.md
- 測試檔案管理: @docs-ai/operations/development/test-file-management.md
- AI WCS 測試實作: `app/ai_wcs_ws/CLAUDE.md`
- ROS 2 開發: @docs-ai/operations/development/ros2-development.md
- 容器開發: @docs-ai/operations/development/docker-development.md
- 核心開發原則: @docs-ai/operations/development/core-principles.md

---
📅 **生效日期**: 2025-08-11  
🔄 **最後更新**: 2025-08-11  
👤 **制定者**: RosAGV 開發團隊