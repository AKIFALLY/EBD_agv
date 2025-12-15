# ROS 2 工作空間測試結構規範

## 🎯 適用場景
- ROS 2 工作空間中測試檔案的標準組織結構
- 區分臨時測試和正式測試的存放位置
- 為 AI Agent 和開發人員提供明確的測試檔案管理指導

## 📋 測試檔案分類與位置

### 測試檔案類型對照表

| 測試類型 | 存放位置 | 檔案命名 | 用途說明 | 生命週期 |
|---------|---------|----------|---------|----------|
| **正式模組測試** | `app/<workspace>/src/<package>/test/` | `test_*.py` | 單元測試、整合測試、功能測試 | 永久保留 |
| **臨時實驗測試** | `~/EBD_agv/agents/` | `test_*.py` | AI Agent 實驗、功能驗證、POC | 定期清理 |
| **ROS 2 標準測試** | `app/<workspace>/src/<package>/test/` | `test_copyright.py`<br>`test_flake8.py`<br>`test_pep257.py` | 代碼品質檢查 | 永久保留 |

## 🏗️ ROS 2 工作空間標準測試結構

### 標準目錄結構
```
app/<workspace_name>_ws/           # 工作空間根目錄
├── src/                           # 源代碼目錄
│   └── <package_name>/            # ROS 2 套件
│       ├── <package_name>/        # Python 模組目錄
│       │   ├── __init__.py
│       │   └── *.py               # 模組程式碼
│       ├── test/                  # 測試目錄 ⭐ 正式測試位置
│       │   ├── __init__.py        # 測試套件初始化
│       │   ├── README.md          # 測試說明文檔
│       │   ├── test_*.py          # 功能測試檔案
│       │   ├── test_copyright.py  # ROS 2 版權檢查
│       │   ├── test_flake8.py     # 代碼風格檢查
│       │   └── test_pep257.py     # 文檔字串檢查
│       ├── launch/                 # Launch 檔案
│       ├── config/                 # 配置檔案
│       ├── package.xml             # ROS 2 套件描述
│       └── setup.py                # Python 套件設定
├── pytest.ini                      # pytest 配置（可選）
├── run_tests.sh                    # 測試執行腳本（可選）
└── ❌ 不應有散落的 .py 測試檔案   # 工作空間根目錄保持清潔

```

### 實際範例對照

#### ✅ 正確結構範例
```bash
# tafl_wcs_ws 的標準結構
app/tafl_wcs_ws/
├── src/tafl_wcs/
│   ├── tafl_wcs/              # 主程式碼
│   └── test/                  # 測試目錄
│       └── test_tafl_executor.py

# db_proxy_ws 的標準結構
app/db_proxy_ws/
├── src/db_proxy/
│   ├── db_proxy/              # 主程式碼
│   └── test/                  # 測試目錄
│       ├── test_base_crud.py
│       ├── test_connection_pool_manager.py
│       └── test_license.py

# tafl_wcs_ws 的標準結構（整理後）
app/tafl_wcs_ws/
├── src/tafl_wcs/
│   ├── tafl_wcs/              # 主程式碼
│   └── test/                  # 測試目錄
│       ├── test_tafl_system.py
│       ├── test_simple_db.py
│       └── test_check_all_data.py
```

#### ❌ 錯誤結構範例
```bash
# 錯誤：測試檔案散落在工作空間根目錄
app/some_ws/
├── test_something.py          # ❌ 不應在此
├── check_data.py              # ❌ 不應在此
├── simple_test.py             # ❌ 不應在此
└── src/...

# 錯誤：測試檔案在 src 目錄下但不在 test 子目錄
app/some_ws/
└── src/package/
    ├── module.py
    └── test_module.py         # ❌ 應該在 test/ 目錄下
```

## 🔧 測試檔案組織最佳實踐

### 1. 建立新工作空間時的測試結構
```bash
# 使用 ROS 2 命令建立套件時會自動建立 test 目錄
cd app/<workspace_name>_ws/src
ros2 pkg create --build-type ament_python <package_name>

# 檢查自動生成的測試結構
ls <package_name>/test/
# 應該看到：test_copyright.py, test_flake8.py, test_pep257.py
```

### 2. 添加功能測試檔案
```python
# 在 test/ 目錄下創建功能測試
# test/test_<functionality>.py

#!/usr/bin/env python3
"""
<功能名稱>測試模組
測試 <package_name> 的 <functionality> 功能
"""

import pytest
import sys
import os

# 添加套件路徑
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from <package_name>.<module> import <function_or_class>

def test_basic_functionality():
    """測試基本功能"""
    assert <function_or_class>() is not None

if __name__ == '__main__':
    pytest.main([__file__])
```

### 3. 建立測試配置檔案

#### pytest.ini (工作空間根目錄)
```ini
[pytest]
testpaths = src/<package_name>/test
python_files = test_*.py
python_classes = Test*
python_functions = test_*
addopts = -v --tb=short
markers =
    unit: Unit tests
    integration: Integration tests
    database: Database tests
    slow: Slow running tests
```

#### test/__init__.py
```python
"""
<Package Name> 測試套件

包含的測試模組：
- test_<functionality1>: <功能1>測試
- test_<functionality2>: <功能2>測試
"""
```

#### test/README.md
```markdown
# <Package Name> 測試目錄

## 測試檔案組織

### 單元測試
- `test_copyright.py` - 版權聲明檢查
- `test_flake8.py` - 代碼風格檢查
- `test_pep257.py` - 文檔字串檢查

### 功能測試
- `test_<functionality>.py` - <功能>測試

## 執行測試

### 執行所有測試
```bash
colcon test --packages-select <package_name>
```

### 執行特定測試
```bash
python3 -m pytest src/<package_name>/test/test_<functionality>.py -v
```
```

### 4. 測試執行腳本 (可選)
```bash
#!/bin/bash
# run_tests.sh - 工作空間根目錄

source /app/setup.bash
source install/setup.bash 2>/dev/null

TEST_TYPE=${1:-all}

case $TEST_TYPE in
    unit)
        python3 -m pytest src/<package_name>/test/ -m unit -v
        ;;
    integration)
        python3 -m pytest src/<package_name>/test/ -m integration -v
        ;;
    all)
        colcon test --packages-select <package_name>
        colcon test-result --verbose
        ;;
esac
```

## 📊 測試檔案清理策略

### 正式測試檔案（永不清理）
位於 `src/<package>/test/` 的檔案是套件的一部分，應該：
- ✅ 納入版本控制
- ✅ 隨套件一起維護
- ✅ 保持與程式碼同步更新
- ❌ 不應該被清理或刪除

### 臨時測試檔案（定期清理）
位於 `~/EBD_agv/agents/` 的檔案是臨時實驗，應該：
- ⚠️ 定期檢查和清理
- ⚠️ 完成驗證後及時刪除
- ❌ 不納入版本控制
- ❌ 不應該長期保留

## 🚨 常見錯誤與修正

### 錯誤 1：測試檔案放錯位置
**問題**：在工作空間根目錄創建測試檔案
```bash
# 錯誤
app/my_ws/test_something.py
```

**修正**：移動到正確位置
```bash
# 正確
mv app/my_ws/test_something.py app/my_ws/src/my_package/test/
```

### 錯誤 2：混淆臨時測試和正式測試
**問題**：將實驗性測試放在套件測試目錄
```bash
# 錯誤：實驗性測試不應該在這裡
app/my_ws/src/my_package/test/experiment_test.py
```

**修正**：實驗性測試應該放在 agents 目錄
```bash
# 正確
mv app/my_ws/src/my_package/test/experiment_test.py ~/EBD_agv/agents/
```

### 錯誤 3：測試檔案命名不規範
**問題**：測試檔案沒有遵循 `test_*.py` 命名規範
```bash
# 錯誤
check_data.py
simple.py
final_check.py
```

**修正**：重命名為標準格式
```bash
# 正確
test_check_data.py
test_simple.py
test_final_check.py
```

## 💡 AI Agent 開發指導

### 創建測試檔案決策樹
```
需要創建測試檔案
├── 是正式的模組功能測試？
│   ├── 是 → 放在 src/<package>/test/test_*.py
│   └── 否 ↓
├── 是臨時實驗或驗證？
│   ├── 是 → 放在 ~/EBD_agv/agents/test_*.py
│   └── 否 ↓
└── 重新評估測試目的
```

### 整理現有測試檔案步驟
1. **識別散落的測試檔案**
   ```bash
   find app/<workspace>/ -name "*.py" -type f | grep -E "(test|check)" | grep -v "/test/"
   ```

2. **分類測試檔案**
   - 功能測試 → 移到 `src/<package>/test/`
   - 實驗測試 → 移到 `~/EBD_agv/agents/`
   - 臨時腳本 → 評估後刪除或歸檔

3. **重命名不規範的檔案**
   ```bash
   # 在容器內執行
   cd /app/<workspace>/src/<package>/test/
   for file in *.py; do
       if [[ ! "$file" =~ ^test_ ]]; then
           mv "$file" "test_$file"
       fi
   done
   ```

4. **建立標準配置檔案**
   - 創建 `test/__init__.py`
   - 創建 `test/README.md`
   - 創建 `pytest.ini`（如需要）

## 🔗 交叉引用
- 臨時測試檔案管理: docs-ai/operations/development/testing/test-file-management.md
- pytest 測試標準: docs-ai/operations/development/testing/testing-standards.md
- ROS 2 開發指導: docs-ai/operations/development/ros2/ros2-development.md
- 核心開發原則: docs-ai/operations/development/core/core-principles.md

---
📅 **創建日期**: 2025-08-22  
🔄 **最後更新**: 2025-08-22  
👤 **作者**: AI Agent (Claude)