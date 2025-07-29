# AI WCS 測試目錄

## 🎯 pytest 專用測試框架

**⚠️ 重要政策：自 2025-07-29 起，AI WCS 專案統一使用 pytest 測試框架**

所有新的測試檔案必須遵循 pytest 標準，詳細規範請參考 @docs-ai/operations/development/testing-standards.md。

## 📂 目錄結構

```
test/
├── @docs-ai/operations/development/testing-standards.md  # pytest 測試標準規範 ⭐
├── README.md                      # 本文檔
├── test_ai_wcs_pytest.py         # pytest 標準測試檔案 ⭐
├── test_ai_wcs_ros2.py           # ROS 2 相容測試（unittest）
├── run_tests.py                   # 統一測試執行器
├── unit/                          # 單元測試目錄
│   ├── test_business_flow_priority.py
│   ├── test_task_decision.py
│   ├── test_work_id_category.py
│   └── test_parameter_manager_unit.py
├── integration/                   # 整合測試目錄
│   └── test_decision_engine_integration.py
└── functional/                    # 功能測試目錄（預留）
```

## 🚀 快速開始

### 1. 執行 pytest 標準測試
```bash
# 進入 AGVC 容器
agvc_enter && all_source
cd /app/ai_wcs_ws/src/ai_wcs

# 執行 pytest 標準測試
python3 -m pytest test/test_ai_wcs_pytest.py -v
```

### 2. 執行所有測試
```bash
# ROS 2 標準方式（推薦）
cd /app/ai_wcs_ws
colcon test --packages-select ai_wcs
colcon test-result --verbose

# 直接使用 pytest
cd /app/ai_wcs_ws/src/ai_wcs
python3 -m pytest test/ -v
```

### 3. 執行特定類型測試
```bash
# 單元測試
python3 -m pytest test/ -m unit

# 整合測試
python3 -m pytest test/ -m integration

# 排除慢速測試
python3 -m pytest test/ -m "not slow"
```

## 📋 測試檔案說明

### ⭐ 主要測試檔案

#### `test_ai_wcs_pytest.py`
- **框架**: pytest（標準）
- **測試數量**: 13 個核心功能測試
- **覆蓋範圍**: 完整的 AI WCS 核心功能
- **狀態**: ✅ 全部通過

#### `@docs-ai/operations/development/testing-standards.md`
- **用途**: pytest 測試標準規範
- **重要性**: 所有新測試必須遵循
- **包含**: 程式碼範例、最佳實踐、遷移指南

### 🔄 向下相容檔案

#### `test_ai_wcs_ros2.py`
- **框架**: unittest（向下相容）
- **測試數量**: 8 個基礎測試
- **狀態**: 維持現狀，不再新增

#### `unit/` 和 `integration/` 目錄
- **框架**: unittest（舊有檔案）
- **狀態**: 保持運作，但不再擴展
- **政策**: 重構時轉換為 pytest

## ⚠️ 開發指導

### 新測試開發
1. **必須使用 pytest**: 所有新測試檔案使用 pytest 框架
2. **參考標準**: 以 `test_ai_wcs_pytest.py` 為範本
3. **遵循規範**: 嚴格遵循 @docs-ai/operations/development/testing-standards.md
4. **適當標記**: 使用 `@pytest.mark.unit` 等標記

### 程式碼審查
- [ ] 新測試使用 pytest 框架
- [ ] 使用 Python 原生 `assert` 語句  
- [ ] 適當使用 fixtures
- [ ] 正確標記測試類型
- [ ] 包含文檔字串

### 禁止行為
- ❌ 新檔案中使用 `unittest.TestCase`
- ❌ 新檔案中使用 `self.assertEqual()` 等
- ❌ 新檔案中使用 `setUp()` 和 `tearDown()`

## 📊 當前測試統計

- **pytest 測試**: 13 個核心功能測試 ✅
- **總測試數**: 57 個測試（包含 unittest 和靜態檢查）
- **通過率**: 100% (0 errors, 0 failures, 0 skipped)
- **測試時間**: < 1 秒

## 🔗 相關文檔

- [AI WCS 主要文檔](../CLAUDE.md)
- [pytest 測試標準](@docs-ai/operations/development/testing-standards.md)
- [ROS 2 測試官方文檔](https://docs.ros.org/en/jazzy/Tutorials/Testing.html)

---
📅 **最後更新**: 2025-07-29  
🎯 **測試框架}: pytest 專用