# 測試檔案管理規範

## 🎯 適用場景
- AI Agent 創建臨時測試檔案時的存放位置指導
- 開發人員進行功能測試和實驗時的檔案管理
- 保持專案目錄結構整潔有序

## 📋 測試檔案存放規範

### 專用測試目錄
**所有臨時性測試檔案必須存放在 `~/RosAGV/agents/` 目錄**

```bash
~/RosAGV/
├── agents/                    # 臨時檔案統一管理目錄
│   ├── test_*.py             # Python 測試腳本
│   ├── test_*.html           # HTML 測試頁面
│   ├── test_*.md             # 測試報告文檔
│   ├── temp.sh               # 臨時 Shell 腳本
│   ├── *.png                 # 測試截圖
│   └── wcs-tafl-editor/    # 特定功能測試子目錄
├── app/                      # 生產代碼目錄（禁止放置臨時檔案）
└── docs-ai/                  # 文檔目錄（禁止放置臨時檔案）
```

### 路徑對應關係
- **宿主機路徑**: `~/RosAGV/agents/`
- **容器內路徑**: `/app/agents/` (通過 Docker volume 掛載)
- **統一管理**: 所有臨時檔案（測試腳本、temp.sh、測試頁面等）都在此目錄

### 命名規範

#### Python 測試檔案
```bash
# ✅ 正確命名
test_tafl_editor.py
test_tafl_editor_interactions.py
test_dsl_phase2_3.py

# ❌ 避免命名
temp.py
test.py
1.py
```

#### HTML 測試頁面
```bash
# ✅ 正確命名
test_tafl_editor_fix.html
tafl_editor_visual_fixes_test.html

# ❌ 避免命名
test.html
page.html
```

#### 測試報告
```bash
# ✅ 正確命名
test_report_final.md
tafl_editor_test_summary.md

# ❌ 避免命名
report.md
result.txt
```

## 🔧 檔案類型指導

### Python 測試腳本
- **用途**: Playwright 自動化測試、功能驗證腳本
- **位置**: `~/RosAGV/agents/test_*.py`
- **範例**: `test_tafl_editor_simple.py`

### HTML 測試頁面
- **用途**: 獨立的前端功能測試頁面
- **位置**: `~/RosAGV/agents/*.html`
- **範例**: `tafl_editor_unified_dark_theme_fix.html`

### 測試截圖
- **用途**: 自動化測試產生的截圖證據
- **位置**: `~/RosAGV/agents/*.png`
- **範例**: `test_drag_drop_debug.png`

### 測試報告
- **用途**: 測試結果記錄和分析報告
- **位置**: `~/RosAGV/agents/*.md` 或 `*.json`
- **範例**: `test_report_final.md`, `tafl_editor_interaction_report.json`

## ⚠️ 禁止事項

### 不可在以下目錄創建臨時檔案
```bash
# ❌ 禁止位置（包括但不限於）
/home/ct/RosAGV/app/                    # 生產代碼根目錄
/home/ct/RosAGV/app/*/src/              # 工作空間源碼目錄
/home/ct/RosAGV/docs-ai/                # 文檔目錄
/home/ct/RosAGV/scripts/                # 系統腳本目錄
```

### 正確做法
```bash
# ✅ 統一放在 agents 目錄
~/RosAGV/agents/temp.sh                 # 臨時 Shell 腳本
~/RosAGV/agents/test_feature.py         # 測試腳本
~/RosAGV/agents/experiment.html         # 實驗頁面
```

### 避免的行為
- ❌ 在生產代碼目錄創建 `test.py`、`temp.py` 或 `temp.sh`
- ❌ 在工作空間 src 目錄下創建臨時檔案
- ❌ 混淆臨時檔案和生產代碼
- ❌ 忘記清理過時的臨時檔案

## 🧹 清理策略

### 定期清理
```bash
# 查看 agents 目錄大小
du -sh ~/RosAGV/agents/

# 列出超過 7 天的測試檔案
find ~/RosAGV/agents/ -type f -mtime +7 -name "test_*"

# 清理特定模式的檔案（謹慎使用）
# rm ~/RosAGV/agents/test_old_*.py
```

### 保留原則
- **保留**: 正在使用的測試腳本
- **保留**: 有參考價值的測試報告
- **清理**: 已完成功能的臨時測試檔案
- **清理**: 重複或過時的測試截圖

## 📊 現有測試檔案分類

### TAFL Editor 相關測試
```bash
agents/
├── tafl_editor_*.html              # TAFL Editor UI 測試頁面
├── test_tafl_editor*.py            # 自動化測試腳本
├── tafl_editor_tests_*.md          # TAFL Editor 測試報告
└── wcs-tafl-editor/                # TAFL Editor 專用測試目錄
```

### DSL 相關測試
```bash
agents/
├── test_dsl_*.py                     # DSL 功能測試
├── test_flowparser_*.py              # Flow 解析器測試
└── test_flowstore_*.py               # Flow 存儲測試
```

### 交互測試
```bash
agents/
├── test_*_interactions.py            # 用戶交互測試
├── test_*_clicks*.py                 # 點擊事件測試
└── test_drag_drop*.py                # 拖放功能測試
```

## 💡 最佳實踐

### AI Agent 創建測試檔案時
1. **確認目錄**: 始終在 `~/RosAGV/agents/` 創建測試檔案
2. **明確命名**: 使用描述性的檔案名稱
3. **添加註解**: 在測試檔案開頭說明用途
4. **記錄結果**: 生成對應的測試報告

### 測試檔案頭部範例
```python
#!/usr/bin/env python3
"""
測試檔案: test_tafl_editor_connection.py
用途: 測試 TAFL Editor 連線渲染功能
創建日期: 2025-08-11
AI Agent: Claude
狀態: 臨時測試檔案，功能驗證後可刪除
"""
```

## 🔗 交叉引用
- ROS 2 工作空間測試結構: docs-ai/operations/development/testing/ros2-workspace-test-structure.md
- 測試標準: docs-ai/operations/development/testing/testing-standards.md
- 測試程序: docs-ai/operations/development/testing/testing-procedures.md
- 開發原則: docs-ai/operations/development/core/core-principles.md