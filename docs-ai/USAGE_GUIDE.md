# Prompts 使用指南

## 🎯 設計理念

RosAGV Prompts Library 採用 @docs-ai/路徑 語法，讓 CLAUDE.md 可以保持簡潔，同時動態載入豐富的 contextual prompts。

## 📝 @ 引用語法

### 基本語法
```markdown
@docs-ai/category/subcategory/prompt-name.md
```

### 在 CLAUDE.md 中使用
```markdown
# 模組 CLAUDE.md

## 📚 Context Loading
@docs-ai/context/system/technology-stack.md
@docs-ai/context/workspaces/agv-workspaces.md
@docs-ai/operations/development/docker-development.md

## 模組概述
簡潔的模組描述...

## 詳細開發指導
詳細架構說明請參考: @docs-ai/context/workspaces/agv-workspaces.md
```

## 🗂️ Prompts 三層架構系統

### Context (上下文) - 系統和業務層級
```markdown
# 系統層級
@docs-ai/context/system/rosagv-overview.md          # RosAGV 系統概覽
@docs-ai/context/system/dual-environment.md         # 雙環境設計
@docs-ai/context/system/technology-stack.md         # 技術棧架構

# 工作空間層級
@docs-ai/context/workspaces/agv-workspaces.md       # AGV 工作空間
@docs-ai/context/workspaces/agvc-workspaces.md      # AGVC 工作空間
```

### Operations (操作) - 開發和維護
```markdown
# 開發操作
@docs-ai/operations/development/ros2-development.md     # ROS 2 開發
@docs-ai/operations/development/docker-development.md  # Docker 開發
@docs-ai/operations/development/web-development.md     # Web 開發
@docs-ai/operations/development/database-operations.md # 資料庫操作
@docs-ai/operations/development/testing-procedures.md  # 測試程序

# 維護操作
@docs-ai/operations/maintenance/system-diagnostics.md  # 系統診斷
@docs-ai/operations/maintenance/troubleshooting.md     # 故障排除
@docs-ai/operations/maintenance/log-analysis.md        # 日誌分析
```

### Knowledge (知識) - 領域和協定
```markdown
# AGV 領域知識
@docs-ai/knowledge/agv-domain/vehicle-types.md      # 車型規格
@docs-ai/knowledge/agv-domain/navigation-systems.md # 導航系統

# 通訊協定
@docs-ai/knowledge/protocols/zenoh-rmw.md           # Zenoh 通訊
@docs-ai/knowledge/protocols/ros2-interfaces.md     # ROS 2 介面
```

### Context (層級載入) - 依 CLAUDE.md 層級
```markdown
@docs-ai/context/level1/system-overview.md          # 系統概述層級
@docs-ai/context/level2/workspace-agv_ws.md         # 工作空間層級
@docs-ai/context/level3/package-agv_base.md         # 套件詳細層級
```

## 🎯 智能選擇策略

### 根據開發任務選擇
| 開發任務 | 推薦 Prompts 組合 |
|---------|------------------|
| **新手導入** | `AI_LEARNING_GUIDE.md` + `tools/rosagv-unified-tools.md` |
| **日常運維** | `tools/rosagv-unified-tools.md` + `tools/diagnostics/system-health-check.md` |
| **故障排除** | `tools/diagnostics/system-health-check.md` + `tools/logging/log-analysis.md` + `tools/network/connectivity-testing.md` |
| **容器管理** | `tools/docker/container-management.md` + `tools/diagnostics/system-health-check.md` |
| **開發工作** | `tools/development/build-and-test.md` + `tools/docker/container-management.md` |
| **AI 助理學習** | `AI_LEARNING_GUIDE.md` + 所有 `tools/` 系列 |

### 根據問題類型選擇
```markdown
# 系統健康檢查
@docs-ai/operations/maintenance/system-diagnostics.md

# 容器管理問題
@docs-ai/operations/development/docker-development.md
@docs-ai/operations/maintenance/system-diagnostics.md

# 網路連接問題
@docs-ai/operations/maintenance/system-diagnostics.md

# 日誌分析問題
@docs-ai/operations/maintenance/log-analysis.md
@docs-ai/operations/maintenance/system-diagnostics.md

# 開發建置問題
@docs-ai/operations/development/ros2-development.md
@docs-ai/operations/development/docker-development.md

# AI 助理學習
@docs-ai/AI_LEARNING_GUIDE.md
@docs-ai/operations/maintenance/system-diagnostics.md
```

## 📋 CLAUDE.md 精簡模板

### 模板範例
```markdown
# [模組名稱] CLAUDE.md

## 📚 Context Loading
@docs-ai/context/system/technology-stack.md
@docs-ai/context/workspaces/[相關工作空間].md
@docs-ai/operations/[development|maintenance]/[相關操作].md

## 概述
簡潔的模組描述，專注於核心功能和用途

## 關鍵特色
- 核心功能點 1
- 核心功能點 2
- 核心功能點 3

## 快速開始
```bash
# 基本操作指令
quick_command_example
```

## 詳細指導
具體操作請參考: @docs-ai/operations/development/[相關技術]-development.md

## 故障排除
基本除錯請參考: @docs-ai/operations/maintenance/system-diagnostics.md

### 常見問題
```bash
# 問題 1 解決方案
solution_command_1

# 問題 2 解決方案  
solution_command_2
```

詳細除錯流程請參考相關 prompts 檔案。
```

## 🔧 Prompts 開發規範

### Prompt 檔案結構
```markdown
# [功能名稱] Prompt

## 🎯 適用場景
- 場景 1
- 場景 2
- 場景 3

## 📋 核心概念/工具概述
核心概念說明或工具功能介紹

## 🔧 開發指導/操作指南
具體的開發指導或操作步驟

## 📂 關鍵檔案位置/快速指令
重要檔案路徑或常用指令

## 🚀 最佳實踐/進階功能
推薦的實作模式和注意事項

## 🚨 故障排除/常見問題
常見問題和解決方案

## 📋 檢查清單
開發或操作檢查項目
```

### 命名規範
- **任務導向**: `[功能]-development.md`, `[功能]-context.md`
- **工具導向**: `[工具名稱]-management.md`, `[工具名稱]-troubleshooting.md`
- **除錯導向**: `[問題類型]-debugging.md`, `[系統]-health-check.md`

## 💡 使用最佳實踐

### CLAUDE.md 精簡原則
1. **Context Loading 區塊**: 文件開頭列出所有相關 prompts
2. **概述簡潔**: 只保留核心功能描述
3. **指令精選**: 只保留最常用的基本指令
4. **詳細參考**: 用 @docs-ai/路徑 引用詳細內容
5. **故障快速**: 提供最緊急的快速解決方案

### 模組化組合
```markdown
# 根據開發階段組合不同 prompts

## 學習階段
@docs-ai/context/system/technology-stack.md
@docs-ai/context/system/rosagv-overview.md

## 開發階段
@docs-ai/context/workspaces/[相關工作空間].md
@docs-ai/operations/development/[相關技術]-development.md

## 除錯階段
@docs-ai/operations/maintenance/system-diagnostics.md
@docs-ai/operations/maintenance/log-analysis.md
@docs-ai/operations/maintenance/troubleshooting.md
```

### 動態載入策略
- **基礎載入**: 每個 CLAUDE.md 至少載入系統架構 prompt
- **功能載入**: 根據模組功能載入對應的專案 prompts  
- **工具載入**: 根據常用操作載入相關工具 prompts
- **除錯載入**: 根據常見問題載入診斷 prompts

## 🔄 維護和更新

### Prompts 更新原則
1. **保持同步**: Prompts 內容應與實際代碼和工具保持同步
2. **版本追蹤**: 重要變更應在 prompts 中註明
3. **交叉引用**: 相關 prompts 之間應有適當的交叉引用
4. **實例更新**: 保持指令範例和檔案路徑的準確性

### CLAUDE.md 維護
1. **定期審查**: 定期檢查 @prompts 引用是否有效
2. **內容平衡**: 確保 CLAUDE.md 保持簡潔但資訊完整
3. **使用者回饋**: 根據實際使用經驗調整 prompts 組合
4. **文檔一致性**: 確保所有 CLAUDE.md 遵循統一的引用模式