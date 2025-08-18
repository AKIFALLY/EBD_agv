# RosAGV CLAUDE.md

## ⚠️ AI Agent 必讀規則
1. **文檔優先原則**：在修改任何代碼前，必須先查看相關 @docs-ai/ 文檔
2. **禁止假設**：不可基於代碼片段做業務邏輯假設，必須查文檔確認
3. **規格權威性**：@docs-ai/ 是系統規格的唯一權威來源

## 📚 核心規格文檔（修改前必查）
- 產品和載具規格：@docs-ai/knowledge/agv-domain/vehicle-types.md
- 資料庫設計：@docs-ai/knowledge/agv-domain/wcs-database-design.md
- Work ID 系統：@docs-ai/knowledge/agv-domain/wcs-workid-system.md

## 📚 Context Loading
@docs-ai/context/system/rosagv-overview.md
@docs-ai/context/system/dual-environment.md
@docs-ai/context/system/technology-stack.md
@docs-ai/knowledge/protocols/keyence-plc-protocol.md
@docs-ai/operations/development/plc-communication.md
@docs-ai/knowledge/system/linear-flow-advanced-features.md
@docs-ai/operations/development/linear-flow-troubleshooting-cases.md

## ⚠️ 重要開發注意事項
**所有 ROS 2 程式必須在 Docker 容器內執行，宿主機無 ROS 2 環境。**

**容器內指令執行格式**: @docs-ai/operations/development/ros2-container-commands.md

## 開發指導
@docs-ai/operations/development/ros2-development.md
@docs-ai/operations/development/docker-development.md

## 維護支援
@docs-ai/operations/maintenance/system-diagnostics.md
@docs-ai/operations/maintenance/troubleshooting.md

## 領域知識
@docs-ai/knowledge/agv-domain/vehicle-types.md
@docs-ai/knowledge/protocols/zenoh-rmw.md

## AI 開發助手指導

### 🔍 核心開發原則
@docs-ai/operations/development/core-principles.md

### 🔧 統一工具系統
@docs-ai/operations/tools/unified-tools.md

### 📋 模組文檔索引
@docs-ai/context/structure/module-index.md

### 🐍 Python 開發環境
- **uv**: 高效能 Python 套件管理器
- **.venv**: 虛擬環境 (含 playwright 等測試工具)
- **使用**: `source .venv/bin/activate` 啟動環境

### 📁 測試檔案管理
- **專用目錄**: `~/RosAGV/agents/` - 所有暫時性測試檔案必須存放於此
- **詳細規範**: @docs-ai/operations/development/test-file-management.md

## Language Configuration
@docs-ai/context/system/language-configuration.md