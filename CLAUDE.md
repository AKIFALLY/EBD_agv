# RosAGV CLAUDE.md

## 🚀 常用工作流程（90% 日常操作）

### 容器操作基礎
```bash
# 進入 AGVC 容器（從宿主機）
cd ~/RosAGV
docker compose -f docker-compose.agvc.yml exec agvc_server bash
source /app/setup.bash && agvc_source

# 或一行完成（使用 bash -i 確保 alias 載入）
docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c "source /app/setup.bash && agvc_source && [指令]"
```

### 常用 Alias
```bash
ba = build_all         # 建置所有工作空間
sa = all_source        # 載入所有工作空間
agvc_source           # 載入 AGVC 工作空間
agv_source            # 載入 AGV 工作空間
```

### 核心工作流程
```bash
# 重建並重啟 Web 服務（最常用）
manage_web_api_launch stop && ba && sa && manage_web_api_launch start

# 快速重啟服務
manage_web_api_launch restart

# TAFL Editor 重建
cd /app/web_api_ws && colcon build --packages-select agvcui && manage_web_api_launch restart

# 檢查系統狀態（宿主機）
r agvc-check
r quick-diag

# 查看服務日誌
docker compose -f docker-compose.agvc.yml logs -f agvc_server
```

## ⚠️ AI Agent 核心規則
1. **先查文檔，不要猜** - @docs-ai/ 是唯一權威
2. **用現成工具，不要造** - 檢查 scripts/ 和 r 命令
3. **批量處理，不要重複** - MultiEdit > 多次 Edit
4. **自動化，不要手動** - Git hooks, manage_* 命令
5. **🔥 Linus Torvalds 思維** - @docs-ai/operations/development/linus-torvalds-ai-agent-principles.md

## 🔍 問題診斷決策樹
```
遇到問題時：
1. 這個問題以前解決過嗎？ → 查 troubleshooting 文檔
2. 有現成工具嗎？ → 執行 `r` 查看工具列表
3. 可以自動化嗎？ → 檢查 scripts/ 目錄
4. 需要批量處理嗎？ → 使用 MultiEdit 或寫腳本
5. 真的需要手動嗎？ → 才開始手動操作
```


## 📚 核心規格文檔（修改前必查）
- 產品和載具規格：@docs-ai/knowledge/agv-domain/vehicle-types.md
- 資料庫設計：@docs-ai/knowledge/agv-domain/wcs-database-design.md
- Work ID 系統：@docs-ai/knowledge/agv-domain/wcs-workid-system.md

## 📚 核心系統文檔（必要載入）
@docs-ai/context/system/dual-environment.md
@docs-ai/operations/development/core-principles.md
@docs-ai/operations/development/linus-torvalds-ai-agent-principles.md
@docs-ai/operations/tools/unified-tools.md
@docs-ai/operations/development/docker-development.md

## ⚠️ 重要開發注意事項
**所有 ROS 2 程式必須在 Docker 容器內執行，宿主機無 ROS 2 環境。**

**容器內指令執行格式**: @docs-ai/operations/development/ros2-container-commands.md

## 📁 參考文檔路徑（需要時查閱）
- **PLC 通訊**: `docs-ai/knowledge/protocols/keyence-plc-protocol.md`
- **ROS 2 開發**: `docs-ai/operations/development/ros2-development.md`
- **系統診斷**: `docs-ai/operations/maintenance/system-diagnostics.md`
- **故障排除**: `docs-ai/operations/maintenance/troubleshooting.md`
- **車型規格**: `docs-ai/knowledge/agv-domain/vehicle-types.md`
- **Zenoh 通訊**: `docs-ai/knowledge/protocols/zenoh-rmw.md`

## AI 開發助手指導


### 🐍 Python 開發環境
- **uv**: 高效能 Python 套件管理器
- **.venv**: 虛擬環境 (含 playwright 等測試工具)
- **使用**: `source .venv/bin/activate` 啟動環境

### 📁 測試檔案管理
- **專用目錄**: `~/RosAGV/agents/` - 所有暫時性測試檔案必須存放於此
- **詳細規範**: @docs-ai/operations/development/test-file-management.md

## Language Configuration
@docs-ai/context/system/language-configuration.md