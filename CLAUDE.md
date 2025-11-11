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

### 📌 服務管理標準化（重要）
**所有 ROS2 服務管理函數 (manage_*) 已標準化，採用 4+6 階段設計：**

```bash
# 統一介面（所有服務）
manage_<service_name> start     # 4階段啟動（幂等性 → 依賴 → 啟動 → 驗證）
manage_<service_name> stop      # 6階段停止（TERM → KILL → 僵屍 → 殘留 → 端口 → 臨時文件）
manage_<service_name> restart   # 完整重啟
manage_<service_name> status    # 詳細狀態檢查
manage_<service_name> logs      # 即時日誌查看

# 已標準化的服務（6個核心服務）
manage_plc_service_agvc        # PLC 服務
manage_ecs_core                # ECS 核心服務
manage_rcs_core                # RCS 核心服務
manage_agvc_database_node      # 資料庫代理
manage_tafl_wcs                # TAFL WCS 流程控制
manage_room_task_build         # Room Task Build

# 參考範例
manage_web_api_launch          # Web 服務（最佳實踐範例）
```

**開發新服務管理函數時必讀：**
- 📖 [manage-function-standard.md](docs-ai/operations/development/ros2/manage-function-standard.md) - 開發標準
- 📖 [manage-function-template.md](docs-ai/operations/development/ros2/manage-function-template.md) - 實作模板

## ⚠️ AI Agent 核心規則
1. **先查文檔，不要猜** - @docs-ai/ 是唯一權威
2. **用現成工具，不要造** - 檢查 scripts/ 和 r 命令
3. **批量處理，不要重複** - MultiEdit > 多次 Edit
4. **自動化，不要手動** - Git hooks, manage_* 命令
6. **📊 保持索引同步** - 修改 docs-ai 後執行 generate-docs-ai-index.py

## 🔍 問題診斷決策樹
```
遇到問題時：
1. 這個問題以前解決過嗎？ → 查 troubleshooting 文檔
2. 有現成工具嗎？ → 執行 `r` 查看工具列表
3. 可以自動化嗎？ → 檢查 scripts/ 目錄
4. 需要批量處理嗎？ → 使用 MultiEdit 或寫腳本
5. 真的需要手動嗎？ → 才開始手動操作
```


## 📚 核心系統文檔（必要載入）
# 🔝 通用層級：AI Agent 核心規則與開發指導（6個強引用 + 8個參考文檔）
# 所有 AI Agent 必須理解的基礎知識

# AI Agent 核心規則與開發指導 - 6個
@ai-agents/container-development-rules.md    # 容器開發規則
@ai-agents/database-operations-rules.md      # 資料庫操作規則
@ai-agents/ros2-development-rules.md         # ROS2 開發規則
@ai-agents/tafl-language-rules.md           # TAFL 語言規則
@ai-agents/unified-tools-usage.md           # 統一工具使用
@ai-agents/web-api-development-rules.md     # Web API 開發規則

# 系統架構（參考文檔）- 3個
docs-ai/context/system/rosagv-overview.md              # 系統概覽
docs-ai/context/system/dual-environment.md             # 雙環境架構
docs-ai/context/system/technology-stack.md             # 技術棧

# 核心開發原則（參考文檔）- 2個
docs-ai/operations/development/core/core-principles.md # 核心開發原則
docs-ai/operations/development/core/linus-torvalds-ai-agent-principles.md # Linus 思維

# 通用工具與操作（參考文檔）- 3個
docs-ai/operations/tools/unified-tools.md              # 統一工具系統
docs-ai/operations/development/docker-development.md   # Docker 開發
docs-ai/operations/guides/troubleshooting.md           # 故障排除與診斷

## 📖 分層架構說明
# 🏗️ RosAGV 採用三層文檔引用架構：
# 1️⃣ 通用層（本文件）：所有 AI Agent 的基礎知識
# 2️⃣ 工作空間層（_ws/CLAUDE.md）：領域特定知識
# 3️⃣ 專業層（src/*/CLAUDE.md）：模組實作細節

### 工作空間層文檔（在對應 _ws 目錄查看）
# 各工作空間的 CLAUDE.md 會包含：
# - 工作空間架構（agv-workspaces.md, agvc-workspaces.md）
# - 領域特定知識（如 AGV 狀態機、PLC 協議、TAFL 語言等）
# - 開發流程文檔（ROS2 開發、測試標準、資料庫操作等）

### 專業實作層文檔（在 src 目錄查看）
# 各模組的 CLAUDE.md 會包含：
# - 高度專業化的實作細節
# - 特定演算法和邏輯
# - 模組特定的協議和介面

## ⚠️ 重要開發注意事項
**所有 ROS 2 程式必須在 Docker 容器內執行，宿主機無 ROS 2 環境。**
# 詳細容器內指令執行請參考工作空間層的 CLAUDE.md 文檔

### 📂 查找特定領域文檔
# 工作空間相關文檔應查看對應的 _ws/CLAUDE.md：
# - PLC 通訊 → plc_proxy_ws/CLAUDE.md 或 keyence_plc_ws/CLAUDE.md
# - AGV 控制 → agv_ws/CLAUDE.md
# - WCS 系統 → tafl_wcs_ws/CLAUDE.md
# - Web 開發 → web_api_ws/CLAUDE.md
# - 資料庫 → db_proxy_ws/CLAUDE.md
# - KUKA Fleet → kuka_fleet_ws/CLAUDE.md

## AI 開發助手指導


### 🐍 Python 開發環境
- **uv**: 高效能 Python 套件管理器
- **.venv**: 虛擬環境 (含 playwright 等測試工具)
- **使用**: `source .venv/bin/activate` 啟動環境

### 📁 測試檔案管理
- **專用目錄**: `~/RosAGV/agents/` - 所有暫時性測試檔案必須存放於此
# 詳細規範: docs-ai/operations/development/testing/test-file-management.md

## 📚 業務流程文檔系統
- **文檔目錄**: `~/RosAGV/design/business-process-docs/` - RosAGV 業務流程文檔中心
- **Web 訪問**: `http://agvc.ui/docs/index.html` (動態 Markdown 載入系統)
  - ⚠️ 注意：`http://agvc.ui/docs` 是 FastAPI 自動文檔，不是業務文檔
- **內容結構**: 使用 Markdown 檔案動態載入（`content/` 目錄）
  - getting-started/ - 快速入門指南
  - business-processes/ - 業務流程說明
  - agv-vehicles/ - AGV 車輛相關
  - system-architecture/ - 系統架構
  - operations/ - 操作指南
  - technical-details/ - 技術細節
# 詳細配置: docs-ai/operations/deployment/nginx-configuration.md

## 📊 AI 知識庫索引維護
**✅ 已啟用自動更新：透過 Git Pre-commit Hook 自動維護索引**

### 自動更新機制
當提交以下文件變更時，系統會自動更新知識庫索引：
- 📚 `docs-ai/**/*.md` - 知識文檔
- 📋 `**/CLAUDE.md` - 架構文件
- 🤖 `ai-agents/**/*.md` - AI Agent 規則

### 自動更新流程
```bash
# 1. 正常修改和提交（無需手動操作）
git add docs-ai/some-doc.md
git commit -m "docs: 更新文檔"

# 2. Pre-commit Hook 自動執行：
#    ✅ 檢測文檔變更
#    ✅ 執行 update-indexes.sh
#    ✅ 更新 docs-ai-index.json
#    ✅ 更新 claude-architecture.json
#    ✅ 自動加入索引到 commit
```

### 手動更新（備用）
如需手動更新索引（例如測試或疑難排解）：
```bash
cd ~/RosAGV/design/business-process-docs
./update-indexes.sh

# 檢視更新統計
cat js/docs-ai-index.json | jq '.stats'
cat js/claude-architecture.json | jq '.summary'
```

### 查看更新結果
- 訪問 `http://agvc.ui/docs/index.html`
- 切換到「🤖 AI 知識庫」頁籤
- 確認兩種統計資訊：
  - 📋 **架構層統計**：CLAUDE.md 檔案分佈（27個檔案）
  - 📚 **知識層統計**：docs-ai 文檔適用層級（67個文檔）

### 索引內容說明
- **引用統計**: 自動掃描所有 CLAUDE.md 檔案中的 docs-ai 引用
- **重要性分類**: 根據引用次數自動分類（≥10次為關鍵，≥5次為重要）
- **分類整理**: 自動將文檔分為核心原則、系統架構、操作指南等類別
- **架構統計**: 統計三層架構中 CLAUDE.md 的分佈和引用關係

## 📦 工作空間專業知識指引
**進入特定工作空間時，請務必查看該工作空間的 CLAUDE.md 檔案以載入專業知識：**

### 核心控制系統
- **agv_ws/CLAUDE.md**: AGV 狀態機、車輛控制
- **agv_cmd_service_ws/CLAUDE.md**: AGV 命令服務、手動控制
- **tafl_wcs_ws/CLAUDE.md**: TAFL 流程控制、WCS 系統
- **rcs_ws/CLAUDE.md**: 機器人控制系統、任務調度

### Web 與資料服務
- **web_api_ws/CLAUDE.md**: Web 服務、API 開發、Socket.IO 整合
- **db_proxy_ws/CLAUDE.md**: PostgreSQL 資料庫操作、SQLAlchemy ORM

### 外部系統整合
- **kuka_fleet_ws/CLAUDE.md**: KUKA Fleet 整合、機器人控制
- **ecs_ws/CLAUDE.md**: 設備控制系統、門控管理

### PLC 與硬體控制
- **keyence_plc_ws/CLAUDE.md**: Keyence PLC 通訊協定
- **plc_proxy_ws/CLAUDE.md**: PLC 代理服務、通訊橋接
- **joystick_ws/CLAUDE.md**: 搖桿控制、手動操作
- **uno_gpio_ws/CLAUDE.md**: 研華 UNO-137 工業電腦 GPIO 控制

### 視覺與感測系統
- **sensorpart_ws/CLAUDE.md**: 3D 相機、OCR 識別

### 系統支援
- **launch_ws/CLAUDE.md**: ROS 2 Launch 系統管理
- **shared_constants_ws/CLAUDE.md**: 共享常數定義、系統參數

每個工作空間 CLAUDE.md 都包含該領域的專業文檔引用和開發指導。

## 📊 分層引用架構總結
# 本文件為第一層（通用層），包含 6 個核心文檔 + 8 個參考文檔
# 第二層（工作空間層）文檔請查看各 _ws/CLAUDE.md
# 第三層（專業層）文檔請查看各 src/*/CLAUDE.md
# 詳細分層架構說明: docs-ai/README.md (三層架構設計章節)