# 模組文檔索引

## 🎯 適用場景
- 快速定位特定功能領域的詳細文檔
- 理解系統模組間的關係和職責分工
- 為開發和故障排除提供導航指引

## 📋 模組文檔索引

### 🏭 業務領域知識
當需要理解 RosAGV 實際應用場景和業務流程時，請參考以下文檔：

- **眼鏡生產流程**: docs-ai/knowledge/business/eyewear-production-process.md - 射出機作業、OPUI叫車、KUKA AGV配送的完整業務流程

### 🚗 AGV 車載系統
當涉及 AGV 車載控制、狀態機、硬體整合相關功能時，請參考以下文檔：

- **AGV狀態機**: `app/agv_ws/src/agv_base/CLAUDE.md` - 3層狀態架構詳解
- **車型實現**: 
  - `app/agv_ws/src/cargo_mover_agv/CLAUDE.md` - Cargo Mover AGV
  - `app/agv_ws/src/loader_agv/CLAUDE.md` - Loader AGV  
  - `app/agv_ws/src/unloader_agv/CLAUDE.md` - Unloader AGV
- **手動控制**: `app/agv_cmd_service_ws/CLAUDE.md` - 手動指令服務
- **搖桿控制**: `app/joystick_ws/CLAUDE.md` - USB搖桿整合
- **感測器處理**: `app/sensorpart_ws/CLAUDE.md` - 感測器數據處理

### 🖥️ AGVC 管理系統  
當涉及車隊管理、Web 服務、資料庫操作相關功能時，請參考以下文檔：

- **Web API**: `app/web_api_ws/CLAUDE.md` - FastAPI + Socket.IO 詳解
  - **AGVUI 監控系統**: docs-ai/knowledge/system/agvui-monitoring-system.md - AGV 車載監控界面 (Port 8003)
- **資料庫操作**: `app/db_proxy_ws/CLAUDE.md` - PostgreSQL ORM和CRUD
  - **資料庫指導**: docs-ai/operations/development/database-operations.md - 通用資料庫操作最佳實踐
- **設備控制**: `app/ecs_ws/CLAUDE.md` - 門控系統和設備管理
- **🎯 TAFL WCS (目前系統)**: `app/tafl_wcs_ws/CLAUDE.md` - **TAFL WCS 目前使用的實作**
  - **WCS 統一架構**: docs-ai/knowledge/agv-domain/wcs-system-design.md - TAFL WCS 完整架構
  - **TAFL Editor**: 在 `app/web_api_ws/src/agvcui/` 中的新視覺化流程設計器，產生 TAFL 檔案
  - **TAFL 核心**: `app/tafl_ws/CLAUDE.md` - TAFL 語言核心實作（解析器、執行器、驗證器）
  - **資料庫設計**: docs-ai/knowledge/agv-domain/wcs-database-design.md - WCS 資料表架構
  - **Work ID 系統**: docs-ai/knowledge/agv-domain/wcs-workid-system.md - 任務分類管理
- **機器人控制**: `app/rcs_ws/CLAUDE.md` - RCS和交通管理

### 🔗 通訊與整合
當涉及外部系統整合、通訊協定相關功能時，請參考以下文檔：

- **PLC通訊**: `app/keyence_plc_ws/CLAUDE.md` - Keyence PLC協議
  - **PLC協議詳解**: docs-ai/knowledge/protocols/keyence-plc-protocol.md - Keyence 協議規範和指令集
  - **PLC開發實踐**: docs-ai/operations/development/ros2/plc-communication.md - PLC 通訊開發最佳實踐
- **PLC代理**: `app/plc_proxy_ws/CLAUDE.md` - ROS 2 PLC服務
- **KUKA整合**: `app/kuka_fleet_ws/CLAUDE.md` - KUKA Fleet Adapter
  - **KUKA Fleet API**: docs-ai/knowledge/protocols/kuka-fleet-api.md - KUKA Fleet Manager API 完整規格
  - **KUKA Fleet 回調**: docs-ai/knowledge/protocols/kuka-fleet-callback.md - 任務狀態回調處理規範
- **路徑規劃**: `app/path_algorithm/CLAUDE.md` - A*演算法實現

### 🔧 基礎服務
當涉及系統基礎設施、介面定義相關功能時，請參考以下文檔：

- **ROS 2 介面**: `app/agv_ws/src/agv_interfaces/CLAUDE.md` - 訊息和服務定義
- **服務啟動配置**: `app/launch_ws/CLAUDE.md` - Web API 群組和 ECS 系統的 ROS 2 Launch 編排

## 🚀 快速導航

### 按問題類型導航
- **狀態機異常** → `agv_base/CLAUDE.md` + 對應車型文檔
- **Web API 問題** → `web_api_ws/CLAUDE.md` + docs-ai/operations/development/web/web-api-launch-management.md
- **Web API Launch 管理** → docs-ai/operations/development/web/web-api-launch-management.md
- **AGVUI 監控問題** → docs-ai/knowledge/system/agvui-monitoring-system.md + `app/web_api_ws/src/agvui/CLAUDE.md`
- **資料庫錯誤** → `db_proxy_ws/CLAUDE.md` + docs-ai/operations/development/database-operations.md
- **PLC 通訊故障** → `keyence_plc_ws/CLAUDE.md` + `plc_proxy_ws/CLAUDE.md` + docs-ai/knowledge/protocols/keyence-plc-protocol.md
- **🎯 WCS 相關問題** → `tafl_wcs_ws/CLAUDE.md` (**目前使用的 WCS 系統**)
  - **WCS 架構理解** → docs-ai/knowledge/agv-domain/wcs-system-design.md
  - **架台旋轉邏輯** → docs-ai/knowledge/system/rack-rotation-logic.md
  - **TAFL Editor 問題** → `app/web_api_ws/src/agvcui/CLAUDE.md` (視覺化流程設計，產生 TAFL 檔案)
- **KUKA Fleet 整合** → `kuka_fleet_ws/CLAUDE.md` + docs-ai/knowledge/protocols/kuka-fleet-api.md + docs-ai/knowledge/protocols/kuka-fleet-callback.md

### 按開發階段導航
- **需求分析** → 領域知識文檔 (docs-ai/knowledge/)
- **架構設計** → 系統架構文檔 (docs-ai/context/system/)
- **實作開發** → 對應模組的 CLAUDE.md
- **測試驗證** → 開發操作文檔 (docs-ai/operations/development/)
  - **測試標準**: docs-ai/operations/development/testing/testing-standards.md - pytest 統一測試規範
- **部署維護** → 維護操作文檔 (docs-ai/operations/guides/)

## 🛠️ 系統工具和腳本

### 資料庫工具 (db_proxy_ws/scripts/)
當需要資料庫初始化、狀態檢查時，請參考以下工具：

- **init_database.sh** - 資料庫用戶和資料庫初始化
  - 功能: 創建 agvc 用戶、agvc 資料庫、test_db 資料庫
  - 使用: `cd /home/ct/RosAGV/app/db_proxy_ws/scripts && ./init_database.sh`
  - 注意: 在宿主機執行，自動檢查和驗證

- **check_db_status.sh** - 資料庫狀態檢查
- **test_connection.py** - 連接測試

### 系統診斷工具 (scripts/)
當需要系統診斷、效能監控時，請參考以下工具：

#### 統一診斷工具 (r 指令)
- **r agvc-check** - AGVC 系統健康檢查
- **r containers-status** - 容器狀態檢查
- **r network-check** - 網路連接檢查
- **r quick-diag** - 快速綜合診斷

#### 專業工具集
- **system-tools/** - 系統健康檢查工具集
- **network-tools/** - 網路診斷工具集
- **docker-tools/** - 容器管理工具集
- **log-tools/** - 日誌分析工具集
- **dev-tools/** - 開發工作流工具集

### 工具使用最佳實踐

#### AI Agent 搜尋策略
**重要教訓**: 避免搜尋盲區，採用多維度搜尋

```bash
# ✅ 好的搜尋方式
find . -name "*init*" -o -name "*database*" -o -name "scripts"
rg "CREATE USER|CREATE DATABASE" --type sh --type sql
find . -type f -name "*.sh" | xargs rg -l "關鍵字"

# ❌ 不完整的搜尋方式
rg "pattern" --type py  # 只搜尋 Python 檔案
```

#### 工具發現原則
1. **檔案類型多樣化**: 不只搜尋 .py，也要包含 .sh, .sql, .yml
2. **目錄結構探索**: 重點關注 scripts/, tools/, config/ 目錄
3. **關鍵字交叉驗證**: 使用多個相關關鍵字組合搜尋
4. **工具腳本優先**: 很多重要功能在腳本中實現

#### 藏匿工具探索
**常見藏匿目錄**:
- `scripts/` - 系統初始化和管理腳本
- `tools/` - 開發和部署工具
- `bin/` - 可執行檔案
- `config/` - 配置腳本和樣板
- `docs/examples/` - 實用範例腳本

## 📋 文檔維護指南

### 模組文檔規範
每個模組的 CLAUDE.md 應包含：
- **Context Loading**: 引用相關的 docs-ai 文檔
- **模組概述**: 簡要說明模組功能和定位
- **核心特色**: 突出該模組的獨特功能
- **開發環境**: 容器使用和工具整合
- **快速開始**: 基本使用方法
- **故障排除**: 常見問題和解決方案
- **交叉引用**: 相關模組和文檔連結

### 更新維護
- **定期檢查**: 確保引用的 docs-ai 文檔存在且內容正確
- **內容同步**: 當系統架構變更時及時更新索引
- **測試驗證**: 定期驗證文檔中的指令和路徑有效性

## 🔗 交叉引用
- 系統架構: docs-ai/context/system/rosagv-overview.md
- 雙環境設計: docs-ai/context/system/dual-environment.md
- 工作空間: docs-ai/context/workspaces/agv-workspaces.md, docs-ai/context/workspaces/agvc-workspaces.md