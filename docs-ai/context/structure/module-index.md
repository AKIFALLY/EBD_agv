# 模組文檔索引

## 🎯 適用場景
- 快速定位特定功能領域的詳細文檔
- 理解系統模組間的關係和職責分工
- 為開發和故障排除提供導航指引

## 📋 模組文檔索引

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
- **資料庫操作**: `app/db_proxy_ws/CLAUDE.md` - PostgreSQL ORM和CRUD
  - **資料庫指導**: @docs-ai/operations/development/database-operations.md - 通用資料庫操作最佳實踐
- **設備控制**: `app/ecs_ws/CLAUDE.md` - 門控系統和設備管理
- **倉庫控制**: `app/wcs_ws/CLAUDE.md` - WCS智能調度系統
- **AI 倉庫控制**: `app/ai_wcs_ws/CLAUDE.md` - AI WCS決策引擎
- **機器人控制**: `app/rcs_ws/CLAUDE.md` - RCS和交通管理

### 🔗 通訊與整合
當涉及外部系統整合、通訊協定相關功能時，請參考以下文檔：

- **PLC通訊**: `app/keyence_plc_ws/CLAUDE.md` - Keyence PLC協議
  - **PLC協議詳解**: @docs-ai/knowledge/protocols/keyence-plc-protocol.md - Keyence 協議規範和指令集
  - **PLC開發實踐**: @docs-ai/operations/development/plc-communication.md - PLC 通訊開發最佳實踐
- **PLC代理**: `app/plc_proxy_ws/CLAUDE.md` - ROS 2 PLC服務
- **KUKA整合**: `app/kuka_fleet_ws/CLAUDE.md` - KUKA Fleet Adapter
  - **KUKA Fleet API**: @docs-ai/knowledge/protocols/kuka-fleet-api.md - KUKA Fleet Manager API 完整規格
  - **KUKA Fleet 回調**: @docs-ai/knowledge/protocols/kuka-fleet-callback.md - 任務狀態回調處理規範
- **路徑規劃**: `app/path_algorithm/CLAUDE.md` - A*演算法實現

### 🔧 基礎服務
當涉及系統基礎設施、介面定義相關功能時，請參考以下文檔：

- **ROS 2 介面**: `app/agv_ws/src/agv_interfaces/CLAUDE.md` - 訊息和服務定義
- **服務啟動配置**: `app/launch_ws/CLAUDE.md` - Web API 群組和 ECS 系統的 ROS 2 Launch 編排

## 🚀 快速導航

### 按問題類型導航
- **狀態機異常** → `agv_base/CLAUDE.md` + 對應車型文檔
- **Web API 問題** → `web_api_ws/CLAUDE.md`
- **資料庫錯誤** → `db_proxy_ws/CLAUDE.md` + @docs-ai/operations/development/database-operations.md
- **PLC 通訊故障** → `keyence_plc_ws/CLAUDE.md` + `plc_proxy_ws/CLAUDE.md` + @docs-ai/knowledge/protocols/keyence-plc-protocol.md
- **車隊管理問題** → `wcs_ws/CLAUDE.md` 或 `ai_wcs_ws/CLAUDE.md`
- **KUKA Fleet 整合** → `kuka_fleet_ws/CLAUDE.md` + @docs-ai/knowledge/protocols/kuka-fleet-api.md + @docs-ai/knowledge/protocols/kuka-fleet-callback.md

### 按開發階段導航
- **需求分析** → 領域知識文檔 (@docs-ai/knowledge/)
- **架構設計** → 系統架構文檔 (@docs-ai/context/system/)
- **實作開發** → 對應模組的 CLAUDE.md
- **測試驗證** → 開發操作文檔 (@docs-ai/operations/development/)
- **部署維護** → 維護操作文檔 (@docs-ai/operations/maintenance/)

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
- 系統架構: @docs-ai/context/system/rosagv-overview.md
- 雙環境設計: @docs-ai/context/system/dual-environment.md
- 工作空間: @docs-ai/context/workspaces/agv-workspaces.md, @docs-ai/context/workspaces/agvc-workspaces.md