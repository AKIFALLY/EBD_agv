# RosAGV 專案規劃文檔中心

## 📋 概述

本目錄包含 RosAGV 專案的完整規劃文檔，為 AI Agent 開發提供結構化的參考依據。所有文檔基於實際程式碼現況，支援增量開發和功能擴展模式。

## 🏗️ 專案背景

- **專案類型**: 大型 ROS 2 專案，包含 15 個工作空間
- **架構模式**: 雙環境容器化架構 (AGV 車載系統 + AGVC 管理系統)
- **技術棧**: ROS 2 Jazzy + Zenoh RMW + PostgreSQL + FastAPI + 雙 Web UI
- **當前狀態**: 所有工作空間 README.md 已標準化完成

## 📁 文檔結構

### 1. 系統架構文檔 (`/architecture/`)
完整的系統設計和技術架構說明
- `system-overview.md` - 整體系統設計圖和模組關係
- `container-architecture.md` - 雙環境容器架構詳細說明
- `technology-stack.md` - 技術棧選擇和整合方式
- `workspace-dependencies.md` - 15個工作空間依賴關係圖
- `communication-protocols.md` - Zenoh RMW、PLC、KUKA Fleet 通訊協定

### 2. 功能需求規格 (`/requirements/`)
詳細的功能需求和業務邏輯規範
- `functional-requirements.md` - 按工作空間分類的功能需求
- `business-processes.md` - AGV 任務流程和車隊管理邏輯
- `non-functional-requirements.md` - 效能、安全性、可靠性需求
- `user-interface-requirements.md` - AGVCUI 和 OPUI 介面需求
- `integration-requirements.md` - 外部系統整合需求

### 3. 技術規格文檔 (`/specifications/`)
詳細的技術實作規範和介面定義
- `ros2-interfaces.md` - ROS 2 服務、訊息、動作介面定義
- `database-schema.md` - PostgreSQL 結構和 SQLModel ORM 設計
- `web-api-specification.md` - FastAPI 端點和資料格式規範
- `plc-communication.md` - Keyence PLC 通訊協定和記憶體映射
- `data-formats.md` - 跨系統資料交換格式規範

### 4. 開發計劃 (`/planning/`)
詳細的開發計劃和資源分配
- `work-breakdown-structure.md` - 基於15個工作空間的任務分解
- `priority-matrix.md` - 功能優先級分類和開發順序
- `milestone-roadmap.md` - 里程碑定義和時程規劃
- `risk-assessment.md` - 技術風險評估和緩解策略
- `resource-allocation.md` - 開發資源分配和技能需求

### 5. 進度追蹤 (`/tracking/`)
統一的進度管理和品質追蹤
- `completion-status.md` - 完成狀態標記系統和進度儀表板
- `milestone-achievements.md` - 里程碑達成記錄和成果展示
- `change-log.md` - 變更歷史和決策記錄
- `quality-metrics.md` - 程式碼品質和效能指標追蹤
- `technical-debt.md` - 技術債務追蹤和重構計劃

### 6. 測試規格 (`/testing/`)
完整的測試策略和驗收標準
- `test-strategy.md` - 整體測試策略和測試金字塔
- `unit-test-specifications.md` - 單元測試計劃和覆蓋率要求
- `integration-test-plans.md` - 整合測試和系統測試
- `acceptance-criteria.md` - 功能驗收標準和使用者驗收測試
- `performance-benchmarks.md` - 效能測試基準和負載測試
- `automated-testing.md` - CI/CD 自動化測試腳本

### 7. 維護指南 (`/maintenance/`)
系統維護和運維管理指南
- `troubleshooting-guide.md` - 故障排除指南和常見問題
- `deployment-procedures.md` - 部署流程和環境配置
- `monitoring-alerting.md` - 系統監控和警報機制
- `backup-recovery.md` - 資料備份和災難恢復計劃

## 🎯 使用指南

### 狀態標記系統
- ✅ **完成**: 功能已實作並通過測試
- 🚧 **進行中**: 正在開發或測試階段
- ⚠️ **待處理**: 已規劃但尚未開始
- ❌ **已取消**: 不再需要或已廢棄
- 🔄 **重構中**: 需要重新設計或改善

### 文檔維護原則
1. **基於實際**: 所有內容基於實際程式碼現況
2. **增量更新**: 支援增量開發和功能擴展
3. **保留記錄**: 已完成功能標記為 ✅ 但不刪除
4. **交叉引用**: 使用相對路徑建立文檔導航網路
5. **AI 友善**: 包含可執行指令和自動化驗證步驟

### 優先級建議
1. **高優先級**: architecture/, requirements/, specifications/
2. **中優先級**: planning/, tracking/
3. **低優先級**: testing/, maintenance/

## 📝 文檔模板

每種文檔類型都有標準化的 Markdown 模板，確保格式一致性和 AI Agent 可讀性。

## 🔗 相關資源

- **工作空間 README**: `/app/*_ws/README.md` (已標準化)
- **配置檔案**: `/app/config/` (實際配置參數)
- **啟動腳本**: `/app/startup.*.bash` (實際啟動狀態)
- **標準化規則**: `/.augment/rules/update-readme.md`

---

**最後更新**: 2025-01-17  
**維護責任**: AI Agent 開發團隊  
**版本**: v1.0.0
