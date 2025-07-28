# RosAGV AI Agent 記憶系統

## 🎯 設計目標
將 CLAUDE.md 重新定位為 AI Agent 的主要記憶文件，透過 @docs-ai/ 引用機制提供模組化的專案指導資訊，避免與 .augment-guidelines 內容重複。

## 🏗️ 三層架構設計

### 📚 Context 文件系列 - AI Agent 背景知識庫
為 AI Agent 提供專案背景知識，確保理解系統架構、技術棧和業務邏輯。

```
docs-ai/context/
├── system/                          # 系統層級背景
│   ├── rosagv-overview.md           # RosAGV 專案整體概覽
│   ├── dual-environment.md          # 雙環境架構詳解
│   ├── technology-stack.md          # 技術棧和依賴關係
│   └── deployment-architecture.md   # 部署架構和網路配置
├── workspaces/                      # 工作空間層級背景
│   ├── agv-workspaces.md           # AGV 車載工作空間概覽
│   ├── agvc-workspaces.md          # AGVC 管理工作空間概覽
│   └── shared-components.md        # 共用組件和通訊機制
└── business/                       # 業務邏輯背景
    ├── agv-control-logic.md        # AGV 控制邏輯和狀態機
    ├── fleet-management.md         # 車隊管理和任務調度
    └── external-integration.md     # 外部系統整合邏輯
```

### 🔧 Prompts 文件系列 - AI Agent 操作指導
提供具體的操作指導和最佳實踐，涵蓋開發、維護、部署、整合等面向。

```
docs-ai/operations/
├── development/                     # 開發操作指導
│   ├── ros2-development.md         # ROS 2 節點開發指導
│   ├── docker-development.md       # Docker 容器開發指導
│   ├── database-operations.md      # 資料庫操作指導
│   ├── web-development.md          # Web API 開發指導
│   └── testing-procedures.md       # 測試和驗證程序
├── maintenance/                     # 維護操作指導
│   ├── system-diagnostics.md       # 系統診斷程序
│   ├── troubleshooting.md          # 故障排除流程
│   ├── log-analysis.md             # 日誌分析方法
│   ├── performance-monitoring.md   # 效能監控指導
│   └── backup-recovery.md          # 備份和恢復程序
├── deployment/                      # 部署操作指導
│   ├── container-management.md     # 容器管理操作
│   ├── environment-setup.md        # 環境設定指導
│   ├── configuration-management.md # 配置管理操作
│   └── version-control.md          # 版本控制最佳實踐
└── integration/                     # 整合操作指導
    ├── external-systems.md         # 外部系統整合指導
    ├── api-integration.md          # API 整合操作
    └── data-migration.md           # 資料遷移程序
```

### 🧠 Knowledge 文件系列 - 領域專業知識
提供 AGV 控制、工業自動化、通訊協定等專業領域知識。

```
docs-ai/knowledge/
├── agv-domain/                      # AGV 領域知識
│   ├── vehicle-types.md            # 車型特性和應用場景
│   ├── navigation-systems.md       # 導航系統和路徑規劃
│   ├── safety-protocols.md         # 安全協定和規範
│   └── performance-optimization.md # 效能優化策略
├── automation/                      # 工業自動化知識
│   ├── plc-communication.md        # PLC 通訊協定
│   ├── sensor-integration.md       # 感測器整合技術
│   ├── control-systems.md          # 控制系統設計
│   └── fleet-coordination.md       # 車隊協調機制
└── protocols/                       # 通訊協定知識
    ├── zenoh-rmw.md                # Zenoh RMW 通訊機制
    ├── ros2-interfaces.md          # ROS 2 介面設計
    ├── web-protocols.md            # Web 通訊協定
    └── database-schemas.md         # 資料庫結構設計
```

## 🔄 與現有文件系統的整合

### 職責分工
- **`.augment-guidelines`**: 環境識別和基礎開發規範（宿主機 vs 容器內）
- **`README.md`**: 專案概覽和快速開始指南
- **CLAUDE.md**: AI Agent 主要記憶文件，透過 @docs-ai/ 引用載入詳細指導
- **`docs-ai/`**: 模組化的 AI Agent 操作指導庫

### 📝 @ 引用語法

#### 基本語法
```markdown
@docs-ai/context/system/rosagv-overview.md
@docs-ai/operations/development/ros2-development.md
@docs-ai/knowledge/agv-domain/vehicle-types.md
```

#### CLAUDE.md 引用範例
```markdown
# AGV Base CLAUDE.md

## 📚 Context Loading
@docs-ai/context/system/dual-environment.md
@docs-ai/context/workspaces/agv-workspaces.md
@docs-ai/knowledge/agv-domain/vehicle-types.md

## 開發指導
@docs-ai/operations/development/ros2-development.md
@docs-ai/operations/development/testing-procedures.md

## 維護支援
@docs-ai/operations/maintenance/system-diagnostics.md
@docs-ai/operations/maintenance/troubleshooting.md
```

## 🎯 智能選擇指南

### 根據任務類型選擇 Prompts 組合

| 任務類型 | Context | Operations | Knowledge |
|---------|---------|------------|-----------|
| **新手導入** | `system/rosagv-overview.md` | `development/ros2-development.md` | `agv-domain/vehicle-types.md` |
| **AGV 開發** | `workspaces/agv-workspaces.md` | `development/ros2-development.md` | `agv-domain/navigation-systems.md` |
| **Web 開發** | `workspaces/agvc-workspaces.md` | `development/web-development.md` | `protocols/web-protocols.md` |
| **系統維護** | `system/deployment-architecture.md` | `maintenance/system-diagnostics.md` | `automation/control-systems.md` |
| **故障排除** | `system/dual-environment.md` | `maintenance/troubleshooting.md` | `protocols/zenoh-rmw.md` |
| **外部整合** | `business/external-integration.md` | `integration/external-systems.md` | `protocols/ros2-interfaces.md` |

## 📋 檔案命名規範

### Context 文件命名
- **系統層級**: `[系統名稱]-[概念].md` (如: `rosagv-overview.md`)
- **工作空間層級**: `[環境]-workspaces.md` (如: `agv-workspaces.md`)
- **業務邏輯**: `[功能領域]-[邏輯類型].md` (如: `fleet-management.md`)

### Operations 文件命名
- **開發操作**: `[技術棧]-development.md` (如: `ros2-development.md`)
- **維護操作**: `[維護類型].md` (如: `system-diagnostics.md`)
- **部署操作**: `[部署對象]-management.md` (如: `container-management.md`)
- **整合操作**: `[整合對象]-[操作類型].md` (如: `external-systems.md`)

### Knowledge 文件命名
- **領域知識**: `[概念]-[類型].md` (如: `vehicle-types.md`)
- **技術知識**: `[技術]-[方面].md` (如: `plc-communication.md`)
- **協定知識**: `[協定名稱]-[用途].md` (如: `zenoh-rmw.md`)

## 🚀 CLAUDE.md 引用模板

### 根目錄 CLAUDE.md 模板
```markdown
# RosAGV CLAUDE.md

## 📚 Context Loading
@docs-ai/context/system/rosagv-overview.md
@docs-ai/context/system/dual-environment.md
@docs-ai/context/system/technology-stack.md

## 系統概述
簡潔的專案描述...

## 核心架構
關鍵架構要點...

## 開發指導
@docs-ai/operations/development/ros2-development.md
@docs-ai/operations/deployment/environment-setup.md

## 維護支援
@docs-ai/operations/maintenance/system-diagnostics.md
@docs-ai/operations/maintenance/troubleshooting.md

## 領域知識
@docs-ai/knowledge/agv-domain/vehicle-types.md
@docs-ai/knowledge/automation/fleet-coordination.md
```

### 工作空間層級 CLAUDE.md 模板
```markdown
# [工作空間名稱] CLAUDE.md

## 📚 Context Loading
@docs-ai/context/workspaces/[agv|agvc]-workspaces.md
@docs-ai/context/business/[相關業務邏輯].md
@docs-ai/knowledge/[相關領域]/[相關知識].md

## 工作空間概述
工作空間功能和職責...

## 開發指導
@docs-ai/operations/development/[相關技術]-development.md
@docs-ai/operations/development/testing-procedures.md

## 維護支援
@docs-ai/operations/maintenance/[相關維護].md

## 快速開始
基本操作指令...
```

### 套件層級 CLAUDE.md 模板
```markdown
# [套件名稱] CLAUDE.md

## 📚 Context Loading
@docs-ai/context/workspaces/[相關工作空間].md
@docs-ai/knowledge/[相關領域]/[相關知識].md

## 套件概述
套件功能和用途...

## 開發指導
具體開發指導...

## API 參考
關鍵 API 和介面...

## 故障排除
常見問題和解決方案...
```

## 🔧 實施步驟

### 第一階段：重構現有結構
1. 建立新的三層目錄結構
2. 遷移和改進現有有價值內容
3. 移除過時和無效內容

### 第二階段：建立核心文件
1. 建立 Context 文件系列
2. 建立 Operations 文件系列
3. 建立 Knowledge 文件系列

### 第三階段：整合和測試
1. 更新所有 CLAUDE.md 引用
2. 建立交叉引用機制
3. 測試引用有效性

## 📋 維護原則

### 內容同步
- 定期檢查 prompts 內容與實際程式碼的同步性
- 建立文件更新的標準流程
- 確保引用的一致性和準確性

### 避免重複
- Context 文件專注於背景知識
- Operations 文件專注於具體操作
- Knowledge 文件專注於領域專業知識
- 與 `.augment-guidelines` 保持職責分工