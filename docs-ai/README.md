# RosAGV AI Agent 記憶系統

## 🎯 設計目標
將 CLAUDE.md 重新定位為 AI Agent 的主要記憶文件，透過 docs-ai/ 引用機制提供模組化的專案指導資訊。

## 🏗️ 三層架構設計

### 🎯 分層引入架構
根據知識的通用性和適用範圍，docs-ai 文檔採用兩種分層視角：

**📁 物理層（目錄結構）**：Context / Operations / Knowledge
**🧠 邏輯層（引用層級）**：Layer 1 通用層 / Layer 2 工作空間層 / Layer 3 專業層

```
🌐 Layer 1 (通用層) → 根目錄 CLAUDE.md 引用
   系統架構、核心原則、通用工具 (11個文檔)

🔧 Layer 2 (工作空間層) → 工作空間 CLAUDE.md 引用
   領域知識、開發流程、通用協議 (54個文檔)

🔬 Layer 3 (專業層) → 模組 CLAUDE.md 引用
   特定實作、專業細節、模組特定 (1個文檔)
```

**引入原則**：
- 向上相容：深層可以引用上層文檔
- 最小必要：只引入必要的文檔
- 避免重複：上層已引用的下層不重複

### 📚 Context 文件系列 - AI Agent 背景知識庫
為 AI Agent 提供專案背景知識，確保理解系統架構、技術棧和業務邏輯。

```
docs-ai/context/
├── structure/                       # 結構化文檔
│   └── module-index.md             # 模組索引
├── system/                          # 系統層級背景
│   ├── rosagv-overview.md           # RosAGV 專案整體概覽
│   ├── dual-environment.md          # 雙環境架構詳解
│   ├── technology-stack.md          # 技術棧和依賴關係
│   └── language-configuration.md    # 語言配置設定
└── workspaces/                      # 工作空間層級背景
    ├── agv-workspaces.md           # AGV 車載工作空間概覽
    └── agvc-workspaces.md          # AGVC 管理工作空間概覽
```

### 🔧 Operations 文件系列 - AI Agent 操作指導
提供具體的操作指導和最佳實踐，涵蓋開發、部署、指導、工具等面向。

```
docs-ai/operations/
├── deployment/                      # 部署操作指導
│   ├── container-management.md     # 容器管理操作
│   ├── docker-compose-configuration.md # Docker Compose 配置
│   └── nginx-configuration.md      # Nginx 配置管理
├── development/                     # 開發操作指導
│   ├── core/                       # 核心開發原則
│   │   ├── core-principles.md      # 核心開發原則
│   │   ├── documentation-standards.md # 文檔標準
│   │   └── linus-torvalds-ai-agent-principles.md # Linus 思維
│   ├── ros2/                       # ROS 2 開發指導
│   │   ├── plc-communication.md    # PLC 通訊開發
│   │   └── ros2-development.md     # ROS 2 開發指南
│   ├── testing/                    # 測試程序文檔
│   │   ├── ros2-workspace-test-structure.md # 測試結構
│   │   ├── test-file-management.md # 測試檔案管理
│   │   ├── testing-procedures.md   # 測試程序
│   │   └── testing-standards.md    # 測試標準（含 ROS 2 pytest 指令）
│   ├── web/                        # Web 開發指導
│   │   ├── web-api-launch-management.md # Web API 啟動管理
│   │   └── web-development.md      # Web 開發指南
│   ├── build-and-test.md           # 建置和測試指南
│   ├── database-operations.md      # 資料庫操作指導
│   └── docker-development.md       # Docker 容器開發指導
├── guides/                          # 操作指南
│   ├── log-analysis.md             # 日誌分析指南
│   ├── performance-monitoring.md   # 效能監控指導
│   ├── rack-management-guide.md    # Rack 管理操作
│   ├── system-diagnostics.md       # 系統診斷程序
│   └── troubleshooting.md          # 故障排除流程
└── tools/                           # 工具系統
    └── unified-tools.md            # 統一工具系統
```

### 🧠 Knowledge 文件系列 - 領域專業知識
提供 AGV 控制、工業自動化、通訊協定等專業領域知識。

```
docs-ai/knowledge/
├── agv-domain/                      # AGV 領域知識
│   ├── agv-state-machine.md        # AGV 狀態機設計
│   ├── license-table-design.md     # 授權表設計
│   ├── magic-value-analysis.md     # 魔術值分析
│   ├── robot-pgno-rules.md         # Robot PGNO 規則
│   ├── vehicle-types.md            # 車型特性和應用場景
│   ├── wcs-database-design.md      # WCS 資料庫設計
│   ├── wcs-system-design.md        # WCS 系統設計
│   ├── wcs-workid-system.md        # WCS WorkID 系統
│   └── write-path-state-analysis.md # 寫入路徑狀態分析
├── business/                        # 業務流程知識
│   └── eyewear-production-process.md # 眼鏡生產業務流程
├── protocols/                       # 通訊協定知識
│   ├── keyence-plc-protocol.md     # Keyence PLC 協定
│   ├── kuka-agv-rack-rotation.md   # KUKA AGV Rack 旋轉
│   ├── kuka-fleet-api.md           # KUKA Fleet API
│   ├── kuka-fleet-callback.md      # KUKA Fleet 回調
│   ├── plc-ros2-interfaces.md      # PLC ROS 2 介面
│   ├── ros2-interfaces.md          # ROS 2 介面設計
│   └── zenoh-rmw.md                # Zenoh RMW 通訊機制
└── system/                          # 系統相關知識
    ├── tafl/                        # TAFL 語言相關
    │   ├── tafl-api-reference.md   # TAFL API 參考
    │   ├── tafl-development-history.md # TAFL 開發歷史
    │   ├── tafl-editor-specification.md # TAFL 編輯器規格
    │   ├── tafl-language-specification.md # TAFL 語言規格
    │   └── tafl-user-guide.md      # TAFL 使用者指南
    ├── agvui-monitoring-system.md  # AGVUI 監控系統
    ├── hmi-system-design.md        # HMI 系統設計
    ├── manual-rack-management.md   # 手動 Rack 管理
    ├── rack-management-architecture.md # Rack 管理架構
    └── rack-rotation-logic.md      # Rack 旋轉邏輯
```

## 🚀 AI 助理快速上手指南

### 第一步：掌握統一工具入口
**關鍵概念**: 只需要記住一個字母 `r`

```bash
# 最重要的 4 個命令
r                    # 顯示工具選單
r agvc-check         # 每日健康檢查
r containers-status  # 檢查容器狀態
r quick-diag         # 快速診斷
```

**AI 助理應該知道**:
- `r` 是統一工具入口，適用於日常運維和故障排除
- 這是宿主機環境的工具，不需要進入容器
- 所有常用診斷和管理功能都可以透過 `r` 存取

### 第二步：理解雙環境架構
**關鍵概念**: AGV 車載系統 + AGVC 管理系統

- **AGV 車載系統**: 即時控制、硬體整合、PLC 通訊
- **AGVC 管理系統**: 車隊管理、Web 服務、資料庫管理
- **跨環境通訊**: 透過 Zenoh RMW 實現無縫通訊

### 📋 常用命令速查表

#### 🔍 系統診斷
| 命令 | 用途 | 適用場景 |
|------|------|----------|
| `r agvc-check` | AGVC 健康檢查 | 每日例行檢查 |
| `r system-health` | 完整健康檢查 | 深度系統診斷 |
| `r quick-diag` | 快速綜合診斷 | 故障排除 |

#### 🐳 容器管理
| 命令 | 用途 | 適用場景 |
|------|------|----------|
| `r containers-status` | 檢查容器狀態 | 確認系統運行 |
| `r agvc-start` | 啟動 AGVC 系統 | 系統啟動 |
| `r agvc-stop` | 停止 AGVC 系統 | 系統關閉 |

#### 🌐 網路診斷
| 命令 | 用途 | 適用場景 |
|------|------|----------|
| `r network-check` | 端口連接檢查 | 網路問題診斷 |
| `r zenoh-check` | Zenoh 連接檢查 | 通訊問題排除 |

#### 📋 日誌分析
| 命令 | 用途 | 適用場景 |
|------|------|----------|
| `r log-scan` | 日誌錯誤掃描 | 發現系統問題 |
| `r log-errors` | 高級錯誤掃描 | 深度錯誤分析 |

### 🚨 故障排除快速流程

1. **第一階段：快速評估** (1-2分鐘)
   ```bash
   r quick-diag           # 快速綜合診斷
   r containers-status    # 容器運行狀態
   r agvc-check          # 關鍵服務檢查
   ```

2. **第二階段：問題定位** (3-5分鐘)
   ```bash
   r log-errors          # 深度日誌分析
   r network-check       # 網路連接檢查
   r zenoh-check         # Zenoh 連接專項檢查
   ```

3. **第三階段：問題解決** (5-15分鐘)
   - 根據診斷結果執行對應解決方案
   - 參考相關 docs-ai/ 文檔進行詳細操作

## 🔄 與現有文件系統的整合

### 職責分工
- **`README.md`**: 專案概覽和快速開始指南
- **CLAUDE.md**: AI Agent 主要記憶文件，透過 docs-ai/ 引用載入詳細指導
- **`docs-ai/`**: 模組化的 AI Agent 操作指導庫（context/, knowledge/, operations/）

## 📚 文檔引用使用指南

### 設計理念
RosAGV AI Agent 記憶系統採用 docs-ai/路徑 語法，讓 CLAUDE.md 可以保持簡潔，同時動態載入豐富的 contextual 文檔。

### 📝 引用語法

**⚠️ 注意**：本文檔中的所有引用示例使用 `docs-ai/` 格式（弱引用）。
強引用格式 `@docs-ai/` 僅在 CLAUDE.md 文件中使用。
詳細規範請參考：`docs-ai/operations/development/core/documentation-standards.md`

#### 基本語法
```markdown
docs-ai/context/system/rosagv-overview.md
docs-ai/operations/development/ros2/ros2-development.md
docs-ai/knowledge/agv-domain/vehicle-types.md
```

#### CLAUDE.md 引用範例
```markdown
# AGV Base CLAUDE.md

## 📚 Context Loading
docs-ai/context/system/dual-environment.md
docs-ai/context/workspaces/agv-workspaces.md
docs-ai/knowledge/agv-domain/vehicle-types.md

## 開發指導
docs-ai/operations/development/ros2/ros2-development.md
docs-ai/operations/development/testing/testing-procedures.md

## 維護支援
docs-ai/operations/guides/system-diagnostics.md
docs-ai/operations/guides/troubleshooting.md
```

## 🎯 選擇指南

### 根據任務類型選擇文檔組合

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
docs-ai/context/system/rosagv-overview.md
docs-ai/context/system/dual-environment.md
docs-ai/context/system/technology-stack.md

## 系統概述
簡潔的專案描述...

## 核心架構
關鍵架構要點...

## 開發指導
docs-ai/operations/development/ros2/ros2-development.md
docs-ai/operations/deployment/container-management.md

## 維護支援
docs-ai/operations/guides/system-diagnostics.md
docs-ai/operations/guides/troubleshooting.md

## 領域知識
docs-ai/knowledge/agv-domain/vehicle-types.md
docs-ai/knowledge/protocols/kuka-fleet-api.md
```

### 工作空間層級 CLAUDE.md 模板
```markdown
# [工作空間名稱] CLAUDE.md

## 📚 Context Loading
docs-ai/context/workspaces/[agv|agvc]-workspaces.md
docs-ai/knowledge/business/[相關業務流程].md
docs-ai/knowledge/[相關領域]/[相關知識].md

## 工作空間概述
工作空間功能和職責...

## 開發指導
docs-ai/operations/development/[相關技術]-development.md
docs-ai/operations/development/testing/testing-procedures.md

## 維護支援
docs-ai/operations/guides/[相關維護].md

## 快速開始
基本操作指令...
```

### 套件層級 CLAUDE.md 模板
```markdown
# [套件名稱] CLAUDE.md

## 📚 Context Loading
docs-ai/context/workspaces/[相關工作空間].md
docs-ai/knowledge/[相關領域]/[相關知識].md

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
- 定期檢查文檔內容與實際程式碼的同步性
- 建立文件更新的標準流程
- 確保引用的一致性和準確性
- 使用 `scripts/check-claude-references.sh` 驗證引用有效性

### 避免重複（階層繼承原則）
- Context 文件專注於背景知識
- Operations 文件專注於具體操作
- Knowledge 文件專注於領域專業知識
- **上層已引用的文檔，下層不再重複引用**
- 根層 → 工作空間層 → 套件層 形成繼承關係

## 📖 核心文檔引用清單

### 🔥 必讀文檔 (AI 助理優先級 1)
- `docs-ai/operations/guides/system-diagnostics.md` - 統一工具使用指南
- `docs-ai/operations/development/docker-development.md` - Docker 容器管理
- `docs-ai/operations/guides/troubleshooting.md` - 故障排除指導

### 🔧 專業工具 (優先級 2)
- `docs-ai/operations/guides/log-analysis.md` - 日誌分析
- `docs-ai/operations/development/ros2/ros2-development.md` - ROS 2 開發建置
- `docs-ai/operations/development/testing/testing-procedures.md` - 測試程序

### 📖 系統架構 (優先級 3)
- `docs-ai/context/system/technology-stack.md` - ROS 2 + Zenoh 架構
- `docs-ai/context/system/dual-environment.md` - 雙環境設計

### 🎯 常用引用組合

#### AGV 車載開發
```markdown
docs-ai/context/workspaces/agv-workspaces.md
docs-ai/knowledge/agv-domain/vehicle-types.md
docs-ai/operations/development/ros2/ros2-development.md
```

#### AGVC 管理開發
```markdown
docs-ai/context/workspaces/agvc-workspaces.md
docs-ai/operations/development/web/web-development.md
docs-ai/operations/development/database-operations.md
```

#### 系統診斷維護
```markdown
docs-ai/context/system/dual-environment.md
docs-ai/operations/guides/system-diagnostics.md
docs-ai/operations/guides/troubleshooting.md
```

## 🔍 快速文檔定位

### 按問題類型定位
- **狀態機問題** → `docs-ai/context/workspaces/agv-workspaces.md`
- **Web API 問題** → `docs-ai/operations/development/web/web-development.md`
- **資料庫問題** → `docs-ai/operations/development/database-operations.md`
- **PLC 通訊問題** → `docs-ai/knowledge/protocols/keyence-plc-protocol.md`
- **容器問題** → `docs-ai/operations/development/docker-development.md`
- **網路通訊問題** → `docs-ai/knowledge/protocols/zenoh-rmw.md`

### 按開發階段定位
- **需求分析** → `docs-ai/knowledge/` 領域知識文檔
- **架構設計** → `docs-ai/context/system/` 系統架構文檔
- **實作開發** → `docs-ai/operations/development/` 開發指導文檔
- **測試驗證** → `docs-ai/operations/development/testing/testing-procedures.md`
- **部署維護** → `docs-ai/operations/guides/` 維護操作文檔