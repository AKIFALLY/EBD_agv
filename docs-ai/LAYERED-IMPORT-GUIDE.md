# 📚 RosAGV 文檔分層引入架構

## 🎯 設計理念
根據知識的通用性和適用範圍，將 docs-ai 文檔分為三個層級，確保 AI Agent 在不同層級載入適當的知識。

**📁 相關文檔**：`docs-ai/STRUCTURE.md` 提供物理目錄結構導航（人類視角），本文檔則關注邏輯載入架構（AI Agent 視角）。

```
🌐 通用層 (Layer 1) - 系統架構、核心原則、通用工具
🔧 工作空間層 (Layer 2) - 領域知識、開發流程、通用協議
🔬 專業層 (Layer 3) - 特定實作、專業細節、模組特定
```

## 📊 當前文檔分佈 (2025-09-19 更新)
- **總文檔數**: 68 個
- **🌐 通用層**: 11 個文檔 (16.2%)
- **🔧 工作空間層**: 56 個文檔 (82.4%)
- **🔬 專業層**: 1 個文檔 (1.4%)

## 📊 文檔專業度分層

### 🌐 第一層：通用知識 (Layer 1 - 根目錄 CLAUDE.md)
**適用對象**：所有 AI Agent 操作、初次接觸系統
**知識特點**：系統級概念、核心原則、通用工具
**實際文檔數**：11 個

#### 當前通用層文檔（已實施）
```yaml
# 系統架構 (4個)
- docs-ai/context/system/rosagv-overview.md          # 系統概覽
- docs-ai/context/system/dual-environment.md         # 雙環境架構
- docs-ai/context/system/technology-stack.md         # 技術棧
- docs-ai/context/system/language-configuration.md   # 語言配置

# 核心開發原則 (3個)
- docs-ai/operations/development/core/core-principles.md                    # 核心開發原則
- docs-ai/operations/development/core/linus-torvalds-ai-agent-principles.md # Linus 思維
- docs-ai/operations/development/core/documentation-standards.md            # 文檔標準

# 通用工具與操作 (4個)
- docs-ai/operations/tools/unified-tools.md               # 統一工具系統
- docs-ai/operations/development/docker-development.md     # Docker 開發
- docs-ai/operations/guides/system-diagnostics.md         # 系統診斷
- docs-ai/operations/guides/troubleshooting.md            # 故障排除
```

### 🔧 第二層：工作空間知識 (Layer 2 - 工作空間 CLAUDE.md)
**適用對象**：特定工作空間開發、領域相關操作
**知識特點**：領域知識、通用協議、工作流程
**實際文檔數**：56 個

#### 工作空間層核心文檔範例
```yaml
# 工作空間架構 (2個)
- docs-ai/context/workspaces/agv-workspaces.md      # AGV 工作空間
- docs-ai/context/workspaces/agvc-workspaces.md     # AGVC 工作空間

# 通用協議和介面 (7個)
- docs-ai/knowledge/protocols/ros2-interfaces.md     # ROS2 介面
- docs-ai/knowledge/protocols/zenoh-rmw.md          # Zenoh 通訊
- docs-ai/knowledge/protocols/keyence-plc-protocol.md # PLC 協議
- docs-ai/knowledge/protocols/kuka-fleet-api.md      # KUKA API

# 開發流程 (多個)
- docs-ai/operations/development/ros2/ros2-development.md        # ROS2 開發
- docs-ai/operations/development/testing/testing-standards.md    # 測試標準
- docs-ai/operations/development/database-operations.md          # 資料庫操作
```

#### 領域特定文檔 (根據工作空間選擇性引用)
```yaml
# AGV 相關工作空間
agv_ws, agv_cmd_service_ws:
  - docs-ai/knowledge/agv-domain/vehicle-types.md    # 車型定義
  - docs-ai/knowledge/agv-domain/agv-state-machine.md # 狀態機
  - docs-ai/knowledge/system/manual-rack-management.md # Rack 管理

# WCS 相關工作空間
tafl_wcs_ws, flow_wcs_ws:
  - docs-ai/knowledge/agv-domain/wcs-system-design.md # WCS 設計
  - docs-ai/knowledge/agv-domain/wcs-workid-system.md # WorkID 系統
  - docs-ai/knowledge/agv-domain/wcs-database-design.md # 資料庫設計

# PLC 相關工作空間
plc_proxy_ws, keyence_plc_ws:
  - docs-ai/knowledge/protocols/keyence-plc-protocol.md # Keyence 協議
  - docs-ai/operations/development/ros2/plc-communication.md # PLC 通訊

# KUKA Fleet 相關工作空間
kuka_fleet_ws:
  - docs-ai/knowledge/protocols/kuka-fleet-api.md    # Fleet API
  - docs-ai/knowledge/protocols/kuka-fleet-callback.md # Fleet 回調
  - docs-ai/knowledge/protocols/kuka-agv-rack-rotation.md # Rack 旋轉

# Web API 相關工作空間
web_api_ws:
  - docs-ai/knowledge/system/hmi-system-design.md    # HMI 設計
  - docs-ai/knowledge/system/agvui-monitoring-system.md # 監控系統
  - docs-ai/operations/development/web/web-development.md # Web 開發

# TAFL 相關工作空間
tafl_ws:
  - docs-ai/knowledge/system/tafl/tafl-language-specification.md
  - docs-ai/knowledge/system/tafl/tafl-quick-start-guide.md
```

### 🔬 第三層：專業知識 (Layer 3 - 模組深層 CLAUDE.md)
**適用對象**：特定模組實作、專業功能開發
**知識特點**：具體實作、專業細節、模組特定
**實際文檔數**：1 個 (目前僅有 ros2-pytest-testing.md)

#### 專業層文檔特徵
- 高度專業化的實作細節
- 特定模組的深層技術文檔
- 僅在需要深入實作時引用

#### 當前專業層文檔
```yaml
# 測試專業文檔
- operations/development/testing/ros2-pytest-testing.md  # ROS2 Pytest 測試

# AGV 基礎狀態機 (agv_ws/src/agv_base)
agv_base:
  - docs-ai/knowledge/agv-domain/agv-state-machine.md # 狀態機詳解
  - docs-ai/knowledge/agv-domain/magic-value-analysis.md # 魔術值分析

# ROS2 介面定義 (agv_ws/src/agv_interfaces)
agv_interfaces:
  - docs-ai/knowledge/protocols/ros2-interfaces.md    # 介面規範
  - docs-ai/knowledge/protocols/plc-ros2-interfaces.md # PLC 介面

# Web 界面實作 (web_api_ws/src/*)
agvcui:
  - docs-ai/knowledge/system/agvui-monitoring-system.md
  - docs-ai/knowledge/system/tafl/tafl-editor-specification.md

opui:
  - docs-ai/knowledge/business/eyewear-production-process.md
  - 操作員界面特定文檔

web_api:
  - docs-ai/knowledge/protocols/kuka-fleet-api.md
  - docs-ai/knowledge/protocols/kuka-fleet-callback.md

# 資料庫代理 (db_proxy_ws/src/db_proxy)
db_proxy:
  - docs-ai/knowledge/agv-domain/wcs-database-design.md
  - docs-ai/operations/development/database-operations.md
```

## 📝 實施建議

### 1. 引入原則
- **向上相容**：深層可以引用上層的文檔
- **最小必要**：只引入必要的文檔，避免過度載入
- **專業匹配**：文檔專業度要與層級匹配

### 2. 引入格式範例

**⚠️ 引用語法使用規則**：
- 在 **CLAUDE.md** 中：可以使用 `@docs-ai/` 或 `docs-ai/` 格式
- 在 **其他所有文檔**中：統一使用 `docs-ai/` 弱引用格式
- 詳細規範請參考：`docs-ai/operations/development/core/documentation-standards.md`

#### 根目錄 CLAUDE.md
```markdown
# RosAGV CLAUDE.md

## 📚 核心系統文檔（必要載入）
docs-ai/context/system/dual-environment.md
docs-ai/operations/development/core/core-principles.md
docs-ai/operations/tools/unified-tools.md
```

#### 工作空間 CLAUDE.md (例：plc_proxy_ws)
```markdown
# plc_proxy_ws CLAUDE.md

## 📚 Context Loading
../../CLAUDE.md  # 引用根目錄通用知識
docs-ai/knowledge/protocols/keyence-plc-protocol.md
docs-ai/knowledge/protocols/plc-ros2-interfaces.md
docs-ai/operations/development/ros2/plc-communication.md
```

#### 深層 CLAUDE.md (例：agv_base)
```markdown
# agv_base CLAUDE.md

## 📚 Context Loading
../../../../CLAUDE.md  # 引用根目錄通用知識
../../CLAUDE.md  # 引用 agv_ws 層知識
docs-ai/knowledge/agv-domain/agv-state-machine.md
docs-ai/knowledge/agv-domain/magic-value-analysis.md
# 其他高度專業化的文檔
```

### 3. 維護建議
1. **定期檢查**：確保引用路徑正確
2. **避免重複**：上層已引用的不要在下層重複
3. **及時更新**：新增文檔時更新相應層級的引用
4. **專業度評估**：定期評估文檔的專業度分類是否合適

## 📊 最新文檔統計 (2025-09-19)

### 層級分佈統計
```
總計：68 個文檔
├── 🌐 通用層 (Layer 1)：11 個 (16.2%)
│   └── 系統架構、核心原則、通用工具
├── 🔧 工作空間層 (Layer 2)：56 個 (82.4%)
│   └── 領域知識、開發流程、通用協議
└── 🔬 專業層 (Layer 3)：1 個 (1.4%)
    └── 特定實作、專業細節

引用統計：
- 強引用總數：257
- 弱引用總數：430
- 總引用次數：687
- 關鍵文檔 (≥10次引用)：11 個
- 重要文檔 (≥5次引用)：18 個
- 未引用文檔：0 個
```

### 實際引用分配
- **根目錄 CLAUDE.md**：11 個通用層文檔（已實施）
- **工作空間 CLAUDE.md**：根據領域選擇 5-10 個文檔
- **模組 CLAUDE.md**：根據需要引用 3-5 個專業文檔

## 🗂️ 文檔專業度分類表

### 通用層文檔 (根目錄引用)
| 文檔路徑 | 描述 | 重要性 |
|---------|------|--------|
| context/system/dual-environment.md | 雙環境架構 | ⭐⭐⭐ |
| context/system/rosagv-overview.md | 系統概覽 | ⭐⭐⭐ |
| operations/development/core/core-principles.md | 核心開發原則 | ⭐⭐⭐ |
| operations/tools/unified-tools.md | 統一工具系統 | ⭐⭐⭐ |
| operations/development/docker-development.md | Docker 開發 | ⭐⭐ |

### 工作空間層文檔
| 文檔路徑 | 適用工作空間 | 專業領域 |
|---------|------------|---------|
| knowledge/protocols/ros2-interfaces.md | 所有 _ws | ROS2 |
| knowledge/protocols/keyence-plc-protocol.md | plc_*_ws | PLC |
| knowledge/protocols/kuka-fleet-api.md | kuka_fleet_ws | KUKA |
| knowledge/agv-domain/wcs-system-design.md | *_wcs_ws | WCS |

### 專業層文檔
| 文檔路徑 | 適用模組 | 專業度 |
|---------|---------|--------|
| knowledge/agv-domain/agv-state-machine.md | agv_base | 高 |
| knowledge/agv-domain/magic-value-analysis.md | agv_base | 高 |
| knowledge/system/tafl/tafl-api-reference.md | tafl 實作 | 高 |

## 🔥 關鍵文檔排行 (基於加權分數)

### Top 5 最常被引用文檔
1. **系統診斷操作指導** [🌐 通用層] - 加權分數: 18.1
   - `operations/guides/system-diagnostics.md`
2. **ROS 2 開發操作指導** [🔧 工作空間層] - 加權分數: 16.3
   - `operations/development/ros2/ros2-development.md`
3. **Keyence PLC 通訊協議** [🔧 工作空間層] - 加權分數: 14.3
   - `knowledge/protocols/keyence-plc-protocol.md`
4. **RosAGV 雙環境架構詳解** [🌐 通用層] - 加權分數: 14.2
   - `context/system/dual-environment.md`
5. **PLC 通訊開發最佳實踐** [🔧 工作空間層] - 加權分數: 13.4
   - `operations/development/ros2/plc-communication.md`

## 🔄 更新歷史
- 2024-11-18：初始版本，建立三層引入架構
- 2025-09-19：更新為最新統計數據，反映實際層級分配