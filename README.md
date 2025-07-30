# RosAGV - 企業級 AGV 控制系統

<div align="center">

**基於 ROS 2 Jazzy 和 Zenoh RMW 的現代化 AGV 車隊管理系統**

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![Docker](https://img.shields.io/badge/Docker-Compose%20V2-blue)](https://docs.docker.com/compose/)
[![Python](https://img.shields.io/badge/Python-3.12-green)](https://www.python.org/)
[![Zenoh RMW](https://img.shields.io/badge/RMW-Zenoh-orange)](https://zenoh.io/)

</div>

## 📋 專案概述

**RosAGV** 是一個企業級 AGV（自動導引車）控制系統，採用雙環境容器化架構，分離車載控制與中央管理功能，為工業自動化提供完整的車隊管理、任務調度和設備控制解決方案。

### 🎯 核心特色

- **🚗 雙環境架構**: 分離 AGV 車載控制與中央 AGVC 管理系統
- **🔧 現代技術棧**: ROS 2 Jazzy + Zenoh RMW + Docker 容器化 + Python 3.12
- **🚛 多車型支援**: Cargo Mover、Loader、Unloader 三種 AGV 車型
- **🔗 外部系統整合**: 無縫整合 KUKA Fleet 和工業 PLC 系統
- **💻 完整 Web 界面**: AGVCUI 管理台 + OPUI 操作界面
- **⚡ 高效能通訊**: 基於 Zenoh RMW 的低延遲跨網路通訊

### 🏭 應用場景

- **智慧工廠**: 自動化物料搬運和產線物流
- **倉儲管理**: 智能倉庫貨物分揀和運輸
- **製造業**: 產線間自動化物料配送
- **物流中心**: 大規模貨物自動化處理

## 🏗️ 系統架構

RosAGV 採用創新的雙環境設計，將系統分為兩個獨立但協同的環境：

```
🚗 AGV 車載系統 (On-board)          🖥️ AGVC 管理系統 (Control Center)
├─ 實時控制和狀態管理                ├─ 車隊管理和任務調度
├─ PLC 設備直接通訊                 ├─ 資料庫管理和資料持久化
├─ 感測器資料處理                   ├─ Web 管理介面
├─ 手動控制（搖桿）支援             ├─ 外部系統整合（KUKA Fleet）
└─ 路徑規劃和導航                   └─ 系統監控和日誌管理
         ↕️ Zenoh RMW 通訊 ↕️
```

### 📦 容器架構

- **AGV 環境**: `docker-compose.yml` - 9個專用工作空間，Host 網路模式
- **AGVC 環境**: `docker-compose.agvc.yml` - 11個專用工作空間，Bridge 網路模式
- **共用組件**: Zenoh Router, PostgreSQL, PLC 通訊模組

> 📖 **詳細架構說明**: [@docs-ai/context/system/dual-environment.md](docs-ai/context/system/dual-environment.md)

## 🚀 快速開始

### 環境要求

- **作業系統**: Ubuntu 24.04 LTS
- **容器**: Docker Engine + Docker Compose V2
- **硬體**: 最少 8GB RAM，4 CPU 核心

### 一鍵啟動

```bash
# 🚗 AGV 車載系統（通常部署在 AGV 車輛上）
docker compose -f docker-compose.yml up -d

# 🖥️ AGVC 管理系統（部署在中央伺服器）
docker compose -f docker-compose.agvc.yml up -d
```

### 系統驗證

```bash
# 使用統一工具檢查系統狀態
r                    # 顯示可用工具
r agvc-check         # AGVC 系統健康檢查
r containers-status  # 檢查所有容器狀態
```

### Web 界面存取

- **AGVCUI 管理台**: http://localhost:8001
- **OPUI 操作界面**: http://localhost:8002
- **API 文檔**: http://localhost:8000/docs

## 🔧 開發指導

### 開發環境設定

**⚠️ 重要**: 所有 ROS 2 開發必須在 Docker 容器內進行，宿主機無 ROS 2 環境。

```bash
# 進入開發容器
docker compose -f docker-compose.agvc.yml exec agvc_server bash

# 載入開發環境
all_source          # 智能載入工作空間
check_system_status # 檢查系統狀態
```

### 工作空間結構

```
app/
├── agv_ws/                    # AGV 核心控制（車載）
├── agv_cmd_service_ws/        # 手動指令服務
├── joystick_ws/               # 搖桿控制整合
├── web_api_ws/                # Web API 服務（管理）
├── db_proxy_ws/               # 資料庫代理服務
├── ecs_ws/                    # 設備控制系統
├── rcs_ws/                    # 機器人控制系統
├── ai_wcs_ws/                 # AI 倉庫控制系統
├── kuka_fleet_ws/             # KUKA Fleet 整合
└── keyence_plc_ws/            # Keyence PLC 通訊
```

### 開發最佳實踐

- **基於實際程式碼開發**: 使用 `rg` 工具搜尋變數定義，避免推測
- **容器內開發**: 在對應容器內進行 ROS 2 節點開發和測試
- **統一工具使用**: 使用 `r` 命令進行系統診斷和管理

> 📖 **完整開發指導**: 
> - [@docs-ai/operations/development/ros2-development.md](docs-ai/operations/development/ros2-development.md)
> - [@docs-ai/operations/development/docker-development.md](docs-ai/operations/development/docker-development.md)
> - [@docs-ai/operations/development/core-principles.md](docs-ai/operations/development/core-principles.md)

## 🛠️ 統一工具系統

RosAGV 提供強大的統一工具系統，只需記住一個字母 `r` 即可存取所有管理功能。

### 🚀 快速開始 - 一個字母搞定一切

```bash
r                    # 顯示工具選單
r agvc-check         # 每日健康檢查
r containers-status  # 檢查容器狀態
r quick-diag         # 快速綜合診斷
```

> 💡 **提示**: `r` 是 `rosagv-tools.sh` 的快捷方式，提供完整的系統管理功能！

### 📋 工具分類

#### 🔍 系統診斷工具
| 命令 | 功能 | 適用環境 |
|------|------|----------|
| `r agvc-check` | AGVC 管理系統健康檢查 | AGVC 主機 |
| `r agv-check` | AGV 車載系統健康檢查 | AGV 車輛 |
| `r system-health` | 完整系統健康檢查 | 通用 |
| `r quick-diag` | 快速綜合診斷 | 故障排除 |

#### 🐳 容器管理工具
| 命令 | 功能 |
|------|------|
| `r containers-status` | 檢查所有容器狀態 |
| `r agv-start` / `r agv-stop` | AGV 容器啟停 |
| `r agvc-start` / `r agvc-stop` | AGVC 系統啟停 |

#### 🌐 網路診斷工具
| 命令 | 功能 |
|------|------|
| `r network-check` | 系統端口檢查 |
| `r zenoh-check` | Zenoh 連接檢查 |

#### 📋 日誌分析工具
| 命令 | 功能 |
|------|------|
| `r log-scan` | 日誌錯誤掃描 |
| `r log-errors` | 高級錯誤分析 |

### 💡 實用範例

#### 日常運維
```bash
# 每日系統檢查
r agvc-check && r containers-status && r network-check

# 系統啟動
r agvc-start

# 檢查系統健康
r system-health
```

#### 故障排除
```bash
# 快速問題診斷
r quick-diag

# 深度分析
r log-errors && r zenoh-check

# 網路問題排查
r network-check
```

### 🔧 工具特色

- ✅ **零安裝**: 直接使用，無需修改系統配置
- ✅ **環境安全**: 不會關閉終端，正確顯示錯誤訊息
- ✅ **智能檢測**: 自動識別 AGV/AGVC 環境
- ✅ **完整選單**: 內建幫助和詳細說明

> 📖 **專業工具集**: [@docs-ai/operations/tools/unified-tools.md](docs-ai/operations/tools/unified-tools.md)

## 📚 文檔和資源

### 🎯 核心文檔

#### 系統架構
- **系統概覽**: [@docs-ai/context/system/rosagv-overview.md](docs-ai/context/system/rosagv-overview.md)
- **雙環境架構**: [@docs-ai/context/system/dual-environment.md](docs-ai/context/system/dual-environment.md)
- **技術棧詳解**: [@docs-ai/context/system/technology-stack.md](docs-ai/context/system/technology-stack.md)
- **語言配置**: [@docs-ai/context/system/language-configuration.md](docs-ai/context/system/language-configuration.md)
- **系統現狀**: [@docs-ai/knowledge/system/current-system-status.md](docs-ai/knowledge/system/current-system-status.md)

#### 工作空間結構
- **AGV 工作空間**: [@docs-ai/context/workspaces/agv-workspaces.md](docs-ai/context/workspaces/agv-workspaces.md)
- **AGVC 工作空間**: [@docs-ai/context/workspaces/agvc-workspaces.md](docs-ai/context/workspaces/agvc-workspaces.md)

### 🔧 開發文檔

#### 核心開發指導
- **核心開發原則**: [@docs-ai/operations/development/core-principles.md](docs-ai/operations/development/core-principles.md)
- **ROS 2 開發**: [@docs-ai/operations/development/ros2-development.md](docs-ai/operations/development/ros2-development.md)
- **Docker 開發**: [@docs-ai/operations/development/docker-development.md](docs-ai/operations/development/docker-development.md)

#### 技術專項開發
- **Web 開發**: [@docs-ai/operations/development/web-development.md](docs-ai/operations/development/web-development.md)
- **資料庫操作**: [@docs-ai/operations/development/database-operations.md](docs-ai/operations/development/database-operations.md)
- **PLC 通訊開發**: [@docs-ai/operations/development/plc-communication.md](docs-ai/operations/development/plc-communication.md)

#### 測試和建置
- **測試程序**: [@docs-ai/operations/development/testing-procedures.md](docs-ai/operations/development/testing-procedures.md)
- **測試標準**: [@docs-ai/operations/development/testing-standards.md](docs-ai/operations/development/testing-standards.md)
- **建置和測試**: [@docs-ai/operations/development/build-and-test.md](docs-ai/operations/development/build-and-test.md)

### 🛠️ 運維和維護

#### 系統診斷和維護
- **系統診斷**: [@docs-ai/operations/maintenance/system-diagnostics.md](docs-ai/operations/maintenance/system-diagnostics.md)
- **故障排除**: [@docs-ai/operations/maintenance/troubleshooting.md](docs-ai/operations/maintenance/troubleshooting.md)
- **日誌分析**: [@docs-ai/operations/maintenance/log-analysis.md](docs-ai/operations/maintenance/log-analysis.md)
- **維護工具指南**: [@docs-ai/operations/maintenance/unified-tools.md](docs-ai/operations/maintenance/unified-tools.md)

#### 開發和工具系統
- **統一工具系統**: [@docs-ai/operations/tools/unified-tools.md](docs-ai/operations/tools/unified-tools.md)

#### 部署和容器管理
- **容器管理**: [@docs-ai/operations/deployment/container-management.md](docs-ai/operations/deployment/container-management.md)

### 🧠 領域知識

#### AGV 車型和系統
- **AGV 車型**: [@docs-ai/knowledge/agv-domain/vehicle-types.md](docs-ai/knowledge/agv-domain/vehicle-types.md)  
- **WCS 系統設計**: [@docs-ai/knowledge/agv-domain/wcs-system-design.md](docs-ai/knowledge/agv-domain/wcs-system-design.md)
- **WCS 資料庫設計**: [@docs-ai/knowledge/agv-domain/wcs-database-design.md](docs-ai/knowledge/agv-domain/wcs-database-design.md)
- **WCS WorkID 系統**: [@docs-ai/knowledge/agv-domain/wcs-workid-system.md](docs-ai/knowledge/agv-domain/wcs-workid-system.md)
- **Robot PGNO 規則**: [@docs-ai/knowledge/agv-domain/robot-pgno-rules.md](docs-ai/knowledge/agv-domain/robot-pgno-rules.md)

#### 業務領域知識
- **眼鏡生產流程**: [@docs-ai/knowledge/business/eyewear-production-process.md](docs-ai/knowledge/business/eyewear-production-process.md)

#### 通訊協定
- **PLC 通訊**: [@docs-ai/knowledge/protocols/keyence-plc-protocol.md](docs-ai/knowledge/protocols/keyence-plc-protocol.md)
- **Zenoh RMW**: [@docs-ai/knowledge/protocols/zenoh-rmw.md](docs-ai/knowledge/protocols/zenoh-rmw.md)
- **KUKA Fleet API**: [@docs-ai/knowledge/protocols/kuka-fleet-api.md](docs-ai/knowledge/protocols/kuka-fleet-api.md)
- **KUKA Fleet 回調**: [@docs-ai/knowledge/protocols/kuka-fleet-callback.md](docs-ai/knowledge/protocols/kuka-fleet-callback.md)
- **ROS 2 介面**: [@docs-ai/knowledge/protocols/ros2-interfaces.md](docs-ai/knowledge/protocols/ros2-interfaces.md)
- **PLC-ROS2 介面**: [@docs-ai/knowledge/protocols/plc-ros2-interfaces.md](docs-ai/knowledge/protocols/plc-ros2-interfaces.md)

### 📋 模組索引

按功能領域快速定位相關文檔：

> 📖 **完整模組索引**: [@docs-ai/context/structure/module-index.md](docs-ai/context/structure/module-index.md)

## 🚨 故障排除

### 🔥 緊急故障處理

```bash
# 第一階段：快速評估 (1-2分鐘)
r quick-diag && r containers-status && r agvc-check

# 第二階段：問題定位 (3-5分鐘)  
r log-errors && r network-check && r zenoh-check

# 第三階段：問題解決 (5-15分鐘)
# 根據診斷結果執行對應解決方案
```

### 🎯 常見問題快速定位

| 問題類型 | 診斷命令 | 相關文檔 |
|----------|----------|----------|
| **容器無法啟動** | `r containers-status` | [@docs-ai/operations/deployment/container-management.md](docs-ai/operations/deployment/container-management.md) |
| **網路連接問題** | `r network-check` | [@docs-ai/knowledge/protocols/zenoh-rmw.md](docs-ai/knowledge/protocols/zenoh-rmw.md) |
| **Zenoh 通訊故障** | `r zenoh-check` | [@docs-ai/operations/maintenance/troubleshooting.md](docs-ai/operations/maintenance/troubleshooting.md) |
| **資料庫連接失敗** | `r agvc-check` | [@docs-ai/operations/development/database-operations.md](docs-ai/operations/development/database-operations.md) |
| **PLC 通訊異常** | `r log-scan` | [@docs-ai/operations/development/plc-communication.md](docs-ai/operations/development/plc-communication.md) |
| **Web 服務無回應** | `r system-health` | [@docs-ai/operations/development/web-development.md](docs-ai/operations/development/web-development.md) |

### 📖 完整故障排除指南

> 📖 **詳細故障排除**: [@docs-ai/operations/maintenance/troubleshooting.md](docs-ai/operations/maintenance/troubleshooting.md)
> 
> 📖 **系統診斷**: [@docs-ai/operations/maintenance/system-diagnostics.md](docs-ai/operations/maintenance/system-diagnostics.md)

## 🤝 貢獻指南

### 開發流程

1. **環境準備**: 使用 Docker 容器進行開發
2. **代碼開發**: 遵循 [@docs-ai/operations/development/core-principles.md](docs-ai/operations/development/core-principles.md)
3. **測試驗證**: 使用 [@docs-ai/operations/development/testing-procedures.md](docs-ai/operations/development/testing-procedures.md)
4. **代碼審查**: 提交 Pull Request 前進行完整測試

### 技術規範

- **語言**: Python 3.12, ROS 2 Jazzy
- **代碼風格**: 遵循 PEP 8 和 ROS 2 編碼規範
- **容器化**: 所有服務必須容器化部署
- **文檔**: 使用 @docs-ai/ 引用系統維護文檔

## 📄 授權

本專案採用 [MIT License](LICENSE) 授權。

---

<div align="center">

**🚀 RosAGV - 推動工業自動化的未來**

[📖 完整文檔](docs-ai/README.md) | [🛠️ 工具指南](CLAUDE.md) | [🐛 問題回報](https://github.com/your-repo/issues)

</div>