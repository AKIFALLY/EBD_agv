# GEMINI.md

## Project Overview
**RosAGV**: An enterprise-grade AGV (Automated Guided Vehicle) control system built on ROS 2 Jazzy and Zenoh RMW. It features a dual-environment containerized architecture for AGV on-board systems and AGVC (AGV Control) management systems, providing comprehensive fleet management, task dispatch, and equipment control solutions for industrial automation.

## Key Features
- **Dual-Environment Architecture**: Separates AGV on-board control from central AGVC management.
- **Modern Tech Stack**: Leverages ROS 2 Jazzy, Zenoh RMW for communication, and Docker for containerization.
- **Multi-Vehicle Support**: Designed for Cargo, Loader, and Unloader AGV types.
- **External System Integration**: Seamlessly integrates with KUKA Fleet systems.
- **Comprehensive Web Interfaces**: Includes AGVCUI (management console) and OPUI (operator interface).

## Quick Start

### 🚗 AGV On-board System
```bash
docker compose -f docker-compose.yml up -d
```

### 🖥️ AGVC Management System
```bash
docker compose -f docker-compose.agvc.yml up -d
```

## System Architecture
- **AGV On-board**: Focuses on AGV state control, PLC communication, sensor integration, joystick control, and path planning.
- **AGVC Management**: Handles fleet management (RCS), warehouse control (WCS), equipment control (ECS), KUKA Fleet integration, web interfaces, and database management.
- **Shared Infrastructure**: Both environments utilize Zenoh RMW, PostgreSQL, and Nginx.

## Project Structure
- **`docker-compose.yml`**: Defines the AGV on-board system.
- **`docker-compose.agvc.yml`**: Defines the AGVC management system.
- **`app/`**: Contains all application code, including ROS 2 workspaces (`*_ws/`), configuration files (`config/`), and startup scripts (`startup.*.bash`).
- **`routerconfig.json5`**: Zenoh Router configuration.

## 🛠️ 開發者工具套件

RosAGV 提供完整的開發者工具套件，涵蓋配置管理、容器操作、系統診斷、日誌分析、網路診斷和開發工作流：

### 📋 工具集概覽

| 工具集 | 描述 | 主要功能 |
|--------|------|----------|
| **🔧 config-tools** | 配置管理工具 | Zenoh配置、硬體映射、AGV/AGVC配置管理 |
| **🐳 docker-tools** | Docker容器管理 | 容器啟停、狀態檢查、快速診斷 |
| **🔍 system-tools** | 系統診斷監控 | 健康檢查、服務監控、系統診斷 |
| **📊 log-tools** | 日誌分析工具 | 智能日誌分析、錯誤檢測、統計分析 |
| **🌐 network-tools** | 網路診斷工具 | Zenoh通訊診斷、端口檢查、連接測試 |
| **💻 dev-tools** | 開發工作流工具 | 建置、測試、分析、部署自動化 |

### 🚀 快速使用

#### 系統狀態檢查
```bash
# 完整系統健康檢查
scripts/system-tools/health-check.sh --quick

# 容器狀態檢查
scripts/docker-tools/container-status.sh all

# 網路連接檢查
scripts/network-tools/port-check.sh system
```

#### 開發工作流
```bash
# 代碼品質檢查
scripts/dev-tools/code-analyzer.sh style --workspace agv_ws

# 快速建置
scripts/dev-tools/build-helper.sh fast

# 測試執行
scripts/dev-tools/test-runner.sh unit --coverage

# 部署到開發環境
scripts/dev-tools/deploy-helper.sh deploy development
```

#### 配置管理
```bash
# Zenoh 配置檢查
scripts/config-tools/zenoh-config.sh validate

# 硬體映射概覽
scripts/config-tools/hardware-mapping.sh overview

# 編輯 AGV 配置
scripts/config-tools/edit-agv-config.sh cargo01
```

#### 故障診斷
```bash
# 日誌分析
scripts/log-tools/log-analyzer.sh all --stats

# 網路診斷
scripts/network-tools/zenoh-network.sh full-check

# Zenoh 通訊故障排除
scripts/network-tools/network-tools.sh troubleshoot zenoh
```

### 📚 詳細文檔

- **完整工具使用說明**: 參見 [`CLAUDE.md`](CLAUDE.md) 中的工具集章節
- **開發環境設置**: 參見 [`app/README.md`](app/README.md)
- **工具開發路線圖**: 參見 [`scripts/TODO_TOOLS_ROADMAP.md`](scripts/TODO_TOOLS_ROADMAP.md)

## Troubleshooting

### 基本故障排除
- **Container startup issues**: Check `docker logs` for specific service errors.
- **Network connectivity**: Verify Docker network configurations and port mappings.
- **Zenoh connection**: Ensure the Zenoh Router is running and configured correctly (port 7447).
- **Database issues**: Check PostgreSQL container status and logs.

### 使用工具集診斷
```bash
# 系統全面診斷
scripts/system-tools/health-check.sh --fix

# 快速問題診斷
scripts/docker-tools/docker-tools.sh && quick_diagnose

# 網路問題診斷
scripts/network-tools/network-tools.sh && network_emergency_check "問題描述"
```

### 智能問題定位

根據問題類型使用對應工具：

| 問題類型 | 推薦工具 | 快速命令 |
|---------|---------|---------|
| **容器無法啟動** | docker-tools | `container-status.sh all` |
| **服務無響應** | system-tools | `health-check.sh --quick` |
| **網路連接問題** | network-tools | `port-check.sh system` |
| **Zenoh 通訊故障** | network-tools | `zenoh-network.sh full-check` |
| **建置失敗** | dev-tools | `build-helper.sh discover` |
| **代碼品質問題** | dev-tools | `code-analyzer.sh style` |
| **配置檔案錯誤** | config-tools | `zenoh-config.sh validate` |
| **日誌錯誤分析** | log-tools | `log-analyzer.sh all --stats` |

For detailed development guides and troubleshooting, refer to [`CLAUDE.md`](CLAUDE.md) and [`app/README.md`](app/README.md).