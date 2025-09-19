# RosAGV 專案整體概覽

## 🎯 適用場景
- AI Agent 初次接觸 RosAGV 專案時的背景知識載入
- 理解專案整體架構和設計理念
- 為具體開發任務提供系統性背景

## 📋 專案核心概念

### 專案定位
**RosAGV** 是一個企業級 AGV（自動導引車）控制系統，基於 ROS 2 Jazzy 和 Zenoh RMW 構建，採用雙環境容器化架構，為工業自動化提供完整的車隊管理、任務調度和設備控制解決方案。

### 開發單位
**擎添工業 (Ching Tech)** - 專注於工業自動化和 AGV 系統解決方案的技術公司，致力於提供高效能、可靠的智慧製造技術。

### 核心特色
- **雙環境架構**: 分離 AGV 車載控制與中央 AGVC 管理
- **現代技術棧**: ROS 2 Jazzy + Zenoh RMW + Docker 容器化
- **多車型支援**: Cargo、Loader、Unloader 三種 AGV 車型
- **外部系統整合**: 無縫整合 KUKA Fleet 系統
- **完整 Web 界面**: AGVCUI 管理台 + OPUI 操作界面

### 業務價值
- **提升效率**: 自動化物料搬運，減少人工操作
- **降低成本**: 優化路徑規劃，提高設備利用率
- **增強安全**: 智能避障和安全協定
- **易於擴展**: 模組化設計，支援多種車型和應用場景

## 🏗️ 系統架構概覽

### 雙環境設計理念
```
🚗 AGV 車載系統 (On-board)
├─ 實時控制和狀態管理
├─ PLC 設備直接通訊
├─ 感測器資料處理
├─ 手動控制（搖桿）支援
└─ 路徑規劃和導航

🖥️ AGVC 管理系統 (Control Center)
├─ 車隊管理和任務調度
├─ 資料庫管理和資料持久化
├─ Web 管理介面
├─ 外部系統整合（KUKA Fleet）
└─ 系統監控和日誌管理
```

### 技術架構層次
```
應用層: AGVCUI, OPUI, 車型特定邏輯
服務層: Web API, Socket.IO, ROS 2 Services
通訊層: Zenoh RMW, PLC 協定, HTTP/WebSocket
資料層: PostgreSQL, 配置檔案, 日誌系統
基礎層: Docker 容器, Ubuntu 24.04, 硬體設備
```

## 🔧 核心技術棧

### ROS 2 生態系統
- **ROS 2 Jazzy**: 最新的 ROS 2 發行版
- **Zenoh RMW**: 高效能的中間件實作
- **Python 3.12**: 主要開發語言
- **Colcon**: 建置系統

### 容器化技術
- **Docker Compose V2**: 容器編排
- **Ubuntu 24.04**: 基礎映像
- **Multi-stage Build**: 優化映像大小
- **Volume Mounting**: 開發時程式碼同步

### Web 技術棧
- **FastAPI**: 高效能 Web 框架
- **Socket.IO**: 即時通訊
- **SQLAlchemy**: ORM 框架
- **PostgreSQL**: 主要資料庫
- **Nginx**: 反向代理

### 工業通訊
- **Keyence PLC**: 設備控制
- **Modbus TCP**: 工業協定
- **KUKA Fleet**: 外部系統整合

## 📂 專案結構概覽

### 根目錄結構
```
RosAGV/
├── docker-compose.yml          # AGV 車載系統配置
├── docker-compose.agvc.yml     # AGVC 管理系統配置
├── Dockerfile                  # AGV 容器映像
├── Dockerfile.agvc             # AGVC 容器映像
├── app/                        # 應用程式碼目錄
├── scripts/                    # 工具腳本集合
├── docs-ai/                    # AI Agent 指導文件
├── CLAUDE.md                   # AI Agent 主要記憶文件
└── README.md                   # 專案概覽
```

### 應用程式碼結構
```
app/
├── *_ws/                       # ROS 2 工作空間
├── config/                     # 配置檔案
├── startup.*.bash              # 啟動腳本
├── setup.bash                  # 環境設定腳本
└── routerconfig.json5          # Zenoh 路由配置
```

## 🚀 部署環境

### AGV 車載環境
- **部署位置**: AGV 車輛上的邊緣計算設備
- **網路模式**: Host 網路，直接存取硬體
- **主要職責**: 即時控制、感測器處理、PLC 通訊
- **容器名稱**: `rosagv`

### AGVC 管理環境
- **部署位置**: 中央管理伺服器或雲端
- **網路模式**: Bridge 網路，隔離安全
- **主要職責**: 車隊管理、Web 服務、資料庫管理
- **容器名稱**: `agvc_server`, `postgres_container`, `nginx`

## 🔄 開發工作流程

### 環境要求
- **宿主機**: Ubuntu 24.04, Docker Compose V2
- **開發環境**: 必須在 Docker 容器內進行 ROS 2 開發
- **工具支援**: 統一工具系統 (`r` 命令)

### 基本開發流程
1. **環境啟動**: 使用 Docker Compose 啟動對應環境
2. **進入容器**: 使用便捷工具進入開發環境
3. **載入工作空間**: 使用 `all_source` 智能載入
4. **開發測試**: 在容器內進行開發和測試
5. **部署驗證**: 驗證功能並部署到目標環境

## 📋 關鍵概念

### 工作空間概念
- **AGV 工作空間**: 9 個專用於車載系統的 ROS 2 工作空間
- **AGVC 工作空間**: 11 個專用於管理系統的 ROS 2 工作空間
- **共用工作空間**: PLC 通訊、路徑規劃等共用功能

### 狀態機架構
- **3 層狀態機**: Base 層、AGV 層、Robot 層
- **車型特化**: 每種車型有專用的狀態機實作
- **事件驅動**: 基於事件的狀態轉換機制

### 通訊機制
- **Zenoh RMW**: 跨容器的 ROS 2 通訊
- **Socket.IO**: Web 界面的即時通訊
- **PLC 協定**: 與工業設備的通訊
- **REST API**: 標準的 Web API 介面

## 🎯 使用此文件的建議

### 對於 AI Agent
1. **首次載入**: 作為理解 RosAGV 專案的起點
2. **背景參考**: 在處理具體任務時提供系統性背景
3. **架構理解**: 理解雙環境設計和技術選型的原因

### 對於開發者
1. **新手導入**: 快速理解專案整體架構
2. **技術決策**: 理解技術棧選擇的背景和原因
3. **開發指導**: 為具體開發工作提供方向指引

### 交叉引用
- 詳細的雙環境架構: docs-ai/context/system/dual-environment.md
- 技術棧詳細說明: docs-ai/context/system/technology-stack.md
- 部署架構詳解: docs-ai/operations/deployment/docker-compose-configuration.md
- 工作空間詳細資訊: docs-ai/context/workspaces/agv-workspaces.md, docs-ai/context/workspaces/agvc-workspaces.md
