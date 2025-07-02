# 擎添工業 ROS AGV 專案

基於 ROS 2 Jazzy 和 Zenoh 通訊協定的自動導引車（AGV）控制系統，支援多種 AGV 類型包括 Loader、Cargo Mover 等。

## 🚀 專案特色

- **ROS 2 Jazzy**: 使用最新的 ROS 2 框架
- **Zenoh 通訊**: 採用 rmw_zenoh_cpp 作為 RMW 實作，提供高效能通訊
- **Docker 容器化**: 完整的容器化部署方案
- **多 AGV 支援**: 支援不同類型的 AGV（Loader、Cargo Mover）
- **狀態機架構**: 基於狀態機的 AGV 控制邏輯
- **Web UI**: 提供 Web 介面進行監控和控制

## 📁 專案結構

```
RosAGV/
├── app/                    # 主要開發環境
│   ├── agv_ws/            # AGV 工作空間
│   ├── agvc/              # AGV 控制器
│   ├── db_proxy_ws/       # 資料庫代理服務
│   ├── web_api_ws/        # Web API 服務
│   ├── ecs_ws/            # ECS 系統工作空間
│   ├── plc_proxy_ws/      # PLC 代理服務
│   ├── keyence_plc_ws/    # Keyence PLC 通訊
│   ├── joystick_ws/       # 搖桿控制
│   ├── config/            # 配置檔案
│   └── ui/                # 使用者介面
├── Design/                # 設計圖檔和流程圖
├── nginx/                 # Nginx 配置
└── shortcut/              # 桌面快捷方式
```

## 🛠️ 技術棧

- **ROS 2 Jazzy**: 機器人作業系統
- **Zenoh**: 高效能通訊中介軟體
- **Python 3.12**: 主要開發語言
- **FastAPI**: Web API 框架
- **SQLModel**: 資料庫 ORM
- **Docker**: 容器化部署
- **Nginx**: 反向代理伺服器

## 🚀 快速開始

### 使用 Docker 部署

1. **AGV 節點部署**:
```bash
docker-compose up -d
```

2. **AGVC 伺服器部署**:
```bash
docker-compose -f docker-compose.agvc.yml up -d
```

### 手動啟動

1. **環境設定**:
```bash
source app/setup.bash
```

2. **啟動 AGV**:
```bash
./app/agv_startup.bash
```

3. **啟動 AGVC 伺服器**:
```bash
./app/startup.agvc.bash
```

## 🏗️ AGV 類型

### Loader AGV
- 負責貨物裝載作業
- 支援多種裝載狀態機
- 整合視覺定位系統

### Cargo Mover AGV
- 負責貨物搬運作業
- 支援 Hokuyo 雷射感測器
- 具備安全防護機制

## 🔧 配置檔案

- `config/path.yaml`: 路徑規劃配置
- `config/hokuyo_dms_config.yaml`: 雷射感測器配置
- `config/ecs_config.yaml`: ECS 系統配置
- `config/door_config.yaml`: 門控系統配置
- `routerconfig.json5`: Zenoh 路由器配置

## 🌐 Web 介面

專案提供兩個主要的 Web 介面：
- **AGVC UI**: AGV 控制中心介面
- **OP UI**: 操作員介面

## 📊 監控與日誌

- 日誌檔案位於 `app/logs/` 目錄
- 支援即時狀態監控
- 提供詳細的錯誤追蹤

## 🔒 安全特性

- SSH 服務（Port 2200）
- 使用者權限管理
- 安全狀態檢查機制

## 📝 開發說明

### 環境需求
- Ubuntu 22.04 或更高版本
- ROS 2 Jazzy
- Python 3.12
- Docker & Docker Compose

### 開發工作流程
1. 修改程式碼
2. 重新建置工作空間
3. 測試功能
4. 提交變更

## 🤝 貢獻指南

歡迎提交 Issue 和 Pull Request 來改善專案。

## 📄 授權

本專案為擎添工業內部專案。
