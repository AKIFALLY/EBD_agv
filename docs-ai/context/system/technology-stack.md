# RosAGV 技術棧和依賴關係

## 🎯 適用場景
- 理解 RosAGV 技術選型的背景和原因
- 為技術決策和架構設計提供參考
- 解決技術相關的相容性和整合問題

## 📋 核心技術棧概覽

### 技術架構分層
```
┌─────────────────────────────────────────────┐
│ 應用層: AGVCUI, OPUI, 車型特定邏輯           │
├─────────────────────────────────────────────┤
│ 服務層: Web API, Socket.IO, ROS 2 Services │
├─────────────────────────────────────────────┤
│ 通訊層: Zenoh RMW, PLC 協定, HTTP/WebSocket │
├─────────────────────────────────────────────┤
│ 資料層: PostgreSQL, 配置檔案, 日誌系統      │
├─────────────────────────────────────────────┤
│ 基礎層: Docker 容器, Ubuntu 24.04, 硬體設備 │
└─────────────────────────────────────────────┘
```

## 🤖 ROS 2 生態系統

### ROS 2 Jazzy
- **版本**: ROS 2 Jazzy Jalapa (最新 LTS)
- **選擇原因**: 長期支援、穩定性、最新功能
- **核心特性**: 
  - 改進的 DDS 實作
  - 更好的即時性能
  - 增強的安全性
  - 更完善的工具鏈

### Zenoh RMW (rmw_zenohd)
- **版本**: 基於 Zenoh 0.11.x
- **選擇原因**: 
  - 高效能、低延遲通訊
  - 跨網路的無縫通訊
  - 自動服務發現
  - 更好的網路適應性
- **配置**: `/app/routerconfig.json5`
- **端口**: 7447 (Zenoh Router)

### Python 3.12
- **選擇原因**: 
  - ROS 2 Jazzy 官方支援
  - 最新語言特性
  - 更好的效能
  - 豐富的生態系統
- **虛擬環境**: `/opt/pyvenv_env`
- **套件管理**: pip3, 虛擬環境隔離

### Colcon 建置系統
- **用途**: ROS 2 工作空間建置和管理
- **特性**: 
  - 並行建置
  - 依賴關係管理
  - 多語言支援
  - 測試整合

## 🐳 容器化技術

### Docker Compose V2
- **版本**: Docker Compose V2 (docker compose)
- **選擇原因**: 
  - 更好的效能
  - 改進的 CLI 體驗
  - 更好的錯誤處理
  - 官方推薦版本
- **配置檔案**: 
  - `docker-compose.yml` (AGV 車載)
  - `docker-compose.agvc.yml` (AGVC 管理)

### Ubuntu 24.04 LTS
- **基礎映像**: ubuntu:24.04
- **選擇原因**: 
  - 最新 LTS 版本
  - ROS 2 Jazzy 官方支援
  - 長期安全更新
  - 現代化的系統工具
- **Python 版本**: Python 3.12 (系統預設)

### 容器架構設計
```
Multi-stage Build
├── Base Stage: 系統依賴和 ROS 2 安裝
├── Development Stage: 開發工具和除錯工具
├── Production Stage: 精簡的生產環境
└── Runtime Stage: 最終執行環境
```

### 網路配置
- **AGV 容器**: Host 網路模式 (直接硬體存取)
- **AGVC 容器**: Bridge 網路模式 (192.168.100.0/24)
- **Volume 掛載**: 程式碼同步、配置檔案、資料持久化

## 🌐 Web 技術棧

### FastAPI
- **版本**: FastAPI 0.104+
- **選擇原因**: 
  - 高效能 (基於 Starlette 和 Pydantic)
  - 自動 API 文檔生成
  - 現代 Python 特性支援
  - 優秀的型別檢查
- **用途**: 核心 API 服務 (Port 8000)

### Socket.IO
- **版本**: python-socketio 5.x
- **選擇原因**: 
  - 即時雙向通訊
  - 自動重連機制
  - 跨瀏覽器相容性
  - 房間和命名空間支援
- **用途**: AGVCUI 和 OPUI 即時通訊

### SQLAlchemy + SQLModel
- **版本**: SQLAlchemy 2.x, SQLModel 0.0.14+
- **選擇原因**: 
  - 現代 ORM 設計
  - FastAPI 原生整合
  - 型別安全
  - 異步支援
- **用途**: 資料庫操作和模型定義

### PostgreSQL
- **版本**: PostgreSQL 16
- **選擇原因**: 
  - 企業級可靠性
  - 豐富的資料型別
  - 優秀的效能
  - 完整的 ACID 支援
- **配置**: 
  - 容器: postgres
  - 端口: 5432
  - 管理工具: pgAdmin4 (Port 5050)

### Nginx
- **版本**: nginx:latest
- **容器配置**:
  - 容器名稱: nginx
  - IP 地址: 192.168.100.252
  - 端口: 80
- **主要功能**: 
  - 反向代理 (將域名導向內部服務)
  - WebSocket 支援 (Socket.IO 通訊)
  - 靜態檔案服務 (文檔系統)
  - CORS 處理
  - gzip 壓縮
  - 安全標頭設定
- **虛擬主機配置**:
  - `agvc.webapi` → :8000 (Web API)
  - `localhost/agvc.ui` → :8001 (AGVCUI)
  - `op.ui` → :8002 (OPUI)
  - `/docs/` → 動態文檔系統
- **掛載目錄**:
  - 配置檔案: `/home/ct/EBD_agv/nginx`
  - 文檔檔案: `/home/ct/EBD_agv/design/business-process-docs`

## 🏭 工業通訊技術

### Keyence PLC 通訊
- **協定**: Keyence 專有協定
- **連接方式**: TCP/IP 乙太網路
- **用途**: 
  - 設備狀態監控
  - 控制指令發送
  - 感測器資料讀取
- **實作**: `keyence_plc_ws`

### Modbus TCP
- **協定**: Modbus TCP/IP
- **用途**: 通用工業設備通訊
- **特性**: 
  - 標準化協定
  - 廣泛設備支援
  - 簡單可靠

### KUKA Fleet 整合
- **協定**: KUKA Fleet API
- **用途**: 外部機器人系統整合
- **功能**: 
  - 任務同步
  - 狀態監控
  - 協調控制
- **實作**: `kuka_fleet_ws`

## 🔧 開發和工具技術

### 程式碼品質工具
```bash
# 靜態分析
pylint              # 程式碼品質檢查
mypy                # 型別檢查
black               # 程式碼格式化
isort               # 導入排序

# 測試框架
pytest              # 單元測試
pytest-cov          # 測試覆蓋率
pytest-asyncio      # 異步測試

# 安全掃描
bandit              # 安全漏洞掃描
safety              # 依賴安全檢查
```

### 監控和日誌
```bash
# 日誌處理
python-logging      # Python 標準日誌
loguru              # 增強日誌庫

# 監控工具
psutil              # 系統監控
docker stats        # 容器監控
```

### 開發工具
```bash
# 編輯器支援
VS Code             # 主要開發環境
Python Extension    # Python 語言支援
ROS Extension       # ROS 2 支援

# 除錯工具
pdb                 # Python 除錯器
gdb                 # C++ 除錯器
ros2 bag            # ROS 2 資料記錄
```

## 📦 依賴管理

### Python 套件管理
```bash
# 系統套件 (apt)
ros-jazzy-*         # ROS 2 核心套件
python3-pip         # Python 套件管理器
python3-venv        # 虛擬環境

# Python 套件 (pip)
fastapi[all]        # Web 框架
sqlalchemy[asyncio] # 資料庫 ORM
python-socketio     # Socket.IO 支援
psycopg2-binary     # PostgreSQL 驅動
```

### 虛擬環境配置
```bash
# 虛擬環境位置
/opt/pyvenv_env/

# 自動載入機制
PYTHONPATH=/opt/pyvenv_env/lib/python3.12/site-packages

# 套件安裝
/opt/pyvenv_env/bin/pip3 install package_name
```

### 容器依賴
```dockerfile
# 基礎依賴
FROM ubuntu:24.04
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common

# ROS 2 安裝
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

## 🔄 版本相容性

### 主要版本對應
```
ROS 2 Jazzy ←→ Ubuntu 24.04 ←→ Python 3.12
FastAPI 0.104+ ←→ SQLAlchemy 2.x ←→ Pydantic 2.x
Docker Compose V2 ←→ Docker Engine 20.10+
PostgreSQL 16 ←→ psycopg2 2.9+
```

### 升級策略
- **ROS 2**: 跟隨 LTS 版本，定期評估升級
- **Python**: 使用系統預設版本，確保相容性
- **Web 框架**: 定期更新，注意破壞性變更
- **資料庫**: 保守升級，重視資料相容性

## 🚀 效能考量

### 系統效能
- **ROS 2 + Zenoh**: 低延遲通訊 (<1ms)
- **FastAPI**: 高併發處理 (>10k req/s)
- **PostgreSQL**: 企業級效能
- **Docker**: 接近原生效能

### 資源使用
- **記憶體**: 每個容器 2-4GB
- **CPU**: 多核心並行處理
- **網路**: Gigabit 乙太網路
- **儲存**: SSD 推薦，支援快速 I/O

### 最佳化策略
- **容器最佳化**: Multi-stage build, 精簡映像
- **資料庫最佳化**: 索引設計, 查詢最佳化
- **網路最佳化**: Zenoh 配置調整
- **程式碼最佳化**: 異步處理, 快取機制

## 📋 技術決策記錄

### 為什麼選擇 Zenoh RMW?
- **效能**: 比傳統 DDS 更高效能
- **網路適應性**: 更好的跨網路通訊
- **簡化配置**: 減少 DDS 配置複雜性
- **未來導向**: ROS 2 的發展方向

### 為什麼選擇 FastAPI?
- **現代設計**: 基於現代 Python 特性
- **效能**: 接近 Node.js 和 Go 的效能
- **開發體驗**: 自動文檔、型別檢查
- **生態系統**: 豐富的插件和整合

### 為什麼選擇 Docker Compose V2?
- **官方推薦**: Docker 官方主推版本
- **效能改進**: 更快的啟動和操作
- **功能增強**: 更多功能和更好的錯誤處理
- **未來支援**: V1 已停止維護

## 🔗 交叉引用
- 雙環境架構: docs-ai/context/system/dual-environment.md
- 部署架構: docs-ai/operations/deployment/docker-compose-configuration.md
- 容器管理: docs-ai/operations/deployment/container-management.md
