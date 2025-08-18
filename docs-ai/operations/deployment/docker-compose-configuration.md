# Docker Compose 配置詳解

## 🎯 適用場景
- 理解 RosAGV 的容器化部署架構
- 配置和管理 Docker 容器環境
- 解決容器相關的網路和資源問題
- 優化容器部署和運行策略

## 📋 雙環境 Docker Compose 架構

RosAGV 採用雙環境架構，分別使用兩個 Docker Compose 檔案：
- **docker-compose.yml**: AGV 車載系統
- **docker-compose.agvc.yml**: AGVC 管理系統

## 🚗 AGV 車載系統配置 (docker-compose.yml)

### 容器配置
```yaml
services:
  rosagv:
    image: yazelin/agv:latest
    container_name: rosagv
    privileged: true
    network_mode: "host"
    restart: unless-stopped
```

### 關鍵配置說明

#### 網路模式
- **network_mode: "host"**: 使用主機網路模式
- **原因**: 需要直接存取 SICK 感測器和其他硬體設備
- **優點**: 低延遲、直接硬體存取
- **缺點**: 較少的網路隔離

#### 權限設定
- **privileged: true**: 給予容器完整的裝置存取權限
- **devices**: 掛載 `/dev/input` 讓容器可以讀取輸入設備（如搖桿）

#### 環境變數
```yaml
environment:
  CONTAINER_TYPE: "agv"
  ZENOH_ROUTER_CONFIG_URI: "/app/routerconfig.json5"
  RMW_IMPLEMENTATION: "rmw_zenoh_cpp"
  SDL_AUDIODRIVER: dummy  # 避免 pygame 的 ALSA 錯誤
```

#### 掛載目錄
```yaml
volumes:
  - ~/RosAGV/app:/app  # 主要應用程式目錄
```

#### 互動設定
- **tty: true**: 允許互動式終端
- **stdin_open: true**: 保持 STDIN 開啟
- **restart: unless-stopped**: 自動重啟策略

## 🖥️ AGVC 管理系統配置 (docker-compose.agvc.yml)

### 服務架構總覽
AGVC 系統包含 4 個主要服務：
1. **nginx**: 反向代理伺服器
2. **agvc_server**: 核心 AGVC 服務
3. **postgres**: PostgreSQL 資料庫
4. **pgadmin**: 資料庫管理工具

### 網路配置
```yaml
networks:
  bridge_network:
    driver: bridge
    ipam:
      config:
        - subnet: 192.168.100.0/24
          gateway: 192.168.100.1
```

## 📦 各服務詳細配置

### 1. Nginx 服務
```yaml
nginx:
  image: nginx:latest
  container_name: nginx
  ports:
    - "80:80"
  volumes:
    - ~/RosAGV/nginx:/etc/nginx/conf.d:ro
    - ~/RosAGV/design/business-process-docs:/usr/share/nginx/html/docs:ro
  restart: always
  networks:
    bridge_network:
      ipv4_address: 192.168.100.252
```

**主要功能**:
- 反向代理各個 Web 服務
- 托管靜態文檔系統
- 提供統一的 80 端口入口

### 2. AGVC Server
```yaml
agvc_server:
  image: yazelin/agvc:latest
  container_name: agvc_server
  restart: always
  networks:
    bridge_network:
      ipv4_address: 192.168.100.100
      mac_address: "02:42:00:00:00:01"
```

**端口映射**:
- 7447: Zenoh Router
- 2200: SSH Server
- 8000: FastAPI Server (Web API)
- 8001: AGVCUI
- 8002: OPUI
- 5173: Vue 開發伺服器

**環境變數**:
```yaml
environment:
  CONTAINER_TYPE: "agvc"
  DISPLAY: $DISPLAY
  ZENOH_ROUTER_CONFIG_URI: "/app/routerconfig.json5"
  RMW_IMPLEMENTATION: "rmw_zenoh_cpp"
```

**掛載目錄**:
```yaml
volumes:
  - ~/RosAGV/app:/app:rw  # 主程式目錄
  - ~/RosAGV/docker-compose.agvc.yml:/app/host/docker-compose.agvc.yml:ro
  - ~/RosAGV/docker-compose.yml:/app/host/docker-compose.yml:ro
  - ~/RosAGV/Dockerfile:/app/host/Dockerfile:ro
  - ~/RosAGV/Dockerfile.agvc:/app/host/Dockerfile.agvc:ro
  - ~/RosAGV/README.md:/app/host/README.md:rw
  - ~/RosAGV/.augment-guidelines:/app/host/.augment-guidelines:rw
  - /tmp/.X11-unix:/tmp/.X11-unix:rw  # X11 顯示支援
```

### 3. PostgreSQL 資料庫
```yaml
postgres:
  image: postgres:latest
  container_name: postgres
  restart: always
  networks:
    bridge_network:
      ipv4_address: 192.168.100.254
  environment:
    POSTGRES_USER: agvc
    POSTGRES_PASSWORD: password
    POSTGRES_DB: agvc
  ports:
    - "5432:5432"
  volumes:
    - postgres_data:/var/lib/postgresql/data
```

**資料庫配置**:
- 使用者: agvc
- 密碼: password
- 預設資料庫: agvc
- 資料持久化: postgres_data volume

### 4. pgAdmin 管理工具
```yaml
pgadmin:
  image: dpage/pgadmin4
  container_name: pgadmin
  restart: always
  networks:
    bridge_network:
      ipv4_address: 192.168.100.101
  environment:
    PGADMIN_DEFAULT_EMAIL: yazelin@ching-tech.com
    PGADMIN_DEFAULT_PASSWORD: password
  ports:
    - "5050:80"
  volumes:
    - pgadmin_data:/var/lib/pgadmin
```

**存取配置**:
- Web 介面: http://localhost:5050
- 登入帳號: yazelin@ching-tech.com
- 登入密碼: password

## 🌐 網路架構摘要

### AGVC Bridge 網路 IP 分配
| 服務 | 容器名稱 | IP 地址 | 主要端口 |
|------|---------|---------|----------|
| AGVC Server | agvc_server | 192.168.100.100 | 7447, 8000-8002 |
| pgAdmin | pgadmin | 192.168.100.101 | 5050 (映射到 80) |
| Nginx | nginx | 192.168.100.252 | 80 |
| PostgreSQL | postgres | 192.168.100.254 | 5432 |

### AGV Host 網路
- 使用主機網路模式
- 無固定 IP（使用主機 IP）
- 直接存取硬體設備

## 📊 Volume 管理

### Named Volumes
```yaml
volumes:
  postgres_data:  # PostgreSQL 資料持久化
  pgadmin_data:   # pgAdmin 配置持久化
```

### Bind Mounts
- **程式碼目錄**: `~/RosAGV/app:/app`
- **Nginx 配置**: `~/RosAGV/nginx:/etc/nginx/conf.d`
- **文檔目錄**: `~/RosAGV/design/business-process-docs:/usr/share/nginx/html/docs`

## 🔧 部署和管理

### 啟動服務
```bash
# 啟動 AGV 車載系統
docker compose -f docker-compose.yml up -d

# 啟動 AGVC 管理系統
docker compose -f docker-compose.agvc.yml up -d
```

### 查看狀態
```bash
# 查看 AGV 容器狀態
docker compose -f docker-compose.yml ps

# 查看 AGVC 容器狀態
docker compose -f docker-compose.agvc.yml ps
```

### 查看日誌
```bash
# AGV 日誌
docker compose -f docker-compose.yml logs -f rosagv

# AGVC 各服務日誌
docker compose -f docker-compose.agvc.yml logs -f agvc_server
docker compose -f docker-compose.agvc.yml logs -f postgres
docker compose -f docker-compose.agvc.yml logs -f nginx
```

### 重啟服務
```bash
# 重啟特定服務
docker compose -f docker-compose.agvc.yml restart nginx
docker compose -f docker-compose.agvc.yml restart agvc_server
```

## 🚨 注意事項

### MAC 地址設定
- AGVC Server 設定固定 MAC: `02:42:00:00:00:01`
- Docker 自動產生的 MAC 以 `02:42` 開頭
- 確保在 Docker 範圍內唯一

### X11 顯示支援
- Ubuntu 系統需要執行: `xhost +local:`
- 掛載 `/tmp/.X11-unix` 目錄
- 設定 `DISPLAY` 環境變數

### 容器名稱策略
- 容器名稱與映像名稱保持一致
- 減少 Claude AI 混淆的機會
- 例如: postgres 容器使用 postgres 名稱

## 📋 配置最佳實踐

### 網路隔離
- AGVC 使用 Bridge 網路提供隔離
- AGV 使用 Host 網路直接存取硬體
- 固定 IP 確保服務間穩定通訊

### 資料持久化
- 使用 Named Volumes 儲存資料庫
- 重要配置檔案使用唯讀掛載
- 程式碼目錄使用讀寫掛載

### 安全性考量
- 生產環境應修改預設密碼
- 限制不必要的端口暴露
- 使用唯讀掛載保護配置檔案

## 🔗 交叉引用
- 雙環境架構: @docs-ai/context/system/dual-environment.md
- Nginx 配置: @docs-ai/operations/deployment/nginx-configuration.md
- 容器管理: @docs-ai/operations/deployment/container-management.md
- 技術棧: @docs-ai/context/system/technology-stack.md
- 系統診斷: @docs-ai/operations/maintenance/system-diagnostics.md