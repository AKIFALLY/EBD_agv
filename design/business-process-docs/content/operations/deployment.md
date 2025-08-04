# 部署指導

## 🎯 RosAGV 系統部署指南

本指南提供 RosAGV 系統的完整部署流程，包括環境準備、系統安裝、配置設定和驗證測試。

## 📋 部署概覽

### 雙環境部署架構
```
部署架構
├── 🚗 AGV 車載環境 (Edge Deployment)
│   ├── 部署位置: AGV 車輛邊緣計算設備
│   ├── 容器配置: docker-compose.yml
│   └── 服務: AGV 控制、PLC 通訊、感測器
└── 🖥️ AGVC 管理環境 (Central Deployment)
    ├── 部署位置: 中央伺服器或雲端
    ├── 容器配置: docker-compose.agvc.yml
    └── 服務: Web API、資料庫、管理介面
```

## 🛠️ 環境準備

### 系統要求

#### AGV 車載環境
```yaml
硬體要求:
  CPU: ARM64 或 x86_64 (4核心以上)
  記憶體: 8GB RAM (建議 16GB)
  儲存: 64GB SSD (建議 256GB)
  網路: Gigabit 乙太網路

軟體要求:
  作業系統: Ubuntu 24.04 LTS
  Docker: 24.0+
  Docker Compose: V2
  Python: 3.12 (由容器提供)
```

#### AGVC 管理環境
```yaml
硬體要求:
  CPU: x86_64 (8核心以上)
  記憶體: 16GB RAM (建議 32GB)
  儲存: 256GB SSD (建議 1TB)
  網路: Gigabit 乙太網路

軟體要求:
  作業系統: Ubuntu 24.04 LTS
  Docker: 24.0+
  Docker Compose: V2
  PostgreSQL: 16 (由容器提供)
```

### 網路規劃
```
網路配置
├── AGV 車載網路: Host 模式 (直接硬體存取)
├── AGVC 管理網路: Bridge 模式 (192.168.100.0/24)
├── PLC 通訊網路: 192.168.2.0/24
└── Zenoh 通訊端口: 7447
```

## 🚀 安裝步驟

### 1. 基礎環境安裝

#### 安裝 Docker 和 Docker Compose
```bash
# 更新系統
sudo apt update && sudo apt upgrade -y

# 安裝必要工具
sudo apt install -y curl wget git vim

# 安裝 Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# 安裝 Docker Compose V2
sudo curl -L "https://github.com/docker/compose/releases/latest/download/docker-compose-linux-$(uname -m)" -o /usr/local/bin/docker-compose
sudo chmod +x /usr/local/bin/docker-compose

# 設定用戶權限
sudo usermod -aG docker $USER
newgrp docker

# 驗證安裝
docker --version
docker compose version
```

### 2. 系統程式碼部署

#### 克隆 RosAGV 專案
```bash
# 克隆專案
git clone https://github.com/your-org/RosAGV.git
cd RosAGV

# 檢查分支
git branch -a
git checkout main  # 或其他穩定分支
```

#### 設定環境變數
```bash
# 創建環境配置檔案
cp .env.example .env

# 編輯環境變數
vim .env

# 基本環境變數範例
POSTGRES_DB=agvc
POSTGRES_USER=agvc
POSTGRES_PASSWORD=your_secure_password
ZENOH_ROUTER_PORT=7447
WEB_API_PORT=8000
```

### 3. AGV 車載環境部署

#### 配置 AGV 環境
```bash
# 編輯 AGV 配置
vim docker-compose.yml

# 設定硬體映射 (根據實際硬體調整)
vim app/config/hardware_mapping.yaml

# 設定 PLC 通訊參數
vim app/config/plc_config.yaml
```

#### 部署 AGV 服務
```bash
# 建置容器映像
docker compose -f docker-compose.yml build

# 啟動 AGV 服務
docker compose -f docker-compose.yml up -d

# 檢查服務狀態
docker compose -f docker-compose.yml ps
docker compose -f docker-compose.yml logs -f
```

### 4. AGVC 管理環境部署

#### 初始化資料庫
```bash
# 啟動資料庫服務
docker compose -f docker-compose.agvc.yml up -d postgres

# 等待資料庫就緒
sleep 30

# 執行資料庫初始化
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc -c "SELECT version();"
```

#### 部署管理服務
```bash
# 建置管理系統映像
docker compose -f docker-compose.agvc.yml build

# 啟動所有管理服務
docker compose -f docker-compose.agvc.yml up -d

# 檢查服務狀態
docker compose -f docker-compose.agvc.yml ps
```

## ⚙️ 配置設定

### 網路配置

#### AGV 網路設定
```bash
# 設定靜態 IP (如需要)
sudo vim /etc/netplan/01-network-manager-all.yaml

network:
  version: 2
  ethernets:
    eth0:
      addresses:
        - 192.168.1.100/24
      gateway4: 192.168.1.1
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]

# 應用網路設定
sudo netplan apply
```

#### 防火牆配置
```bash
# 開放必要端口
sudo ufw allow 7447   # Zenoh 通訊
sudo ufw allow 8501   # PLC 通訊
sudo ufw allow 22     # SSH

# AGVC 管理系統額外端口
sudo ufw allow 8000   # Web API
sudo ufw allow 8001   # AGVCUI
sudo ufw allow 8002   # OPUI
sudo ufw allow 5432   # PostgreSQL (僅內部)
```

### 服務配置

#### Zenoh Router 配置
```json5
// app/routerconfig.json5
{
  "mode": "router",
  "listen": {
    "endpoints": ["tcp/0.0.0.0:7447"]
  },
  "routing": {
    "face": {
      "unicast": {
        "accept_timeout": 10000
      }
    }
  }
}
```

#### PLC 通訊配置
```yaml
# app/config/plc_config.yaml
plc_connections:
  - name: "cargo_plc_01"
    ip: "192.168.2.101"
    port: 8501
    timeout: 5
    retry_count: 3
  - name: "loader_plc_01"
    ip: "192.168.2.102"
    port: 8501
    timeout: 5
    retry_count: 3
```

## 🔍 部署驗證

### 系統健康檢查

#### 基本服務檢查
```bash
# 檢查容器狀態
docker ps -a

# 檢查服務日誌
docker compose logs --tail=100

# 檢查網路連接
ping 192.168.2.101  # PLC 連接測試
telnet localhost 7447  # Zenoh 連接測試
```

#### Web 服務驗證
```bash
# 檢查 Web API
curl http://localhost:8000/health

# 檢查管理介面
curl http://localhost:8001/
curl http://localhost:8002/

# 檢查 API 文檔
open http://localhost:8000/docs
```

### 功能測試

#### ROS 2 通訊測試
```bash
# 進入容器
docker compose exec rosagv bash

# 載入 ROS 2 環境
source /app/setup.bash

# 檢查 ROS 2 節點
ros2 node list

# 檢查主題
ros2 topic list
ros2 topic echo /agv_status
```

#### PLC 通訊測試
```bash
# 測試 PLC 連接
docker compose exec rosagv bash -c "source /app/setup.bash && python3 -c 'from keyence_plc import KeyencePlcCom; plc = KeyencePlcCom(\"192.168.2.101\", 8501); print(plc.connect())'"
```

## 🚨 故障排除

### 常見問題

#### 容器啟動失敗
```bash
# 檢查端口衝突
sudo netstat -tulpn | grep :8000

# 檢查磁碟空間
df -h

# 檢查記憶體使用
free -h

# 重新建置容器
docker compose down
docker compose build --no-cache
docker compose up -d
```

#### 網路連接問題
```bash
# 檢查 Docker 網路
docker network ls
docker network inspect rosagv_agvc_network

# 重置 Docker 網路
docker compose down
docker network prune -f
docker compose up -d
```

#### 資料庫連接問題
```bash
# 檢查資料庫狀態
docker compose exec postgres pg_isready -U agvc

# 重置資料庫連接
docker compose restart postgres
sleep 30
docker compose restart agvc_server
```

## 📊 監控和維護

### 日常監控
```bash
# 系統資源監控
docker stats

# 服務狀態監控
docker compose ps

# 日誌監控
docker compose logs --tail=100 -f
```

### 備份策略
```bash
#!/bin/bash
# 資料庫備份腳本
DATE=$(date +%Y%m%d_%H%M%S)
docker compose exec postgres pg_dump -U agvc agvc > backup_${DATE}.sql

# 配置檔案備份
tar -czf config_backup_${DATE}.tar.gz app/config/
```

### 更新部署
```bash
# 系統更新流程
git pull origin main
docker compose build
docker compose up -d --force-recreate

# 滾動更新 (生產環境)
docker compose up -d --no-deps agvc_server
sleep 30
docker compose up -d --no-deps rosagv
```

## 📋 檢查清單

### 部署前檢查
- [ ] 硬體規格符合要求
- [ ] 作業系統版本正確
- [ ] Docker 和 Docker Compose 已安裝
- [ ] 網路連接正常
- [ ] 防火牆設定完成

### 部署後驗證
- [ ] 所有容器正常運行
- [ ] Web 介面可以存取
- [ ] PLC 通訊正常
- [ ] ROS 2 服務運行正常
- [ ] 資料庫連接正常
- [ ] 日誌沒有錯誤訊息

### 生產環境額外檢查
- [ ] 監控系統配置完成
- [ ] 備份策略已實施
- [ ] 安全性設定完成
- [ ] 效能基準測試通過
- [ ] 災難恢復計劃就緒

---

**相關文檔：**
- [系統架構](../system-architecture/dual-environment.md) - 了解架構設計
- [開發環境](development.md) - 開發環境設定
- [維護操作](maintenance.md) - 日常維護指導
- [故障排除](troubleshooting.md) - 問題診斷和解決