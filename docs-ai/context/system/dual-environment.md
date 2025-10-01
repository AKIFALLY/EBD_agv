# RosAGV 雙環境架構詳解

## 🎯 適用場景
- 理解 RosAGV 雙環境設計理念和實作細節
- 為跨環境開發提供架構指導
- 解決環境相關的部署和通訊問題

## 📋 雙環境設計理念

### 設計原則
**分離關注點**: 將即時控制與管理功能分離，提高系統穩定性和可維護性
- **AGV 車載系統**: 專注於即時控制、硬體整合、安全性
- **AGVC 管理系統**: 專注於車隊管理、資料處理、用戶介面

### 架構優勢
- **高可用性**: 車載系統獨立運行，不依賴中央系統
- **可擴展性**: 管理系統可以管理多台 AGV
- **維護性**: 各環境可以獨立更新和維護
- **安全性**: 車載系統與管理系統網路隔離

## 🚗 AGV 車載系統 (On-board Environment)

### 部署特性
```yaml
容器名稱: rosagv
網路模式: host
部署位置: AGV 車輛上的邊緣計算設備
Docker Compose: docker-compose.yml
```

### 核心職責
- **即時控制**: AGV 狀態機、運動控制、安全監控
- **硬體整合**: PLC 通訊、感測器資料處理、執行器控制
- **本地決策**: 避障、路徑跟隨、緊急停止
- **手動控制**: 搖桿操作、手動模式支援

### 網路模式選擇原因  
**為什麼 AGV 使用 Host 網路？**
- **SICK 感測器存取**: 需要直接存取 SICK SLAM 感測器
- **感測器軟體獨立**: SICK 有自己的軟體，不透過我們的軟體運行
- **硬體直接存取**: 需要直接硬體存取權限，減少抽象層
- **即時性要求**: 減少網路層級延遲，確保即時控制響應

### 工作空間配置 (9個)
```
agv_ws/                    # 核心 AGV 控制
├── agv_base              # 基礎狀態機架構
├── cargo_mover_agv       # Cargo 車型實作
├── loader_agv            # Loader 車型實作
└── unloader_agv          # Unloader 車型實作

agv_cmd_service_ws/       # 手動指令服務
joystick_ws/              # 搖桿控制整合
sensorpart_ws/            # 感測器資料處理
keyence_plc_ws/           # Keyence PLC 通訊
plc_proxy_ws/             # PLC 代理服務
path_algorithm/           # 路徑規劃演算法
```

### 硬體存取
- **直接硬體存取**: 透過 host 網路模式直接存取 PLC、感測器
- **USB 設備**: 搖桿、感測器等 USB 設備直接掛載
- **網路設備**: 直接存取工業乙太網路設備

### 啟動流程
```bash
# 啟動 AGV 車載系統
docker compose -f docker-compose.yml up -d

# 容器內自動執行
startup.agv.bash
├── SSH 服務啟動
├── Zenoh Router 啟動
├── 載入 AGV 工作空間
└── 啟動 AGV 核心服務
```

## 🖥️ AGVC 管理系統 (Control Center Environment)

### 部署特性
```yaml
容器名稱: agvc_server, postgres, nginx
網路模式: bridge (192.168.100.0/24)
部署位置: 中央管理伺服器或雲端
Docker Compose: docker-compose.agvc.yml
```

### 網路模式選擇原因
**為什麼 AGVC 使用 Bridge 網路？**
- **KUKA 系統整合**: KUKA 外部系統也是 Docker 部署在同台電腦上
- **固定 IP 需求**: 需要固定 IP 地址確保與 KUKA 系統穩定連接  
- **企業級隔離**: Bridge 網路提供更好的隔離性，適合企業級部署
- **網路管理**: 便於管理多個服務之間的網路通訊

### 核心職責
- **車隊管理**: 多台 AGV 的協調和調度
- **任務管理**: 任務分配、優先級管理、執行監控
- **資料管理**: 資料庫操作、歷史資料、報表生成
- **用戶介面**: Web 管理台、操作員介面
- **外部整合**: KUKA Fleet、WMS、ERP 系統整合

### 容器服務架構
```
AGVC 系統 (4個容器)
├── nginx (192.168.100.252)
│   ├── 反向代理服務
│   ├── 靜態文檔托管
│   └── WebSocket 支援
├── agvc_server (192.168.100.100)
│   ├── ROS 2 核心服務
│   ├── Web API (8000)
│   ├── AGVCUI (8001)
│   └── OPUI (8002)
├── postgres (192.168.100.254)
│   └── PostgreSQL 資料庫
└── pgadmin (192.168.100.101)
    └── 資料庫管理介面
```

### 工作空間配置 (11個)
```
web_api_ws/               # Web API 和 Socket.IO
├── web_api              # 核心 API 服務
├── agvcui               # 管理員界面
└── opui                 # 操作員界面

db_proxy_ws/              # 資料庫代理服務
ecs_ws/                   # 設備控制系統
rcs_ws/                   # 機器人控制系統
# (wcs_ws 已整合至 tafl_wcs_ws)
kuka_fleet_ws/            # KUKA Fleet 整合
tafl_ws/                  # TAFL 解析器和執行器
tafl_wcs_ws/              # TAFL WCS 整合系統
keyence_plc_ws/           # PLC 通訊 (共用)
plc_proxy_ws/             # PLC 代理 (共用)
path_algorithm/           # 路徑規劃 (共用)
```

### 服務架構
```
Port 8000: 核心 API 服務 (FastAPI + Socket.IO)
Port 8001: AGVCUI 管理員界面
Port 8002: OPUI 操作員界面
Port 5432: PostgreSQL 資料庫
Port 5050: pgAdmin4 管理工具
Port 80:   Nginx 反向代理
```

### 啟動流程
```bash
# 啟動 AGVC 管理系統 (4個服務容器)
docker compose -f docker-compose.agvc.yml up -d
# 啟動: nginx, agvc_server, postgres, pgadmin

# 容器內自動執行 (agvc_server)
startup.agvc.bash
├── SSH 服務啟動 (Port 2200)
├── Zenoh Router 啟動 (Port 7447)
├── 載入 AGVC 工作空間
└── Web 服務啟動 (Ports 8000-8002)
```

## 🌐 跨環境通訊機制

### Zenoh RMW 通訊
```
Zenoh Router (Port 7447)
├── AGV 車載系統 ←→ AGVC 管理系統
├── 跨容器 ROS 2 通訊
├── 自動服務發現
└── 高效能資料傳輸
```

### 通訊配置
- **Zenoh 配置**: `/app/routerconfig.json5`
- **自動連接**: 兩個環境自動建立 Zenoh 連接
- **服務發現**: ROS 2 服務和主題自動發現
- **資料同步**: 狀態資訊、指令、感測器資料

### 網路架構
```
AGV 車載 (Host 網路) - 直接硬體存取模式
├── 直接存取硬體網路 (SICK 感測器、PLC 設備)
├── Zenoh Router: 0.0.0.0:7447
├── 優點: 低延遲、直接硬體存取
└── 用途: 即時控制、感測器整合

AGVC 管理 (Bridge 網路: 192.168.100.0/24) - 企業級隔離模式
├── agvc_server: 192.168.100.100
├── postgres: 192.168.100.254  
├── nginx: 192.168.100.252
├── Zenoh Router: 192.168.100.100:7447
├── 優點: 網路隔離、固定 IP、安全管理
└── 用途: 企業系統整合、KUKA Fleet 通訊
```

## 🔧 開發環境配置

### 環境識別
```bash
# 檢查當前環境
pwd                    # /home/ct/RosAGV (宿主機)
                      # /app (容器內)

# 環境特徵
宿主機: 可執行 docker compose, git, sudo
容器內: 可執行 ros2, colcon, python3
```

### 工作空間載入
```bash
# 自動載入 (自動檢測環境)
all_source             # 或別名: sa

# 強制載入特定環境
agv_source            # 載入 AGV 工作空間
agvc_source           # 載入 AGVC 工作空間
```

### 開發工具
```bash
# 宿主機工具 (統一入口)
r                     # 顯示可用工具
r agvc-check          # AGVC 健康檢查
r containers-status   # 容器狀態檢查

# 容器內工具
check_system_status   # 系統狀態檢查
check_zenoh_status    # Zenoh 通訊檢查
```

## 🚀 部署策略

### 生產環境部署
- **AGV 車載**: 部署在每台 AGV 的邊緣計算設備
- **AGVC 管理**: 部署在中央伺服器或雲端平台
- **網路連接**: 透過工業 WiFi 或 5G 網路連接

### 開發環境部署
- **單機開發**: 在同一台機器上運行兩個環境
- **分散式開發**: AGV 環境在實體設備，AGVC 環境在開發機
- **模擬環境**: 使用模擬器替代實體 AGV 硬體

### 擴展策略
- **水平擴展**: 增加更多 AGV 車載系統
- **垂直擴展**: 提升 AGVC 管理系統的處理能力
- **地理分散**: 多個 AGVC 管理中心管理不同區域

## 📋 最佳實踐

### 開發原則
1. **環境隔離**: 在對應環境中開發對應功能
2. **通訊測試**: 定期測試跨環境通訊
3. **獨立測試**: 每個環境可以獨立測試
4. **版本同步**: 確保兩個環境的版本相容性

### 故障排除
1. **環境檢查**: 確認在正確的環境中操作
2. **通訊診斷**: 檢查 Zenoh 連接狀態
3. **服務狀態**: 檢查各環境的服務運行狀態
4. **日誌分析**: 分析兩個環境的日誌資訊

### 交叉引用
- 技術棧詳細說明: docs-ai/context/system/technology-stack.md
- 部署架構詳解: docs-ai/operations/deployment/docker-compose-configuration.md
- AGV 工作空間: docs-ai/context/workspaces/agv-workspaces.md
- AGVC 工作空間: docs-ai/context/workspaces/agvc-workspaces.md
- 容器管理操作: docs-ai/operations/deployment/container-management.md
