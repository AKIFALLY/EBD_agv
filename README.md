# RosAGV 智能車隊管理系統

## 📋 專案總覽

RosAGV 是一個基於 ROS 2 Jazzy 和 Zenoh 通訊中介軟體的企業級自動導引車（AGV）控制系統，專為工業自動化環境設計。系統採用雙環境容器化架構，支援 AGV 車載系統和 AGVC 管理系統，提供完整的車隊管理、任務調度、設備控制解決方案。

### 🎯 核心價值
- **雙環境架構**：AGV 車載系統 + AGVC 管理系統分離部署
- **現代化技術棧**：ROS 2 Jazzy + Zenoh RMW + Docker 容器化
- **多車型支援**：Cargo、Loader、Unloader 三種車型
- **外部系統整合**：支援 KUKA Fleet 系統整合
- **完整 Web 介面**：AGVCUI 管理控制台 + OPUI 操作介面

### 🏭 應用場景
- 智能工廠物料搬運
- 倉庫自動化管理
- 生產線物流調度
- 多車協同作業

## 🏗️ 系統架構

### 雙環境運行模式

```
┌─────────────────────────────────────────────────────────────────┐
│                    RosAGV 智能車隊管理系統                        │
├─────────────────────────────────────────────────────────────────┤
│  🚗 AGV 車載系統          │  🖥️ AGVC 管理系統                    │
│  ├─ AGV 狀態控制          │  ├─ 車隊管理 (RCS)                   │
│  ├─ PLC 通訊              │  ├─ 倉庫控制 (WCS)                   │
│  ├─ 感測器整合            │  ├─ 設備控制 (ECS)                   │
│  ├─ 搖桿控制              │  ├─ KUKA Fleet 整合                  │
│  └─ 路徑規劃              │  ├─ Web 管理介面                     │
│                           │  └─ 資料庫管理                       │
├─────────────────────────────────────────────────────────────────┤
│  共用基礎設施：Zenoh RMW + PostgreSQL + Nginx                    │
└─────────────────────────────────────────────────────────────────┘
```

## 🔧 環境需求

### 系統需求
- **作業系統**: Ubuntu 24.04 LTS
- **容器化**: Docker 24.0+ + Docker Compose V2
- **記憶體**: 最少 8GB RAM (建議 16GB+)
- **儲存空間**: 最少 50GB 可用空間
- **網路**: 支援 Zenoh Router 通訊 (port 7447)

### 硬體需求
- **AGV 車載系統**: 支援 USB 搖桿、PLC 設備
- **AGVC 管理系統**: 支援 PostgreSQL、Web 服務

### 網路配置
- **SSH 存取**: port 2200 (容器內)
- **Zenoh Router**: port 7447
- **Web 介面**: port 80 (Nginx)
- **PostgreSQL**: port 5432
- **pgAdmin4**: port 5050

## 🚀 快速部署

### 🚗 AGV 車載系統部署
```bash
# 啟動 AGV 車載系統
docker compose -f docker-compose.yml up -d

# 查看運行狀態
docker compose -f docker-compose.yml ps

# 查看啟動日誌
docker compose -f docker-compose.yml logs -f rosagv

# 進入容器進行開發
docker compose -f docker-compose.yml exec rosagv bash
```

### 🖥️ AGVC 管理系統部署
```bash
# 啟動 AGVC 管理系統
docker compose -f docker-compose.agvc.yml up -d

# 查看運行狀態
docker compose -f docker-compose.agvc.yml ps

# 查看啟動日誌
docker compose -f docker-compose.agvc.yml logs -f agvc_server

# 進入容器進行管理
docker compose -f docker-compose.agvc.yml exec agvc_server bash
```

## 🔄 容器管理

### 啟動和停止
```bash
# 啟動服務
docker compose -f <compose-file> up -d

# 停止服務
docker compose -f <compose-file> down

# 重啟服務
docker compose -f <compose-file> restart

# 強制重建並啟動
docker compose -f <compose-file> up -d --build --force-recreate
```

### 日誌和監控
```bash
# 查看即時日誌
docker compose -f <compose-file> logs -f [service_name]

# 查看最近 100 行日誌
docker compose -f <compose-file> logs --tail=100 [service_name]

# 查看容器資源使用情況
docker stats

# 查看容器詳細資訊
docker compose -f <compose-file> ps -a
```

### 健康檢查
```bash
# 檢查容器狀態
docker compose -f docker-compose.yml exec rosagv bash -c "ps aux | grep -E '(ssh|zenoh|ros2)'"
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "ps aux | grep -E '(ssh|zenoh|postgres)'"

# 檢查 Zenoh Router 狀態
docker compose -f <compose-file> exec <container> bash -c "cat /tmp/zenoh_router.pid && pgrep -f rmw_zenohd"

# 檢查 ROS 2 環境
docker compose -f <compose-file> exec <container> bash -c "printenv | grep -E '(ROS_DISTRO|RMW_IMPLEMENTATION)'"
```

## 🚨 故障排除

### 常見部署問題

#### 1. 容器啟動失敗
```bash
# 檢查 Docker 服務狀態
sudo systemctl status docker

# 檢查 Docker Compose 版本
docker compose version

# 查看詳細錯誤日誌
docker compose -f <compose-file> logs <service_name>

# 清理並重新啟動
docker compose -f <compose-file> down -v
docker compose -f <compose-file> up -d --build
```

#### 2. 網路連線問題
```bash
# 檢查 Docker 網路
docker network ls
docker network inspect <network_name>

# 檢查端口佔用
sudo netstat -tulpn | grep -E "(2200|7447|5432|5050|80)"

# 測試容器間連線
docker compose -f <compose-file> exec <container> ping <target_container>
```

#### 3. Zenoh Router 連線問題
```bash
# 檢查 Zenoh 配置
docker compose -f <compose-file> exec <container> cat /app/routerconfig.json5

# 重啟 Zenoh Router
docker compose -f <compose-file> exec <container> bash -c "
pkill -f rmw_zenohd
sleep 2
nohup ros2 run rmw_zenoh_cpp rmw_zenohd > /tmp/zenoh_router.log 2>&1 &
echo \$! > /tmp/zenoh_router.pid
"
```

#### 4. 資料庫連線問題 (AGVC 系統)
```bash
# 檢查 PostgreSQL 狀態
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "pg_isready -h localhost -p 5432"

# 查看資料庫日誌
docker compose -f docker-compose.agvc.yml logs postgres

# 重啟資料庫服務
docker compose -f docker-compose.agvc.yml restart postgres
```

## 📁 專案結構

```
RosAGV/
├── README.md                      # 本檔案 - 部署和管理指南
├── docker-compose.yml             # AGV 車載系統配置
├── docker-compose.agvc.yml        # AGVC 管理系統配置
├── Dockerfile                     # AGV 車載系統映像
├── Dockerfile.agvc                # AGVC 管理系統映像
└── app/                           # 應用程式碼目錄
    ├── README.md                  # 開發者技術文檔
    ├── startup.agv.bash           # AGV 車載系統啟動腳本
    ├── startup.agvc.bash          # AGVC 管理系統啟動腳本
    ├── setup.bash                 # 通用開發環境腳本

    ├── setup.augment.bash         # AI 助理專用腳本
    ├── routerconfig.json5         # Zenoh Router 配置
    ├── *_ws/                      # ROS 2 工作空間
    └── config/                    # 系統配置檔案
```

### 重要檔案說明
- **docker-compose.yml**: AGV 車載系統的容器編排配置
- **docker-compose.agvc.yml**: AGVC 管理系統的容器編排配置
- **app/README.md**: 詳細的開發者技術文檔和 ROS 2 指南
- **routerconfig.json5**: Zenoh Router 的網路通訊配置
- **startup.*.bash**: 容器啟動時自動執行的初始化腳本

## 📚 進階使用

### 開發環境設定
詳細的開發環境設定、ROS 2 工作空間管理、Zenoh 配置等技術細節，請參考 [app/README.md](app/README.md)。

### 系統整合
- KUKA Fleet 整合配置
- PLC 設備連線設定
- 資料庫架構設計
- Web 介面客製化

### 效能調優
- Zenoh Router 效能配置
- Docker 資源限制設定
- PostgreSQL 效能調優
- 網路頻寬最佳化

## 🤝 支援和貢獻

### 技術支援
- 部署問題：檢查本文檔的故障排除章節
- 開發問題：參考 [app/README.md](app/README.md)
- 系統整合：聯繫技術團隊

### 貢獻指南
1. Fork 專案並建立功能分支
2. 確認目標部署環境（AGV 車載 vs AGVC 管理）
3. 遵循 Docker 容器化開發規範
4. 新增對應的測試案例
5. 更新相關文檔
6. 提交 Pull Request

## 📖 相關文檔

### 技術文檔
- [開發者指南](app/README.md) - 詳細的 ROS 2 開發和 Zenoh 配置指南
- [API 文檔](app/web_api_ws/README.md) - Web API 介面說明
- [PLC 通訊指南](app/plc_proxy_ws/README.md) - PLC 設備整合說明

### 配置文檔
- [Docker 配置說明](docker-compose.yml) - AGV 車載系統容器配置
- [AGVC 配置說明](docker-compose.agvc.yml) - AGVC 管理系統容器配置
- [Zenoh 配置](app/routerconfig.json5) - Zenoh Router 網路配置

### 快速參考
```bash
# 常用容器指令
docker compose -f docker-compose.yml up -d          # 啟動 AGV 車載系統
docker compose -f docker-compose.agvc.yml up -d     # 啟動 AGVC 管理系統
docker compose -f <file> exec <container> bash      # 進入容器
docker compose -f <file> logs -f <service>          # 查看日誌
docker compose -f <file> down                       # 停止服務

# 容器內開發指令 (進入容器後執行)
source setup.bash                                   # 載入開發環境
build_all                                          # 建置所有工作空間
test_all                                           # 測試所有工作空間
check_system_status                                # 檢查系統狀態
```

## 🔄 版本歷史

### v1.0.0 (2025-01-17)
- ✅ 完成雙環境容器化架構
- ✅ 整合 ROS 2 Jazzy + Zenoh RMW
- ✅ 實現 AGV 車載系統基礎功能
- ✅ 建立 AGVC 管理系統框架
- ✅ 完成 PLC 通訊整合
- ✅ 建立完整的文檔體系

### 未來版本規劃
- 🔄 v1.1.0: 完成 AGVC 管理系統所有服務啟動
- 🔄 v1.2.0: 完成 KUKA Fleet 系統整合
- 🔄 v2.0.0: 支援多工廠部署和雲端管理

---

**版本**: v1.0.0
**最後更新**: 2025-01-17
**維護團隊**: RosAGV 開發團隊
**授權**: MIT License
