# RosAGV 車隊管理系統

## 📋 專案總覽

RosAGV 是一個基於 ROS 2 Jazzy 的企業級自動導引車（AGV）控制系統，專為工業自動化環境設計。系統採用雙環境架構，支援 AGV 車載系統和 AGVC 管理系統，整合了車隊管理、任務調度、設備控制、Web 介面等完整功能。

## 🏗️ 系統架構概述

`/app/` 目錄包含多個專案，這些專案在實際運行時分為兩種不同的運行環境，每種環境僅使用部分專案：

### 🚗 第一種運行環境：AGV 車載系統
**運行位置**：AGV 車輛上的電腦 Docker 環境
**啟動方式**：`docker-compose -f docker-compose.yml up -d`
**Docker 配置**：使用自定義 Docker image（參考 `Dockerfile`）
**內含環境**：ROS2 Jazzy + Python 虛擬環境 (pyvenv)

#### 車型分類與任務處理
目前支援三種車型，各自處理不同任務：
- **Cargo** - Rack架搬運車
- **Loader** - 房間內入料車
- **Unloader** - 房間內出料車

#### 程式架構特點
- 每種車型的程式僅在對應車型上運行
- 採用狀態模式 (State Pattern) 控制
- 三層狀態架構：
  - **Base 層**：所有車種共用的基礎邏輯狀態
  - **AGV 層**：依據實際車型決定的專用狀態
  - **Robot 層**：車上機器人執行取放任務時的狀態控制

### 🖥️ 第二種運行環境：AGVC 管理系統
**運行位置**：AGVC 管理電腦的 Docker 環境
**啟動方式**：`docker-compose -f docker-compose.agvc.yml up -d`
**Docker 配置**：使用自定義 Docker image（參考 `Dockerfile.agvc`）
**內含環境**：ROS2 Jazzy + Nginx + pgAdmin4 + PostgreSQL + Python 虛擬環境

#### 系統組件
- **ECS** (Equipment Control System) - 設備控制系統
- **RCS** (Robot Control System) - 機器人控制系統
- **TAFL** (Task Automation Flow Language) - 新一代任務流程語言
- **TAFL WCS** - 基於 TAFL 的倉庫控制系統（替代舊 WCS）
- **管理 UI 介面** - agvcui、opui、agvui、webapi

#### 外部系統整合
- **整合對象**：KUKA Fleet 系統
- **支援車型**：KUKA400i
- **控制方式**：由外部 KUKA Fleet 系統控制車輛運行邏輯
- **整合工具**：Kuka Fleet Adapter
- **功能**：透過 API 對 KUKA400i 下達指令與監控狀態
- **狀態回報**：KUKA 任務執行期間透過 WebAPI 回報任務執行狀態

### 工作空間結構與實際啟動分配
```
RosAGV/app/
├── shared_constants_ws/       # 🚗🖥️ 共用 - 共享常數定義 (最優先載入)
├── agv_ws/                    # 🚗🖥️ 共用 - AGV 核心控制系統
│   ├── agv_base              # 基礎狀態機架構
│   ├── cargo_mover_agv       # Cargo 車型實作
│   ├── loader_agv            # Loader 車型實作
│   └── unloader_agv          # Unloader 車型實作
├── agv_cmd_service_ws/        # 🚗 AGV 車載系統 - 手動指令服務
├── db_proxy_ws/               # 🖥️ AGVC 管理系統 - 資料庫代理服務
├── ecs_ws/                    # 🖥️ AGVC 管理系統 - 設備控制系統
├── joystick_ws/               # 🚗 AGV 車載系統 - 搖桿控制系統
├── keyence_plc_ws/            # 🚗🖥️ 共用 - Keyence PLC 通訊
├── kuka_fleet_ws/             # 🖥️ AGVC 管理系統 - KUKA 車隊整合
├── launch_ws/                 # 🖥️ AGVC 管理系統 - 系統啟動管理
├── plc_proxy_ws/              # 🚗🖥️ 共用 - PLC 代理服務
├── rcs_ws/                    # 🖥️ AGVC 管理系統 - 車隊控制系統
│   ├── rcs                   # 核心 RCS 服務
│   ├── rcs_interfaces        # RCS 介面定義
│   └── traffic_manager       # 交通管理模組
├── sensorpart_ws/             # 🚗 AGV 車載系統 - 感測器整合系統
├── tafl_ws/                   # 🖥️ AGVC 管理系統 - TAFL 解析器和執行器
├── tafl_wcs_ws/               # 🖥️ AGVC 管理系統 - TAFL WCS 整合系統
├── uno_gpio_ws/               # 🚗 AGV 車載系統 - 研華 UNO-137 GPIO 控制
├── web_api_ws/                # 🖥️ AGVC 管理系統 - Web API 和前端介面
│   ├── agvcui                # 管理員完整控制台
│   ├── agvui                 # AGV 車載 UI 介面
│   ├── opui                  # 操作員簡化介面
│   └── web_api               # 核心 API 服務
├── path_algorithm/            # 🚗🖥️ 共用 - 路徑規劃演算法
├── agv_ui/                    # 🚗 AGV 車載系統 - AGV UI 介面 (測試中)
├── config/                    # 🚗🖥️ 共用 - 統一配置文件
├── scripts/                   # 🚗🖥️ 共用 - 工具腳本集
├── agents/                    # 🚗🖥️ 共用 - 測試腳本目錄
├── docs/                      # 📚 技術文檔
├── routerconfig.json5         # 🚗🖥️ 共用 - Zenoh Router 配置
├── setup.bash                 # 🚗🖥️ 共用 - 環境設定腳本
├── startup.agv.bash           # 🚗 AGV 車載系統啟動腳本
└── startup.agvc.bash          # 🖥️ AGVC 管理系統啟動腳本
```

**圖例說明**：
- 🚗 AGV 車載系統專用
- 🖥️ AGVC 管理系統專用
- 🚗🖥️ 兩個環境共用
- 📚 文檔資源

**注意事項**：
- `shared_constants_ws` 在兩個環境中都是最優先載入的工作空間
- 實際載入的工作空間由 `setup.bash` 中的 `agv_source()` 和 `agvc_source()` 函數控制

## 🚀 快速開始

### 系統需求
- **作業系統**: Ubuntu 24.04 LTS
- **ROS 2**: Jazzy
- **RMW 實作**: rmw_zenoh_cpp (預設使用 Zenoh 通訊中介軟體)
- **Python**: 3.12+ (容器內使用虛擬環境)
- **資料庫**: PostgreSQL 12+
- **容器化**: Docker + Docker Compose V2
- **硬體**: 支援 USB 搖桿、PLC 設備
- **網路**: 支援 Zenoh Router 通訊 (預設 port 7447)

### 🚗 AGV 車載系統啟動
```bash
# 使用 Docker Compose V2 啟動 AGV 車載系統
docker compose -f docker-compose.yml up -d

# 查看運行狀態
docker compose -f docker-compose.yml ps

# 查看日誌
docker compose -f docker-compose.yml logs -f

# 進入容器檢查服務狀態
docker compose -f docker-compose.yml exec rosagv bash
```

**容器自動啟動流程**：
1. 執行 `/app/startup.agv.bash` 腳本
2. 啟動 SSH 服務 (port 2200)
3. 啟動 Zenoh Router (port 7447)
4. 載入 AGV 工作空間 (`agv_source()` 函數)：
   - keyence_plc_ws （PLC 通訊）
   - plc_proxy_ws （PLC 代理）
   - agv_cmd_service_ws （手動指令）
   - joystick_ws （搖桿控制）
   - agv_ws （AGV 核心）
   - path_algorithm （路徑規劃）
5. 選項: AGV UI 啟動 (AUTO_START_AGVUI=true)

### 🖥️ AGVC 管理系統啟動
```bash
# 使用 Docker Compose V2 啟動 AGVC 管理系統
docker compose -f docker-compose.agvc.yml up -d

# 查看運行狀態
docker compose -f docker-compose.agvc.yml ps

# 查看日誌
docker compose -f docker-compose.agvc.yml logs -f

# 進入容器檢查服務狀態
docker compose -f docker-compose.agvc.yml exec agvc_server bash
```

**容器自動啟動流程**：
1. 執行 `/app/startup.agvc.bash` 腳本
2. 啟動 SSH 服務 (port 2200)
3. 啟動 Zenoh Router (port 7447)
4. 載入完整 setup.bash 環境
5. 載入 AGVC 工作空間 (`agvc_source()` 函數)：
   - shared_constants_ws （共享常數，最優先）
   - keyence_plc_ws、plc_proxy_ws （PLC 系統）
   - path_algorithm、agv_ws （基礎功能）
   - db_proxy_ws （資料庫代理）
   - ecs_ws、rcs_ws （控制系統）
   - tafl_ws、tafl_wcs_ws （TAFL 系統）
   - web_api_ws、kuka_fleet_ws、launch_ws （應用層）
6. Web 服務啟動 (manage_web_api_launch)

### Zenoh 配置驗證

#### 🚗 AGV 車載系統驗證
```bash
# 檢查 Zenoh Router 程序狀態
docker compose -f docker-compose.yml exec rosagv bash -c "ps aux | grep rmw_zenohd"

# 檢查 Zenoh PID 檔案
docker compose -f docker-compose.yml exec rosagv bash -c "cat /tmp/zenoh_router.pid 2>/dev/null && echo ' (PID 檔案存在)' || echo 'PID 檔案不存在'"

# 檢查 Zenoh Router 日誌
docker compose -f docker-compose.yml exec rosagv bash -c "tail -10 /tmp/zenoh_router.log 2>/dev/null || echo 'Zenoh 日誌檔案不存在'"

# 檢查環境變數
docker compose -f docker-compose.yml exec rosagv bash -c "printenv | grep -E '(RMW_IMPLEMENTATION|ZENOH_ROUTER_CONFIG_URI)'"

# 檢查 Zenoh 配置檔案
docker compose -f docker-compose.yml exec rosagv bash -c "ls -la /app/routerconfig.json5 && echo '配置檔案存在'"

# 檢查 ROS 2 環境和節點通訊（需要等待啟動完成）
docker compose -f docker-compose.yml exec rosagv bash -c "source /opt/ros/\$ROS_DISTRO/setup.bash && source /opt/ws_rmw_zenoh/install/setup.bash && ros2 node list"
```

#### 🖥️ AGVC 管理系統驗證
```bash
# 檢查 Zenoh Router 程序狀態
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "ps aux | grep rmw_zenohd"

# 檢查 Zenoh PID 檔案
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "cat /tmp/zenoh_router.pid 2>/dev/null && echo ' (PID 檔案存在)' || echo 'PID 檔案不存在'"

# 檢查 Zenoh Router 日誌
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "tail -10 /tmp/zenoh_router.log 2>/dev/null || echo 'Zenoh 日誌檔案不存在'"

# 檢查環境變數
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "printenv | grep -E '(RMW_IMPLEMENTATION|ZENOH_ROUTER_CONFIG_URI)'"

# 檢查 Zenoh 配置檔案
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "ls -la /app/routerconfig.json5 && echo '配置檔案存在'"

# 檢查 ROS 2 環境和節點通訊
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "source /opt/ros/\$ROS_DISTRO/setup.bash && source /opt/ws_rmw_zenoh/install/setup.bash && ros2 node list"
```

### 開發環境建置（本地開發用）
```bash
# 注意：容器已自動執行啟動腳本，無需手動執行 startup.bash

# 1. 進入容器
# AGV 車載系統
docker compose -f docker-compose.yml exec rosagv bash
# AGVC 管理系統
docker compose -f docker-compose.agvc.yml exec agvc_server bash

# 2. 設定環境（容器內已自動載入）
source /app/setup.bash

# 3. 載入所有工作空間（容器內已自動執行）
all_source

# 4. 建置專案（如需重新建置）
build_all
```

## 🔧 核心功能

### 🚗 AGV 車載系統功能

#### 1. 多類型 AGV 支援
- **Cargo** - Rack架搬運車：負責料架的搬運和定位
- **Loader** - 房間內入料車：專門處理物料裝載任務
- **Unloader** - 房間內出料車：專門處理物料卸載任務

#### 2. 三層狀態架構控制
- **Base 層**：所有車種共用的基礎邏輯狀態
  - 初始化、待機、移動、停止等基本狀態
- **AGV 層**：依據實際車型決定的專用狀態
  - Cargo：搬運、定位、對接狀態
  - Loader：入料、裝載、確認狀態
  - Unloader：出料、卸載、清空狀態
- **Robot 層**：車上機器人執行取放任務時的狀態控制
  - 機械臂控制、夾具操作、安全檢查

#### 3. 設備整合
- **PLC 通訊**：支援 Modbus、Ethernet/IP、Keyence 協定
- **感測器融合**：雷射、視覺、距離感測器整合
- **搖桿控制**：手動操作和緊急控制

### 🖥️ AGVC 管理系統功能

#### 1. 核心管理系統
- **ECS** (Equipment Control System)：設備控制與監控
- **RCS** (Robot Control System)：機器人任務調度與控制
- **WCS** (Warehouse Control System)：倉庫作業流程管理

#### 2. KUKA Fleet 整合
- **支援車型**：KUKA400i 自動導引車
- **控制方式**：外部 KUKA Fleet 系統主導車輛運行邏輯
- **API 整合**：透過 Kuka Fleet Adapter 進行指令下達與狀態監控
- **狀態回報**：即時回報任務執行狀態與車輛位置

#### 3. Web 管理介面
- **AGVCUI**：管理員完整控制台
- **OPUI**：操作員簡化介面
- **WebAPI**：RESTful API 服務
- **即時監控**：WebSocket 即時資料推送

#### 4. 資料庫管理
- **PostgreSQL**：主要資料存儲
- **pgAdmin4**：資料庫管理介面
- **資料代理**：統一的資料存取服務

## 📊 專案完成度分析

### 🚗 AGV 車載系統專案狀態

| 專案 | 啟動狀態 | 核心功能 | 完成度 | 說明 |
|------|----------|----------|--------|------|
| agv_ws | ✅ 自動載入 | AGV 狀態控制 | 85% | 三層狀態架構，包含 agv_base、車型實作 |
| agv_cmd_service_ws | ✅ 自動載入 | 命令服務 | 80% | 手動指令介面定義完成 |
| joystick_ws | ✅ 自動載入 | 搖桿控制 | 75% | USB 搖桿手動控制完成 |
| keyence_plc_ws | ✅ 自動載入 | Keyence PLC | 90% | PLC 通訊穩定，共用模組 |
| plc_proxy_ws | ✅ 自動載入 | PLC 代理 | 95% | 多協定支援，共用模組 |
| path_algorithm | ✅ 自動載入 | 路徑規劃 | 85% | A* 演算法實現 |
| sensorpart_ws | ❌ 手動載入 | 感測器整合 | 70% | 3D 相機、OCR 識別 |
| uno_gpio_ws | ❌ 手動載入 | GPIO 控制 | 60% | 研華 UNO-137 工業電腦 GPIO |
| agv_ui | 🧑 測試中 | AGV UI | 50% | 車載 UI 介面測試中 |

### 🖥️ AGVC 管理系統專案狀態

| 專案 | 啟動狀態 | 核心功能 | 完成度 | 說明 |
|------|----------|----------|--------|------|
| shared_constants_ws | ✅ 優先載入 | 共享常數 | 95% | 系統共用常數定義 |
| db_proxy_ws | ✅ 自動載入 | 資料庫服務 | 90% | PostgreSQL 代理、連線池 |
| ecs_ws | ✅ 自動載入 | 設備控制 | 75% | 門控、設備管理 |
| rcs_ws | ✅ 自動載入 | 車隊控制 | 80% | 任務派發、交通管理 |
| tafl_ws | ✅ 自動載入 | TAFL 引擎 | 85% | TAFL 解析器和執行器 |
| tafl_wcs_ws | ✅ 自動載入 | TAFL WCS | 75% | 新一代 WCS 實作 |
| web_api_ws | ✅ 自動載入 | Web 服務 | 85% | FastAPI、Socket.IO、UI |
| kuka_fleet_ws | ✅ 自動載入 | KUKA 整合 | 60% | KUKA Fleet API 整合 |
| launch_ws | ✅ 自動載入 | 啟動管理 | 80% | ROS 2 Launch 配置 |

### 🚗🖥️ 共用專案狀態

| 專案 | 啟動狀態 | 核心功能 | 完成度 | 說明 |
|------|----------|----------|--------|------|
| keyence_plc_ws | ✅ 兩側載入 | Keyence PLC | 90% | PLC 通訊協定 |
| plc_proxy_ws | ✅ 兩側載入 | PLC 代理 | 95% | 多協定支援 |
| path_algorithm | ✅ 兩側載入 | 路徑規劃 | 85% | A* 演算法已實現 |
| agv_ws | ✅ AGVC 載入 | AGV 核心 | 85% | AGVC 也載入作為基礎 |
| config | ✅ 配置可用 | 配置管理 | 90% | YAML/JSON5 配置 |
| scripts | ✅ 工具可用 | 工具腳本 | 85% | 系統工具集 |

### 容器架構和網路配置

#### 🐳 Docker 容器配置

| 容器名稱 | IP 位址 | 服務 Port | 功能說明 |
|---------|----------|-----------|----------|
| nginx | 192.168.100.252 | 80 | 反向代理、靜態文件服務 |
| agvc_server | 192.168.100.100 | 7447, 2200, 8000-8002 | 核心 ROS 2 服務 |
| postgres | 192.168.100.254 | 5432 | PostgreSQL 資料庫 |
| pgadmin | 192.168.100.101 | 5050 | 資料庫管理介面 |

#### Zenoh 通訊配置

| 組件 | 狀態 | 說明 |
|------|------|------|
| rmw_zenoh_cpp | ✅ 已安裝 | 預建置在 Docker image 中 |
| Zenoh Router | ✅ 自動啟動 | Port 7447，跨容器通訊 |
| routerconfig.json5 | ✅ 已配置 | JSON5 格式，支援多節點 |

## 🔗 專案依賴關係與啟動順序

### 🚗 AGV 車載系統實際啟動流程
```mermaid
graph TD
    A[startup.agv.bash] --> B[SSH Service]
    A --> C[Zenoh Router]
    A --> D[載入工作空間]
    D --> E[keyence_plc_ws]
    D --> F[plc_proxy_ws]
    D --> G[agv_cmd_service_ws]
    D --> H[joystick_ws]
    D --> I[agv_ws]
    D --> J[path_algorithm]
    I --> K[ros2 launch loader_agv]
```

**AGV 車載系統實際啟動順序**：
1. **基礎服務**: SSH (port 2200), Zenoh Router (port 7447)
2. **工作空間載入**: keyence_plc_ws → plc_proxy_ws → agv_cmd_service_ws → joystick_ws → agv_ws → path_algorithm
3. **AGV 啟動**: `ros2 launch loader_agv launch.py`

### 🖥️ AGVC 管理系統實際啟動流程
```mermaid
graph TD
    A[startup.agvc.bash] --> B[SSH Service]
    A --> C[Zenoh Router]
    A --> D[載入 setup.bash]
    D --> E[agvc_source 函數]
    E --> F[shared_constants_ws]
    E --> G[keyence_plc_ws + plc_proxy_ws]
    E --> H[path_algorithm + agv_ws]
    E --> I[db_proxy_ws]
    E --> J[ecs_ws + rcs_ws]
    E --> K[tafl_ws + tafl_wcs_ws]
    E --> L[web_api_ws + kuka_fleet_ws]
    E --> M[launch_ws]
    A --> N[manage_web_api_launch]
    N --> O[Web Services :8000-8002]
```

**AGVC 管理系統實際啟動順序**：
1. **基礎服務**: SSH (port 2200), Zenoh Router (port 7447)
2. **setup.bash 載入**: 完整環境設定和工具函數
3. **agvc_source() 載入順序**:
   - shared_constants_ws (最優先)
   - 基礎工作空間: keyence_plc_ws, plc_proxy_ws, path_algorithm, agv_ws, db_proxy_ws
   - 應用工作空間: ecs_ws, rcs_ws, tafl_ws, tafl_wcs_ws, web_api_ws, kuka_fleet_ws, launch_ws
4. **Web 服務啟動**: manage_web_api_launch 管理 API (8000), AGVCUI (8001), OPUI (8002)

## 📝 開發指南

### Docker 環境開發
```bash
# 進入 AGV 車載系統容器
docker compose -f docker-compose.yml exec rosagv bash

# 進入 AGVC 管理系統容器
docker compose -f docker-compose.agvc.yml exec agvc_server bash

# 查看容器內掛載的配置檔案
ls -la /app/host/

# 檢查容器內服務狀態
ps aux | grep -E "(ssh|zenoh|ros2)"

# 查看啟動日誌
tail -f /tmp/zenoh_router.log
tail -f /tmp/agv.log
```

### Zenoh 開發注意事項
- **RMW 實作**: 系統預設使用 `rmw_zenoh_cpp`，無需手動設定
- **Router 配置**: 修改 `/app/routerconfig.json5` 來調整 Zenoh 連線設定
- **節點通訊**: 所有 ROS 2 節點自動使用 Zenoh 進行通訊
- **網路配置**: 確保 port 7447 可用於 Zenoh Router

### 程式碼規範
- 使用 Python 3.12+ 語法 (容器內版本)
- 完整的錯誤處理和日誌記錄
- 所有 ROS 2 節點支援 Zenoh RMW
- 狀態模式實現需遵循三層架構設計

### 測試策略
```bash
# AGV 車載系統測試
docker compose -f docker-compose.yml exec rosagv bash -c "cd /app && test_all"

# AGVC 管理系統測試
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "cd /app && test_all"

# 單一專案測試
cd <workspace> && colcon test

# Zenoh 連線測試（在容器內執行）
# AGV 車載系統
docker compose -f docker-compose.yml exec rosagv bash -c "source /opt/ros/\$ROS_DISTRO/setup.bash && source /opt/ws_rmw_zenoh/install/setup.bash && ros2 node list"
docker compose -f docker-compose.yml exec rosagv bash -c "source /opt/ros/\$ROS_DISTRO/setup.bash && source /opt/ws_rmw_zenoh/install/setup.bash && ros2 topic list"

# AGVC 管理系統
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "source /opt/ros/\$ROS_DISTRO/setup.bash && source /opt/ws_rmw_zenoh/install/setup.bash && ros2 node list"
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "source /opt/ros/\$ROS_DISTRO/setup.bash && source /opt/ws_rmw_zenoh/install/setup.bash && ros2 topic list"
```

### ECS 依賴關係說明
**重要發現**：`ecs_ws` 雖然未在容器啟動腳本中自動啟動，但其核心依賴已經載入：

```bash
# ECS 的依賴關係
ecs_ws → plc_proxy_ws (✅ 已載入)
ecs_ws → keyence_plc_ws (✅ 已載入)
ecs_ws → db_proxy_ws (❌ 未載入)
```

**手動啟動 ECS 服務**：
```bash
# 在 AGVC 管理系統容器中
docker compose exec agvc_server bash

# 建置 ecs_ws (如果尚未建置)
cd /app/ecs_ws && colcon build && source install/setup.bash

# 啟動 ECS 服務
ros2 launch ecs_launch launch.py

# 或單獨啟動 ECS 核心節點
ros2 run ecs ecs_core --ros-args -p db_url_agvc:="postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
```

### Zenoh 通訊診斷

#### 🔧 使用統一工具診斷（推薦）
```bash
# 快速 Zenoh 連接檢查
r zenoh-check

# Zenoh Router 配置管理
r zenoh-config           # 顯示配置概況
r zenoh-config validate  # 驗證配置檔案
r zenoh-config status    # 檢查服務狀態

# 專業 Zenoh 網路診斷
scripts/network-tools/zenoh-network.sh diagnose    # 全面診斷
scripts/network-tools/zenoh-network.sh test        # 連接測試
scripts/network-tools/zenoh-network.sh monitor     # 效能監控
```

#### 🔍 手動診斷方法（進階）
```bash
# 容器內 Zenoh 狀態檢查
docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c "
source /app/setup.bash && agvc_source
check_zenoh_status
"

# 檢查 Zenoh Router 日誌
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "
tail -f /tmp/zenoh_router.log
"

# 重啟 Zenoh Router (使用管理函數)
docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c "
source /app/setup.bash
manage_zenoh restart
"

# 檢查 ROS 2 節點通訊
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "
source /app/setup.bash && agvc_source
ros2 node list
ros2 topic list
"
```

### 系統健康檢查

#### 🏥 使用統一工具（推薦）
```bash
# 快速綜合診斷
r quick-diag

# 系統專用檢查
r agvc-check            # AGVC 系統健康檢查
r agv-check             # AGV 系統健康檢查
r containers-status     # 容器狀態檢查
r network-check         # 網路連接檢查

# 載入工具集後的專業檢查
source scripts/system-tools/system-tools.sh
system_health           # 完整健康檢查
system_quick_check      # 快速診斷
```

#### 📊 手動檢查方法（備用）
```bash
# Docker 容器狀態
docker compose -f docker-compose.yml ps
docker compose -f docker-compose.agvc.yml ps

# 容器內服務狀態 (使用 bash -i 確保環境載入)
docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c "
source /app/setup.bash && agvc_source
check_system_status
"

# Web 服務狀態
docker compose -f docker-compose.agvc.yml exec agvc_server bash -i -c "
source /app/setup.bash && agvc_source
manage_web_api_launch status
"

# 資料庫連接測試
docker compose -f docker-compose.agvc.yml exec postgres psql -U agvc -d agvc -c "SELECT version();"
```

### 貢獻流程
1. Fork 專案並建立功能分支
2. 確認目標運行環境（AGV 車載 vs AGVC 管理）
3. 了解當前啟動腳本中實際載入的工作空間
4. 遵循 Zenoh RMW 和容器化開發規範
5. 新增對應的測試案例
6. 更新相關文檔
7. 提交 Pull Request 到 GitHub 倉庫

## 🔧 維護與支援

### 建議監控指標（目標）
- **系統可用性**: 99.5% 目標
- **任務完成率**: 95% 目標
- **平均回應時間**: < 100ms 目標
- **錯誤率**: < 1% 目標

### 備份策略建議
- **資料庫**: 建議每日自動備份
- **配置文件**: Git 版本控制管理
- **日誌文件**: 建議 7 天保留期

### 技術支援
- **專案維護者**: yaze lin <yazelin@ching-tech.com>
- **開發單位**: 擎添工業 (Ching Tech)
- **GitHub 倉庫**: https://github.com/yazelin/RosAGV
- **問題回報**: GitHub Issues
- **技術文檔路徑**:
  - `/home/ct/RosAGV/docs/` - 系統文檔
  - `/home/ct/RosAGV/app/docs/` - 應用文檔
  - `/home/ct/RosAGV/docs-ai/` - AI Agent 文檔
  - `/home/ct/RosAGV/design/` - 設計文檔

## 📈 未來規劃（基於實際開發狀態）

### 🚗 AGV 車載系統發展規劃

#### 短期目標 (1-3 個月) - 完成現有功能
- [ ] **sensorpart_ws** (70%→85%): 修復自動載入，完成 3D 相機整合
- [ ] **uno_gpio_ws** (60%→80%): 穩定 GPIO 控制，加入自動載入
- [ ] **agv_ui** (50%→70%): 完成基本車載介面功能
- [ ] 修復狀態機錯誤處理中的已知問題
- [ ] 增加單元測試覆蓋率至 60%

#### 中期目標 (3-6 個月) - 系統穩定化
- [ ] 完成三種車型（Cargo、Loader、Unloader）的完整測試
- [ ] 建立車載系統健康檢查機制
- [ ] 優化 PLC 通訊穩定性（目前 90%→95%）
- [ ] 降低 CPU 使用率 10%（效能優化）
- [ ] 編寫操作手冊和故障排除指南

### 🖥️ AGVC 管理系統發展規劃

#### 短期目標 (1-3 個月) - 提升完成度
- [✓] AGVC 工作空間已全部啟用 (13 個工作空間)
- [ ] **kuka_fleet_ws** (60%→80%): 完成 KUKA API 整合測試
- [ ] **TAFL Editor**: 修復拖放問題，改善使用體驗
- [ ] **OPUI** (85%→95%): 根據操作員回饋優化介面
- [ ] 修復資料庫連線池偶發性洩漏問題

#### 中期目標 (3-6 個月) - 功能完善
- [ ] 實作任務優先級排程（基於現有 rcs_ws）
- [ ] 建立系統監控儀表板（使用現有 FastAPI + Socket.IO）
- [ ] 完成 Web API 效能優化（目標：<100ms 回應時間）
- [ ] 增加 PostgreSQL 查詢最佳化
- [ ] 建立自動備份和恢復機制

### 🛠️ 技術債務和品質改善（持續進行）

#### 程式碼品質
- [ ] 更新所有 setup.py 中的 'TODO: Package description'
- [x] 移除已廢棄的 wcs_ws 相關引用
- [ ] 統一錯誤處理和日誌格式
- [ ] 降低程式碼重複率至 5% 以下

#### 測試和驗證
- [ ] 自動化測試覆蓋率：60%→70%→80%（分階段提升）
- [ ] 整合測試：AGV ↔ AGVC 通訊穩定性
- [ ] 壓力測試：支援 5 台→10 台 AGV 同時運行
- [ ] 建立每日自動化測試流程

### 📊 實際可達成的系統目標

#### 生產環境準備 (6 個月)
- [ ] 完成生產環境部署文檔
- [ ] 建立系統監控和告警（基於日誌分析）
- [ ] 多 AGV 協同測試（3-5 台實機測試）
- [ ] 操作員培訓材料製作
- [ ] 建立標準作業程序 (SOP)

#### 持續改進項目
- [ ] **TAFL 系統**: 根據使用情況新增實用動詞
- [ ] **文檔同步**: 確保 docs-ai/ 與實際代碼同步
- [ ] **效能監控**: 建立基準測試和定期檢測
- [ ] **使用者回饋**: 每月收集並處理操作員建議

### ⚠️ 務實的限制認知
- 暫不考慮 AI/ML 功能（無相關基礎設施）
- 專注本地部署（不追求雲端化）
- 使用現有技術棧（ROS 2、FastAPI、PostgreSQL）
- 優先解決實際問題而非新功能開發

---

**版本**: 2.0.0
**最後更新**: 2025-09-22
**授權**: Proprietary - © 2025 Ching Tech Industrial Co., Ltd. All rights reserved.
**維護者**: 擎添工業 (Ching Tech)
