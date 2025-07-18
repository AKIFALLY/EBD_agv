# RosAGV 智能車隊管理系統

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
- **WCS** (Warehouse Control System) - 倉庫控制系統
- **管理 UI 介面** - agvcui、opui、webapi

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
├── agv_ws/                    # 🚗 AGV 車載系統 - AGV 核心控制系統 (實際啟動)
├── agv_cmd_service_ws/        # 🚗 AGV 車載系統 - AGV 命令服務 (實際啟動)
├── db_proxy_ws/               # 🖥️ AGVC 管理系統 - 資料庫代理服務 (未在啟動腳本中)
├── ecs_ws/                    # 🖥️ AGVC 管理系統 - 設備控制系統 (未在啟動腳本中)
├── joystick_ws/               # 🚗 AGV 車載系統 - 搖桿控制系統 (實際啟動)
├── keyence_plc_ws/            # 🚗🖥️ 共用 - Keyence PLC 通訊 (實際啟動)
├── kuka_fleet_ws/             # 🖥️ AGVC 管理系統 - KUKA 車隊整合 (未在啟動腳本中)
├── launch_ws/                 # 🚗🖥️ 共用 - 系統啟動管理 (有 ecs_launch 但未自動啟動)
├── plc_proxy_ws/              # 🚗🖥️ 共用 - PLC 代理服務 (實際啟動)
├── rcs_ws/                    # 🖥️ AGVC 管理系統 - 車隊控制系統 (未在啟動腳本中)
├── sensorpart_ws/             # 🚗 AGV 車載系統 - 感測器整合系統 (未在啟動腳本中)
├── wcs_ws/                    # 🖥️ AGVC 管理系統 - 倉庫控制系統 (未在啟動腳本中)
├── web_api_ws/                # 🖥️ AGVC 管理系統 - Web API 和前端介面 (未在啟動腳本中)
├── path_algorithm/            # 🚗🖥️ 共用 - 路徑規劃演算法 (實際啟動)
├── config/                    # 🚗🖥️ 共用 - 統一配置文件
├── routerconfig.json5         # 🚗🖥️ 共用 - Zenoh Router 配置
├── startup.agv.bash           # 🚗 AGV 車載系統啟動腳本
└── startup.agvc.bash          # 🖥️ AGVC 管理系統啟動腳本
```

**圖例說明**：
- 🚗 AGV 車載系統專用
- 🖥️ AGVC 管理系統專用
- 🚗🖥️ 兩個環境共用
- **(實際啟動)** 表示在容器啟動腳本中被載入的工作空間
- **(未在啟動腳本中)** 表示目前未被自動啟動的工作空間

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
4. 載入工作空間環境 (keyence_plc_ws, plc_proxy_ws, agv_cmd_service_ws, joystick_ws, agv_ws, path_algorithm)
5. 啟動 AGV Launch (`ros2 launch loader_agv launch.py`)

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
4. 載入工作空間環境 (keyence_plc_ws, plc_proxy_ws, agv_cmd_service_ws, joystick_ws, agv_ws, path_algorithm)
5. 啟動 PostgreSQL、pgAdmin4、Nginx 服務

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
| agv_ws | ✅ 自動啟動 | AGV 狀態控制 | 85% | 三層狀態架構，loader_agv launch |
| agv_cmd_service_ws | ✅ 自動載入 | 命令服務 | 80% | 介面定義完成，服務節點可用 |
| joystick_ws | ✅ 自動載入 | 搖桿控制 | 75% | 手動控制功能完成 |
| keyence_plc_ws | ✅ 自動載入 | Keyence PLC | 90% | PLC 通訊穩定，被 ecs_ws 依賴 |
| plc_proxy_ws | ✅ 自動載入 | PLC 代理 | 95% | 多協定支援，被 ecs_ws 依賴 |
| sensorpart_ws | ❌ 未啟動 | 感測器整合 | 70% | 基本感測器已整合 |

### 🖥️ AGVC 管理系統專案狀態

| 專案 | 啟動狀態 | 核心功能 | 完成度 | 說明 |
|------|----------|----------|--------|------|
| db_proxy_ws | ❌ 未啟動 | 資料庫服務 | 90% | 完整 CRUD、連線池管理 |
| ecs_ws | ⚠️ 依賴已載入 | 設備控制 | 75% | 強依賴 plc_proxy_ws & keyence_plc_ws |
| rcs_ws | ❌ 未啟動 | 車隊控制 | 80% | 任務派發邏輯已實現 |
| wcs_ws | ❌ 未啟動 | 倉庫控制 | 70% | KUKA 整合進行中 |
| web_api_ws | ❌ 未啟動 | Web 介面 | 85% | API、前端功能完整 |
| kuka_fleet_ws | ❌ 未啟動 | KUKA 車隊 | 60% | API 整合開發中 |

### 🚗🖥️ 共用專案狀態

| 專案 | 啟動狀態 | 核心功能 | 完成度 | 說明 |
|------|----------|----------|--------|------|
| launch_ws | ❌ 未啟動 | 啟動管理 | 80% | 基本啟動流程完成 |
| path_algorithm | ✅ 自動載入 | 路徑規劃 | 85% | A* 演算法已實現 |
| config | ✅ 配置可用 | 配置管理 | 90% | 統一配置架構完成 |

### Zenoh 相關配置

| 組件 | 狀態 | 說明 |
|------|------|------|
| rmw_zenoh_cpp | ✅ 已安裝 | 預建置在 Docker image 中 |
| Zenoh Router | ✅ 自動啟動 | 容器啟動時自動執行 |
| routerconfig.json5 | ✅ 已配置 | 支援多節點連線 |

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
    A --> D[載入工作空間]
    D --> E[keyence_plc_ws]
    D --> F[plc_proxy_ws]
    D --> G[agv_cmd_service_ws]
    D --> H[joystick_ws]
    D --> I[agv_ws]
    D --> J[path_algorithm]
    A --> K[Docker Services]
    K --> L[PostgreSQL]
    K --> M[pgAdmin4]
    K --> N[Nginx]
```

**AGVC 管理系統實際啟動順序**：
1. **基礎服務**: SSH (port 2200), Zenoh Router (port 7447)
2. **工作空間載入**: keyence_plc_ws → plc_proxy_ws → agv_cmd_service_ws → joystick_ws → agv_ws → path_algorithm
3. **Docker 服務**: PostgreSQL, pgAdmin4, Nginx
4. **注意**: 目前 AGVC 專用工作空間 (db_proxy_ws, ecs_ws, rcs_ws, wcs_ws, web_api_ws 等) 未在啟動腳本中自動啟動
5. **ECS 依賴**: ecs_ws 雖未自動啟動，但其依賴的 plc_proxy_ws 和 keyence_plc_ws 已載入，可手動啟動 ECS 服務

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
- 遵循 `.augment-guidelines` 中的開發原則
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

### Zenoh 故障排除指令

#### 常見問題診斷
```bash
# 1. 檢查 Zenoh Router 是否正確啟動
docker compose -f docker-compose.yml exec rosagv bash -c "
if [ -f /tmp/zenoh_router.pid ]; then
    pid=\$(cat /tmp/zenoh_router.pid)
    if ps -p \$pid > /dev/null; then
        echo '✅ Zenoh Router 運行中 (PID: '\$pid')'
    else
        echo '❌ Zenoh Router PID 檔案存在但程序未運行'
    fi
else
    echo '❌ Zenoh Router PID 檔案不存在'
fi"

# 2. 檢查 Zenoh Router 日誌中的錯誤
docker compose -f docker-compose.yml exec rosagv bash -c "
if [ -f /tmp/zenoh_router.log ]; then
    echo '=== Zenoh Router 日誌 (最後 20 行) ==='
    tail -20 /tmp/zenoh_router.log
else
    echo '❌ Zenoh Router 日誌檔案不存在'
fi"

# 3. 檢查 ROS 2 環境是否正確載入
docker compose -f docker-compose.yml exec rosagv bash -c "
echo '=== ROS 2 環境檢查 ==='
echo 'ROS_DISTRO: '\$ROS_DISTRO
echo 'RMW_IMPLEMENTATION: '\$RMW_IMPLEMENTATION
echo 'ZENOH_ROUTER_CONFIG_URI: '\$ZENOH_ROUTER_CONFIG_URI
which ros2 && echo '✅ ros2 指令可用' || echo '❌ ros2 指令不可用'
"

# 4. 手動重啟 Zenoh Router（如果需要）
docker compose -f docker-compose.yml exec rosagv bash -c "
echo '重啟 Zenoh Router...'
pkill -f rmw_zenohd 2>/dev/null
sleep 2
source /opt/ros/\$ROS_DISTRO/setup.bash
source /opt/ws_rmw_zenoh/install/setup.bash
nohup ros2 run rmw_zenoh_cpp rmw_zenohd > /tmp/zenoh_router.log 2>&1 &
echo \$! > /tmp/zenoh_router.pid
echo '✅ Zenoh Router 已重啟'
"
```

### 健康檢查指令
```bash
# 檢查容器啟動狀態
docker compose -f docker-compose.yml ps
docker compose -f docker-compose.agvc.yml ps

# 檢查 Zenoh Router 是否運行
docker compose -f docker-compose.yml exec rosagv bash -c "pgrep -f rmw_zenohd"
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "pgrep -f rmw_zenohd"

# 檢查 SSH 服務
docker compose -f docker-compose.yml exec rosagv bash -c "pgrep -f sshd"
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "pgrep -f sshd"

# 檢查 ROS 2 環境變數
docker compose -f docker-compose.yml exec rosagv bash -c "printenv | grep RMW_IMPLEMENTATION"
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "printenv | grep RMW_IMPLEMENTATION"

# 檢查工作空間載入狀態
docker compose -f docker-compose.yml exec rosagv bash -c "source /opt/ros/\$ROS_DISTRO/setup.bash && source /opt/ws_rmw_zenoh/install/setup.bash && ros2 pkg list | grep -E '(agv|plc|keyence|ecs)'"
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "source /opt/ros/\$ROS_DISTRO/setup.bash && source /opt/ws_rmw_zenoh/install/setup.bash && ros2 pkg list | grep -E '(agv|plc|keyence|ecs)'"

# 檢查 ECS 依賴是否可用（AGVC 管理系統）
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "source /opt/pyvenv_env/bin/activate && python3 -c 'from plc_proxy.plc_client import PlcClient; from keyence_plc.keyence_plc_memory import PlcMemory; print(\"✅ ECS 依賴可用\")'"
```

### 貢獻流程
1. Fork 專案並建立功能分支
2. 確認目標運行環境（AGV 車載 vs AGVC 管理）
3. 了解當前啟動腳本中實際載入的工作空間
4. 遵循 Zenoh RMW 和容器化開發規範
5. 新增對應的測試案例
6. 更新相關文檔
7. 提交 Pull Request

## 🔧 維護與支援

### 監控指標
- **系統可用性**: 99.5% 目標
- **任務完成率**: 95% 目標
- **平均回應時間**: < 100ms
- **錯誤率**: < 1%

### 備份策略
- **資料庫**: 每日自動備份
- **配置文件**: Git 版本控制
- **日誌文件**: 7 天保留期

### 技術支援
- **維護者**: root <yaze.lin.j303@gmail.com>
- **文檔**: `/docs` 目錄
- **問題回報**: GitHub Issues
- **緊急聯絡**: 24/7 支援熱線

## 📈 未來規劃

### 🚗 AGV 車載系統發展規劃

#### 短期目標 (1-3 個月)
- [ ] 完善三層狀態架構的錯誤處理機制
- [ ] 啟用 sensorpart_ws 感測器整合
- [ ] 優化 Zenoh 通訊效能
- [ ] 建立完整的車載系統測試覆蓋

#### 中期目標 (3-6 個月)
- [ ] 支援更多 AGV 車型變體 (cargo_mover_agv, unloader_agv)
- [ ] 實現預測性維護功能
- [ ] 新增機器學習路徑最佳化
- [ ] 建立車載診斷系統

### 🖥️ AGVC 管理系統發展規劃

#### 短期目標 (1-3 個月)
- [ ] 啟用 AGVC 專用工作空間 (db_proxy_ws, ecs_ws, rcs_ws, wcs_ws, web_api_ws)
- [ ] 完成 startup.agvc.bash 中的管理系統啟動邏輯
- [ ] 將 ecs_ws 加入 AGVC 管理系統的自動啟動流程
- [ ] 建立完整的 Zenoh 多節點通訊架構
- [ ] 實現 CI/CD 自動化流程

#### 中期目標 (3-6 個月)
- [ ] 完成 KUKA Fleet 完整整合
- [ ] 建立完整的監控儀表板
- [ ] 實現智能任務調度演算法
- [ ] 新增系統效能分析工具

### 🚗🖥️ 整體系統長期目標 (6-12 個月)
- [ ] 雲端部署支援
- [ ] 多工廠管理功能
- [ ] AI 驅動的任務調度
- [ ] 國際化和多語言支援
- [ ] 跨平台車隊統一管理

### Zenoh 架構優化目標
- [ ] 優化 routerconfig.json5 多節點連線配置
- [ ] 建立 Zenoh 效能監控機制
- [ ] 實現 Zenoh 故障轉移機制
- [ ] 新增 Zenoh 安全認證機制

---

**版本**: 1.0.0
**最後更新**: 2025-07-17
**授權**: Apache 2.0
