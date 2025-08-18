# RosAGV 統一節點管理系統

## 系統概覽

RosAGV 統一節點管理系統提供了三層架構的節點管理解決方案，支援本地 AGVC 節點和遠端 AGV 節點的統一管理。

## 架構設計

```
┌─────────────────────────────────────────────┐
│          宿主機工具層 (Host Tools)           │
│         rosagv-tools.sh / r command         │
└─────────────────────────────────────────────┘
                      ↓
┌─────────────────────────────────────────────┐
│      容器管理函數層 (Container Functions)     │
│           /app/setup.bash functions         │
└─────────────────────────────────────────────┘
                      ↓
┌─────────────────────────────────────────────┐
│         ROS 2 節點層 (ROS 2 Nodes)          │
│       ros2 launch / ros2 run / ros2 node    │
└─────────────────────────────────────────────┘
```

## 核心組件

### 1. 節點註冊表 (`/app/config/node_registry.yaml`)

- **本地節點定義**: web_api_launch, flow_wcs, ecs_core, rcs_core, db_proxy, kuka_fleet
- **遠端 AGV 配置**: cargo02, loader02, unloader02
- **節點群組管理**: all, web_services, core_services, integration
- **系統服務**: zenoh_router, ssh_service

### 2. 容器管理函數 (`/app/setup.bash`)

新增的管理函數:
- `manage_all_nodes` - 管理所有節點
- `manage_ecs_core` - 管理 ECS 設備控制系統
- `manage_db_proxy` - 管理資料庫代理服務
- `manage_rcs` - 管理機器人控制系統
- `manage_kuka_fleet` - 管理 KUKA Fleet 整合
- `manage_agv_launch` - 管理遠端 AGV 節點

### 3. 宿主機工具 (`rosagv-tools.sh`)

新增的命令:
- `r node-status` - 查看所有節點狀態
- `r node-start [name]` - 啟動特定節點
- `r node-stop [name]` - 停止特定節點
- `r node-restart [name]` - 重啟節點
- `r agv-nodes [name] [action]` - 管理遠端 AGV

### 4. Web API 整合 (`/app/web_api_ws/src/web_api/web_api/routers/nodes.py`)

RESTful API 端點:
- `GET /api/nodes/` - 獲取所有節點列表
- `GET /api/nodes/status` - 獲取所有節點狀態
- `GET /api/nodes/node/{name}` - 獲取特定節點狀態
- `POST /api/nodes/node/{name}/start` - 啟動節點
- `POST /api/nodes/node/{name}/stop` - 停止節點
- `POST /api/nodes/node/{name}/restart` - 重啟節點
- `GET /api/nodes/group/{name}` - 獲取節點群組狀態
- `POST /api/nodes/group/{name}/start` - 啟動節點群組
- `POST /api/nodes/group/{name}/stop` - 停止節點群組
- `GET /api/nodes/agv/{name}` - 獲取遠端 AGV 狀態
- `POST /api/nodes/agv/{name}/{action}` - 控制遠端 AGV
- `POST /api/nodes/reload-registry` - 重新載入註冊表
- `GET /api/nodes/health` - 健康檢查

## 使用指南

### 快速開始

```bash
# 查看所有節點狀態
r node-status

# 啟動 Web API 服務群組
r node-start web_api_launch

# 停止特定節點
r node-stop flow_wcs

# 重啟節點
r node-restart ecs_core

# 管理遠端 AGV
r agv-nodes cargo02 status
r agv-nodes loader02 start
r agv-nodes unloader02 stop
```

### 容器內操作

```bash
# 進入 AGVC 容器
docker compose -f docker-compose.agvc.yml exec agvc_server bash
source /app/setup.bash
agvc_source

# 使用管理函數
manage_all_nodes status      # 查看所有節點狀態
manage_web_api_launch start  # 啟動 Web API
manage_flow_wcs restart      # 重啟 Flow WCS
manage_agv_launch cargo02 status  # 查看遠端 AGV
```

### Web API 使用

```bash
# 獲取所有節點狀態
curl http://localhost:8000/api/nodes/status

# 啟動特定節點
curl -X POST http://localhost:8000/api/nodes/node/web_api_launch/start

# 控制遠端 AGV
curl -X POST http://localhost:8000/api/nodes/agv/cargo02/start

# 管理節點群組
curl http://localhost:8000/api/nodes/group/core_services
curl -X POST http://localhost:8000/api/nodes/group/web_services/start
```

## 節點詳細資訊

### Web 服務群組
- **web_api_launch**: Web API 服務群組 (3個節點)
  - agvc_ui_server (Port 8001)
  - op_ui_server (Port 8002)
  - web_api_server (Port 8000)

### 核心服務
- **flow_wcs**: Flow WCS 倉儲控制系統
- **ecs_core**: ECS 設備控制核心
- **rcs_core**: RCS 機器人控制系統
- **db_proxy**: 資料庫代理服務

### 整合服務
- **kuka_fleet**: KUKA Fleet 整合服務
- **plc_proxy_agvc**: PLC 通訊代理服務

### 遠端 AGV
- **cargo02**: IP 192.168.10.11, Port 2222
- **loader02**: IP 192.168.10.12, Port 2222
- **unloader02**: IP 192.168.10.13, Port 2222

## 系統特性

1. **三層架構設計**: 宿主機工具 → 容器函數 → ROS 節點
2. **統一管理介面**: 一致的命令格式和 API 設計
3. **遠端 AGV 支援**: 透過 SSH 管理遠端 AGV 節點
4. **群組管理**: 支援節點群組的批量操作
5. **Web API 整合**: RESTful API 支援程式化管理
6. **健康檢查**: 自動檢測節點運行狀態
7. **配置驅動**: 基於 YAML 配置的靈活管理

## 測試驗證

使用提供的測試腳本驗證系統:

```bash
# 執行完整測試
/home/ct/RosAGV/scripts/test-node-management.sh
```

測試涵蓋:
- 宿主機工具層測試
- 容器管理函數層測試
- 節點註冊表配置驗證
- Web API 整合測試
- 實際節點狀態檢查

## 故障排除

### 節點無法啟動
1. 檢查工作空間是否已載入: `agvc_source`
2. 確認節點在註冊表中: `cat /app/config/node_registry.yaml`
3. 查看詳細錯誤: `manage_[node_name] status`

### 遠端 AGV 連接失敗
1. 檢查網路連接: `ping [agv_ip]`
2. 驗證 SSH 端口: `telnet [agv_ip] 2222`
3. 確認認證資訊正確

### Web API 無回應
1. 檢查 API 服務狀態: `manage_web_api_launch status`
2. 驗證端口可用: `netstat -tlnp | grep 8000`
3. 查看 API 日誌: `docker logs agvc_server`

## 5. AGVCUI 界面整合 (`/app/web_api_ws/src/agvcui/agvcui/routers/nodes.py`)

AGVCUI 節點管理界面功能：
- **Web 界面**: 訪問 http://localhost:8001/nodes 查看節點管理界面
- **卡片式顯示**: 節點狀態以卡片形式展示
- **實時更新**: 每 30 秒自動刷新節點狀態
- **批量操作**: 支援啟動/停止所有節點
- **API 代理**: 所有操作通過 Web API (Port 8000) 執行

### AGVCUI 界面特性
- **狀態統計**: 顯示活躍、停止、錯誤節點數量
- **個別控制**: 每個節點卡片有啟動/停止/重啟按鈕
- **遠端 AGV**: 獨立區域顯示遠端 AGV 節點
- **響應式設計**: 使用 Bulma CSS 框架
- **導航整合**: 在主導航欄添加 "Nodes" 連結

### 使用 AGVCUI 界面

```bash
# 啟動 AGVCUI 服務
docker compose -f docker-compose.agvc.yml exec agvc_server bash -c "source /app/setup.bash && agvc_source && python3 /app/web_api_ws/src/agvcui/agvcui/agvc_ui_server.py"

# 訪問節點管理界面
firefox http://localhost:8001/nodes
```

## 未來擴展

- [x] AGVCUI 界面整合 (已完成)
- [ ] 節點依賴管理
- [ ] 自動故障恢復
- [ ] 節點性能監控
- [ ] 配置版本管理
- [ ] 批量部署支援

## 相關文件

- `/app/config/node_registry.yaml` - 節點註冊表配置
- `/app/setup.bash` - 容器管理函數定義
- `/home/ct/RosAGV/rosagv-tools.sh` - 宿主機工具腳本
- `/app/web_api_ws/src/web_api/web_api/routers/nodes.py` - Web API 實現
- `/app/web_api_ws/src/agvcui/agvcui/routers/nodes.py` - AGVCUI 節點管理路由
- `/app/web_api_ws/src/agvcui/agvcui/templates/nodes.html` - AGVCUI 節點管理模板
- `/home/ct/RosAGV/scripts/test-node-management.sh` - 完整測試腳本
- `/home/ct/RosAGV/scripts/test-agvcui-node-integration.sh` - AGVCUI 整合測試

---

*文件版本: 1.1*
*更新日期: 2025-08-15*
*作者: RosAGV Team*