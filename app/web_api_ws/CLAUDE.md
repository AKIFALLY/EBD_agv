# web_api_ws CLAUDE.md

## 📚 Context Loading
../../CLAUDE.md  # 引用根目錄通用層知識（系統架構、核心原則、通用工具）

## 🔧 工作空間層文檔（第二層）
# Web 開發領域知識
@docs-ai/knowledge/system/agvui-monitoring-system.md    # AGVCUI 監控系統
@docs-ai/knowledge/system/hmi-system-design.md          # HMI 系統設計
@docs-ai/operations/development/web/web-development.md  # Web 開發指南
@docs-ai/operations/development/web/web-api-launch-management.md # API Launch 管理

# 外部系統整合
@docs-ai/knowledge/protocols/kuka-fleet-api.md          # KUKA Fleet API
@docs-ai/knowledge/protocols/kuka-fleet-callback.md     # KUKA Fleet 回調
@docs-ai/knowledge/protocols/kuka-agv-rack-rotation.md  # KUKA AGV Rack 旋轉

# 工作空間通用文檔
@docs-ai/context/workspaces/agvc-workspaces.md         # AGVC 工作空間架構
@docs-ai/operations/development/testing/testing-standards.md # 測試標準

# 設備授權和權限管理
@docs-ai/knowledge/agv-domain/license-table-design.md      # 授權表設計

## 📋 模組概述

**Web API 服務系統** - 完整的 Web 服務群組，提供 RESTful API、Socket.IO 實時通訊、用戶界面與外部系統整合，是 AGVC 管理系統的核心 Web 服務層。

### Web API 服務群組工作空間特有功能
- **🌐 Web 服務群組**: 整合四個主要 Web 服務提供完整功能
- **🔗 系統整合中心**: 連接 PLC、KUKA Fleet、資料庫等外部系統
- **🖥️ 用戶界面提供**: AGVCUI (管理員) + OPUI (操作員) + AGVUI (車載)
- **🎭 API Gateway**: 統一的 API 入口和外部系統整合

## 📂 專案結構 (實際驗證)

```
src/
├── web_api/              # 🌐 API Gateway 服務 (Port 8000)
│   ├── routers/          # API 路由模組
│   │   ├── kuka.py      # KUKA Fleet 整合 API
│   │   ├── plc.py       # PLC 控制 API
│   │   ├── door.py      # 門控系統 API
│   │   ├── traffic.py   # 交通管制 API
│   │   ├── map_importer.py # 地圖匯入 API
│   │   └── nodes.py     # ROS 2 節點管理 API (統一控制)
│   └── api_server.py     # FastAPI 主伺服器
├── agvcui/              # 🖥️ 車隊管理系統 (Port 8001)
│   ├── database/        # 資料库操作層 (完整 CRUD)
│   ├── routers/         # 完整 API 路由系統
│   │   ├── tafl_editor.py # ⚠️ 已棄用 - TAFL 編輯器 API (路由: /tafl/editor)
│   │   ├── tafl_editor_direct.py # ⚠️ 已棄用 - TAFL 直接編輯 API
│   │   └── ...          # 其他路由器 (agvs, tasks, racks 等)
│   ├── static/          # 前端靜態資源
│   ├── templates/       # Jinja2 模板
│   ├── agvc_ui_server.py # FastAPI 主伺服器
│   └── agvc_ui_socket.py # Socket.IO 實時通訊
├── opui/                # 👨‍💼 操作員界面 (Port 8002)
│   ├── core/            # 核心伺服器模組
│   │   ├── op_ui_server.py # FastAPI 主伺服器
│   │   └── op_ui_socket.py # Socket.IO 實時通訊
│   ├── monitoring/      # 監控服務層
│   │   └── task_monitor.py # 任務狀態監控服務
│   ├── services/        # 業務邏輯服務層
│   │   └── opui_task_service.py # OPUI 任務業務邏輯
│   ├── frontend/        # 前端界面資源
│   ├── api/             # API 客戶端
│   └── tests/           # 標準 pytest 測試
└── agvui/               # 🚗 AGV 車載監控界面 (Port 8003)
    ├── agv_ui_server.py # FastAPI 主伺服器
    ├── agv_ui_socket.py # Socket.IO 實時通訊
    ├── agv_ui_ros.py    # ROS 2 節點整合
    ├── static/          # 監控界面資源
    └── templates/       # HTML 模板
```

### 架構特性
- **多服務協同**: 四個獨立服務協同提供完整 Web 功能
- **統一技術棧**: 基於 FastAPI + Socket.IO + PostgreSQL
- **分層設計**: API Gateway + 業務界面 + 專業界面的分層架構
- **ROS 2 整合**: 完整的 ROS 2 套件結構和 Launch 支援

## 🚀 服務架構詳解

### Web 服務端口配置 (基於實際檔案驗證)
- **Port 8000**: `web_api` - API Gateway (外部系統整合中心)
  - PLC 控制整合、KUKA Fleet 整合、門控系統、交通管制
- **Port 8001**: `agvcui` - 車隊管理系統 (完整 MVC 架構)
  - 完整資料庫操作、用戶管理、任務調度、地圖監控
- **Port 8002**: `opui` - 操作員界面 (任務管理專用)
  - 操作員友好界面、任務分派、狀態監控
- **Port 8003**: `agvui` - AGV 車載監控
  - AGV 本地監控界面、狀態顯示、實時更新

### Web 服務技術棧
- **後端框架**: FastAPI (高效能 Web 框架)
- **即時通訊**: Socket.IO (雙向即時通訊)
- **資料庫**: PostgreSQL + SQLAlchemy/SQLModel
- **前端技術**: Bulma CSS + Vanilla JavaScript + Leaflet
- **ROS 2 整合**: 完整的 ROS 2 套件支援

## 🚀 Web API 專用開發

**⚠️ 通用開發環境請參考**: ../../CLAUDE.md 開發指導章節

### Web 服務管理快速指令
```bash
# 【推薦方式】透過根目錄統一工具
# 參考: ../../CLAUDE.md 開發指導

# 【AGVC 系統管理】
agvc_start                   # 啟動 AGVC 系統
agvc_health                  # 健康檢查

# 【Web 服務狀態檢查】
curl http://localhost:8000/health    # API Gateway
curl http://localhost:8001/          # AGVCUI
curl http://localhost:8002/          # OPUI
curl http://localhost:8003/          # AGVUI
```

### 容器內操作 (ROS 2 和 Web 開發)

#### 環境設定 (AGVC容器內)
```bash
source /app/setup.bash
agvc_source  # 載入AGVC工作空間 (或使用 all_source 自動檢測)
cd /app/web_api_ws
```

#### 服務啟動 (容器內執行)
```bash
# 【方法1: 透過宿主機工具】(推薦)
# 在宿主機執行：
source scripts/docker-tools/docker-tools.sh
agvc_enter  # 自動進入 AGVC 容器並載入環境

# 或使用快速執行
quick_agvc "python3 src/web_api/web_api/api_server.py"      # API Gateway
quick_agvc "python3 src/agvcui/agvcui/agvc_ui_server.py"   # AGVCUI 系統
quick_agvc "python3 src/opui/opui/opui_server.py"          # OPUI 界面

# 【方法2: 手動進入容器】
# 前提：在 ~/RosAGV 目錄執行
cd ~/RosAGV
# 必須先進入AGVC容器並載入環境
docker compose -f docker-compose.agvc.yml exec agvc_server bash
source /app/setup.bash && agvc_source

# API Gateway服務 (實際路徑驗證)
python3 src/web_api/web_api/api_server.py

# AGVCUI車隊管理系統
python3 src/agvcui/agvcui/agvc_ui_server.py

# OPUI操作員界面  
python3 src/opui/opui/opui_server.py

# 或使用ROS2啟動 (如果已建置)
ros2 run web_api api_server
ros2 run agvcui agvc_ui_server
ros2 run opui opui_server
ros2 run agvui agv_ui_server

# 【方法3: 使用Web API Launch管理】(推薦)
# 在AGVC容器內
manage_web_api_launch start     # 啟動所有Web服務
manage_web_api_launch stop      # 停止所有Web服務
manage_web_api_launch restart   # 重啟所有Web服務
manage_web_api_launch status    # 檢查服務狀態
```

#### 構建與測試

##### 宿主機建置和測試 (推薦)
```bash
# 使用開發工具進行建置
source scripts/dev-tools/dev-tools.sh
dev_build --workspace web_api_ws    # 建置 Web API 工作空間
dev_test --workspace web_api_ws     # 測試 Web API 工作空間

# 或直接使用工具腳本
scripts/dev-tools/build-helper.sh fast --workspace web_api_ws
scripts/dev-tools/test-runner.sh unit --workspace web_api_ws

# Web 服務狀態檢查
curl http://localhost:8000/health    # web_api健康檢查
curl http://localhost:8001/         # agvcui界面
curl http://localhost:8002/         # opui界面
curl http://localhost:8003/         # agvui界面
```

##### 容器內建置 (setup.bash驗證可用)
```bash
# 【方法1: 透過宿主機工具】(推薦)
quick_agvc "build_ws web_api_ws"     # 在 AGVC 容器內建置

# 【方法2: 手動進入容器】
agvc_enter  # 進入容器
build_ws web_api_ws                  # 使用setup.bash中的函數
```

## API開發指南 (基於實際架構)

### Web API服務 (API Gateway模式)
**實際路徑**: `src/web_api/routers/` (驗證存在)
- `door.py` - 門控系統API
- `plc.py` - PLC設備API
- `traffic.py` - 交通管理API
- `kuka.py` - KUKA Fleet整合API
- `nodes.py` - ROS 2節點管理API (統一節點控制)

### AGVCUI系統 (完整MVC架構)
**實際路徑**: `src/agvcui/` (驗證存在完整結構)
- `database/` - 資料庫操作層
- `routers/` - 完整的API路由系統
  - `tafl_editor.py` - TAFL編輯器API (路由: /tafl/editor)
  - `tafl_editor_direct.py` - TAFL直接編輯API
  - 其他路由器 (agvs, tasks, racks, carriers 等)
- `static/` & `templates/` - 前端資源

### 新增API端點流程
1. **Web API**: 在 `src/web_api/routers/` 添加新路由檔案
2. **AGVCUI**: 在 `src/agvcui/routers/` 實現完整CRUD操作
3. **註冊**: 在對應的 `*_server.py` 中註冊路由

### 實際API結構範例
```python
# web_api/routers/nodes.py (ROS 2節點管理)
@router.get("/api/nodes")
async def list_nodes():
    # 列出所有ROS 2節點

@router.post("/api/nodes/{node_name}/restart")
async def restart_node(node_name: str):
    # 重啟指定節點

# web_api/routers/door.py (外部系統整合)
@router.get("/door/{door_id}/status")
async def get_door_status(door_id: str):
    # PLC整合邏輯

# ⚠️ 已棄用 (2025-11-18) - 使用 kuka_wcs_ws 替代
# agvcui/routers/tafl_editor.py (TAFL編輯器)
# @router.get("/tafl/editor")
# async def tafl_editor_page():
#     # TAFL視覺化編輯器頁面
```

## Socket.IO事件管理

### 事件命名空間
- `/agv` - AGV狀態更新
- `/system` - 系統狀態事件
- `/task` - 任務管理事件

### 新增Socket.IO事件
1. 在對應socket類別添加事件處理器
2. 更新前端JavaScript事件監聽
3. 測試事件流與資料格式

```python
@socketio.on('connect', namespace='/agv')
def handle_agv_connect():
    emit('status', {'message': 'AGV Socket Connected'})
```

## 前端開發指南

### OPUI (操作員界面)
- **架構**: 分層架構 (monitoring/services/core)
- **監控層**: task_monitor.py 提供任務狀態監控
- **服務層**: opui_task_service.py 處理業務邏輯
- **實時更新**: Socket.IO 即時通訊

### AGVCUI (車隊管理系統)
- **功能**: 車隊管理與監控
- ~~**TAFL編輯器**~~: ⚠️ 已棄用 - 視覺化流程編輯器 (/tafl/editor)
- **節點管理**: 統一ROS 2節點控制
- **即時更新**: Socket.IO實時資料

### AGVUI (AGV車載監控)
- **功能**: AGV本地狀態監控
- **ROS 2整合**: agv_ui_ros.py 節點背景服務
- **輕量級**: 為車載資源限制設計
- **即時通訊**: Socket.IO 狀態更新

## 配置管理

### 服務配置 (docker-compose.agvc.yml驗證)
- **端口映射**: 8000-8002:8000-8002 (實際配置)
- **資料庫**: PostgreSQL透過docker compose管理 (`docker compose -f docker-compose.agvc.yml [up -d|stop] postgres`)
- **環境變數**: CONTAINER_TYPE="agvc" (容器環境檢測)

### 實際配置檔案
- **Web API配置**: `/app/config/web_api_config.yaml` (實際存在)
- **硬體映射**: `/app/config/hardware_mapping.yaml`
- **Zenoh配置**: `/app/routerconfig.json5`

## 🗄️ 資料庫管理 (pgAdmin)

### pgAdmin 服務配置
- **容器名稱**: `pgadmin` (192.168.100.101)
- **端口映射**: 5050:80 (宿主機:容器)
- **Nginx 反向代理**: `http://agvc.ui/pgadmin/`
- **登入資訊**:
  - Email: `yazelin@ching-tech.com`
  - Password: `password`

### 訪問方式

#### 方式 1: 透過 AGVCUI 界面 (推薦)
1. 訪問 AGVCUI: `http://agvc.ui/` 或 `http://localhost:8001/`
2. 使用管理員帳號登入系統
3. 點擊右上角用戶選單 → 「資料庫管理」
4. pgAdmin 自動在新分頁開啟 (`http://agvc.ui/pgadmin/`)
5. 使用 pgAdmin 登入資訊進入資料庫管理界面

#### 方式 2: 直接訪問
```bash
# 透過 Nginx 反向代理 (推薦)
http://agvc.ui/pgadmin/

# 直接端口訪問 (開發測試用)
http://localhost:5050/
```

### PostgreSQL 連接配置
在 pgAdmin 中新增伺服器連接：
- **主機名稱**: `192.168.100.254` 或 `postgres`
- **端口**: `5432`
- **維護資料庫**: `agvc`
- **用戶名稱**: `agvc`
- **密碼**: `password`

### 常用資料庫操作
```bash
# 檢查資料庫連接
PGPASSWORD=password psql -h 192.168.100.254 -U agvc -d agvc -c "\dt"

# 查看資料表統計
PGPASSWORD=password psql -h 192.168.100.254 -U agvc -d agvc -c "
SELECT
  schemaname,
  tablename,
  pg_size_pretty(pg_total_relation_size(schemaname||'.'||tablename)) AS size
FROM pg_tables
WHERE schemaname = 'public'
ORDER BY pg_total_relation_size(schemaname||'.'||tablename) DESC;
"
```

### 架構優勢
✅ **統一入口**: 透過 Nginx 反向代理整合到 AGVCUI 界面
✅ **權限控制**: 只有管理員用戶可見資料庫管理選單
✅ **新分頁開啟**: 不影響 AGVCUI 主界面操作
✅ **標準化訪問**: 與其他 Web 服務保持一致的訪問模式

## 🔍 Web 服務專項測試

**⚠️ 通用測試指導請參考**: ../../CLAUDE.md 測試章節

### Web 服務快速測試
```bash
# Web 服務健康檢查
curl http://localhost:8000/health    # API Gateway
curl http://localhost:8001/          # AGVCUI 界面
curl http://localhost:8002/          # OPUI 界面
curl http://localhost:8003/          # AGVUI 界面
curl http://localhost:8000/docs      # API 文檔

# Socket.IO 連接測試
python -c "import socketio; sio = socketio.Client(); sio.connect('http://localhost:8000')"
```

### Web 特定調試工具
- **FastAPI 自動文檔**: http://localhost:8000/docs
- **Socket.IO 事件監控**: 瀏覽器開發工具 Network 分頁
- **Vue 組件調試**: Vue Devtools (OPUI)

## 部署注意事項

### 容器內服務
- 所有服務運行在AGVC容器內
- 通過nginx反向代理對外服務
- 支援SSL終端與負載均衡

### 安全考量
- CORS政策正確配置
- API認證與授權機制
- 敏感資料環境變數管理

## 🚨 Web 服務專項故障排除

**⚠️ 通用故障排除請參考**: ../../CLAUDE.md 故障排除章節

### Web 服務特有問題診斷
```bash
# Web 服務健康檢查
agvc_health                          # AGVC 系統健康檢查
curl http://localhost:8000/health    # API Gateway 健康檢查

# 端口和網路檢查
netstat -tlnp | grep -E "(8000|8001|8002|8003)"  # 檢查 Web 服務端口
scripts/network-tools/port-check.sh --port 8000-8003

# 服務重啟和日誌
agvc_restart                         # 重啟 AGVC 系統
agvc_logs                           # 查看系統日誌
```

### Web 服務關鍵依賴
- **AGVC 容器**: 所有 Web 服務必須在 AGVC 容器內運行
- **資料庫連接**: PostgreSQL 服務正常運行
- **ROS 2 環境**: 正確載入 AGVC 工作空間
- **端口可用性**: 8000-8003 端口未被佔用

## 💡 Web 開發要點

- **Web 服務群組**: 四個 Web 服務協同提供完整功能
- **AGVC 容器運行**: 所有 Web 服務必須在 AGVC 容器內執行
- **多端口服務**: 8000 (API), 8001 (AGVCUI), 8002 (OPUI), 8003 (AGVUI)
- **統一管理**: manage_web_api_launch 統一啟動/停止所有服務
- **即時通訊**: Socket.IO 提供雙向即時資料交換
- **系統整合**: 與 PLC、KUKA Fleet、資料庫等外部系統整合

## 📊 主要API端點整理

### Web API (Port 8000)
```bash
# 節點管理API
GET  /api/nodes                     # 列出所有ROS 2節點
GET  /api/nodes/{node_name}         # 節點詳情
POST /api/nodes/{node_name}/restart # 重啟節點
POST /api/nodes/{node_name}/stop    # 停止節點

# PLC控制API
GET  /plc/status                    # PLC狀態
POST /plc/read_data                 # 讀取PLC數據
POST /plc/write_data                # 寫入PLC數據

# KUKA Fleet整合
POST /interfaces/api/amr/missionStateCallback  # 任務狀態回調
```

### AGVCUI (Port 8001)
```bash
# ⚠️ 已棄用 (2025-11-18) - TAFL Editor 已被 KUKA WCS 取代
# GET  /tafl/editor                   # TAFL視覺化編輯器
# GET  /tafl/verbs                    # TAFL動詞列表
# POST /tafl/validate                 # 驗證TAFL流程
# POST /tafl/save                     # 保存TAFL流程

# AGV管理
GET  /api/agvs                      # AGV列表
GET  /api/agvs/{agv_id}            # AGV詳情
```

## 🔗 交叉引用
- AGVC 工作空間: @docs-ai/context/workspaces/agvc-workspaces.md
- KUKA Fleet API: @docs-ai/knowledge/protocols/kuka-fleet-api.md
- KUKA Fleet 回調: @docs-ai/knowledge/protocols/kuka-fleet-callback.md
- ROS 2 介面: @docs-ai/knowledge/protocols/ros2-interfaces.md
- 資料庫操作: @docs-ai/operations/development/database-operations.md
- Web API 啟動管理: @docs-ai/operations/development/web/web-api-launch-management.md
- Web 開發: @docs-ai/operations/development/web/web-development.md
- 測試標準: @docs-ai/operations/development/testing/testing-standards.md
- 系統診斷: @docs-ai/operations/guides/system-diagnostics.md
- 監控系統: @docs-ai/knowledge/system/agvui-monitoring-system.md
- HMI 設計: @docs-ai/knowledge/system/hmi-system-design.md