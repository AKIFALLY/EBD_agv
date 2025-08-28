# web_api_ws CLAUDE.md

## 📚 Context Loading
../../CLAUDE.md  # 引用根目錄系統文档
@docs-ai/knowledge/protocols/kuka-fleet-api.md
@docs-ai/knowledge/protocols/kuka-fleet-callback.md
@docs-ai/knowledge/business/eyewear-production-process.md
@docs-ai/knowledge/protocols/kuka-agv-rack-rotation.md
@docs-ai/operations/maintenance/troubleshooting.md

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
│   │   └── map_importer.py # 地圖匯入 API
│   ├── api_server.py     # FastAPI 主伺服器
│   └── tests/           # API 測試套件
├── agvcui/              # 🖥️ 車隊管理系統 (Port 8001)
│   ├── database/        # 資料库操作層 (完整 CRUD)
│   ├── routers/         # 完整 API 路由系統
│   ├── static/          # 前端靜態資源
│   ├── templates/       # Jinja2 模板
│   ├── agvc_ui_server.py # FastAPI 主伺服器
│   └── agvc_ui_socket.py # Socket.IO 實時通訊
├── opui/                # 👨‍💼 操作員界面 (Port 8002)
│   ├── core/            # 核心伺服器模組
│   ├── frontend/        # 前端界面資源
│   ├── api/             # API 客戶端
│   └── services/        # 業務邏輯服務
└── agvui/               # 🚗 AGV 車載監控界面
    ├── agv_ui_server.py # AGV 監控伺服器
    └── static/          # 監控界面資源
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
- **Port 8003**: `agvui` - AGV 車載監控 (可選)
  - AGV 本地監控界面、狀態顯示

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

### AGVCUI系統 (完整MVC架構)
**實際路徑**: `src/agvcui/` (驗證存在完整結構)
- `database/` - 資料庫操作層
- `routers/` - 完整的API路由系統
- `static/` & `templates/` - 前端資源

### 新增API端點流程
1. **Web API**: 在 `src/web_api/routers/` 添加新路由檔案
2. **AGVCUI**: 在 `src/agvcui/routers/` 實現完整CRUD操作
3. **註冊**: 在對應的 `*_server.py` 中註冊路由

### 實際API結構範例
```python
# web_api/routers/door.py (外部系統整合)
@router.get("/door/{door_id}/status")
async def get_door_status(door_id: str):
    # PLC整合邏輯
    
# agvcui/routers/*.py (完整業務邏輯)  
@router.get("/agv/{agv_id}/status")
async def get_agv_status(agv_id: str):
    # 完整的資料庫查詢和業務邏輯
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

### OPUI (Vue.js 3)
- **架構**: 完整重構的現代化界面
- **組件**: 模組化Vue組件設計
- **狀態管理**: Vuex/Pinia整合
- **開發**: 支援熱重載開發模式

### AGVCUI
- **功能**: 車隊管理與監控
- **整合**: 與核心API緊密整合
- **即時更新**: Socket.IO實時資料

## 配置管理

### 服務配置 (docker-compose.agvc.yml驗證)
- **端口映射**: 8000-8002:8000-8002 (實際配置)
- **資料庫**: PostgreSQL透過setup.bash的start_db/stop_db管理
- **環境變數**: CONTAINER_TYPE="agvc" (容器環境檢測)

### 實際配置檔案
- **Web API配置**: `/app/config/web_api_config.yaml` (實際存在)
- **硬體映射**: `/app/config/hardware_mapping.yaml`
- **Zenoh配置**: `/app/routerconfig.json5`

## 🔍 Web 服務專項測試

**⚠️ 通用測試指導請參考**: ../../CLAUDE.md 測試章節

### Web 服務快速測試
```bash
# Web 服務健康檢查
curl http://localhost:8000/health    # API Gateway
curl http://localhost:8001/          # AGVCUI 界面  
curl http://localhost:8002/          # OPUI 界面
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
netstat -tlnp | grep -E "(8000|8001|8002)"  # 檢查 Web 服務端口
scripts/network-tools/port-check.sh --port 8000-8002

# 服務重啟和日誌
agvc_restart                         # 重啟 AGVC 系統
agvc_logs                           # 查看系統日誌
```

### Web 服務關鍵依賴
- **AGVC 容器**: 所有 Web 服務必須在 AGVC 容器內運行
- **資料庫連接**: PostgreSQL 服務正常運行
- **ROS 2 環境**: 正確載入 AGVC 工作空間
- **端口可用性**: 8000-8002 端口未被佔用

## 💡 Web 開發要點

- **Web 服務群組**: 四個 Web 服務協同提供完整功能
- **AGVC 容器運行**: 所有 Web 服務必須在 AGVC 容器內執行
- **多端口服務**: 8000 (API), 8001 (AGVCUI), 8002 (OPUI), 8003 (AGVUI)
- **即時通訊**: Socket.IO 提供雙向即時資料交換
- **系統整合**: 與 PLC、KUKA Fleet、資料庫等外部系統整合

## 🔗 交叉引用
- 系統概覽: @docs-ai/context/system/rosagv-overview.md
- 雙環境架構: @docs-ai/context/system/dual-environment.md
- 技術棧詳解: @docs-ai/context/system/technology-stack.md
- KUKA Fleet API: @docs-ai/knowledge/protocols/kuka-fleet-api.md
- KUKA Fleet 回調: @docs-ai/knowledge/protocols/kuka-fleet-callback.md
- ROS 2 介面: @docs-ai/knowledge/protocols/ros2-interfaces.md
- Web 開發指導: @docs-ai/operations/development/web-development.md
- 資料庫操作: @docs-ai/operations/development/database-operations.md
- Docker 開發: @docs-ai/operations/development/docker-development.md
- 系統診斷: @docs-ai/operations/maintenance/system-diagnostics.md
- 故障排除: @docs-ai/operations/maintenance/troubleshooting.md
- 統一工具: @docs-ai/operations/tools/unified-tools.md