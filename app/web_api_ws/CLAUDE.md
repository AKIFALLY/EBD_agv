# web_api_ws CLAUDE.md

## 📚 Context Loading
@docs-ai/context/system/rosagv-overview.md
@docs-ai/context/system/dual-environment.md
@docs-ai/context/system/technology-stack.md
@docs-ai/knowledge/protocols/kuka-fleet-api.md
@docs-ai/knowledge/protocols/kuka-fleet-callback.md
@docs-ai/knowledge/protocols/ros2-interfaces.md
@docs-ai/operations/development/web-development.md
@docs-ai/operations/development/database-operations.md

## 📋 模組概述

**Web API 服務系統** - 完整的 Web 服務群組，提供 RESTful API、Socket.IO 實時通訊、用戶界面與外部系統整合，是 AGVC 管理系統的核心 Web 服務層。

### 核心定位
- **Web 服務群組**: 整合三個主要 Web 服務提供完整功能
- **系統整合中心**: 連接 PLC、KUKA Fleet、資料庫等外部系統
- **用戶界面提供**: 管理員界面 (AGVCUI) 和操作員界面 (OPUI)
- **API Gateway**: 統一的 API 入口和外部系統整合

詳細系統架構說明請參考: @docs-ai/context/system/rosagv-overview.md

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

### 技術棧整合
詳細技術棧說明請參考: @docs-ai/context/system/technology-stack.md

- **後端框架**: FastAPI (高效能 Web 框架)
- **實時通訊**: Socket.IO (雙向即時通訊)
- **資料庫**: PostgreSQL + SQLAlchemy/SQLModel
- **前端技術**: Bulma CSS + Vanilla JavaScript + Leaflet
- **ROS 2 整合**: 完整的 ROS 2 套件支援

## 🔧 開發環境設定

詳細開發環境設定請參考: @docs-ai/operations/development/docker-development.md

### 容器環境要求
**⚠️ 重要**: 所有 Web API 服務必須在 AGVC Docker 容器內執行，詳細說明請參考: @docs-ai/context/system/dual-environment.md

### 宿主機統一工具使用
詳細工具系統請參考: @docs-ai/operations/tools/unified-tools.md

### AGVC 容器管理
詳細容器管理指導請參考: @docs-ai/operations/development/docker-development.md

**常用 AGVC 管理指令**：
```bash
# 載入工具並進入開發環境
source scripts/docker-tools/docker-tools.sh
agvc_enter                   # 進入 AGVC 容器 (自動載入環境)

# 系統管理
agvc_start                   # 啟動 AGVC 系統
agvc_health                  # 健康檢查
agvc_logs                    # 查看日誌

# 快速執行容器內指令
quick_agvc "build_ws web_api_ws"      # 建置工作空間
quick_agvc "curl http://localhost:8000/health"  # API 健康檢查
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

## 🔍 測試與調試

### 系統診斷和測試
詳細測試和診斷指導請參考: @docs-ai/operations/maintenance/system-diagnostics.md

### 快速 Web 服務測試
```bash
# Web 服務健康檢查
curl http://localhost:8000/health    # API Gateway
curl http://localhost:8001/          # AGVCUI 界面  
curl http://localhost:8002/          # OPUI 界面
curl http://localhost:8000/docs      # API 文檔界面

# 系統狀態檢查
source scripts/docker-tools/docker-tools.sh
agvc_health                          # AGVC 系統健康檢查
```

### 容器內調試工具

#### API 測試 (容器內)
```bash
# 【方法1: 透過宿主機工具】(推薦)
quick_agvc "curl http://localhost:8000/health"       # API 健康檢查
quick_agvc "python -c \"import socketio; sio = socketio.Client(); sio.connect('http://localhost:8000')\""

# 【方法2: 手動進入容器】
agvc_enter  # 進入容器
curl http://localhost:8000/docs      # FastAPI自動文檔
# Socket.IO測試
python -c "import socketio; sio = socketio.Client(); sio.connect('http://localhost:8000')"
```

### 前端調試
- **瀏覽器開發工具**: 檢查Network與Console
- **Vue Devtools**: 組件狀態調試  
- **Socket.IO調試**: 查看事件流

## 部署注意事項

### 容器內服務
- 所有服務運行在AGVC容器內
- 通過nginx反向代理對外服務
- 支援SSL終端與負載均衡

### 安全考量
- CORS政策正確配置
- API認證與授權機制
- 敏感資料環境變數管理

## 🛠️ 故障排除

詳細故障排除指導請參考: @docs-ai/operations/maintenance/troubleshooting.md

### 系統診斷和故障排除
詳細診斷和故障排除指導請參考: 
- @docs-ai/operations/maintenance/system-diagnostics.md - 系統診斷工具和流程
- @docs-ai/operations/maintenance/troubleshooting.md - 故障排除指導

### 快速診斷工具
```bash
# 統一診斷入口 (宿主機執行)
r agvc-check                         # AGVC 系統健康檢查
r containers-status                  # 容器狀態檢查
r network-check                      # 網路連接檢查
r quick-diag                         # 快速綜合診斷

# AGVC 專用工具
source scripts/docker-tools/docker-tools.sh
agvc_health                          # AGVC 系統健康檢查
agvc_status                          # 容器狀態檢查
```

### 常見 Web 服務問題
```bash
# 端口檢查
scripts/network-tools/port-check.sh --port 8000-8002

# 服務重啟
agvc_restart                         # 重啟整個 AGVC 系統

# 日誌檢查  
agvc_logs                           # 查看系統日誌
scripts/log-tools/log-analyzer.sh agvc --stats  # 日誌分析
```

## 💡 重要提醒

### 開發環境使用原則
- **🖥️ 宿主機**: 使用 `scripts/` 工具進行容器管理、服務診斷、API 測試
- **🐳 容器內**: 執行 Web 服務、ROS 2 相關指令、資料庫操作
- **📡 推薦方式**: 使用 `agvc_enter` 進入容器，使用 `quick_agvc` 執行容器內指令

### Web 開發最佳實踐
詳細 Web 開發指導請參考: @docs-ai/operations/development/web-development.md

- **API 變更**: 需同步更新前端界面和文檔
- **Socket.IO 事件**: 確保向後兼容性和完整測試
- **資料庫變更**: 透過 db_proxy 進行，詳見 @docs-ai/operations/development/database-operations.md
- **服務部署**: 所有 Web 服務必須在 AGVC 容器內運行

### 故障排除最佳實踐
詳細故障排除流程請參考: @docs-ai/operations/maintenance/troubleshooting.md

1. **優先使用宿主機工具**: 快速診斷和服務檢查
2. **多層次檢查**: 容器→服務→端口→網路→資料庫
3. **日誌分析為主**: 使用統一診斷工具進行智能分析
4. **服務隔離**: 分別檢查 API Gateway、AGVCUI、OPUI 服務

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