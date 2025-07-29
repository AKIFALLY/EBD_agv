# agvcui - 車隊管理界面系統

## 📚 Context Loading
../CLAUDE.md  # 引用上層 web_api_ws 工作空間文档

## 📋 套件概述
agvcui 是 Web API 工作空間中的 **車隊管理界面系統**，提供完整的 MVC 架構和企業級管理功能。基於 FastAPI + Socket.IO + Jinja2，提供即時 Web 界面，支援 AGV 車隊監控、任務管理、地圖視覺化等核心管理功能。

**🎯 定位**: Port 8001 的管理員界面，具備完整的資料庫操作和即時通訊能力

## 核心模組

### 後端服務
- **AgvcUIServer** (`agvc_ui_server.py`): 主要FastAPI伺服器
- **AgvcUISocket** (`agvc_ui_socket.py`): WebSocket即時通訊
- **Auth** (`auth.py`): 用戶認證和授權
- **Database** (`database/`): 資料庫操作層
- **Routers** (`routers/`): API路由器

### 前端架構
- **靜態資源** (`static/`): CSS、JavaScript、圖像資源
- **模板** (`templates/`): Jinja2 HTML模板
- **JavaScript模組**: 頁面邏輯和即時更新

## 關鍵檔案

### 後端核心
- `/agvcui/agvc_ui_server.py` - 主要FastAPI應用伺服器
- `/agvcui/agvc_ui_socket.py` - WebSocket連接管理和即時通訊
- `/agvcui/auth.py` - 用戶認證和會話管理
- `/agvcui/db.py` - 資料庫連接配置
- `/agvcui/middleware.py` - 中間件配置

### 資料庫操作層
```
database/
├── connection.py        # 資料庫連接管理
├── agv_ops.py          # AGV操作
├── task_ops.py         # 任務操作
├── rack_ops.py         # 架台操作
├── carrier_ops.py      # 載具操作
├── equipment_ops.py    # 設備操作
├── user_ops.py         # 用戶操作
├── audit_log_ops.py    # 審計日誌操作
└── utils.py            # 工具函數
```

### API路由器
```
routers/
├── auth.py             # 認證API
├── agvs.py             # AGV管理API
├── tasks.py            # 任務管理API
├── racks.py            # 架台管理API
├── carriers.py         # 載具管理API
├── devices.py          # 設備管理API
├── users.py            # 用戶管理API
├── map.py              # 地圖API
├── signals.py          # 信號API
├── works.py            # 工作管理API
├── audit_logs.py       # 審計日誌API
├── rosout_logs.py      # ROS日誌API
└── runtime_logs.py     # 運行時日誌API
```

### 前端資源
```
static/
├── css/                # 樣式表
│   ├── bulma_1_0_4.min.css
│   ├── agvcui-bulma-extend.css
│   ├── dashboardPage.css
│   ├── mapPage.css
│   └── ...
├── js/                 # JavaScript模組
│   ├── mapPage.js      # 地圖頁面邏輯
│   ├── dashboardPage.js # 儀表板邏輯
│   ├── agvsPage.js     # AGV頁面邏輯
│   ├── tasksPage.js    # 任務頁面邏輯
│   ├── socket.js       # WebSocket客戶端
│   └── lib/            # 第三方庫
└── objects/            # 地圖物件
    ├── BaseObject.js
    ├── RackInfoObject.js
    ├── TransferBoxObject.js
    └── ...
```

## 🚀 AGVCUI 專用啟動

### 車隊管理界面啟動
```bash
# 【推薦方式】透過上層工作空間工具
# 參考: ../CLAUDE.md 開發環境設定

# 【直接啟動】AGVCUI 管理界面
cd /app/web_api_ws/src/agvcui
python3 agvcui/agvc_ui_server.py

# 開發模式 (自動重載)
uvicorn agvcui.agvc_ui_server:app --host 0.0.0.0 --port 8001 --reload

# 檢查管理界面
curl http://localhost:8001/
```

### AGVCUI 專項測試
```bash
# 車隊管理功能測試
python3 -m pytest tests/ -v
python3 tests/test_task_status_api.py    # 任務狀態 API 測試

# 前端界面測試 (瀏覽器開啟)
firefox tests/test_cache_verification.html        # 快取驗證測試
firefox tests/test_rack_marker_interaction.html   # Rack 標記互動測試
```

## 📊 AGVCUI 特定配置

### 管理界面服務器配置
```python
# agvc_ui_server.py 車隊管理界面配置
HOST = "0.0.0.0"      # AGVCUI 監聽地址
PORT = 8001           # 管理界面端口 (企業級管理功能)
DATABASE_URL = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
```

### 認證和會話配置
```python
# auth.py JWT 認證配置
SECRET_KEY = "your_secret_key"           # 管理員認證密鑰
ALGORITHM = "HS256"                      # JWT 算法
ACCESS_TOKEN_EXPIRE_MINUTES = 30         # 管理員會話超時
```

### Socket.IO 即時通訊配置
```python
# agvc_ui_socket.py 即時更新配置
SOCKET_PATH = "/socket.io"               # WebSocket 路徑
CORS_ALLOWED_ORIGINS = ["*"]             # 跨域設定 (開發用)
```

## 🔗 AGVCUI 特有整合點

### 車隊管理界面特有整合
- **db_proxy_ws**: 透過完整 database/ 層進行 PostgreSQL CRUD 操作
- **agv_interfaces**: 接收和處理 AGV 狀態更新訊息
- **kuka_fleet_ws**: 整合 KUKA Fleet 任務管理和監控
- **ecs_ws**: 設備控制狀態監控和門控管理
- **Socket.IO 即時通訊**: 支援多用戶同時監控和操作

### WebSocket事件
```javascript
// 客戶端訂閱事件
socket.on('agv_status_update', handleAGVStatusUpdate);
socket.on('task_status_update', handleTaskStatusUpdate);
socket.on('rack_status_update', handleRackStatusUpdate);
socket.on('map_update', handleMapUpdate);

// 客戶端發送事件
socket.emit('subscribe_room', {room: 'agv_monitoring'});
socket.emit('request_agv_status', {agv_id: 'agv01'});
```

### API端點
```bash
# 認證API
POST /auth/login                   # 用戶登入
POST /auth/logout                  # 用戶登出
GET  /auth/me                      # 獲取當前用戶

# AGV管理API
GET  /api/agvs                     # AGV列表
GET  /api/agvs/{agv_id}           # 特定AGV詳情
PUT  /api/agvs/{agv_id}           # 更新AGV配置

# 任務管理API
GET  /api/tasks                    # 任務列表
POST /api/tasks                    # 創建任務
PUT  /api/tasks/{task_id}         # 更新任務
DELETE /api/tasks/{task_id}       # 刪除任務

# 地圖API
GET  /api/map/data                 # 地圖數據
GET  /api/map/objects             # 地圖物件
POST /api/map/update_object       # 更新地圖物件
```

## 測試方法

### 後端API測試
```bash
# 執行後端測試
python3 -m pytest tests/ -v

# 測試認證功能
curl -X POST http://localhost:8001/auth/login \
  -H "Content-Type: application/json" \
  -d '{"username": "admin", "password": "password"}'

# 測試API端點
curl -X GET http://localhost:8001/api/agvs \
  -H "Authorization: Bearer <token>"
```

### 前端功能測試
```bash
# 開啟瀏覽器測試頁面
firefox tests/test_cache_verification.html
firefox tests/test_rack_marker_interaction.html
firefox tests/test_task_status_sync.html

# 地圖功能測試
firefox tests/test_rack_object_debug.html
firefox tests/test_rack_toggle_diagnosis.html
```

### WebSocket測試
```javascript
// 在瀏覽器控制台測試
const socket = io('ws://localhost:8001');
socket.on('connect', () => console.log('Connected'));
socket.emit('subscribe_room', {room: 'agv_monitoring'});
```

### 整合測試
```bash
# 啟動完整系統測試
# 1. 啟動AGVCUI
python3 agvcui/agvc_ui_server.py &

# 2. 開啟瀏覽器
firefox http://localhost:8001

# 3. 測試各功能頁面
# - 儀表板 (Dashboard)
# - AGV監控頁面
# - 任務管理頁面
# - 地圖頁面
# - 設備管理頁面
```

## 🚨 AGVCUI 專項故障排除  

**⚠️ 通用故障排除請參考**: ../CLAUDE.md 故障排除章節

### 車隊管理界面特有問題

#### WebSocket 即時通訊問題
```bash
# 檢查 Socket.IO 連接狀態
curl -i -N -H "Connection: Upgrade" \
  -H "Upgrade: websocket" \
  -H "Sec-WebSocket-Version: 13" \
  -H "Sec-WebSocket-Key: test" \
  http://localhost:8001/socket.io/

# 瀏覽器控制台檢查 WebSocket 事件
```

#### 地圖視覺化問題
```bash
# 檢查地圖數據 API
curl http://localhost:8001/api/map/data
curl http://localhost:8001/api/map/objects

# 檢查 Leaflet.js 地圖初始化
# 瀏覽器開發工具 → Console → 檢查 JavaScript 錯誤
```

#### 認證和會話問題
```bash
# 檢查 JWT 認證設定
curl -X POST http://localhost:8001/auth/login \
  -H "Content-Type: application/json" \
  -d '{"username": "admin", "password": "password"}'

# 檢查會話狀態
curl -X GET http://localhost:8001/auth/me \
  -H "Authorization: Bearer <token>"
```

### AGVCUI 功能特色
- **🎛️ 儀表板**: 系統總覽、AGV 狀態統計、任務進度監控
- **🗺️ 地圖視圖**: 即時地圖、AGV 軌跡、設備狀態視覺化
- **📋 任務管理**: 任務創建、編輯、調度、執行監控
- **🚗 AGV 監控**: 即時位置、狀態、電池電量、工作狀態
- **📊 設備管理**: 架台、載具、設備狀態監控和控制
- **👥 用戶管理**: 多用戶權限、認證、審計日誌追蹤