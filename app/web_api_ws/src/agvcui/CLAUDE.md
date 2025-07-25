# agvcui - AGVC車隊管理界面

## 專案概述
agvcui是RosAGV系統的車隊管理界面，提供完整的AGV車隊監控、任務管理、設備狀態監控等功能。基於FastAPI後端和現代化前端技術，提供即時的Web界面，支援多用戶管理、即時地圖顯示、任務調度等企業級功能。

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

## 開發指令

### 基本操作
```bash
# 進入AGVC容器
docker compose -f docker-compose.agvc.yml exec agvc_server bash
source /app/setup.bash && all_source

# 構建agvcui
build_ws web_api_ws
# 或單獨構建
colcon build --packages-select agvcui

# 啟動AGVCUI伺服器
cd /app/web_api_ws/src/agvcui
python3 agvcui/agvc_ui_server.py
```

### 開發模式啟動
```bash
# 使用uvicorn開發模式 (port 8001)
uvicorn agvcui.agvc_ui_server:app --host 0.0.0.0 --port 8001 --reload

# 檢查AGVCUI界面
curl http://localhost:8001/
```

### 測試指令
```bash
# 執行測試套件
cd /app/web_api_ws/src/agvcui
python3 -m pytest tests/ -v

# 執行特定測試
python3 tests/test_task_status_api.py

# 前端功能測試
open tests/test_cache_verification.html
open tests/test_rack_marker_interaction.html
```

## 配置設定

### 伺服器配置
```python
# agvc_ui_server.py 配置
HOST = "0.0.0.0"
PORT = 8001
DATABASE_URL = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
```

### 認證配置
```python
# auth.py 配置
SECRET_KEY = "your_secret_key"
ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 30
```

### WebSocket配置
```python
# agvc_ui_socket.py 配置
SOCKET_PATH = "/socket.io"
CORS_ALLOWED_ORIGINS = ["*"]
```

### 環境變數
```bash
export AGVCUI_HOST="0.0.0.0"           # AGVCUI伺服器主機
export AGVCUI_PORT="8001"              # AGVCUI伺服器端口  
export DATABASE_URL="postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
export SECRET_KEY="your_secret_key"    # JWT密鑰
```

## 整合點

### 與其他專案整合
- **web_api_ws**: 共享web_api_ws工作空間
- **db_proxy_ws**: 透過database層存取PostgreSQL
- **agv_base**: 接收AGV狀態更新
- **agv_interfaces**: 處理AGV狀態和狀態變更訊息
- **kuka_fleet_ws**: 整合KUKA Fleet任務管理
- **ecs_ws**: 設備控制和狀態監控

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

## 故障排除

### 常見問題

#### 伺服器啟動失敗
```bash
# 檢查端口佔用
netstat -tulpn | grep :8001
sudo lsof -i :8001

# 檢查資料庫連接
python3 -c "
from agvcui.database.connection import get_db_connection
conn = get_db_connection()
print('Database connection OK')"

# 檢查依賴模組
python3 -c "import fastapi, socketio, jinja2; print('Dependencies OK')"
```

#### WebSocket連接問題
```bash
# 檢查WebSocket連接
curl -i -N -H "Connection: Upgrade" \
  -H "Upgrade: websocket" \
  -H "Sec-WebSocket-Version: 13" \
  -H "Sec-WebSocket-Key: test" \
  http://localhost:8001/socket.io/

# 檢查瀏覽器控制台錯誤
# Network標籤查看WebSocket連接狀態
```

#### 前端資源載入失敗
```bash
# 檢查靜態資源路徑
ls -la /app/web_api_ws/src/agvcui/agvcui/static/

# 檢查CSS和JavaScript文件
curl http://localhost:8001/static/css/bulma_1_0_4.min.css
curl http://localhost:8001/static/js/mapPage.js

# 檢查瀏覽器Network標籤
# 確認所有資源載入成功
```

#### 地圖顯示問題
```bash
# 檢查地圖數據API
curl http://localhost:8001/api/map/data

# 檢查地圖物件
curl http://localhost:8001/api/map/objects

# 開啟瀏覽器開發工具檢查JavaScript錯誤
# 查看Leaflet地圖初始化狀態
```

### 除錯技巧
- 使用瀏覽器開發工具檢查Network和Console錯誤
- 監控FastAPI日誌觀察API請求處理
- 使用WebSocket測試工具驗證即時通訊
- 檢查PostgreSQL日誌確認資料庫查詢
- 使用Postman測試API端點功能

### 效能監控
- 頁面載入時間分析
- WebSocket訊息傳輸延遲
- 資料庫查詢效能
- 前端JavaScript執行效能
- 記憶體使用情況
- 並發用戶支援能力

### 使用者介面功能
- **儀表板**: 系統總覽、AGV狀態統計、任務進度
- **AGV監控**: 即時AGV位置、狀態、電池電量
- **任務管理**: 任務創建、編輯、調度、監控
- **地圖視圖**: 即時地圖、AGV軌跡、設備狀態
- **設備管理**: 架台、載具、設備狀態監控
- **用戶管理**: 用戶權限、認證、審計日誌
- **系統日誌**: ROS日誌、運行時日誌、錯誤追蹤