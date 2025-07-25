# opui - 操作員介面系統

## 專案概述
OPUI (Operator User Interface) 是基於 ROS2、FastAPI + Socket.IO 的即時 Web 應用程式，專為 AGV 調度系統的操作員介面設計。提供操作員友善的 Web 介面，用於管理 AGV 任務調度、監控系統狀態，並與後端 RCS 系統協作完成自動化倉儲作業。

## 核心模組

### 前端架構 (模組化設計)
- **index.js**: 共用功能（全域初始化、Store 狀態管理、Socket 連線處理）
- **pages/homePage.js**: Home 頁面專用功能（產品選擇、數量設定、房號選擇）
- **pages/settingPage.js**: Settings 頁面專用功能（產品管理、系統設定）
- **pages/rackPage.js**: Rack 頁面專用功能（料架管理）

### 後端架構 (分離式設計)
- **op_ui_server.py**: FastAPI 伺服器和 HTTP 路由處理
- **op_ui_socket.py**: Socket.IO 事件處理和即時通訊功能
- **task_service.py**: 任務相關業務邏輯
- **device_auth.py**: 設備授權驗證

### 狀態管理系統
- **userStore**: 用戶狀態管理（clientId、machineId、連線狀態）
- **operationStore**: 操作狀態管理（左右側操作狀態）
- **dataStore**: 資料狀態管理（products、machines、rooms、parking）
- **tasksStore**: 任務狀態管理（活躍任務追蹤）
- **uiStore**: UI 狀態管理（載入狀態、通知訊息）

## 關鍵檔案

### 前端核心檔案
- `/app/static/js/index.js` - 全域初始化和共用功能
- `/app/static/js/pages/homePage.js` - Home 頁面功能
- `/app/static/js/pages/settingPage.js` - Settings 頁面功能
- `/app/static/js/pages/rackPage.js` - Rack 頁面功能
- `/app/static/js/miniStore.js` - 輕量級狀態管理

### 後端核心檔案
- `/opui/op_ui_server.py` - FastAPI 主伺服器
- `/opui/op_ui_socket.py` - Socket.IO 事件處理
- `/opui/task_service.py` - 任務業務邏輯
- `/opui/device_auth.py` - 設備認證

### 模板檔案
- `/app/templates/index.html` - 主頁面模板
- `/app/templates/base.html` - 基礎模板
- `/app/static/css/style.css` - 樣式定義

## 開發指令

### 基本構建
```bash
# 進入 AGVC 容器
docker compose -f docker-compose.agvc.yml exec agvc_server bash
source /app/setup.bash && all_source

# 構建 web_api_ws
build_ws web_api_ws

# 單獨構建 opui
cd /app/web_api_ws
colcon build --packages-select opui
```

### 服務啟動
```bash
# 啟動 OPUI 伺服器 (Port 8002)
cd /app/web_api_ws/src/opui
python3 opui/op_ui_server.py

# 或使用 ROS 2 啟動
ros2 run opui op_ui_server

# 開發模式啟動 (自動重載)
python3 opui/op_ui_server.py --reload
```

### 測試指令
```bash
# 執行所有測試
cd /app/web_api_ws/src/opui
python3 -m pytest tests/

# 前端測試
python3 tests/test_frontend.py

# Socket.IO 測試
python3 tests/test_socket_api.py

# 資料庫整合測試
python3 tests/test_database_integration.py
```

## 配置設定

### 伺服器配置
```python
# op_ui_server.py
SERVER_CONFIG = {
    "host": "0.0.0.0",
    "port": 8002,
    "debug": False,
    "reload": False
}

SOCKET_CONFIG = {
    "cors_allowed_origins": "*",
    "async_mode": "asgi"
}
```

### 資料庫配置
```python
# 使用 db_proxy 連線池
from db_proxy.db_proxy.connection_pool_manager import get_connection_pool

DATABASE_CONFIG = {
    "pool_size": 10,
    "max_overflow": 20,
    "pool_timeout": 30
}
```

## 整合點

### 與其他專案整合
- **db_proxy_ws**: 使用 connection_pool_manager 進行資料庫操作
- **rcs_ws**: 透過 Socket.IO 接收任務狀態更新
- **web_api_ws**: 共享 FastAPI 技術棧和配置
- **agvcui**: 共用 Web 服務架構模式

### Socket.IO 事件
```javascript
// 客戶端事件
socket.emit('client_update', clientData);
socket.emit('call_car', taskData);
socket.emit('dispatch_car', dispatchData);

// 伺服器事件
socket.on('client_update', handleClientUpdate);
socket.on('task_status_update', handleTaskUpdate);
socket.on('system_notification', handleNotification);
```

### API 端點
```bash
# HTTP API
GET  /api/products          # 獲取產品列表
POST /api/products          # 新增產品
GET  /api/machines          # 獲取機台列表
GET  /api/rooms             # 獲取房間列表
POST /api/tasks             # 創建任務
```

## 測試方法

### 前端測試
```bash
# 頁面功能測試
python3 tests/test_home_page.py
python3 tests/test_settings_page.py
python3 tests/test_rack_page.py

# 狀態管理測試
python3 tests/test_store_management.py

# Socket.IO 連線測試
python3 tests/test_socket_connection.py
```

### 後端測試
```bash
# API 端點測試
python3 tests/test_api_endpoints.py

# 任務服務測試
python3 tests/test_task_service.py

# 資料庫整合測試
python3 tests/test_database_operations.py
```

## 故障排除

### 常見問題

#### Socket.IO 連線失敗
```bash
# 檢查伺服器狀態
curl http://localhost:8002/health

# 檢查 Socket.IO 端點
curl http://localhost:8002/socket.io/

# 查看伺服器日誌
tail -f /app/logs/opui.log
```

#### 資料庫連線問題
```bash
# 檢查資料庫連線池
python3 -c "from db_proxy.db_proxy.connection_pool_manager import test_connection; test_connection()"

# 檢查資料庫狀態
ros2 service call /db_proxy/test_connection
```

#### 前端載入異常
```bash
# 檢查靜態檔案
ls -la /app/web_api_ws/src/opui/app/static/

# 檢查模板檔案
ls -la /app/web_api_ws/src/opui/app/templates/

# 清除瀏覽器快取
# 開發者工具 -> Network -> Disable cache
```

### 除錯技巧
- 使用瀏覽器開發者工具監控 Socket.IO 事件
- 檢查 FastAPI 自動生成的 API 文檔 `/docs`
- 監控資料庫連線池狀態
- 使用 `console.log()` 追蹤前端狀態變化

### 效能監控
- Socket.IO 事件回應時間應保持在 100ms 內
- 資料庫查詢效能監控
- 前端頁面載入時間分析
- 記憶體使用情況檢查
