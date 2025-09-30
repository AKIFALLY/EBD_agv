# opui - 操作員介面系統

## 📚 Context Loading
../../../../CLAUDE.md  # 引用根目錄系統文档
../../CLAUDE.md  # 引用上層 web_api_ws 工作空間文档

# 操作員界面專業知識（專業層）
@docs-ai/knowledge/business/eyewear-production-process.md  # 生產流程

## 專案概述
OPUI (Operator User Interface) 是基於 ROS2、FastAPI + Socket.IO 的即時 Web 應用程式，專為 AGV 調度系統的操作員介面設計。提供操作員友善的 Web 介面，用於管理 AGV 任務調度、監控系統狀態，並與後端 RCS 系統協作完成自動化倉儲作業。

## 📂 專案結構（實際驗證）
```
opui/
├── opui/                          # 主要 Python 套件目錄
│   ├── core/                     # 核心伺服器模組
│   │   ├── op_ui_server.py      # FastAPI 主伺服器
│   │   ├── op_ui_socket.py      # Socket.IO 事件處理
│   │   ├── task_service.py      # 任務業務邏輯
│   │   ├── device_auth.py       # 設備授權驗證
│   │   ├── socket_handler.py    # Socket 事件處理器
│   │   └── server.py            # 伺服器基礎配置
│   ├── frontend/                 # 前端資源目錄
│   │   ├── static/              # 靜態資源
│   │   │   ├── index.js         # 主入口 JavaScript
│   │   │   ├── css/             # 樣式檔案
│   │   │   │   └── style.css   # 主要樣式
│   │   │   └── js/              # JavaScript 模組
│   │   │       ├── pages/       # 頁面邏輯
│   │   │       │   ├── homePage.js
│   │   │       │   ├── settingPage.js
│   │   │       │   └── rackPage.js
│   │   │       ├── managers/    # 管理器模組
│   │   │       │   ├── StateManager.js
│   │   │       │   ├── UIManager.js
│   │   │       │   ├── EventManager.js
│   │   │       │   └── PageManager.js
│   │   │       ├── lib/         # 函式庫
│   │   │       │   └── miniStore.js
│   │   │       └── store.js     # Store 狀態管理
│   │   └── templates/           # HTML 模板
│   │       ├── base.html        # 基礎模板
│   │       ├── home.html        # 主頁面模板
│   │       ├── hmi.html         # HMI 頁面
│   │       ├── setting.html     # 設定頁面
│   │       ├── rack.html        # 料架頁面
│   │       ├── navbar.html      # 導航欄
│   │       ├── error.html       # 錯誤頁面
│   │       └── unauthorized.html # 未授權頁面
│   ├── api/                      # API 客戶端模組
│   │   └── node.py              # 節點 API 客戶端
│   ├── monitoring/               # 監控服務層
│   │   └── task_monitor.py      # 任務狀態監控
│   ├── services/                 # 業務邏輯服務
│   │   └── opui_task_service.py # OPUI 任務服務
│   ├── database/                 # 資料庫操作層
│   │   └── operations.py        # 資料庫 CRUD 操作
│   └── constants/                # 常數定義
│       └── __init__.py          # 常數模組
├── tests/                        # 標準測試目錄 (pytest)
│   ├── conftest.py              # Pytest 配置和 fixtures
│   ├── __init__.py              # 測試模組初始化
│   ├── test_basic.py            # 基礎測試
│   ├── test_db.py               # 資料庫測試
│   ├── test_op_ui_server.py    # 伺服器測試
│   ├── test_op_ui_socket.py    # Socket 測試
│   ├── test_routers.py         # 路由測試
│   ├── test_integration.py     # 整合測試
│   ├── test_performance.py     # 效能測試
│   ├── test_simple.py          # 簡單測試
│   ├── test_room_button_logic.py  # 房間按鈕邏輯測試
│   ├── test_task_creation.py      # 任務創建測試
│   └── test_task_completion_flow.py # 任務完成流程測試
├── config/                       # 配置檔案目錄
│   └── settings.py              # 系統設定
├── app/                          # 應用相關目錄（僅包含符號連結）
├── resource/                     # ROS2 資源目錄
├── package.xml                   # ROS2 套件配置
├── setup.py                      # Python 套件配置
├── setup.cfg                     # Python 套件設定
├── README.md                     # 專案說明文檔
└── CLAUDE.md                     # AI 開發助手指導文檔
```

## 核心模組

### 前端架構 (模組化設計)
- **frontend/static/index.js**: 共用功能（全域初始化、Store 狀態管理、Socket 連線處理）
- **frontend/static/js/pages/homePage.js**: Home 頁面專用功能（產品選擇、數量設定、房號選擇）
- **frontend/static/js/pages/settingPage.js**: Settings 頁面專用功能（產品管理、系統設定）
- **frontend/static/js/pages/rackPage.js**: Rack 頁面專用功能（料架管理）

### 後端架構 (核心模組設計)
- **core/op_ui_server.py**: FastAPI 主伺服器和 HTTP 路由處理
- **core/op_ui_socket.py**: Socket.IO 事件處理和即時通訊功能
- **core/task_service.py**: 任務相關業務邏輯
- **core/device_auth.py**: 設備授權驗證
- **core/socket_handler.py**: Socket 事件處理器
- **core/server.py**: 伺服器基礎配置

### 業務模組 (分層架構)
- **api/**: API 客戶端模組（node.py 等）
- **monitoring/**: 監控服務模組（task_monitor.py）
- **services/**: 業務邏輯服務（opui_task_service.py）
- **database/**: 資料庫操作層（operations.py）
- **constants/**: 常數定義模組

### 狀態管理系統
- **userStore**: 用戶狀態管理（clientId、machineId、連線狀態）
- **operationStore**: 操作狀態管理（左右側操作狀態）
- **dataStore**: 資料狀態管理（products、machines、rooms、parking）
- **tasksStore**: 任務狀態管理（活躍任務追蹤）
- **uiStore**: UI 狀態管理（載入狀態、通知訊息）

## 關鍵檔案

### 前端核心檔案
- `/opui/frontend/static/index.js` - 全域初始化和共用功能
- `/opui/frontend/static/js/pages/homePage.js` - Home 頁面功能
- `/opui/frontend/static/js/pages/settingPage.js` - Settings 頁面功能
- `/opui/frontend/static/js/pages/rackPage.js` - Rack 頁面功能
- `/opui/frontend/static/js/lib/miniStore.js` - 輕量級狀態管理
- `/opui/frontend/static/js/store.js` - Store 狀態管理
- `/opui/frontend/static/js/managers/` - 管理器模組（StateManager、UIManager、EventManager、PageManager）

### 後端核心檔案
- `/opui/core/op_ui_server.py` - FastAPI 主伺服器
- `/opui/core/op_ui_socket.py` - Socket.IO 事件處理
- `/opui/core/task_service.py` - 任務業務邏輯
- `/opui/core/device_auth.py` - 設備認證
- `/opui/core/socket_handler.py` - Socket 事件處理器
- `/opui/core/server.py` - 伺服器基礎配置

### 模板檔案
- `/opui/frontend/templates/home.html` - 主頁面模板（非 index.html）
- `/opui/frontend/templates/base.html` - 基礎模板
- `/opui/frontend/templates/hmi.html` - HMI 頁面模板
- `/opui/frontend/templates/setting.html` - 設定頁面模板
- `/opui/frontend/templates/rack.html` - 料架頁面模板
- `/opui/frontend/templates/navbar.html` - 導航欄模板
- `/opui/frontend/templates/error.html` - 錯誤頁面模板
- `/opui/frontend/templates/unauthorized.html` - 未授權頁面模板
- `/opui/frontend/static/css/style.css` - 樣式定義

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
# 使用 db_proxy 連線池（實際使用方式）
from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.models import Client, Product, Machine, Room, Rack, Task, Work, TaskStatus
from db_proxy.crud.base_crud import BaseCRUD
from db_proxy.crud.node_crud import node_crud

# 連線池管理
pool_manager = ConnectionPoolManager()
DATABASE_CONFIG = {
    "pool_size": 10,
    "max_overflow": 20,
    "pool_timeout": 30
}
```

## 工作區配置功能

### 架構設計
**工作區與停車格分離設計**：將料架存放區域分為工作區（workspace）和停車格（parking space）兩個獨立部分

#### 資料庫架構
```python
# Machine 表新增欄位（PostgreSQL INTEGER[] 陣列）
workspace_1: List[int]  # 作業員1（左側）的工作區 location ID 陣列
workspace_2: List[int]  # 作業員2（右側）的工作區 location ID 陣列
parking_space_1: int    # 作業員1（左側）的停車格 location ID
parking_space_2: int    # 作業員2（右側）的停車格 location ID

# 範例配置
machine.workspace_1 = [101, 102, 103]  # 左側有3個工作區位置
machine.workspace_2 = [104, 105, 106]  # 右側有3個工作區位置
machine.parking_space_1 = 95          # 左側停車格
machine.parking_space_2 = 96          # 右側停車格
```

### 料架管理邏輯

#### 加入料架（add_rack）邏輯優化
- **優先級順序**：工作區位置 > 停車格（永不使用）
- **自動分配**：自動選擇第一個可用的工作區位置
- **滿載保護**：當所有工作區位置都被佔用時，返回錯誤訊息
- **錯誤處理**：「工作區已滿，請等待料架派送完成」

```python
# op_ui_socket.py 實作邏輯（第614-644行）
if side == "left":
    workspace_locations = machine.workspace_1 or []
else:
    workspace_locations = machine.workspace_2 or []

# 查詢可用的工作區位置
available_location = None
for location_id in workspace_locations:
    existing_rack = rack_crud.get_by_field(session, "location_id", location_id)
    if not existing_rack:  # 位置可用
        available_location = location_id
        break

if not available_location:
    return {"success": False, "message": f"{side_name} 工作區已滿，請等待料架派送完成"}
```

#### 派車（dispatch_full）流程更新
- **移動路徑**：從工作區 → 停車格
- **位置檢查**：確認料架在工作區中
- **目標位置**：根據操作員側選擇對應停車格
- **向後相容**：保持與舊版本的相容性

### 功能特點
1. **容量管理**：每個工作區可配置多個位置，提供彈性的容量管理
2. **職責分離**：工作區用於暫存，停車格用於派送，職責明確
3. **自動化分配**：系統自動管理工作區位置分配，減少人工錯誤
4. **錯誤防護**：完善的滿載檢測和錯誤提示機制

## 整合點

### 與其他專案整合
- **db_proxy_ws**: 使用 connection_pool_manager 進行資料庫操作，支援 INTEGER[] 陣列欄位
- **rcs_ws**: 透過 Socket.IO 接收任務狀態更新
- **web_api_ws**: 共享 FastAPI 技術棧和配置
- **agvcui**: 共用 Web 服務架構模式

### Socket.IO 事件
```javascript
// 客戶端事件
socket.emit('client_update', clientData);
socket.emit('add_rack', rackData);         // 加入料架到工作區
socket.emit('dispatch_full', dispatchData); // 派車（從工作區到停車格）

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
# 頁面功能測試（實際存在的測試檔案）
python3 tests/test_room_button_logic.py    # 房間按鈕邏輯測試
python3 tests/test_task_creation.py        # 任務創建測試
python3 tests/test_task_completion_flow.py # 任務完成流程測試

# Socket.IO 連線測試
python3 tests/test_op_ui_socket.py         # Socket 事件測試
```

### 後端測試
```bash
# API 端點測試（實際存在的測試檔案）
python3 tests/test_routers.py              # 路由測試
python3 tests/test_op_ui_server.py         # 伺服器測試

# 整合測試
python3 tests/test_integration.py          # 整合測試
python3 tests/test_simple.py               # 簡單測試
python3 tests/test_basic.py                # 基礎測試

# 資料庫測試
python3 tests/test_db.py                   # 資料庫操作測試

# 效能測試
python3 tests/test_performance.py          # 效能測試
```

## 故障排除

### 常見問題

#### 工作區配置相關問題
```bash
# 檢查機台工作區配置
psql -h 192.168.100.254 -U agvc -d agvc -c "SELECT id, name, workspace_1, workspace_2 FROM machine WHERE id = 1;"

# 檢查工作區料架狀態
psql -h 192.168.100.254 -U agvc -d agvc -c "SELECT r.id, r.name, r.location_id FROM rack r WHERE r.location_id IN (101, 102, 103);"

# 測試工作區滿載情況
python3 agents/test_workspace_config.py
```

**常見錯誤訊息**：
- "工作區已滿，請等待料架派送完成" - 所有工作區位置都被佔用
- "機台 X 的工作區未配置" - workspace_1 或 workspace_2 為 NULL
- "料架 X 不存在於系統中" - 料架未在資料庫中註冊

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
ls -la /app/web_api_ws/src/opui/opui/frontend/static/

# 檢查模板檔案
ls -la /app/web_api_ws/src/opui/opui/frontend/templates/

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
