# OPUI - 操作員介面系統

OPUI (Operator User Interface) 是一個基於 FastAPI + Socket.IO 的即時 Web 應用程式，專為 AGV 調度系統的操作員介面設計。

## 系統架構

```
┌─────────────────┐    Socket.IO    ┌─────────────────┐    db_proxy    ┌─────────────────┐
│   前端 (Web)    │ ←──────────────→ │   後端 (Python) │ ←─────────────→ │  資料庫 (PostgreSQL) │
│                 │                 │                 │                │                 │
│ • homePage.js   │                 │ • op_ui_server  │                │ • Client        │
│ • settingPage.js│                 │ • op_ui_socket  │                │ • Product       │
│ • miniStore     │                 │ • db.py         │                │ • Machine       │
│ • socket.js     │                 │ • routers       │                │ • Room          │
└─────────────────┘                 └─────────────────┘                └─────────────────┘
```

## 專案結構

```
opui/
├── opui/
│   ├── __init__.py
│   ├── op_ui_server.py          # FastAPI 伺服器主程式
│   ├── op_ui_socket.py          # Socket.IO 事件處理
│   ├── db.py                    # 資料庫操作層
│   ├── routers/                 # API 路由
│   │   ├── process_settings.py
│   │   └── product.py
│   ├── static/                  # 前端靜態資源
│   │   ├── js/
│   │   │   ├── homePage.js      # 主頁面邏輯
│   │   │   ├── settingPage.js   # 設定頁面邏輯
│   │   │   ├── socket.js        # Socket.IO 客戶端
│   │   │   └── store/
│   │   │       ├── index.js     # 狀態管理
│   │   │       └── miniStore.js # 輕量級狀態管理庫
│   │   ├── css/                 # 樣式檔案
│   │   └── socket.io.js         # Socket.IO 客戶端庫
│   └── templates/               # HTML 模板
│       ├── base.html
│       ├── home.html            # 操作主頁
│       └── setting.html         # 設定頁面
├── package.xml                  # ROS2 套件配置
├── setup.py                     # Python 套件配置
└── README.md                    # 本文件
```

## 核心組件

### 1. op_ui_server.py - 伺服器核心

負責建立和配置整個 Web 伺服器：

- **FastAPI 應用程式**：處理 HTTP 請求和路由
- **Socket.IO 伺服器**：處理即時雙向通訊
- **靜態檔案服務**：提供前端資源
- **模板渲染**：使用 Jinja2 渲染 HTML 頁面
- **CORS 設定**：支援跨域請求

```python
class OpUiServer:
    def __init__(self, host="0.0.0.0", port=8002):
        self.sio = socketio.AsyncServer(async_mode="asgi")
        self.app = FastAPI()
        self.sio_app = socketio.ASGIApp(self.sio, self.app)
        self.op_ui_socket = OpUiSocket(self.sio)
```

### 2. op_ui_socket.py - Socket 事件處理

管理所有 Socket.IO 事件和即時通訊：

#### 主要事件處理：
- **連線管理**
  - `connect`: 使用者連線
  - `disconnect`: 使用者離線
  - `login`: 使用者登入驗證

- **資料同步**
  - `client_update`: 更新客戶端設定
  - `notify_products/machines/rooms`: 推送基礎資料

- **AGV 操作**
  - `call_empty`: 叫空車指令（會在資料庫 Task 表中創建任務記錄）
  - `dispatch_full`: 派滿車指令（會在資料庫 Task 表中創建任務記錄）

#### 使用者會話管理：
```python
self.user_sid_map = {}  # clientId -> sid 映射
```

### 3. 前端架構

#### 狀態管理 (miniStore)
使用自製的輕量級狀態管理系統：

```javascript
// 主要 Store
const clientStore = createStore('clientState', {
    op: {
        left: { productSelected: 0, product: [...] },
        right: { productSelected: 0, product: [...] }
    },
    machineId: 1,
    clientId: null
});

const parkingStore = createStore('parkingState', {...});
const productsStore = createStore('productsState', {...});
const machinesStore = createStore('machinesState', {...});
const roomsStore = createStore('roomsState', {...});
```

#### 頁面組件
- **homePage.js**: 主操作介面，處理產品選擇、數量設定、叫車派車
- **settingPage.js**: 設定頁面，處理產品配置、機台選擇
- **socket.js**: Socket.IO 客戶端封裝，提供 API 介面

## 資料流程

### 1. 初始化流程
```
使用者開啟網頁 → Socket 自動連線 → 觸發 login 事件 →
伺服器驗證並建立會話 → 推送基礎資料 → 前端更新狀態 → UI 渲染
```

### 2. 操作流程
```
使用者操作 → 前端事件處理 → Socket 發送事件 →
後端處理邏輯 → 資料庫操作 → 回應結果 → 前端狀態更新
```

### 3. 即時同步
```
資料變更 → 後端檢測 → Socket 廣播 →
所有連線客戶端接收 → 狀態更新 → UI 重新渲染
```

### 4. 任務創建流程
```
使用者點擊叫車/派車 → Socket 事件觸發 →
獲取客戶端資訊 → 準備任務資料 →
創建 Task 記錄 → 回應操作結果
```

## 任務管理功能

### 叫空車任務 (call_empty)

當操作員點擊叫空車按鈕時，系統會：

1. **收集資訊**：
   - 停車位資訊 (parking_space)
   - 客戶端 ID (clientId)
   - 機台 ID (machineId)

2. **創建任務記錄**：
   ```python
   task_data = {
       "name": "叫空車 - 停車位 001",
       "description": "操作員從機台 1 叫空車到停車位 001",
       "work_id": get_call_empty_work_id(),  # 叫空車工作類型
       "status_id": get_default_task_status_id(),  # 待執行狀態
       "priority": 1,
       "parameters": {
           "parking_space": {...},
           "machine_id": 1,
           "client_id": "client_123",
           "task_type": "call_empty"
       }
   }
   ```

3. **資料庫操作**：
   - 插入 Task 表記錄
   - 自動創建 Work 類型（如不存在）
   - 自動創建 TaskStatus（如不存在）

### 派滿車任務 (dispatch_full)

當操作員點擊派滿車按鈕時，系統會：

1. **收集資訊**：
   - 停車位資訊 (parking_space)
   - 產品名稱 (product_name)
   - 數量 (count)
   - 料架 ID (rack_id)
   - 房間 (room)
   - 側邊 (side)

2. **創建任務記錄**：
   ```python
   task_data = {
       "name": "派滿車 - 產品A x32 到停車位 001",
       "description": "操作員從機台 1 派滿車，產品: 產品A，數量: 32",
       "work_id": get_dispatch_full_work_id(),  # 派滿車工作類型
       "status_id": get_default_task_status_id(),  # 待執行狀態
       "priority": 2,  # 較高優先級
       "parameters": {
           "parking_space": {...},
           "product_name": "產品A",
           "count": 32,
           "rack_id": 123,
           "room": 2,
           "side": "left",
           "task_type": "dispatch_full"
       }
   }
   ```

### 任務狀態管理

系統自動管理以下任務狀態：
- **待執行** (pending/created): 任務已創建，等待執行
- **執行中** (running): 任務正在執行
- **已完成** (completed): 任務執行完成
- **失敗** (failed): 任務執行失敗
- **取消** (cancelled): 任務被取消

## 資料庫整合

使用 `db_proxy` 的 `ConnectionPoolManager` 進行資料庫操作：

```python
# 資料庫連線
db_url_agvc = 'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'
connection_pool = ConnectionPoolManager(db_url_agvc)

# CRUD 操作
client_crud = BaseCRUD(Client, id_column="id")
product_crud = BaseCRUD(Product, id_column="id")
machine_crud = BaseCRUD(Machine, id_column="id")
room_crud = BaseCRUD(Room, id_column="id")
```

### 主要資料表
- **Client**: 客戶端/操作員資訊
- **Product**: 產品資料
- **Machine**: 機台資訊
- **Room**: 房間/區域資料
- **Rack**: 料架資訊
- **Task**: 任務資料（叫空車、派滿車等操作記錄）
- **Work**: 工作類型（叫空車、派滿車等）
- **TaskStatus**: 任務狀態（待執行、執行中、已完成等）

## 部署與運行

### 環境需求
- Python 3.8+
- PostgreSQL 資料庫
- ROS2 環境 (Docker 容器)

### 安裝步驟

1. **建置 ROS2 套件**
```bash
cd /app/web_api_ws
colcon build --packages-select opui
source install/setup.bash  # 或執行 all_source
```

2. **運行服務**
```bash
ros2 run opui op_ui_server
```

3. **訪問介面**
- 主頁面: http://localhost:8002/
- 設定頁面: http://localhost:8002/setting

### 配置選項

環境變數：
- `CORS_ALLOWED_ORIGINS`: CORS 允許的來源 (預設: "*")

## API 介面

### Socket.IO 事件

#### 客戶端 → 伺服器
- `login(client_data)`: 使用者登入
- `client_update(data)`: 更新客戶端設定
- `call_empty(data)`: 叫空車
- `dispatch_full(data)`: 派滿車

#### 伺服器 → 客戶端
- `product_list(products)`: 產品列表更新
- `machine_list(machines)`: 機台列表更新
- `room_list(rooms)`: 房間列表更新
- `parking_list(parking)`: 停車位列表更新
- `notify_message(msg)`: 通知訊息
- `error_message(msg)`: 錯誤訊息

### HTTP API
- `GET /`: 主頁面
- `GET /setting`: 設定頁面
- 其他 API 路由定義在 `routers/` 目錄

## 開發指南

### 前端開發
- 使用 ES6 模組系統
- 狀態管理透過 miniStore
- UI 框架使用 Bulma CSS
- 圖示使用 Material Design Icons

### 後端開發
- 遵循 FastAPI 最佳實踐
- 使用 async/await 處理非同步操作
- 資料庫操作透過 db_proxy 統一管理
- Socket.IO 事件處理保持簡潔

### 測試
```bash
# 運行測試 (如果有)
python -m pytest tests/
```

## 故障排除

### 常見問題

1. **Socket 連線失敗**
   - 檢查伺服器是否正常運行
   - 確認防火牆設定
   - 檢查 CORS 配置

2. **資料庫連線錯誤**
   - 確認資料庫服務運行
   - 檢查連線字串設定
   - 驗證資料庫權限

3. **ROS2 套件找不到**
   - 執行 `colcon build`
   - 確保執行 `source install/setup.bash`
   - 檢查 package.xml 配置

## 相關專案

- **rcs_core**: AGV 調度核心系統
- **db_proxy**: 資料庫代理層
- **agvcui**: AGV 控制介面
- **agvui**: AGV 監控介面

## 授權

本專案為內部開發使用。

---

*最後更新: 2025-06-26*
