# RosAGV Web API 技術規格

## 📋 概述

本文檔詳細描述 RosAGV 系統的 Web API 技術規格，基於 `app/web_api_ws/` 和 `app/db_proxy_ws/` 工作空間的實際程式碼實作。系統採用 FastAPI 框架，整合 SQLModel ORM，提供完整的 RESTful API 服務。

## 🏗️ 系統架構

### Web API 服務架構
```mermaid
graph TD
    A[FastAPI 應用程式] --> B[API 路由器]
    B --> C[PLC 控制 API]
    B --> D[門控制 API]
    B --> E[交通管制 API]
    B --> F[地圖匯入 API]
    B --> G[KUKA Fleet API]
    
    A --> H[中介軟體層]
    H --> I[CORS 中介軟體]
    H --> J[認證中介軟體]
    
    A --> K[資料庫層]
    K --> L[SQLModel ORM]
    K --> M[PostgreSQL 連線池]
    
    subgraph "外部整合"
        N[PLC 代理服務]
        O[ECS 門控制]
        P[交通控制器]
        Q[KUKA Fleet 系統]
    end
    
    C --> N
    D --> O
    E --> P
    G --> Q
```

### 服務組件配置
| 組件 | 實作類別 | 端口 | 狀態 |
|------|----------|------|------|
| Web API 主服務 | `ApiServer` | 8000 | ✅ 完成 |
| AGVCUI 服務 | `AgvcUiServer` | 8001 | ✅ 完成 |
| OPUI 服務 | `OpUiServer` | 8002 | ✅ 完成 |
| Vue 開發服務 | - | 5173 | ✅ 完成 |

## 🔧 核心 API 服務 (web_api)

### 應用程式配置
```python
# 主應用程式類別: ApiServer
class ApiServer:
    def __init__(self):
        self.app = FastAPI()
        self.plc_client = PlcClientNode('plc_client', 'agvc')
        self.door_config = DoorControllerConfig()
        self.door_controller = DoorLogic(self.plc_client, self.door_config)
        self.db_pool = ConnectionPoolManager(
            'postgresql+psycopg2://agvc:password@192.168.100.254/agvc')
        self.traffic_controller = TrafficController(self.db_pool)

# 啟動配置
uvicorn.run(app, host="0.0.0.0", port=8000, log_level="debug")
```

### 環境變數配置
```yaml
# 實際使用的環境變數
CONTAINER_TYPE: "agvc"
RMW_IMPLEMENTATION: "rmw_zenoh_cpp"
ZENOH_ROUTER_CONFIG_URI: "/app/routerconfig.json5"
DISPLAY: $DISPLAY
```

## 📡 API 端點規格

### 1. PLC 控制 API (`/plc/`)

#### 1.1 讀取 PLC 資料
```yaml
端點: GET /plc/get_data/{device_type}/{key}
描述: 讀取指定 PLC 裝置的資料
路徑參數:
  - device_type: string (DM 或 MR)
  - key: string (PLC 位址)
回應格式:
  成功 (200):
    {
      "key": "string",
      "value": "string"
    }
  錯誤 (404): {"detail": "Key not found in PLC"}
  錯誤 (500): {"detail": "Failed to reading data from PLC {message}"}
```

#### 1.2 PLC 資料模型
```python
# Pydantic 模型定義
class ForceInput(BaseModel):
    device_type: str
    key: str

class SingleDataInput(BaseModel):
    device_type: str
    key: str
    value: str

class ContinuousDataInput(BaseModel):
    device_type: str
    start_key: str
    values: List[str]
```

### 2. 門控制 API (`/door/`)

#### 2.1 門控制指令
```yaml
端點: POST /door/control
描述: 控制指定門的開啟或關閉
請求體:
  {
    "doorId": "string",
    "isOpen": boolean
  }
回應: 門控制器回應結果
```

#### 2.2 門狀態查詢
```yaml
端點: POST /door/state
描述: 查詢指定門的當前狀態
請求體:
  {
    "doorId": "string"
  }
回應: 門狀態資訊
```

#### 2.3 門控制資料模型
```python
class DoorStateData(BaseModel):
    doorId: str

class DoorControlData(BaseModel):
    doorId: str
    isOpen: bool
```

### 3. 交通管制 API (`/traffic/`)

#### 3.1 取得交管區使用權
```yaml
端點: POST /traffic/acquire
描述: 請求取得指定交管區的使用權
請求體:
  {
    "trafficId": "string",
    "agvId": "string"
  }
回應:
  成功: {"isAllow": true, "success": true}
  失敗: {"detail": "Failed to acquire traffic zone"}
```

#### 3.2 釋放交管區使用權
```yaml
端點: POST /traffic/release
描述: 釋放指定交管區的使用權
請求體:
  {
    "trafficId": "string",
    "agvId": "string"
  }
回應: 操作結果
```

#### 3.3 按名稱操作交管區
```yaml
端點: POST /traffic/acquire_by_name
端點: POST /traffic/release_by_name
描述: 按交管區名稱進行操作
請求體:
  {
    "trafficName": "string",
    "agvId": "string"
  }
```

### 4. KUKA Fleet API 整合

#### 4.0 API 通訊方向說明
KUKA Fleet API 整合涉及雙向通訊：

**主動請求 (RosAGV → KUKA Fleet)**:
- 我們的系統透過 `KukaApiClient` 主動向 KUKA Fleet 系統發送請求
- 包括：任務提交、狀態查詢、機器人管理、容器管理、地圖查詢等
- 目標地址：`http://192.168.11.206:10870`

**被動回調 (KUKA Fleet → RosAGV)**:
- KUKA Fleet 系統主動向我們的系統發送任務狀態回報
- 端點：`POST /interfaces/api/amr/missionStateCallback`
- 我們的系統作為 Web 服務接收這些回調

#### 4.1 KUKA Fleet 系統架構
```mermaid
graph TD
    A[RosAGV 系統] --> B[KUKA Fleet Adapter]
    B --> C[KUKA API Client]
    C -->|"請求方向: → KUKA Fleet"| D[KUKA Fleet 系統<br/>192.168.11.206:10870]

    B --> E[任務管理]
    E --> F[MOVE 任務]
    E --> G[RACK_MOVE 任務]
    E --> H[WORKFLOW 任務]

    B --> I[狀態監控]
    I --> J[機器人狀態查詢]
    I --> K[容器狀態查詢]
    I --> L[任務狀態追蹤]

    D -->|"回調方向: KUKA Fleet →"| M[任務狀態回報]
    M --> N[Web API 回調<br/>/interfaces/api/amr/missionStateCallback]
    N --> O[RosAGV 資料庫<br/>Task.parameters 更新]

    style D fill:#e1f5fe
    style N fill:#f3e5f5
    style C fill:#e8f5e8
```

#### 4.2 KUKA API 客戶端配置
```python
# KukaApiClient 配置
class KukaApiClient:
    def __init__(self,
                 base_url="http://192.168.11.206:10870",
                 username="admin",
                 password="Admin"):
        self.base_url = base_url
        self.token = None
        if username and password:
            self.login(username, password)
```

#### 4.3 認證機制
```yaml
端點: POST /api/login
方向: RosAGV 系統 → KUKA Fleet 系統
描述: KUKA Fleet 系統登入認證
請求體:
  {
    "username": "admin",
    "password": "Admin"
  }
回應:
  {
    "success": true,
    "data": {
      "token": "Bearer_token_string",
      "access_token": "access_token_string"
    },
    "message": "Login successful"
  }
認證標頭: Authorization: {token}
```

#### 4.4 任務提交 API
```yaml
端點: POST /api/amr/submitMission
方向: RosAGV 系統 → KUKA Fleet 系統
描述: 提交任務到 KUKA Fleet 系統
請求體:
  {
    "orgId": "Ching-Tech",              # 組織 ID (必填)
    "requestId": "REQ_001",             # 請求 ID (必填)
    "missionCode": "MISSION_001",       # 任務代碼 (必填)
    "missionType": "MOVE",              # 任務類型 (必填): MOVE, RACK_MOVE
    "viewBoardType": "string",          # 看板顯示類型 (可選)
    "robotModels": ["KMP 400i diffDrive"], # 機器人型號 (可選)
    "robotIds": [1],                    # 機器人 ID 陣列 (可選)
    "robotType": "LIFT",                # 機器人類型 (可選)
    "priority": 1,                      # 優先級 1-99 (可選)
    "containerModelCode": "string",     # 容器型號代碼 (可選)
    "containerCode": "string",          # 容器代碼 (可選)
    "templateCode": "string",           # 模板代碼 (可選)
    "lockRobotAfterFinish": true,       # 完成後鎖定機器人 (可選)
    "unlockRobotId": "string",          # 解鎖機器人 ID (可選)
    "unlockMissionCode": "string",      # 解鎖任務代碼 (可選)
    "idleNode": "string",               # 閒置節點 (可選)
    "missionData": [                    # 任務資料 (可選)
      {
        "sequence": 1,
        "position": "test-test1-1",
        "type": "NODE_POINT",
        "passStrategy": "AUTO"
      }
    ]
  }
回應:
  {
    "success": true,
    "code": "200",
    "data": {},
    "msg": "Mission submitted successfully"
  }
```

#### 4.5 任務管理 API
```yaml
# 任務取消
端點: POST /api/amr/missionCancel
方向: RosAGV 系統 → KUKA Fleet 系統
請求體:
  {
    "missionCode": "MISSION_001",       # 任務代碼 (可選)
    "cancelMode": "FORCE",              # 取消模式 (可選)
    "containerCode": "string",          # 容器代碼 (可選)
    "position": "string",               # 位置 (可選)
    "reason": "string",                 # 取消原因 (可選)
    "requestId": "REQ_001"              # 請求 ID (可選)
  }

# 任務暫停
端點: POST /api/amr/pauseMission
方向: RosAGV 系統 → KUKA Fleet 系統
參數: missionCode=MISSION_001

# 任務恢復
端點: POST /api/amr/recoverMission
方向: RosAGV 系統 → KUKA Fleet 系統
參數: missionCode=MISSION_001

# 任務重定向
端點: POST /api/amr/redirectMission
方向: RosAGV 系統 → KUKA Fleet 系統
請求體:
  {
    "missionCode": "MISSION_001",
    "newTargetPosition": "test-test1-5"
  }
```

#### 4.6 機器人管理 API
```yaml
# 機器人狀態查詢
端點: POST /api/amr/robotQuery
方向: RosAGV 系統 → KUKA Fleet 系統
請求體: {}  # 空物件查詢所有機器人
回應:
  {
    "success": true,
    "data": [
      {
        "robotId": "1",
        "status": 3,                    # 1-離場 2-離線 3-空閒 4-任務中 5-充電中 6-更新中 7-錯誤
        "batteryLevel": 85,
        "currentPosition": "test-test1-1",
        "missionCode": "MISSION_001",
        "x": "10.5",
        "y": "20.3",
        "robotOrientation": "90.0",
        "errorMessage": ""
      }
    ]
  }

# 機器人充電
端點: POST /api/amr/chargeRobot
方向: RosAGV 系統 → KUKA Fleet 系統
請求體:
  {
    "robotId": "1",
    "chargePosition": "CHARGE_STATION_1"
  }

# 機器人入場
端點: POST /api/amr/insertRobot
方向: RosAGV 系統 → KUKA Fleet 系統
請求體:
  {
    "robotId": "1",
    "entryPosition": "ENTRY_POINT_1"
  }

# 機器人離場
端點: POST /api/amr/removeRobot
方向: RosAGV 系統 → KUKA Fleet 系統
請求體:
  {
    "robotId": "1",
    "exitPosition": "EXIT_POINT_1"
  }
```

#### 4.7 任務狀態回報 (回調)
```yaml
端點: POST /interfaces/api/amr/missionStateCallback
方向: RosAGV 系統 ← KUKA Fleet 系統
描述: 接收 KUKA 系統的任務狀態回報 (KUKA Fleet 主動回調我們的系統)
請求體:
  {
    "missionCode": "MISSION_001",       # 必填，最大長度32
    "viewBoardType": "MOVE",            # 可選，任務類型
    "containerCode": "CONT_001",        # 可選，容器代碼
    "currentPosition": "test-test1-3",  # 可選，當前位置
    "slotCode": "SLOT_01",             # 可選，槽位代碼
    "robotId": "1",                    # 可選，機器人ID
    "missionStatus": "COMPLETED",       # 必填，任務狀態
    "message": "任務完成",              # 可選，補充說明
    "missionData": {}                  # 可選，任務自訂資料
  }
回應: 任務更新結果
```

#### 4.8 容器管理 API
```yaml
# 容器入場
端點: POST /api/amr/containerIn
方向: RosAGV 系統 → KUKA Fleet 系統
請求體:
  {
    "requestId": "REQ_CONT_001",
    "containerCode": "CONTAINER_001",
    "position": "test-test1-10",
    "isNew": true
  }

# 容器出場
端點: POST /api/amr/containerOut
方向: RosAGV 系統 → KUKA Fleet 系統
請求體:
  {
    "requestId": "REQ_CONT_002",
    "containerCode": "CONTAINER_001",
    "position": "test-test1-20"
  }

# 容器查詢 (所有)
端點: POST /api/amr/containerQueryAll
方向: RosAGV 系統 → KUKA Fleet 系統
請求體: {}
回應:
  {
    "success": true,
    "data": [
      {
        "containerCode": "CONTAINER_001",
        "position": "test-test1-10",
        "status": "OCCUPIED",
        "robotId": "1"
      }
    ]
  }

# 容器查詢 (指定)
端點: POST /api/amr/containerQuery
方向: RosAGV 系統 → KUKA Fleet 系統
請求體:
  {
    "containerCode": "CONTAINER_001"
  }
```

#### 4.9 地圖和區域管理 API
```yaml
# 查詢所有 WCS 區域
端點: GET /api/amr/areaQuery
方向: RosAGV 系統 → KUKA Fleet 系統
回應:
  {
    "success": true,
    "data": [
      {
        "areaCode": "AREA_001",
        "areaName": "生產區域",
        "mapCode": "test-test1"
      }
    ]
  }

# 區域內點位查詢
端點: POST /api/amr/areaNodesQuery
方向: RosAGV 系統 → KUKA Fleet 系統
請求體:
  {
    "areaCode": "AREA_001"
  }

# 查詢功能點位
端點: POST /api/amr/queryFunctionNode
方向: RosAGV 系統 → KUKA Fleet 系統
請求體:
  {
    "nodeType": "CHARGE_STATION"
  }

# 查詢所有禁行區
端點: GET /api/amr/queryAllForbiddenAreas
方向: RosAGV 系統 → KUKA Fleet 系統
回應:
  {
    "success": true,
    "data": [
      {
        "forbiddenAreaCode": "FORBIDDEN_001",
        "forbiddenAreaId": 1,
        "status": "ACTIVE"
      }
    ]
  }

# 更新禁行區狀態
端點: POST /api/amr/updateForbiddenAreaStatus
方向: RosAGV 系統 → KUKA Fleet 系統
請求體:
  {
    "forbiddenAreaId": 1,
    "status": "INACTIVE"
  }
```

#### 4.10 KUKA Fleet Adapter 常數定義
```python
# AGV 狀態常數
class KukaFleetAdapter:
    STATUS_REMOVED = 1      # 離場
    STATUS_OFFLINE = 2      # 離線
    STATUS_IDLE = 3         # 空閒
    STATUS_RUNNING = 4      # 任務中
    STATUS_CHARGING = 5     # 充電中
    STATUS_UPDATING = 6     # 更新中
    STATUS_ERROR = 7        # 錯誤

    # 任務類型常數
    MISSION_MOVE = "MOVE"           # 移動任務
    MISSION_RACK_MOVE = "RACK_MOVE" # 搬運任務

    # 地圖配置
    MAP_LAYOUT_DISTRICT = "test-test1"  # 地圖佈局區域

    # 機器人配置
    ROBOT_MODEL = "KMP 400i diffDrive"  # 機器人型號
    ROBOT_TYPE = "LIFT"                 # 機器人類型
    ORG_ID = "Ching-Tech"              # 組織 ID
```

### 5. 地圖匯入 API (`/map_importer/`)

#### 5.1 上傳 KUKA 地圖
```yaml
端點: POST /map_importer/upload-kuka-map/
描述: 上傳 KUKA 地圖檔案並匯入資料庫
請求: multipart/form-data 檔案上傳
回應:
  {
    "message": "Upload successful",
    "floors": number,
    "nodes_saved": number,
    "edges_saved": number,
    "node_label_mapping": {}
  }
```

#### 5.2 上傳 CT 地圖
```yaml
端點: POST /map_importer/upload-ct-map/
描述: 上傳 CT 地圖檔案並匯入資料庫
請求: multipart/form-data 檔案上傳
回應:
  {
    "message": "Upload successful",
    "nodes_saved": number,
    "edges_saved": number
  }
```

#### 5.3 刪除 KUKA 地圖
```yaml
端點: DELETE /map_importer/delete-kuka-map
描述: 刪除所有 KUKA 地圖資料
回應:
  {
    "message": "Delete successful"
  }
```

### 6. 系統管理 API

#### 6.1 伺服器關閉
```yaml
端點: GET /shutdown
描述: 關閉 API 伺服器
回應:
  {
    "message": "Server is shutting down..."
  }
```

## 🔍 錯誤處理

### HTTP 狀態碼規範
| 狀態碼 | 說明 | 使用場景 |
|--------|------|----------|
| 200 | 成功 | 操作成功完成 |
| 404 | 未找到 | PLC 鍵值不存在 |
| 500 | 伺服器錯誤 | PLC 通訊失敗、資料庫錯誤 |

### 錯誤回應格式
```json
{
  "detail": "錯誤描述訊息"
}
```

## 🖥️ AGVCUI 服務 API (port 8001)

### 應用程式配置
```python
# AGVCUI 服務類別: AgvcUiServer
class AgvcUiServer:
    def __init__(self):
        self.app = FastAPI()
        self.db_pool = ConnectionPoolManager(
            'postgresql+psycopg2://agvc:password@192.168.100.254/agvc')
        self.socket_manager = SocketManager()

# 啟動配置
uvicorn.run(app, host="0.0.0.0", port=8001, log_level="debug")
```

### 主要功能模組
| 模組 | 路由前綴 | 功能描述 | 狀態 |
|------|----------|----------|------|
| 認證系統 | `/auth/` | 使用者登入、登出、權限管理 | ✅ 完成 |
| AGV 管理 | `/agvs/` | AGV 車輛資訊管理 | ✅ 完成 |
| 任務管理 | `/tasks/` | 任務建立、查詢、狀態更新 | ✅ 完成 |
| 工作管理 | `/works/` | 工作流程定義和管理 | ✅ 完成 |
| 產品管理 | `/products/` | 產品資訊管理 | ✅ 完成 |
| 料架管理 | `/racks/` | 料架位置和狀態管理 | ✅ 完成 |
| 載具管理 | `/carriers/` | 載具資訊管理 | ✅ 完成 |
| 客戶端管理 | `/clients/` | 客戶端設備管理 | ✅ 完成 |
| 設備管理 | `/devices/` | 設備狀態監控 | ✅ 完成 |
| 地圖管理 | `/map/` | 地圖顯示和編輯 | ✅ 完成 |
| 日誌管理 | `/logs/` | 系統日誌查詢 | ✅ 完成 |
| 使用者管理 | `/users/` | 使用者帳號管理 | ✅ 完成 |

### Socket.IO 即時通訊架構
```python
class AgvcUiSocket:
    def __init__(self, sio):
        self.sio = sio
        self.connected_sids = set()     # 活躍連線追蹤
        self.tasks = []                 # 週期性任務列表
        self._task = None               # 背景任務

    def init_socketio(self):
        """初始化 Socket.IO 事件處理器"""
        self.sio.on('connect')(self.connect)
        self.sio.on('disconnect')(self.disconnect)
        self.sio.on('user_login')(self.user_login)
        self.sio.on('user_logout')(self.user_logout)

        # 註冊週期性任務
        self.tasks = [
            {
                "func": self.notify_by_modifylog,
                "interval": 0.1,            # 100ms 間隔
                "last_time": 0
            }
        ]

    async def _periodic_notify(self):
        """週期性資料同步背景任務"""
        while True:
            now = time.perf_counter()
            for task in self.tasks:
                if now - task["last_time"] >= task["interval"]:
                    for sid in list(self.connected_sids):
                        try:
                            await task["func"](sid)
                        except Exception as e:
                            print(f"❌ Error in task {task['func'].__name__} for sid {sid}: {e}")
                    task["last_time"] = now
            await asyncio.sleep(0.05)
```

### AGVCUI Socket.IO 事件規格

#### 1. 連線管理事件

##### 1.1 客戶端連線 (connect)
```yaml
事件: connect
方向: 客戶端 → 伺服器 (自動觸發)
描述: 客戶端建立 Socket.IO 連線
處理邏輯:
  - 將 sid 加入 connected_sids 集合
  - 啟動週期性資料同步背景任務
  - 自動推送所有系統資料給新連線的客戶端
```

##### 1.2 使用者登入 (user_login)
```yaml
事件: user_login
方向: 客戶端 → 伺服器
請求格式:
  {
    "username": "admin",              # 使用者名稱 (必填)
    "password": "password"            # 密碼 (必填)
  }
回應格式:
  {
    "success": true,
    "message": "登入成功，歡迎 管理員",
    "user": {
      "id": 1,
      "username": "admin",
      "role": "admin",
      "full_name": "管理員",
      "is_active": true,
      "isLoggedIn": true,
      "isConnected": true
    },
    "access_token": "eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9..."
  }
處理邏輯:
  - 驗證使用者憑證
  - 生成 JWT access_token
  - 更新使用者最後登入時間
  - 設定使用者連線狀態
```

##### 1.3 使用者登出 (user_logout)
```yaml
事件: user_logout
方向: 客戶端 → 伺服器
請求格式: {}
回應格式:
  {
    "success": true,
    "message": "登出成功"
  }
處理邏輯:
  - 清除使用者連線狀態
  - 撤銷 JWT token (可選)
```

##### 1.4 客戶端斷線 (disconnect)
```yaml
事件: disconnect
方向: 客戶端 → 伺服器 (自動觸發)
描述: 客戶端斷開 Socket.IO 連線
處理邏輯:
  - 從 connected_sids 集合移除 sid
  - 清理相關資源
```

#### 2. 系統資料廣播事件

##### 2.1 地圖資訊廣播 (map_info)
```yaml
事件: map_info
方向: 伺服器 → 客戶端
資料格式:
  {
    "nodes": [                        # CT 節點
      { "id": 1, "x": 10.5, "y": 20.3 }
    ],
    "edges": [                        # CT 邊
      { "id": 1, "from_id": 1, "to_id": 2, "weight": 1.0, "name": "PATH_1_2" }
    ],
    "kukaNodes": [                    # KUKA 節點
      { "id": 1, "uuid": "550e8400-e29b-41d4-a716-446655440000", "x": 12.0, "y": 22.0 }
    ],
    "kukaEdges": [                    # KUKA 邊
      { "id": 1, "from_id": 1, "to_id": 2, "weight": 1.5, "name": "KUKA_PATH_1_2" }
    ],
    "agvs": [                         # AGV 資料
      {
        "id": 1,
        "name": "AGV_001",
        "model": "K400",
        "x": 10.5,
        "y": 20.3,
        "heading": 90.0,
        "last_node": 1,
        "enable": 1
      }
    ]
  }
觸發時機:
  - 客戶端連線時
  - AGV 資料變更時 (透過 modify_log 追蹤)
```

##### 2.2 AGV 列表廣播 (agv_list)
```yaml
事件: agv_list
方向: 伺服器 → 客戶端
資料格式:
  {
    "agvs": [
      {
        "id": 1,
        "name": "AGV_001",
        "model": "K400",              # 車型: K400, Cargo, Loader, Unloader
        "x": 10.5,                    # X 座標
        "y": 20.3,                    # Y 座標
        "heading": 90.0,              # 方向角
        "last_node": 1,               # 最後節點
        "enable": 1,                  # 啟用狀態
        "description": "主要運輸車輛",
        "status_id": 3,               # 狀態 ID
        "battery_level": 85.5,        # 電池電量
        "current_position": "NODE_123" # 當前位置
      }
    ]
  }
觸發時機:
  - 客戶端連線時
  - AGV 資料變更時 (透過 modify_log 追蹤)
```

##### 2.3 任務列表廣播 (task_list)
```yaml
事件: task_list
方向: 伺服器 → 客戶端
資料格式:
  {
    "tasks": [
      {
        "id": 123,
        "parent_task_id": null,       # 父任務 ID
        "work_id": 1,                 # 工作 ID
        "status_id": 3,               # 狀態 ID (0-6, 51-54)
        "room_id": 1,                 # 房間 ID
        "node_id": 101,               # 節點 ID
        "agv_id": 1,                  # AGV ID
        "name": "運輸任務_001",
        "description": "從 A 點運輸到 B 點",
        "mission_code": "MISSION_001", # KUKA 任務代碼
        "priority": 1,                # 優先級
        "parameters": {               # JSON 參數
          "source_location": "A1",
          "target_location": "B2",
          "kuka_mission_status": "COMPLETED"
        },
        "created_at": "2025-01-17T10:30:00Z",
        "updated_at": "2025-01-17T11:00:00Z"
      }
    ]
  }
觸發時機:
  - 客戶端連線時
  - 任務資料變更時 (透過 modify_log 追蹤)
```

##### 2.4 料架列表廣播 (rack_list)
```yaml
事件: rack_list
方向: 伺服器 → 客戶端
資料格式:
  {
    "racks": [
      {
        "id": 1,
        "name": "RACK_001",
        "agv_id": 1,                  # 關聯 AGV ID
        "location_id": 101,           # 位置 ID
        "product_id": 1,              # 產品 ID
        "is_carry": 0,                # 是否被搬運
        "is_in_map": 1,               # 是否在地圖中
        "is_docked": 0,               # 是否停靠
        "status_id": 1,               # 狀態 ID
        "direction": 0,               # 方向
        "size": "S",                  # 產品尺寸 (S/L)
        "product_name": "PRODUCT_A",  # 產品名稱
        "total": 32,                  # 總容量 (S=32, L=16)
        "count": 15                   # 當前數量
      }
    ]
  }
觸發時機:
  - 客戶端連線時
  - 料架資料變更時 (透過 modify_log 追蹤)
```

##### 2.5 其他系統資料廣播
```yaml
# 載具列表廣播 (carrier_list)
事件: carrier_list
資料: { "carriers": [{ "id", "name", "rack_id", "status_id", "enable" }] }

# 機台列表廣播 (machine_list)
事件: machine_list
資料: { "machines": [{ "id", "name", "description", "parking_space_1", "parking_space_2", "enable" }] }

# 房間列表廣播 (room_list)
事件: room_list
資料: { "rooms": [{ "id", "name", "description", "process_settings_id", "enable" }] }

# 信號列表廣播 (signal_list)
事件: signal_list
資料: { "signals": [{ "id", "eqp_id", "name", "value", "type_of_value", "dm_address" }] }

# 位置列表廣播 (location_list)
事件: location_list
資料: { "locations": [{ "id", "name", "description" }] }
```

#### 3. 變更追蹤機制

##### 3.1 修改日誌追蹤 (notify_by_modifylog)
```yaml
功能: 基於資料庫變更日誌的自動資料同步
執行週期: 100ms (0.1 秒)
追蹤範圍:
  - agv 表變更 → 觸發 notify_agvs + notify_map
  - rack 表變更 → 觸發 notify_racks
  - carrier 表變更 → 觸發 notify_carriers
  - signal 表變更 → 觸發 notify_signals
  - task 表變更 → 觸發 notify_tasks
檢查邏輯:
  - 查詢最近 0.2 秒內的 modify_log 記錄
  - 根據 table_name 決定要觸發的通知函數
  - 對所有活躍連線 (connected_sids) 推送更新
```

##### 3.2 變更追蹤配置
```python
# 追蹤的資料表映射
notify_map = {
    "agv": self.notify_agvs,
    "rack": self.notify_racks,
    "carrier": self.notify_carriers,
    "signal": self.notify_signals,
    "task": self.notify_tasks,
}

# 變更檢查時間窗口
check_since = now - timedelta(seconds=0.2)  # 0.2 秒內的變更
recent_updates = [log for log in logs if log.modified_at > check_since]
```

#### 4. 通知事件

##### 4.1 通知訊息 (notify_message)
```yaml
事件: notify_message
方向: 伺服器 → 客戶端
資料格式:
  {
    "message": "AGV 任務已完成"
  }
```

##### 4.2 錯誤訊息 (error_message)
```yaml
事件: error_message
方向: 伺服器 → 客戶端
資料格式:
  {
    "message": "AGV 連線中斷"
  }
```

## 🖥️ AGVCUI 服務 API (port 8001)

### 應用程式配置
```python
# AGVCUI 服務類別: AgvcUiServer
class AgvcUiServer:
    def __init__(self, host="0.0.0.0", port=8001):
        self.host = host
        self.port = port

        # 初始化 Socket.IO
        self.sio = socketio.AsyncServer(
            async_mode="asgi",
            cors_allowed_origins="*"
        )

        # 建立 FastAPI 應用
        self.app = FastAPI(lifespan=lifespan)

        # 將 Socket.IO 整合進 ASGI App
        self.sio_app = socketio.ASGIApp(self.sio, self.app)

        # 啟用 CORS
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )

        # 添加認證中間件
        self.app.add_middleware(AuthMiddleware)

# 啟動配置
uvicorn.run(self.sio_app, host="0.0.0.0", port=8001)
```

### HTTP 路由配置
```python
def register_routes(self):
    """註冊 API 路由"""

    @self.app.get("/", response_class=HTMLResponse)
    async def index(request: Request):
        # 首頁路由

    # 註冊所有路由器
    self.app.include_router(map.get_router(self.templates))
    self.app.include_router(tasks.get_router(self.templates))
    self.app.include_router(works.get_router(self.templates))
    self.app.include_router(rosout_logs.get_router(self.templates))
    self.app.include_router(runtime_logs.get_router(self.templates))
    self.app.include_router(audit_logs.get_router(self.templates))
    self.app.include_router(clients.get_router(self.templates))
    self.app.include_router(racks.get_router(self.templates))
    self.app.include_router(products.get_router(self.templates))
    self.app.include_router(carriers.get_router(self.templates))
    self.app.include_router(agvs.get_router(self.templates))
    self.app.include_router(devices.get_router(self.templates))
    self.app.include_router(signals.get_router(self.templates))
    self.app.include_router(auth.get_router(self.templates))
    self.app.include_router(users.get_router(self.templates))
```

### 認證機制
```python
# JWT Token 認證 (簡化版，永不過期)
class AuthManager:
    SECRET_KEY = "your-secret-key-change-this-in-production"
    ALGORITHM = "HS256"
    ACCESS_TOKEN_EXPIRE_MINUTES = None  # 永不過期

    def create_access_token(self, data: dict) -> str:
        """創建簡單的 access token（永不過期）"""
        to_encode = data.copy()
        token_data = json.dumps(to_encode)
        token_bytes = token_data.encode('utf-8')
        signature = hmac.new(SECRET_KEY.encode('utf-8'),
                           token_bytes, hashlib.sha256).hexdigest()
        token = base64.b64encode(token_bytes).decode('utf-8')
        return f"{token}.{signature}"

    def verify_token(self, token: str) -> Optional[TokenData]:
        """驗證 token"""
        try:
            token_part, signature = token.split('.')
            token_bytes = base64.b64decode(token_part.encode('utf-8'))

            # 驗證簽名
            expected_signature = hmac.new(SECRET_KEY.encode('utf-8'),
                                        token_bytes, hashlib.sha256).hexdigest()
            if not hmac.compare_digest(signature, expected_signature):
                return None

            # 解析 token 數據
            token_data = json.loads(token_bytes.decode('utf-8'))
            username = token_data.get("sub")
            if username is None:
                return None

            return TokenData(username=username)
        except Exception:
            return None

    def authenticate_user(self, username: str, password: str):
        """驗證用戶登入"""
        user = get_user_by_username(username)
        if not user:
            return False
        if not verify_password(password, user.password_hash):
            return False
        if not user.is_active:
            return False
        return user
```

## 📱 OPUI 服務 API (port 8002)

### 應用程式配置
```python
# OPUI 服務類別: OpUiServer
class OpUiServer:
    def __init__(self, host="0.0.0.0", port=8002):
        self.host = host
        self.port = port

        # 初始化 Socket.IO
        self.sio = socketio.AsyncServer(
            async_mode="asgi",
            cors_allowed_origins="*",
            engineio_logger=True,
            logger=True
        )

        # 建立 FastAPI 應用
        self.app = FastAPI(lifespan=lifespan)

        # 將 Socket.IO 整合進 ASGI App
        self.sio_app = socketio.ASGIApp(self.sio, self.app)

# 啟動配置
uvicorn.run(self.sio_app, host="0.0.0.0", port=8002)
```

### HTTP 路由配置
```python
def register_routes(self):
    """註冊 HTTP 路由和 API 端點"""

    @self.app.get("/home", response_class=HTMLResponse)
    async def home(request: Request):
        # 檢查設備授權
        device_id = request.query_params.get("deviceId")
        if not device_id:
            raise HTTPException(status_code=400, detail="缺少 deviceId 參數")

        auth_result = await check_device_authorization(device_id)
        if not auth_result["success"]:
            raise HTTPException(status_code=403, detail=auth_result["message"])

        return self.templates.TemplateResponse("home.html", {
            "request": request,
            "device_id": device_id
        })

    @self.app.get("/setting", response_class=HTMLResponse)
    async def setting(request: Request):
        # 設定頁面路由 (相同的設備授權檢查)

    @self.app.get("/rack", response_class=HTMLResponse)
    async def rack(request: Request):
        # 料架頁面路由 (相同的設備授權檢查)

    # 整合 API 路由器
    self.app.include_router(process_settings.router)
    self.app.include_router(product.router)
    self.app.include_router(license.router)
    self.app.include_router(agv.router, prefix="/api", tags=["agv", "tasks"])
```

### HTTP API 端點

#### 1. 頁面路由
```yaml
# 首頁
端點: GET /home
參數: deviceId (query parameter, 必填)
回應: HTML 頁面
授權: 設備授權檢查

# 設定頁面
端點: GET /setting
參數: deviceId (query parameter, 必填)
回應: HTML 頁面
授權: 設備授權檢查

# 料架頁面
端點: GET /rack
參數: deviceId (query parameter, 必填)
回應: HTML 頁面
授權: 設備授權檢查
```

#### 2. REST API 端點

##### 2.1 產品管理 API
```yaml
# 產品列表
端點: GET /products
回應: List[Product]

# 產品詳情
端點: GET /products/{product_id}
回應: Product

# 建立產品
端點: POST /products
請求體: { "name": "PRODUCT_A", "size": "S", "process_settings_id": 1 }
回應: Product
```

##### 2.2 授權管理 API
```yaml
# 授權列表
端點: GET /licenses
回應: List[License]

# 授權詳情
端點: GET /licenses/{license_id}
回應: License

# 建立授權
端點: POST /licenses
請求體: { "device_id": "device_001", "active": 1 }
回應: License

# 檢查授權
端點: GET /licenses/check/{device_id}
回應: { "success": true, "message": "授權有效" }
```

##### 2.3 AGV 和任務 API (prefix: /api)
```yaml
# AGV 列表
端點: GET /api/agvs
回應: List[AGV]

# AGV 詳情
端點: GET /api/agvs/{agv_id}
回應: AGV

# 建立 AGV
端點: POST /api/agvs
請求體: { "name": "AGV_001", "model": "K400", "x": 10.5, "y": 20.3, "heading": 90.0 }
回應: AGV

# 任務狀態查詢
端點: GET /api/tasks/{task_id}/status
回應: { "success": true, "task_id": 123, "status": 3, "status_name": "執行中" }
```

##### 2.4 製程設定 API
```yaml
# 製程設定列表
端點: GET /process_settings
回應: List[ProcessSettings]

# 製程設定詳情
端點: GET /process_settings/{settings_id}
回應: ProcessSettings

# 建立製程設定
端點: POST /process_settings
請求體: { "soaking_times": 3, "description": "標準製程" }
回應: ProcessSettings
```

### Socket.IO 事件處理架構
```python
class OpUiSocket:
    def __init__(self, sio):
        self.sio = sio
        self.user_sid_map = {}          # clientId -> sid 映射
        self.task_monitor = TaskMonitor()
        self.task_service = TaskService()

    def init_socketio(self):
        """初始化 Socket.IO 事件處理器"""
        self.sio.on('connect')(self.connect)
        self.sio.on('disconnect')(self.disconnect)
        self.sio.on('login')(self.login)
        self.sio.on('client_update')(self.client_update)
        self.sio.on('restore_client_by_id')(self.restore_client_by_id)
        self.sio.on('add_rack')(self.add_rack)
        self.sio.on('del_rack')(self.del_rack)
        self.sio.on('call_empty')(self.call_empty)
        self.sio.on('dispatch_full')(self.dispatch_full)
        self.sio.on('cancel_task')(self.cancel_task)
        self.sio.on('confirm_delivery')(self.confirm_delivery)
        self.sio.on('test_complete_task')(self.test_complete_task)
        self.sio.on('get_task_status')(self.get_task_status)
        self.sio.on('get_active_tasks')(self.get_active_tasks)
```

### Socket.IO 事件規格

#### 1. 連線管理事件

##### 1.1 客戶端連線 (connect)
```yaml
事件: connect
方向: 客戶端 → 伺服器 (自動觸發)
描述: 客戶端建立 Socket.IO 連線
處理邏輯:
  - 啟動任務監控服務
  - 從資料庫恢復進行中的任務
  - 同步活躍任務狀態給客戶端
  - 嘗試恢復客戶端資料
```

##### 1.2 客戶端登入 (login)
```yaml
事件: login
方向: 客戶端 → 伺服器
請求格式:
  {
    "deviceId": "device_001",         # 設備 ID (必填)
    "machineId": 1,                   # 機台 ID (必填)
    "userAgent": "Mozilla/5.0..."     # 瀏覽器資訊 (自動填入)
  }
回應格式:
  {
    "success": true,
    "message": "登入成功，clientId: device_001",
    "client": {
      "clientId": "device_001",
      "machineId": 1,
      "userAgent": "Mozilla/5.0...",
      "op": {
        "left": { "productSelected": 0, "products": [] },
        "right": { "productSelected": 0, "products": [] }
      }
    },
    "clientId": "device_001"
  }
處理邏輯:
  - 生成或使用現有 clientId
  - 建立或更新客戶端記錄
  - 建立 clientId -> sid 映射
  - 發送靜態資料 (產品、機台、房間、停車格列表)
```

##### 1.3 客戶端資料恢復 (restore_client_by_id)
```yaml
事件: restore_client_by_id
方向: 客戶端 → 伺服器
請求格式:
  {
    "clientId": "device_001"
  }
回應格式:
  {
    "success": true,
    "message": "恢復成功",
    "client": { /* 完整客戶端資料 */ }
  }
處理邏輯:
  - 從資料庫查詢客戶端資料
  - 恢復 clientId -> sid 映射
  - 發送靜態資料給客戶端
```

#### 2. 資料同步事件

##### 2.1 客戶端狀態更新 (client_update)
```yaml
事件: client_update
方向: 客戶端 → 伺服器
請求格式:
  {
    "clientId": "device_001",
    "machineId": 1,
    "userAgent": "Mozilla/5.0...",
    "op": {
      "left": {
        "productSelected": 0,
        "products": ["PRODUCT_A", "PRODUCT_B"]
      },
      "right": {
        "productSelected": 1,
        "products": ["PRODUCT_C", "PRODUCT_D"]
      }
    }
  }
回應格式:
  {
    "success": true,
    "message": "設定已儲存",
    "client": { /* 更新後的客戶端資料 */ },
    "clientId": "device_001"
  }
處理邏輯:
  - 向後相容性處理 (product → products 遷移)
  - 更新資料庫客戶端記錄
  - 建立或更新 clientId -> sid 映射
  - 機台變更時更新停車列表
```

#### 3. 伺服器廣播事件

##### 3.1 產品列表廣播 (product_list)
```yaml
事件: product_list
方向: 伺服器 → 客戶端
資料格式:
  {
    "products": [
      { "id": 1, "name": "PRODUCT_A", "size": "S" },
      { "id": 2, "name": "PRODUCT_B", "size": "L" }
    ]
  }
```

##### 3.2 機台列表廣播 (machine_list)
```yaml
事件: machine_list
方向: 伺服器 → 客戶端
資料格式:
  {
    "machines": [
      {
        "id": 1,
        "name": "機台_001",
        "parking_space_1": 101,
        "parking_space_2": 102,
        "parking_space_1_status": 0,    # 0-未佔用 1-佔用中 2-已送達
        "parking_space_2_status": 0
      }
    ]
  }
```

##### 3.3 停車格列表廣播 (parking_list)
```yaml
事件: parking_list
方向: 伺服器 → 客戶端
資料格式:
  {
    "left": [
      { "id": 1, "name": "RACK_001" },
      { "id": 2, "name": "RACK_002" }
    ],
    "right": [
      { "id": 3, "name": "RACK_003" }
    ]
  }
```

#### 4. AGV 操作事件

##### 4.1 叫空車 (call_empty)
```yaml
事件: call_empty
方向: 客戶端 → 伺服器
請求格式:
  {
    "side": "left"                    # 停車格側邊: "left" 或 "right"
  }
回應格式:
  {
    "success": true,
    "message": "叫車成功，任務 ID: 123"
  }
處理邏輯:
  - 驗證客戶端和機台資訊
  - 根據側邊獲取停車格 node_id
  - 建立 call_empty 任務
  - 更新機台停車格狀態為佔用中 (1)
  - 啟動任務監控
  - 廣播機台狀態更新
```

##### 4.2 派車 (dispatch_full)
```yaml
事件: dispatch_full
方向: 客戶端 → 伺服器
請求格式:
  {
    "side": "left",                   # 停車格側邊
    "rack": "RACK_001"                # 料架名稱
  }
回應格式:
  {
    "success": true,
    "message": "派車成功，任務 ID: 124"
  }
處理邏輯:
  - 驗證客戶端和機台資訊
  - 根據側邊獲取停車格 node_id
  - 建立 dispatch_full 任務
  - 更新機台停車格狀態為佔用中 (1)
  - 從停車格移除料架
  - 啟動任務監控
  - 廣播機台狀態和停車格列表更新
```

##### 4.3 取消任務 (cancel_task)
```yaml
事件: cancel_task
方向: 客戶端 → 伺服器
請求格式:
  {
    "side": "left"                    # 停車格側邊
  }
回應格式:
  {
    "success": true,
    "message": "已取消停車位 [101] 的任務"
  }
處理邏輯:
  - 驗證客戶端和機台資訊
  - 根據側邊獲取停車格 node_id
  - 刪除相關任務
  - 重設機台停車格狀態為未佔用 (0)
  - 廣播機台狀態和停車格列表更新
```

##### 4.4 確認送達 (confirm_delivery)
```yaml
事件: confirm_delivery
方向: 客戶端 → 伺服器
請求格式:
  {
    "side": "left"                    # 停車格側邊
  }
回應格式:
  {
    "success": true,
    "message": "已確認停車位 [101] 的rack架已搬移至作業區"
  }
處理邏輯:
  - 驗證客戶端和機台資訊
  - 根據側邊獲取停車格 node_id
  - 停止監聽該停車格的任務
  - 重設機台停車格狀態為未佔用 (0)
  - 推送任務狀態變更為 'confirmed'
  - 廣播機台狀態和停車格列表更新
```

#### 5. 料架管理事件

##### 5.1 新增料架 (add_rack)
```yaml
事件: add_rack
方向: 客戶端 → 伺服器
請求格式:
  {
    "rack": "RACK_001",               # 料架名稱 (支援 rack 或 rackName)
    "side": "left"                    # 停車格側邊
  }
回應格式:
  {
    "success": true,
    "message": "料架 RACK_001 已分配到左側停車格"
  }
處理邏輯:
  - 驗證客戶端和機台資訊
  - 根據側邊決定停車格位置
  - 查詢或建立料架記錄
  - 更新料架的 location_id
  - 廣播停車格列表更新
```

##### 5.2 刪除料架 (del_rack)
```yaml
事件: del_rack
方向: 客戶端 → 伺服器
請求格式:
  {
    "rackId": 1                       # 料架 ID
  }
回應格式:
  {
    "success": true,
    "message": "料架已刪除"
  }
處理邏輯:
  - 驗證客戶端和機台資訊
  - 刪除料架記錄
  - 廣播停車格列表更新
```

#### 6. 任務狀態事件

##### 6.1 活躍任務狀態同步 (active_tasks)
```yaml
事件: active_tasks
方向: 伺服器 → 客戶端
資料格式:
  {
    "left": {
      "task_id": 123,
      "task_type": "call_empty",
      "node_id": 101,
      "status": "pending",            # pending, delivered, confirmed
      "createdAt": 1642406400000
    },
    "right": {
      "task_id": 124,
      "task_type": "dispatch_full",
      "node_id": 102,
      "status": "pending",
      "createdAt": 1642406500000
    }
  }
觸發時機:
  - 客戶端連線時
  - 任務狀態變更時
```

##### 6.2 任務狀態變更通知 (task_status_update)
```yaml
事件: task_status_update
方向: 伺服器 → 客戶端
資料格式:
  {
    "task_id": 123,
    "side": "left",
    "type": "call_empty",
    "status": "delivered",            # pending, delivered, confirmed, completed
    "status_name": "已送達",
    "status_description": "任務已完成，等待確認",
    "machine_id": 1,
    "node_id": 101,
    "updatedAt": "2025-01-17T10:30:00Z"
  }
觸發時機:
  - 任務完成時 (delivered)
  - 確認送達時 (confirmed)
  - 派車完成時 (completed)
```

##### 6.3 任務查詢事件
```yaml
# 查詢任務狀態
事件: get_task_status
請求格式: { "taskId": 123 }

# 查詢活躍任務
事件: get_active_tasks
請求格式: {}

# 測試完成任務
事件: test_complete_task
請求格式: { "taskId": 123 }
```

#### 7. 通知事件

##### 7.1 通知訊息 (notify_message)
```yaml
事件: notify_message
方向: 伺服器 → 客戶端
資料格式:
  {
    "message": "AGV已將rack架送達停車位[101]"
  }
```

##### 7.2 錯誤訊息 (error_message)
```yaml
事件: error_message
方向: 伺服器 → 客戶端
資料格式:
  {
    "message": "找不到客戶端資訊"
  }
```

## 🔐 安全和認證

### 認證流程
```mermaid
sequenceDiagram
    participant C as 客戶端
    participant A as API 服務
    participant D as 資料庫

    C->>A: POST /auth/login (username, password)
    A->>D: 驗證使用者憑證
    D-->>A: 使用者資訊
    A-->>C: JWT Token

    C->>A: API 請求 (Bearer Token)
    A->>A: 驗證 JWT Token
    A-->>C: API 回應
```

### 中介軟體配置
```python
# CORS 中介軟體
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 認證中介軟體
@app.middleware("http")
async def auth_middleware(request: Request, call_next):
    # JWT Token 驗證邏輯
```

## 🧪 API 測試範例

### 1. PLC 資料讀取測試
```bash
# 讀取 DM 資料
curl -X GET "http://localhost:8000/plc/get_data/DM/7600" \
  -H "accept: application/json"

# 預期回應
{
  "key": "7600",
  "value": "123"
}
```

### 2. 門控制測試
```bash
# 開啟門
curl -X POST "http://localhost:8000/door/control" \
  -H "Content-Type: application/json" \
  -d '{"doorId": "DOOR_01", "isOpen": true}'
```

### 3. KUKA 任務狀態回報測試
```bash
# 任務狀態更新
curl -X POST "http://localhost:8000/interfaces/api/amr/missionStateCallback" \
  -H "Content-Type: application/json" \
  -d '{
    "missionCode": "TASK_001",
    "missionStatus": "COMPLETED",
    "robotId": "AGV_01",
    "currentPosition": "STATION_A"
  }'
```

### 4. KUKA Fleet API 測試
```bash
# KUKA Fleet 登入測試 (RosAGV → KUKA Fleet)
curl -X POST "http://192.168.11.206:10870/api/login" \
  -H "Content-Type: application/json" \
  -d '{
    "username": "admin",
    "password": "Admin"
  }'

# 預期回應
{
  "success": true,
  "data": {
    "token": "Bearer_token_string"
  },
  "message": "Login successful"
}

# 機器人狀態查詢測試 (RosAGV → KUKA Fleet)
curl -X POST "http://192.168.11.206:10870/api/amr/robotQuery" \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer_token_string" \
  -d '{}'

# 任務提交測試 (RosAGV → KUKA Fleet)
curl -X POST "http://192.168.11.206:10870/api/amr/submitMission" \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer_token_string" \
  -d '{
    "orgId": "Ching-Tech",
    "requestId": "REQ_001",
    "missionCode": "MISSION_001",
    "missionType": "MOVE",
    "robotModels": ["KMP 400i diffDrive"],
    "robotIds": [1],
    "robotType": "LIFT",
    "priority": 1,
    "missionData": [
      {
        "sequence": 1,
        "position": "test-test1-1",
        "type": "NODE_POINT",
        "passStrategy": "AUTO"
      }
    ]
  }'

# 任務取消測試 (RosAGV → KUKA Fleet)
curl -X POST "http://192.168.11.206:10870/api/amr/missionCancel" \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer_token_string" \
  -d '{
    "missionCode": "MISSION_001",
    "cancelMode": "FORCE",
    "reason": "測試取消"
  }'

# 任務狀態回調測試 (KUKA Fleet → RosAGV)
# 注意：這是 KUKA Fleet 系統向我們的系統發送的回調，不是我們主動調用的
# 測試時可以模擬 KUKA Fleet 系統發送回調到我們的 Web API
curl -X POST "http://localhost:8000/interfaces/api/amr/missionStateCallback" \
  -H "Content-Type: application/json" \
  -d '{
    "missionCode": "MISSION_001",
    "missionStatus": "COMPLETED",
    "robotId": "1",
    "currentPosition": "test-test1-5",
    "containerCode": "CONTAINER_001",
    "message": "任務完成"
  }'
```

### 5. KUKA Fleet Adapter 測試
```bash
# 測試 KUKA Fleet Adapter 節點
ros2 run kuka_fleet_adapter kuka_fleet_adapter

# 檢查節點狀態
ros2 node list | grep kuka_fleet_adapter
ros2 node info /kuka_fleet_adapter

# 檢查參數設定
ros2 param list /kuka_fleet_adapter
ros2 param get /kuka_fleet_adapter base_url
ros2 param get /kuka_fleet_adapter username
```

### 6. OPUI Socket.IO 測試
```javascript
// 建立 Socket.IO 連線測試
const socket = io('http://localhost:8002');

// 連線事件測試
socket.on('connect', () => {
    console.log('✅ Socket.IO 連線成功');

    // 登入測試
    socket.emit('login', {
        deviceId: 'test_device_001',
        machineId: 1,
        userAgent: navigator.userAgent
    }, (response) => {
        console.log('登入回應:', response);
    });
});

// 客戶端狀態更新測試
socket.emit('client_update', {
    clientId: 'test_device_001',
    machineId: 1,
    userAgent: navigator.userAgent,
    op: {
        left: {
            productSelected: 0,
            products: ['PRODUCT_A', 'PRODUCT_B']
        },
        right: {
            productSelected: 1,
            products: ['PRODUCT_C']
        }
    }
}, (response) => {
    console.log('狀態更新回應:', response);
});

// 叫空車測試
socket.emit('call_empty', {
    side: 'left'
}, (response) => {
    console.log('叫車回應:', response);
});

// 派車測試
socket.emit('dispatch_full', {
    side: 'right',
    rack: 'RACK_001'
}, (response) => {
    console.log('派車回應:', response);
});

// 監聽伺服器廣播事件
socket.on('machine_list', (data) => {
    console.log('📥 收到機台列表:', data);
});

socket.on('parking_list', (data) => {
    console.log('📥 收到停車格列表:', data);
});

socket.on('active_tasks', (data) => {
    console.log('📥 收到活躍任務:', data);
});

socket.on('task_status_update', (data) => {
    console.log('📥 收到任務狀態更新:', data);
});

socket.on('notify_message', (data) => {
    console.log('📥 收到通知:', data.message);
});

socket.on('error_message', (data) => {
    console.log('❌ 收到錯誤:', data.message);
});
```

### 7. Socket.IO 連線測試 (Node.js)
```javascript
// 使用 socket.io-client 進行測試
const io = require('socket.io-client');
const socket = io('http://localhost:8002');

socket.on('connect', () => {
    console.log('✅ 連線成功');

    // 測試登入
    socket.emit('login', {
        deviceId: 'test_device_001',
        machineId: 1,
        userAgent: 'Node.js Test Client'
    });
});

socket.on('product_list', (data) => {
    console.log('產品列表:', data.products);
});

socket.on('machine_list', (data) => {
    console.log('機台列表:', data.machines);
});

// 測試料架管理
setTimeout(() => {
    socket.emit('add_rack', {
        rack: 'TEST_RACK_001',
        side: 'left'
    });
}, 2000);

// 測試叫車
setTimeout(() => {
    socket.emit('call_empty', {
        side: 'left'
    });
}, 4000);
```

### 8. AGVCUI Socket.IO 測試
```javascript
// 建立 AGVCUI Socket.IO 連線測試
const socket = io('http://localhost:8001');

// 連線事件測試
socket.on('connect', () => {
    console.log('✅ AGVCUI Socket.IO 連線成功');

    // 使用者登入測試
    socket.emit('user_login', {
        username: 'admin',
        password: 'password'
    }, (response) => {
        console.log('登入回應:', response);
        if (response.success) {
            console.log('🔑 Access Token:', response.access_token);
        }
    });
});

// 監聽系統資料廣播事件
socket.on('map_info', (data) => {
    console.log('🗺️ 收到地圖資訊:');
    console.log('  - CT 節點數量:', data.nodes.length);
    console.log('  - CT 邊數量:', data.edges.length);
    console.log('  - KUKA 節點數量:', data.kukaNodes.length);
    console.log('  - AGV 數量:', data.agvs.length);
});

socket.on('agv_list', (data) => {
    console.log('🚗 收到 AGV 列表:', data.agvs.length, '台 AGV');
    data.agvs.forEach(agv => {
        console.log(`  - ${agv.name} (${agv.model}): 位置(${agv.x}, ${agv.y}), 電量: ${agv.battery_level}%`);
    });
});

socket.on('task_list', (data) => {
    console.log('📋 收到任務列表:', data.tasks.length, '個任務');
    data.tasks.forEach(task => {
        console.log(`  - 任務 ${task.id}: ${task.name}, 狀態: ${task.status_id}, AGV: ${task.agv_id}`);
    });
});

socket.on('rack_list', (data) => {
    console.log('📦 收到料架列表:', data.racks.length, '個料架');
    data.racks.forEach(rack => {
        console.log(`  - ${rack.name}: 產品 ${rack.product_name}, 數量: ${rack.count}/${rack.total}`);
    });
});

socket.on('carrier_list', (data) => {
    console.log('🚛 收到載具列表:', data.carriers.length, '個載具');
});

socket.on('machine_list', (data) => {
    console.log('🏭 收到機台列表:', data.machines.length, '台機台');
});

socket.on('room_list', (data) => {
    console.log('🏢 收到房間列表:', data.rooms.length, '個房間');
});

socket.on('signal_list', (data) => {
    console.log('📡 收到信號列表:', data.signals.length, '個信號');
});

socket.on('location_list', (data) => {
    console.log('📍 收到位置列表:', data.locations.length, '個位置');
});

socket.on('notify_message', (data) => {
    console.log('📥 收到通知:', data.message);
});

socket.on('error_message', (data) => {
    console.log('❌ 收到錯誤:', data.message);
});

// 使用者登出測試
setTimeout(() => {
    socket.emit('user_logout', {}, (response) => {
        console.log('登出回應:', response);
    });
}, 10000);
```

### 9. AGVCUI Socket.IO 連線測試 (Node.js)
```javascript
// 使用 socket.io-client 進行 AGVCUI 測試
const io = require('socket.io-client');
const socket = io('http://localhost:8001');

let dataReceived = {
    map_info: false,
    agv_list: false,
    task_list: false,
    rack_list: false
};

socket.on('connect', () => {
    console.log('✅ AGVCUI 連線成功');

    // 測試登入
    socket.emit('user_login', {
        username: 'admin',
        password: 'password'
    });
});

// 監控資料接收狀態
socket.on('map_info', (data) => {
    dataReceived.map_info = true;
    console.log('📊 地圖資料統計:');
    console.log(`  - CT: ${data.nodes.length} 節點, ${data.edges.length} 邊`);
    console.log(`  - KUKA: ${data.kukaNodes.length} 節點, ${data.kukaEdges.length} 邊`);
    console.log(`  - AGV: ${data.agvs.length} 台`);
});

socket.on('agv_list', (data) => {
    dataReceived.agv_list = true;
    console.log('🚗 AGV 狀態摘要:');
    const statusCount = {};
    data.agvs.forEach(agv => {
        statusCount[agv.status_id] = (statusCount[agv.status_id] || 0) + 1;
    });
    console.log('  狀態分布:', statusCount);
});

socket.on('task_list', (data) => {
    dataReceived.task_list = true;
    console.log('📋 任務狀態摘要:');
    const statusCount = {};
    data.tasks.forEach(task => {
        statusCount[task.status_id] = (statusCount[task.status_id] || 0) + 1;
    });
    console.log('  狀態分布:', statusCount);
});

socket.on('rack_list', (data) => {
    dataReceived.rack_list = true;
    console.log('📦 料架統計:');
    const sizeCount = { S: 0, L: 0 };
    data.racks.forEach(rack => {
        sizeCount[rack.size] = (sizeCount[rack.size] || 0) + 1;
    });
    console.log('  尺寸分布:', sizeCount);
});

// 檢查資料接收完整性
setTimeout(() => {
    console.log('📊 資料接收狀態檢查:');
    Object.entries(dataReceived).forEach(([key, received]) => {
        console.log(`  ${key}: ${received ? '✅' : '❌'}`);
    });
}, 5000);
```

### 10. 地圖上傳測試
```bash
# 上傳 KUKA 地圖
curl -X POST "http://localhost:8000/map_importer/upload-kuka-map/" \
  -H "accept: application/json" \
  -H "Content-Type: multipart/form-data" \
  -F "file=@kuka_map.json"
```

## 📊 效能和監控

### 效能指標
| 指標 | 目標值 | 監控方式 |
|------|--------|----------|
| API 回應時間 | < 100ms | FastAPI 內建監控 |
| 資料庫查詢時間 | < 50ms | SQLModel 查詢日誌 |
| 並發連線數 | > 100 | uvicorn 統計 |
| Socket.IO 連線 | > 50 | Socket.IO 監控 |

### 日誌配置
```python
# 日誌設定
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
```

## 📝 相關文檔

- [資料庫結構設計](./database-schema.md)
- [ROS 2 介面規格](./ros2-interfaces.md)
- [容器架構說明](../architecture/container-architecture.md)
- [功能需求規格](../requirements/functional-requirements.md)

---

**最後更新**: 2025-01-17
**維護責任**: Web API 開發工程師
**版本**: v1.0.0
