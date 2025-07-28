# Web 開發操作指導

## 🎯 適用場景
- FastAPI Web API 開發
- Socket.IO 即時通訊實作
- 前端界面開發 (AGVCUI, OPUI)
- Web 服務部署和最佳化

## 📋 RosAGV Web 技術棧

### 後端技術
- **FastAPI**: 高效能 Web 框架
- **Socket.IO**: 即時雙向通訊
- **SQLModel**: 現代 ORM 框架
- **Pydantic**: 資料驗證和序列化
- **Uvicorn**: ASGI 伺服器

### 前端技術
- **HTML5 + CSS3**: 現代 Web 標準
- **JavaScript ES6+**: 現代 JavaScript
- **Socket.IO Client**: 即時通訊客戶端
- **Bulma CSS**: 響應式 CSS 框架
- **Chart.js**: 資料視覺化

### 部署技術
- **Nginx**: 反向代理和靜態檔案服務
- **Docker**: 容器化部署
- **PostgreSQL**: 資料持久化

## 🚀 FastAPI 開發

### 基本應用結構
```python
# main.py
from fastapi import FastAPI, Depends, HTTPException
from fastapi.middleware.cors import CORSMiddleware
import socketio

# 建立 FastAPI 應用
app = FastAPI(
    title="RosAGV API",
    description="AGV 車隊管理系統 API",
    version="1.0.0"
)

# CORS 設定
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Socket.IO 整合
sio = socketio.AsyncServer(
    async_mode='asgi',
    cors_allowed_origins="*"
)
socket_app = socketio.ASGIApp(sio, app)

# 健康檢查端點
@app.get("/health")
async def health_check():
    return {"status": "healthy", "timestamp": datetime.utcnow()}
```

### 路由組織
```python
# routers/agv.py
from fastapi import APIRouter, Depends, HTTPException
from sqlmodel import Session
from typing import List

router = APIRouter(prefix="/api/v1/agvs", tags=["AGV"])

@router.get("/", response_model=List[AGVResponse])
async def get_agvs(
    skip: int = 0,
    limit: int = 100,
    session: Session = Depends(get_session)
):
    agvs = await get_agvs_from_db(session, skip, limit)
    return agvs

@router.post("/", response_model=AGVResponse)
async def create_agv(
    agv_data: AGVCreate,
    session: Session = Depends(get_session)
):
    agv = await create_agv_in_db(session, agv_data)
    return agv

@router.get("/{agv_id}", response_model=AGVResponse)
async def get_agv(
    agv_id: int,
    session: Session = Depends(get_session)
):
    agv = await get_agv_from_db(session, agv_id)
    if not agv:
        raise HTTPException(status_code=404, detail="AGV not found")
    return agv
```

### 資料模型
```python
# models/agv.py
from sqlmodel import SQLModel, Field
from typing import Optional
from datetime import datetime
from enum import Enum

class AGVStatus(str, Enum):
    IDLE = "idle"
    MOVING = "moving"
    LOADING = "loading"
    UNLOADING = "unloading"
    CHARGING = "charging"
    ERROR = "error"

class AGVBase(SQLModel):
    name: str = Field(max_length=50)
    agv_type: str = Field(max_length=20)
    status: AGVStatus = Field(default=AGVStatus.IDLE)
    battery_level: float = Field(ge=0.0, le=1.0)
    current_location: Optional[str] = None

class AGV(AGVBase, table=True):
    __tablename__ = "agvs"
    
    id: Optional[int] = Field(default=None, primary_key=True)
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: Optional[datetime] = None

class AGVCreate(AGVBase):
    pass

class AGVResponse(AGVBase):
    id: int
    created_at: datetime
    updated_at: Optional[datetime]
```

## 🔌 Socket.IO 即時通訊

### 伺服器端實作
```python
# websocket/events.py
import socketio
from typing import Dict, Any

sio = socketio.AsyncServer(async_mode='asgi')

# 連接事件
@sio.event
async def connect(sid, environ):
    print(f"Client {sid} connected")
    await sio.emit('connection_response', {'status': 'connected'}, room=sid)

# 斷線事件
@sio.event
async def disconnect(sid):
    print(f"Client {sid} disconnected")

# AGV 狀態訂閱
@sio.event
async def subscribe_agv_status(sid, data):
    agv_id = data.get('agv_id')
    if agv_id:
        await sio.enter_room(sid, f"agv_{agv_id}")
        await sio.emit('subscription_confirmed', {
            'agv_id': agv_id,
            'status': 'subscribed'
        }, room=sid)

# 廣播 AGV 狀態更新
async def broadcast_agv_status(agv_id: int, status_data: Dict[str, Any]):
    await sio.emit('agv_status_update', {
        'agv_id': agv_id,
        'data': status_data
    }, room=f"agv_{agv_id}")

# 系統通知
async def broadcast_system_notification(message: str, level: str = "info"):
    await sio.emit('system_notification', {
        'message': message,
        'level': level,
        'timestamp': datetime.utcnow().isoformat()
    })
```

### 客戶端實作
```javascript
// static/js/socket-client.js
class SocketClient {
    constructor(serverUrl = '') {
        this.socket = io(serverUrl);
        this.setupEventHandlers();
    }

    setupEventHandlers() {
        this.socket.on('connect', () => {
            console.log('Connected to server');
            this.onConnected();
        });

        this.socket.on('disconnect', () => {
            console.log('Disconnected from server');
            this.onDisconnected();
        });

        this.socket.on('agv_status_update', (data) => {
            this.handleAGVStatusUpdate(data);
        });

        this.socket.on('system_notification', (data) => {
            this.handleSystemNotification(data);
        });
    }

    subscribeToAGV(agvId) {
        this.socket.emit('subscribe_agv_status', { agv_id: agvId });
    }

    handleAGVStatusUpdate(data) {
        const { agv_id, data: statusData } = data;
        // 更新 UI 中的 AGV 狀態
        this.updateAGVDisplay(agv_id, statusData);
    }

    handleSystemNotification(data) {
        const { message, level, timestamp } = data;
        // 顯示系統通知
        this.showNotification(message, level);
    }

    updateAGVDisplay(agvId, statusData) {
        const agvElement = document.getElementById(`agv-${agvId}`);
        if (agvElement) {
            agvElement.querySelector('.status').textContent = statusData.status;
            agvElement.querySelector('.battery').textContent = `${Math.round(statusData.battery_level * 100)}%`;
        }
    }

    showNotification(message, level) {
        // 使用 Bulma 通知組件
        const notification = document.createElement('div');
        notification.className = `notification is-${level}`;
        notification.innerHTML = `
            <button class="delete"></button>
            ${message}
        `;
        document.querySelector('.notifications-container').appendChild(notification);
        
        // 自動移除通知
        setTimeout(() => {
            notification.remove();
        }, 5000);
    }
}

// 初始化 Socket 客戶端
const socketClient = new SocketClient();
```

## 🎨 前端開發

### AGVCUI 管理員界面
```html
<!-- templates/agvcui/dashboard.html -->
<!DOCTYPE html>
<html lang="zh-TW">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>AGVC 管理台</title>
    <link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/bulma@0.9.4/css/bulma.min.css">
    <link rel="stylesheet" href="/static/css/agvcui.css">
</head>
<body>
    <nav class="navbar is-primary">
        <div class="navbar-brand">
            <a class="navbar-item">
                <strong>AGVC 管理台</strong>
            </a>
        </div>
        <div class="navbar-menu">
            <div class="navbar-start">
                <a class="navbar-item" href="/dashboard">儀表板</a>
                <a class="navbar-item" href="/agvs">AGV 管理</a>
                <a class="navbar-item" href="/tasks">任務管理</a>
                <a class="navbar-item" href="/settings">系統設定</a>
            </div>
        </div>
    </nav>

    <section class="section">
        <div class="container">
            <div class="columns">
                <div class="column is-3">
                    <div class="card">
                        <div class="card-header">
                            <p class="card-header-title">系統狀態</p>
                        </div>
                        <div class="card-content">
                            <div class="content">
                                <p>線上 AGV: <span id="online-agvs">0</span></p>
                                <p>執行中任務: <span id="active-tasks">0</span></p>
                                <p>系統狀態: <span id="system-status" class="tag is-success">正常</span></p>
                            </div>
                        </div>
                    </div>
                </div>
                
                <div class="column is-9">
                    <div class="card">
                        <div class="card-header">
                            <p class="card-header-title">AGV 狀態監控</p>
                        </div>
                        <div class="card-content">
                            <div id="agv-grid" class="columns is-multiline">
                                <!-- AGV 狀態卡片將動態載入 -->
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        </div>
    </section>

    <div class="notifications-container"></div>

    <script src="/socket.io/socket.io.js"></script>
    <script src="/static/js/socket-client.js"></script>
    <script src="/static/js/agvcui.js"></script>
</body>
</html>
```

### OPUI 操作員界面
```javascript
// static/js/opui.js
class OPUIManager {
    constructor() {
        this.socketClient = new SocketClient();
        this.currentMachine = null;
        this.setupEventHandlers();
        this.loadMachineInfo();
    }

    setupEventHandlers() {
        // 叫空車按鈕
        document.getElementById('call-empty-agv').addEventListener('click', () => {
            this.callEmptyAGV();
        });

        // 派車按鈕
        document.getElementById('dispatch-agv').addEventListener('click', () => {
            this.dispatchAGV();
        });

        // 取消任務按鈕
        document.getElementById('cancel-task').addEventListener('click', () => {
            this.cancelCurrentTask();
        });
    }

    async callEmptyAGV() {
        try {
            const response = await fetch('/api/v1/tasks/call-empty-agv', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({
                    machine_id: this.currentMachine.id,
                    priority: 5
                })
            });

            const result = await response.json();
            if (result.success) {
                this.showNotification('空車呼叫成功', 'success');
                this.updateTaskStatus(result.task_id, 'pending');
            } else {
                this.showNotification('空車呼叫失敗: ' + result.message, 'danger');
            }
        } catch (error) {
            this.showNotification('網路錯誤: ' + error.message, 'danger');
        }
    }

    async dispatchAGV() {
        const rackName = document.getElementById('rack-input').value;
        if (!rackName) {
            this.showNotification('請輸入料架名稱', 'warning');
            return;
        }

        try {
            const response = await fetch('/api/v1/tasks/dispatch-agv', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({
                    machine_id: this.currentMachine.id,
                    rack_name: rackName,
                    priority: 5
                })
            });

            const result = await response.json();
            if (result.success) {
                this.showNotification('派車成功', 'success');
                this.updateTaskStatus(result.task_id, 'dispatched');
                document.getElementById('rack-input').value = '';
            } else {
                this.showNotification('派車失敗: ' + result.message, 'danger');
            }
        } catch (error) {
            this.showNotification('網路錯誤: ' + error.message, 'danger');
        }
    }

    updateTaskStatus(taskId, status) {
        const statusElement = document.getElementById('current-task-status');
        const taskIdElement = document.getElementById('current-task-id');
        
        if (statusElement) {
            statusElement.textContent = status;
            statusElement.className = `tag is-${this.getStatusColor(status)}`;
        }
        
        if (taskIdElement) {
            taskIdElement.textContent = taskId || '無';
        }
    }

    getStatusColor(status) {
        const colorMap = {
            'pending': 'warning',
            'dispatched': 'info',
            'in_progress': 'primary',
            'completed': 'success',
            'failed': 'danger',
            'cancelled': 'light'
        };
        return colorMap[status] || 'light';
    }

    showNotification(message, type) {
        const notification = document.createElement('div');
        notification.className = `notification is-${type}`;
        notification.innerHTML = `
            <button class="delete"></button>
            ${message}
        `;
        
        document.querySelector('.notifications-container').appendChild(notification);
        
        // 刪除按鈕事件
        notification.querySelector('.delete').addEventListener('click', () => {
            notification.remove();
        });
        
        // 自動移除
        setTimeout(() => {
            if (notification.parentNode) {
                notification.remove();
            }
        }, 5000);
    }
}

// 初始化 OPUI
document.addEventListener('DOMContentLoaded', () => {
    new OPUIManager();
});
```

## 🔧 開發工具和最佳實踐

### API 文檔
```python
# 自動生成 API 文檔
@app.get("/docs")
async def get_docs():
    return get_swagger_ui_html(openapi_url="/openapi.json", title="RosAGV API")

@app.get("/redoc")
async def get_redoc():
    return get_redoc_html(openapi_url="/openapi.json", title="RosAGV API")
```

### 錯誤處理
```python
# 全域錯誤處理
@app.exception_handler(HTTPException)
async def http_exception_handler(request, exc):
    return JSONResponse(
        status_code=exc.status_code,
        content={
            "error": True,
            "message": exc.detail,
            "status_code": exc.status_code
        }
    )

@app.exception_handler(ValidationError)
async def validation_exception_handler(request, exc):
    return JSONResponse(
        status_code=422,
        content={
            "error": True,
            "message": "Validation error",
            "details": exc.errors()
        }
    )
```

### 測試
```python
# 測試用例
import pytest
from fastapi.testclient import TestClient
from main import app

client = TestClient(app)

def test_health_check():
    response = client.get("/health")
    assert response.status_code == 200
    assert response.json()["status"] == "healthy"

def test_create_agv():
    agv_data = {
        "name": "AGV001",
        "agv_type": "loader",
        "battery_level": 0.85
    }
    response = client.post("/api/v1/agvs/", json=agv_data)
    assert response.status_code == 200
    assert response.json()["name"] == "AGV001"
```

## 🔗 交叉引用
- 資料庫操作: @docs-ai/operations/development/database-operations.md
- AGVC 工作空間: @docs-ai/context/workspaces/agvc-workspaces.md
- 系統診斷: @docs-ai/operations/maintenance/system-diagnostics.md
- 技術棧: @docs-ai/context/system/technology-stack.md
