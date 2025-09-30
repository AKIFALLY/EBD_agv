# AI Agent Web API 開發原則

## Web 服務架構
- **API 服務**: Port 8000 (FastAPI + Socket.IO)
- **AGVCUI**: Port 8001 (管理員界面)
- **OPUI**: Port 8002 (操作員界面)
- **反向代理**: Nginx (Port 80)

## 服務管理
```bash
# 容器內管理（載入 setup.bash 後）
manage_web_api_launch start     # 啟動服務群組
manage_web_api_launch stop      # 停止服務
manage_web_api_launch restart   # 重啟服務
manage_web_api_launch status    # 檢查狀態

# 自動啟動控制
AUTO_START_WEB_API_LAUNCH=true   # 啟用
AUTO_START_WEB_API_LAUNCH=false  # 停用（測試用）
```

## FastAPI 基本架構
```python
from fastapi import FastAPI, Depends
from fastapi.middleware.cors import CORSMiddleware
import socketio

# 建立應用
app = FastAPI(title="RosAGV API", version="1.0.0")

# CORS 設定
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"]
)

# Socket.IO 整合
sio = socketio.AsyncServer(async_mode='asgi', cors_allowed_origins="*")
socket_app = socketio.ASGIApp(sio, app)

# 健康檢查
@app.get("/health")
async def health():
    return {"status": "healthy"}
```

## API 路由組織
```python
# routers/agv.py
from fastapi import APIRouter, HTTPException

router = APIRouter(prefix="/api/v1/agvs", tags=["AGV"])

@router.get("/")
async def get_agvs():
    return {"agvs": []}

@router.get("/{agv_id}")
async def get_agv(agv_id: int):
    return {"id": agv_id}

@router.post("/")
async def create_agv(data: dict):
    return {"created": True}

# main.py 註冊路由
app.include_router(router)
```

## Socket.IO 即時通訊
```python
# 伺服器端
@sio.on('connect')
async def connect(sid, environ):
    print(f"Client {sid} connected")

@sio.on('agv_update')
async def handle_agv_update(sid, data):
    await sio.emit('agv_status', data, room='all')

@sio.on('disconnect')
async def disconnect(sid):
    print(f"Client {sid} disconnected")
```

```javascript
// 客戶端
const socket = io('http://localhost:8000');

socket.on('connect', () => {
    console.log('Connected');
});

socket.emit('agv_update', {id: 1, status: 'idle'});

socket.on('agv_status', (data) => {
    console.log('Status:', data);
});
```

## 資料驗證 (Pydantic/SQLModel)
```python
from sqlmodel import SQLModel, Field
from typing import Optional
from enum import Enum

class AGVStatus(str, Enum):
    IDLE = "idle"
    MOVING = "moving"
    CHARGING = "charging"

class AGVBase(SQLModel):
    name: str = Field(max_length=50)
    status: AGVStatus = Field(default=AGVStatus.IDLE)
    battery: float = Field(ge=0.0, le=1.0)

class AGVCreate(AGVBase):
    pass

class AGVResponse(AGVBase):
    id: int
```

## 開發工作流程
```bash
# 1. 修改代碼
vim /app/web_api_ws/src/web_api/main.py

# 2. 重建工作空間
cd /app/web_api_ws
colcon build --packages-select web_api

# 3. 重啟服務
manage_web_api_launch restart

# 4. 測試 API
curl http://localhost:8000/health
```

## 測試端點
```bash
# Health check
curl http://localhost:8000/health

# API 測試
curl http://localhost:8000/api/v1/agvs
curl -X POST http://localhost:8000/api/v1/agvs \
  -H "Content-Type: application/json" \
  -d '{"name":"AGV001","status":"idle"}'

# 界面訪問
http://localhost:8001  # AGVCUI
http://localhost:8002  # OPUI
```

## Nginx 配置
```nginx
# 虛擬主機配置
server {
    server_name agvc.webapi;
    location / {
        proxy_pass http://192.168.100.100:8000;
    }
}

server {
    server_name agvc.ui;
    location / {
        proxy_pass http://192.168.100.100:8001;
    }
}
```

## 常見問題解決

### 服務無回應
```bash
# 檢查進程
ps aux | grep -E "api_server|agvcui|opui"

# 重啟服務
manage_web_api_launch restart

# 檢查日誌
docker compose -f docker-compose.agvc.yml logs agvc_server
```

### 端口衝突
```bash
# 檢查端口
ss -tulpn | grep -E "(8000|8001|8002)"

# 停止衝突服務
sudo kill -9 <PID>
```

### Socket.IO 連接失敗
```javascript
// 檢查 CORS 設定
// 確保伺服器允許客戶端來源
sio = socketio.AsyncServer(
    cors_allowed_origins="*"  // 或指定來源
)
```

## 關鍵規則
1. **統一管理**: 使用 manage_web_api_launch 管理服務
2. **端口分配**: 8000=API, 8001=AGVCUI, 8002=OPUI
3. **CORS 設定**: 開發環境可用 "*"，生產需指定
4. **資料驗證**: 使用 Pydantic/SQLModel
5. **重啟優先**: 修改後用 restart 而非 stop+start