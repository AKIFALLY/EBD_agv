# AGVCUI 專案重建指南

## 📋 專案概述

AGVCUI 是一個基於 ROS 2 的自動導引車（AGV）控制與監控系統的 Web 介面，專門設計用於工業自動化環境中的物料裝載作業。該系統採用現代化的 Web 技術棧，提供即時監控、任務管理、地圖視覺化和設備控制功能。

### 🎯 核心功能清單

1. **AGV 控制與監控**
   - 即時 AGV 位置追蹤和狀態監控
   - AGV 路徑規劃和導航控制
   - 多 AGV 協調管理

2. **任務管理系統**
   - 任務創建、分配和執行監控
   - 任務狀態即時更新
   - 任務歷史記錄和分析

3. **地圖視覺化**
   - 互動式工廠地圖顯示
   - 即時 AGV 位置和路徑顯示
   - 設備狀態視覺化

4. **設備管理**
   - 工廠設備狀態監控
   - 設備配置和參數管理
   - 故障診斷和報警

5. **載具和貨架管理**
   - 載具位置追蹤
   - 貨架狀態管理
   - 庫存管理

6. **用戶權限系統**
   - 多級用戶權限控制
   - 安全認證機制
   - 操作日誌記錄

7. **即時通訊**
   - Socket.IO 雙向通訊
   - 即時狀態更新
   - 事件驅動架構

## 🏗️ 技術架構說明

### 後端架構

#### 核心技術棧
- **框架**: FastAPI (Python)
- **即時通訊**: Socket.IO
- **資料庫**: PostgreSQL
- **ORM**: SQLAlchemy
- **認證**: JWT + HttpOnly Cookies
- **ROS 整合**: ROS 2 (Humble/Foxy)

#### 架構層次
```
後端架構/
├── agvc_ui_server.py          # FastAPI 主伺服器
├── agvc_ui_socket.py          # Socket.IO 事件處理
├── auth.py                    # 認證和權限管理
├── middleware.py              # 中間件（認證、CORS）
├── routers/                   # API 路由模組
│   ├── auth.py               # 認證路由
│   ├── map.py                # 地圖相關 API
│   ├── tasks.py              # 任務管理 API
│   ├── devices.py            # 設備管理 API
│   ├── agvs.py               # AGV 管理 API
│   ├── carriers.py           # 載具管理 API
│   ├── racks.py              # 貨架管理 API
│   └── users.py              # 用戶管理 API
├── database/                  # 資料庫操作層
│   ├── connection.py         # 資料庫連線管理
│   ├── task_ops.py           # 任務相關操作
│   ├── agv_ops.py            # AGV 相關操作
│   ├── user_ops.py           # 用戶相關操作
│   └── equipment_ops.py      # 設備相關操作
└── utils/                     # 工具模組
    └── permissions.py        # 權限檢查工具
```

### 前端架構

#### 核心技術棧
- **框架**: 原生 JavaScript (ES6+) + 模組化設計
- **UI 框架**: Bulma CSS
- **圖標**: Material Design Icons
- **地圖**: Leaflet.js
- **即時通訊**: Socket.IO Client
- **模板引擎**: Jinja2 (伺服器端渲染)

#### 前端模組結構
```
前端架構/
├── templates/                 # Jinja2 模板
│   ├── base.html             # 基礎模板
│   ├── navbar.html           # 導航欄
│   ├── home.html             # 首頁
│   ├── map.html              # 地圖頁面
│   ├── tasks.html            # 任務管理頁面
│   └── devices.html          # 設備管理頁面
├── static/
│   ├── css/                  # 樣式文件
│   │   ├── bulma_1_0_4.min.css
│   │   ├── materialdesignicons.min.css
│   │   ├── agvcui-bulma-extend.css
│   │   └── mapPage.css
│   ├── js/                   # JavaScript 模組
│   │   ├── index.js          # 主入口文件
│   │   ├── socket.js         # Socket.IO 通訊
│   │   ├── mapPage.js        # 地圖頁面邏輯
│   │   ├── navbar.js         # 導航欄邏輯
│   │   └── notify.js         # 通知系統
│   ├── objects/              # 地圖物件類別
│   │   ├── BaseObject.js     # 基礎物件類別
│   │   ├── RotatingMovingObject.js  # AGV 物件
│   │   └── RackInfoObject.js # 貨架物件
│   └── store/                # 狀態管理
│       ├── index.js          # 狀態 Store 定義
│       └── miniStore.js      # 輕量級狀態管理
```

### 資料庫設計

#### 核心資料表
```sql
-- AGV 相關
agv                    # AGV 基本信息
agv_status            # AGV 狀態記錄

-- 任務管理
task                  # 任務基本信息
task_status          # 任務狀態記錄

-- 地圖和導航
node                 # 地圖節點
edge                 # 地圖邊線
kuka_node           # KUKA 專用節點
kuka_edge           # KUKA 專用邊線

-- 設備管理
eqp                  # 設備基本信息
eqp_port            # 設備端口
eqp_signal          # 設備信號

-- 載具和貨架
carrier             # 載具信息
rack                # 貨架信息
rack_status         # 貨架狀態

-- 用戶和權限
user                # 用戶基本信息
audit_log           # 操作審計日誌

-- 系統日誌
rosout_log          # ROS 系統日誌
runtime_log         # 運行時日誌
modify_log          # 資料修改日誌
```

## 🎨 UI/UX 設計規範

### 設計原則
1. **一致性**: 統一的視覺風格和交互模式
2. **響應式**: 支援多種螢幕尺寸
3. **直觀性**: 清晰的信息層次和操作流程
4. **即時性**: 即時狀態更新和反饋

### 色彩規範
```css
/* 主要色彩 */
--primary-color: #3273dc;      /* 主色調 */
--success-color: #48c774;      /* 成功狀態 */
--warning-color: #ffcc00;      /* 警告狀態 */
--danger-color: #f14668;       /* 錯誤狀態 */
--info-color: #3298dc;         /* 信息狀態 */

/* 狀態標籤色彩 */
.tag.is-success { background-color: #48c774; }  /* 啟用/正常 */
.tag.is-warning { background-color: #ffcc00; }  /* 警告/使用中 */
.tag.is-danger { background-color: #f14668; }   /* 錯誤/停用 */
.tag.is-info { background-color: #3298dc; }     /* 信息/ID */
```

### 組件清單

#### 1. 導航組件
- **Navbar**: 頂部導航欄，包含用戶信息和主要功能入口
- **Sidebar**: 側邊欄（地圖頁面使用）

#### 2. 表格組件
- **統一表格樣式**: 使用 Bulma table 類別
- **分頁組件**: 統一的分頁邏輯
- **狀態標籤**: 彩色標籤顯示各種狀態

#### 3. 表單組件
- **統一表單樣式**: Bulma form 控件
- **驗證提示**: 即時表單驗證
- **提交按鈕**: 統一的按鈕樣式

#### 4. 地圖組件
- **互動地圖**: 基於 Leaflet.js
- **AGV 標記**: 動態 AGV 位置顯示
- **路徑顯示**: 任務路徑視覺化

#### 5. 通知組件
- **Toast 通知**: 操作結果反饋
- **確認對話框**: 重要操作確認

## 🤖 AI 重建提示詞

### 系統提示詞
```
你是一個專業的全端開發工程師，專精於 Python FastAPI、JavaScript、ROS 2 和工業自動化系統開發。
你需要重建一個名為 AGVCUI 的 AGV 控制與監控系統。

專案要求：
1. 使用 FastAPI + Socket.IO 作為後端
2. 使用原生 JavaScript + Bulma CSS 作為前端
3. 整合 ROS 2 系統
4. 實現即時 AGV 監控和控制
5. 提供完整的 Web 管理介面

技術約束：
- 必須支援多 AGV 協調管理
- 必須提供即時地圖視覺化
- 必須實現安全的用戶權限系統
- 必須支援任務管理和設備監控
- 必須使用 PostgreSQL 資料庫
```

### 功能實現提示詞
```
請按照以下順序實現 AGVCUI 系統：

階段一：基礎架構搭建
1. 建立 FastAPI 專案結構
2. 配置 Socket.IO 即時通訊
3. 設計資料庫 Schema
4. 實現基礎認證系統

階段二：核心功能開發
1. 實現 AGV 狀態監控
2. 開發任務管理系統
3. 建立地圖視覺化功能
4. 實現設備管理介面

階段三：進階功能
1. 完善用戶權限系統
2. 實現即時通知機制
3. 開發操作日誌功能
4. 優化效能和使用者體驗

每個階段都要包含完整的測試和文檔。
```

## 📝 逐步任務執行步驟

### 第一階段：專案初始化和基礎架構

#### 1.1 建立 ROS 2 工作空間
```bash
# 建立工作空間
mkdir -p agvcui_ws/src
cd agvcui_ws

# 建立 Python 套件
cd src
ros2 pkg create --build-type ament_python agvcui
```

#### 1.2 設定專案依賴
```python
# setup.py
from setuptools import find_namespace_packages, setup

package_name = 'agvcui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_namespace_packages(exclude=['test']),
    install_requires=[
        'setuptools',
        'fastapi',
        'uvicorn',
        'python-socketio',
        'sqlalchemy',
        'psycopg2-binary',
        'python-jose[cryptography]',
        'python-multipart',
        'jinja2',
        'aiofiles'
    ],
    entry_points={
        'console_scripts': [
            'agvc_ui_server = agvcui.agvc_ui_server:main',
        ],
    },
)
```

#### 1.3 建立基礎目錄結構
```bash
agvcui/
├── agvcui/
│   ├── __init__.py
│   ├── agvc_ui_server.py      # 主伺服器
│   ├── agvc_ui_socket.py      # Socket.IO 處理
│   ├── auth.py                # 認證模組
│   ├── middleware.py          # 中間件
│   ├── database/              # 資料庫模組
│   ├── routers/               # API 路由
│   ├── static/                # 靜態資源
│   ├── templates/             # HTML 模板
│   └── utils/                 # 工具模組
```

### 第二階段：資料庫設計和連線

#### 2.1 資料庫連線配置
```python
# database/connection.py
from db_proxy.connection_pool_manager import ConnectionPoolManager
from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker

# 資料庫 URL
DATABASE_URL = "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"

# 建立連線池
connection_pool = ConnectionPoolManager(DATABASE_URL)
engine = create_engine(DATABASE_URL)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
Base = declarative_base()
```

#### 2.2 核心資料模型
```python
# database/models.py
from sqlalchemy import Column, Integer, String, Float, Boolean, DateTime, ForeignKey
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import relationship

Base = declarative_base()

class AGV(Base):
    __tablename__ = "agv"

    id = Column(Integer, primary_key=True, index=True)
    name = Column(String, unique=True, index=True)
    x = Column(Float, default=0.0)
    y = Column(Float, default=0.0)
    heading = Column(Float, default=0.0)
    status = Column(String, default="idle")
    is_active = Column(Boolean, default=True)

class Task(Base):
    __tablename__ = "task"

    id = Column(Integer, primary_key=True, index=True)
    agv_id = Column(Integer, ForeignKey("agv.id"))
    task_type = Column(String)
    status = Column(String, default="pending")
    created_at = Column(DateTime)
    completed_at = Column(DateTime, nullable=True)

    agv = relationship("AGV", back_populates="tasks")

class User(Base):
    __tablename__ = "user"

    id = Column(Integer, primary_key=True, index=True)
    username = Column(String, unique=True, index=True)
    password_hash = Column(String)
    role = Column(String, default="user")
    is_active = Column(Boolean, default=True)
```

### 第三階段：FastAPI 伺服器實現

#### 3.1 主伺服器設定
```python
# agvc_ui_server.py
import os
import socketio
import uvicorn
from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
from fastapi.templating import Jinja2Templates
from contextlib import asynccontextmanager

class AgvcUiServer:
    def __init__(self, host="0.0.0.0", port=8001):
        self.host = host
        self.port = port

        # 初始化 Socket.IO
        self.sio = socketio.AsyncServer(
            async_mode="asgi",
            cors_allowed_origins="*"
        )

        # FastAPI 生命週期管理
        @asynccontextmanager
        async def lifespan(app: FastAPI):
            self.socket_handler = AgvcUiSocket(self.sio)
            yield
            await self.socket_handler.close()

        # 建立 FastAPI 應用
        self.app = FastAPI(lifespan=lifespan)
        self.sio_app = socketio.ASGIApp(self.sio, self.app)

        # 設定中間件
        self.setup_middleware()

        # 設定靜態文件和模板
        self.setup_static_and_templates()

        # 註冊路由
        self.register_routes()

    def setup_middleware(self):
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )

    def setup_static_and_templates(self):
        base_dir = os.path.dirname(os.path.abspath(__file__))
        self.templates = Jinja2Templates(
            directory=os.path.join(base_dir, "templates")
        )
        self.app.mount(
            "/static",
            StaticFiles(directory=os.path.join(base_dir, "static")),
            name="static"
        )

    def register_routes(self):
        # 註冊各種路由
        from .routers import map, tasks, devices, agvs

        self.app.include_router(map.router)
        self.app.include_router(tasks.router)
        self.app.include_router(devices.router)
        self.app.include_router(agvs.router)

    def run(self):
        uvicorn.run(self.sio_app, host=self.host, port=self.port)

def main():
    server = AgvcUiServer()
    server.run()
```

#### 3.2 Socket.IO 事件處理
```python
# agvc_ui_socket.py
import asyncio
import time
from fastapi.encoders import jsonable_encoder

class AgvcUiSocket:
    def __init__(self, sio):
        self.sio = sio
        self.connected_sids = set()
        self.init_socketio()

        # 定期通知任務
        self.notification_tasks = [
            {"func": self.notify_agvs, "interval": 1.0, "last_time": time.time()},
            {"func": self.notify_tasks, "interval": 2.0, "last_time": time.time()},
            {"func": self.notify_map_data, "interval": 5.0, "last_time": time.time()},
        ]

        self._task = asyncio.create_task(self._periodic_notify())

    def init_socketio(self):
        self.sio.on('connect')(self.connect)
        self.sio.on('disconnect')(self.disconnect)
        self.sio.on('user_login')(self.user_login)
        self.sio.on('agv_command')(self.agv_command)
        self.sio.on('task_create')(self.task_create)

    async def connect(self, sid, environ):
        self.connected_sids.add(sid)
        print(f"🔌 用戶連線: {sid}")

        # 發送初始資料
        await self.notify_agvs(sid)
        await self.notify_map_data(sid)
        await self.notify_tasks(sid)

    async def disconnect(self, sid):
        self.connected_sids.discard(sid)
        print(f"🔌 用戶斷線: {sid}")

    async def user_login(self, sid, data):
        """處理用戶登入"""
        username = data.get('username')
        password = data.get('password')

        # 驗證用戶
        user = authenticate_user(username, password)
        if not user:
            return {"success": False, "message": "登入失敗"}

        # 創建 token
        access_token = create_access_token(data={"sub": user.username})

        return {
            "success": True,
            "message": "登入成功",
            "user": jsonable_encoder(user),
            "access_token": access_token
        }

    async def agv_command(self, sid, data):
        """處理 AGV 控制命令"""
        agv_id = data.get('agv_id')
        command = data.get('command')

        # 發送命令到 ROS 系統
        success = await self.send_agv_command(agv_id, command)

        return {
            "success": success,
            "message": "命令已發送" if success else "命令發送失敗"
        }

    async def task_create(self, sid, data):
        """處理任務創建"""
        task_data = data.get('task')

        # 創建任務
        task = await self.create_task(task_data)

        # 廣播任務更新
        await self.broadcast_task_update(task)

        return {
            "success": True,
            "message": "任務創建成功",
            "task": jsonable_encoder(task)
        }

    async def _periodic_notify(self):
        """定期通知循環"""
        while True:
            try:
                current_time = time.time()

                for task in self.notification_tasks:
                    if current_time - task["last_time"] >= task["interval"]:
                        await task["func"]()
                        task["last_time"] = current_time

                await asyncio.sleep(0.1)
            except Exception as e:
                print(f"❌ 定期通知錯誤: {e}")
                await asyncio.sleep(1.0)

    async def notify_agvs(self, sid=None):
        """通知 AGV 狀態"""
        agvs = get_all_agvs()  # 從資料庫獲取
        payload = {"agvs": agvs}

        if sid:
            await self.sio.emit("agv_list", jsonable_encoder(payload), room=sid)
        else:
            await self.sio.emit("agv_list", jsonable_encoder(payload))

    async def close(self):
        """關閉 Socket 處理器"""
        if hasattr(self, '_task'):
            self._task.cancel()
```

### 第四階段：前端架構實現

#### 4.1 基礎 HTML 模板
```html
<!-- templates/base.html -->
<!DOCTYPE html>
<html lang="zh-TW" data-theme="light">
<head>
    <meta charset="UTF-8">
    <title>{% block title %}AGVCUI{% endblock %}</title>
    <link rel="stylesheet" href="/static/css/bulma_1_0_4.min.css" />
    <link rel="stylesheet" href="/static/css/materialdesignicons.min.css" />
    <link rel="stylesheet" href="/static/css/agvcui-bulma-extend.css" />

    <!-- Socket.IO 和 JavaScript 庫 -->
    <script src="/static/js/lib/socket.io.min.js"></script>
    <script src="/static/js/lib/axios.min.js"></script>
    <script type="module" src="/static/index.js"></script>
</head>
<body>
    <main class="main-root">
        {% include "navbar.html" %}
        {% block content %}{% endblock %}
    </main>
</body>
</html>
```

#### 4.2 導航欄模板
```html
<!-- templates/navbar.html -->
<nav class="navbar is-primary" role="navigation">
    <div class="navbar-brand">
        <a class="navbar-item" href="/">
            <strong>AGVCUI</strong>
        </a>
    </div>

    <div class="navbar-menu">
        <div class="navbar-start">
            <a class="navbar-item" href="/map">地圖</a>
            <a class="navbar-item" href="/tasks">任務</a>
            <a class="navbar-item" href="/devices">設備</a>
            <a class="navbar-item" href="/agvs">AGV</a>
        </div>

        <div class="navbar-end">
            <div class="navbar-item has-dropdown is-hoverable" id="user-dropdown">
                <a class="navbar-link" id="user-info">
                    <span class="icon"><i class="mdi mdi-account"></i></span>
                    <span id="username-display">未登入</span>
                </a>
                <div class="navbar-dropdown">
                    <a class="navbar-item" href="/login" id="login-link">登入</a>
                    <a class="navbar-item" href="#" id="logout-link" style="display: none;">登出</a>
                </div>
            </div>
        </div>
    </div>
</nav>
```

#### 4.3 地圖頁面模板
```html
<!-- templates/map.html -->
{% extends "base.html" %}
{% block title %}地圖 - AGVCUI{% endblock %}

{% block content %}
<section class="hero is-fullheight is-light map-container">
    <div id="map" class="map">
        <div class="grid-overlay"></div>
    </div>

    <!-- 地圖工具列 -->
    <div class="map-toolbar">
        <button class="button" id="map-tool-tasks" title="任務管理">
            <span class="icon"><i class="mdi mdi-format-list-checks"></i></span>
        </button>
        <button class="button" id="map-tool-agvs" title="AGV 管理">
            <span class="icon"><i class="mdi mdi-robot"></i></span>
        </button>
        <button class="button" id="map-tool-racks" title="貨架管理">
            <span class="icon"><i class="mdi mdi-view-grid"></i></span>
        </button>
    </div>

    <!-- 側邊面板 -->
    <div class="map-sidebar" id="map-sidebar">
        <div class="map-sidebar-header">
            <h4 class="title is-5" id="sidebar-title">詳細資訊</h4>
            <button class="map-sidebar-close" id="sidebar-close">
                <i class="mdi mdi-close"></i>
            </button>
        </div>
        <div class="map-sidebar-content" id="sidebar-content">
            <!-- 動態內容 -->
        </div>
    </div>
</section>

<!-- 引入地圖相關 JavaScript 和 CSS -->
<script src="/static/js/lib/leaflet.js"></script>
<link rel="stylesheet" href="/static/css/leaflet.css" />
<link href="/static/css/mapPage.css" rel="stylesheet" />
{% endblock %}
```

#### 4.4 前端狀態管理
```javascript
// static/store/miniStore.js
export function createStore(name, initialState) {
    let state = { ...initialState };
    const listeners = new Set();

    return {
        getState: () => ({ ...state }),

        setState: (newState) => {
            const prevState = { ...state };
            state = { ...state, ...newState };

            // 通知所有監聽器
            listeners.forEach(listener => {
                try {
                    listener(state, prevState);
                } catch (error) {
                    console.error(`Store ${name} listener error:`, error);
                }
            });
        },

        subscribe: (listener) => {
            listeners.add(listener);
            return () => listeners.delete(listener);
        },

        clear: () => {
            state = { ...initialState };
        }
    };
}
```

```javascript
// static/store/index.js
import { createStore } from './miniStore.js';

// AGV 狀態管理
export const agvStore = createStore('agvState', {
    agvs: []
});

// 任務狀態管理
export const taskStore = createStore('taskState', {
    tasks: []
});

// 地圖狀態管理
export const mapStore = createStore('mapState', {
    nodes: [],
    edges: [],
    agvs: [],
    selectedObject: null
});

// 用戶狀態管理
export const userStore = createStore('userState', {
    id: null,
    username: null,
    role: null,
    isLoggedIn: false,
    isConnected: false
});
```

#### 4.5 Socket.IO 通訊層
```javascript
// static/js/socket.js
import { agvStore, taskStore, mapStore, userStore } from '../store/index.js';
import { notify } from './notify.js';

class SocketManager {
    constructor() {
        this.socket = null;
        this.isConnected = false;
        this.reconnectAttempts = 0;
        this.maxReconnectAttempts = 5;
    }

    connect() {
        this.socket = io({
            transports: ['websocket', 'polling'],
            timeout: 20000,
            forceNew: true
        });

        this.setupEventListeners();
    }

    setupEventListeners() {
        // 連線事件
        this.socket.on('connect', () => {
            console.log('✅ Socket 連線成功');
            this.isConnected = true;
            this.reconnectAttempts = 0;
            userStore.setState({ isConnected: true });
        });

        this.socket.on('disconnect', () => {
            console.log('❌ Socket 連線斷開');
            this.isConnected = false;
            userStore.setState({ isConnected: false });
        });

        // 資料更新事件
        this.socket.on('agv_list', (data) => {
            agvStore.setState({ agvs: data.agvs });
        });

        this.socket.on('task_list', (data) => {
            taskStore.setState({ tasks: data.tasks });
        });

        this.socket.on('map_info', (data) => {
            mapStore.setState({
                nodes: data.nodes,
                edges: data.edges,
                agvs: data.agvs
            });
        });
    }

    // API 方法
    async login(username, password) {
        return new Promise((resolve) => {
            this.socket.emit('user_login', { username, password }, (response) => {
                if (response.success) {
                    userStore.setState({
                        ...response.user,
                        isLoggedIn: true
                    });
                    notify.showSuccess(response.message);
                } else {
                    notify.showError(response.message);
                }
                resolve(response);
            });
        });
    }

    async sendAGVCommand(agvId, command) {
        return new Promise((resolve) => {
            this.socket.emit('agv_command', { agv_id: agvId, command }, (response) => {
                if (response.success) {
                    notify.showSuccess(response.message);
                } else {
                    notify.showError(response.message);
                }
                resolve(response);
            });
        });
    }

    async createTask(taskData) {
        return new Promise((resolve) => {
            this.socket.emit('task_create', { task: taskData }, (response) => {
                if (response.success) {
                    notify.showSuccess(response.message);
                } else {
                    notify.showError(response.message);
                }
                resolve(response);
            });
        });
    }
}

// 全域 Socket 管理器
export const socketManager = new SocketManager();

// 自動連線
document.addEventListener('DOMContentLoaded', () => {
    socketManager.connect();
});
```

### 第五階段：地圖視覺化實現

#### 5.1 地圖物件基礎類別
```javascript
// static/objects/BaseObject.js
export class BaseObject {
    constructor(map, position, name, className = '') {
        this.map = map;
        this.position = position;
        this.name = name;
        this.className = className;
        this.marker = null;
        this.data = {};

        this.createMarker();
    }

    createMarker() {
        const icon = L.divIcon({
            className: `custom-marker ${this.className}`,
            html: `<div class="marker-content">${this.name}</div>`,
            iconSize: [40, 40],
            iconAnchor: [20, 20]
        });

        this.marker = L.marker(this.position, { icon })
            .addTo(this.map);

        this.setupEvents();
    }

    setupEvents() {
        this.marker.on('click', () => {
            this.onClick();
        });
    }

    onClick() {
        // 子類別覆寫此方法
        console.log(`Clicked on ${this.name}`);
    }

    setPosition(newPosition) {
        this.position = newPosition;
        if (this.marker) {
            this.marker.setLatLng(newPosition);
        }
    }

    setData(data) {
        this.data = { ...this.data, ...data };
    }

    remove() {
        if (this.marker) {
            this.map.removeLayer(this.marker);
        }
    }
}
```

#### 5.2 AGV 物件類別
```javascript
// static/objects/RotatingMovingObject.js
import { BaseObject } from './BaseObject.js';

export class RotatingMovingObject extends BaseObject {
    constructor(map, position, name, className = '') {
        super(map, position, name, className);

        this.targetPosition = position;
        this.targetHeading = 0;
        this.currentHeading = 0;
        this.animationMode = 'smooth';
        this.lerpSpeed = 4.0;
        this.isAnimating = false;

        this.startAnimation();
    }

    createMarker() {
        const icon = L.divIcon({
            className: `agv-marker ${this.className}`,
            html: `
                <div class="agv-icon" style="transform: rotate(${this.currentHeading}deg)">
                    <i class="mdi mdi-robot"></i>
                    <div class="agv-name">${this.name}</div>
                </div>
            `,
            iconSize: [60, 60],
            iconAnchor: [30, 30]
        });

        this.marker = L.marker(this.position, { icon })
            .addTo(this.map);

        this.setupEvents();
    }

    setTargetPosition(newPosition, heading = 0) {
        this.targetPosition = newPosition;
        this.targetHeading = heading;

        if (this.animationMode === 'instant') {
            this.position = newPosition;
            this.currentHeading = heading;
            this.updateMarker();
        }
    }

    setAnimationMode(mode, speed = 4.0) {
        this.animationMode = mode;
        this.lerpSpeed = speed;
    }

    startAnimation() {
        if (this.isAnimating) return;
        this.isAnimating = true;
        this.animate();
    }

    animate() {
        if (!this.isAnimating) return;

        if (this.animationMode === 'smooth') {
            // 位置插值
            const deltaLat = this.targetPosition.lat - this.position.lat;
            const deltaLng = this.targetPosition.lng - this.position.lng;
            const distance = Math.sqrt(deltaLat * deltaLat + deltaLng * deltaLng);

            if (distance > 0.001) {
                const factor = Math.min(1.0, this.lerpSpeed * 0.016); // 60 FPS
                this.position = L.latLng(
                    this.position.lat + deltaLat * factor,
                    this.position.lng + deltaLng * factor
                );
            }

            // 角度插值
            const deltaHeading = this.normalizeAngle(this.targetHeading - this.currentHeading);
            if (Math.abs(deltaHeading) > 1) {
                const factor = Math.min(1.0, this.lerpSpeed * 0.016);
                this.currentHeading += deltaHeading * factor;
                this.currentHeading = this.normalizeAngle(this.currentHeading);
            }

            this.updateMarker();
        }

        requestAnimationFrame(() => this.animate());
    }

    normalizeAngle(angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    updateMarker() {
        if (this.marker) {
            this.marker.setLatLng(this.position);

            const iconElement = this.marker.getElement();
            if (iconElement) {
                const agvIcon = iconElement.querySelector('.agv-icon');
                if (agvIcon) {
                    agvIcon.style.transform = `rotate(${this.currentHeading}deg)`;
                }
            }
        }
    }

    onClick() {
        // 顯示 AGV 詳細資訊
        if (window.mapInteraction) {
            window.mapInteraction.showAGVDetails(this.data);
        }
    }
}
```

#### 5.3 地圖頁面邏輯
```javascript
// static/js/mapPage.js
import { mapStore, agvStore, taskStore } from '../store/index.js';
import { RotatingMovingObject } from '../objects/RotatingMovingObject.js';
import { notify } from './notify.js';

export const mapPage = (() => {
    let map = null;
    const agvObjects = new Map();
    const nodeObjects = new Map();

    function initMap() {
        // 初始化 Leaflet 地圖
        map = L.map('map', {
            crs: L.CRS.Simple,
            minZoom: -2,
            maxZoom: 4,
            zoomControl: true
        });

        // 載入地圖圖片
        const mapBounds = [[0, 0], [3010, 6320]];
        L.imageOverlay('/static/alan-demo-map.drawio.svg', mapBounds)
            .addTo(map);

        // 設定初始視圖
        const initialBounds = [[1005, 2660], [1805, 3260]];
        map.fitBounds(initialBounds);

        // 設定地圖事件
        setupMapEvents();

        // 訂閱狀態變化
        subscribeToStores();
    }

    function setupMapEvents() {
        map.on('click', (e) => {
            console.log('地圖點擊位置:', e.latlng);
        });

        // 工具列按鈕事件
        document.getElementById('map-tool-tasks')?.addEventListener('click', () => {
            showTaskPanel();
        });

        document.getElementById('map-tool-agvs')?.addEventListener('click', () => {
            showAGVPanel();
        });
    }

    function subscribeToStores() {
        // 訂閱 AGV 狀態變化
        agvStore.subscribe((state) => {
            updateAGVs(state.agvs);
        });

        // 訂閱地圖資料變化
        mapStore.subscribe((state) => {
            updateMapData(state);
        });
    }

    function updateAGVs(agvs) {
        agvs.forEach(agv => {
            if (agvObjects.has(agv.id)) {
                // 更新現有 AGV
                const agvObject = agvObjects.get(agv.id);
                const position = L.latLng(agv.y, agv.x);
                agvObject.setTargetPosition(position, agv.heading);
            } else {
                // 創建新 AGV
                const position = L.latLng(agv.y, agv.x);
                const agvObject = new RotatingMovingObject(
                    map, position, agv.name, 'agv-marker'
                );
                agvObject.setData(agv);
                agvObjects.set(agv.id, agvObject);
            }
        });
    }

    function updateMapData(mapData) {
        // 更新節點和邊線
        updateNodes(mapData.nodes);
        updateEdges(mapData.edges);
    }

    function updateNodes(nodes) {
        nodes.forEach(node => {
            if (!nodeObjects.has(node.id)) {
                const position = L.latLng(node.y, node.x);
                const marker = L.circleMarker(position, {
                    radius: 5,
                    fillColor: '#3388ff',
                    color: '#fff',
                    weight: 2,
                    opacity: 1,
                    fillOpacity: 0.8
                }).addTo(map);

                marker.bindPopup(`節點 ${node.id}`);
                nodeObjects.set(node.id, marker);
            }
        });
    }

    function updateEdges(edges) {
        // 實現邊線繪製邏輯
        edges.forEach(edge => {
            // 根據節點 ID 找到位置並繪製線條
        });
    }

    function showTaskPanel() {
        const sidebar = document.getElementById('map-sidebar');
        const title = document.getElementById('sidebar-title');
        const content = document.getElementById('sidebar-content');

        title.textContent = '任務管理';
        content.innerHTML = `
            <div class="task-panel">
                <button class="button is-primary" onclick="createNewTask()">
                    <span class="icon"><i class="mdi mdi-plus"></i></span>
                    <span>新增任務</span>
                </button>
                <div id="task-list">
                    <!-- 任務列表將在這裡動態載入 -->
                </div>
            </div>
        `;

        sidebar.classList.add('is-active');
        loadTaskList();
    }

    function showAGVPanel() {
        const sidebar = document.getElementById('map-sidebar');
        const title = document.getElementById('sidebar-title');
        const content = document.getElementById('sidebar-content');

        title.textContent = 'AGV 管理';
        content.innerHTML = `
            <div class="agv-panel">
                <div id="agv-list">
                    <!-- AGV 列表將在這裡動態載入 -->
                </div>
            </div>
        `;

        sidebar.classList.add('is-active');
        loadAGVList();
    }

    function loadTaskList() {
        const taskList = document.getElementById('task-list');
        const tasks = taskStore.getState().tasks;

        taskList.innerHTML = tasks.map(task => `
            <div class="task-item box">
                <div class="level">
                    <div class="level-left">
                        <div>
                            <p class="title is-6">任務 ${task.id}</p>
                            <p class="subtitle is-7">${task.task_type}</p>
                        </div>
                    </div>
                    <div class="level-right">
                        <span class="tag ${getTaskStatusClass(task.status)}">
                            ${task.status}
                        </span>
                    </div>
                </div>
            </div>
        `).join('');
    }

    function loadAGVList() {
        const agvList = document.getElementById('agv-list');
        const agvs = agvStore.getState().agvs;

        agvList.innerHTML = agvs.map(agv => `
            <div class="agv-item box">
                <div class="level">
                    <div class="level-left">
                        <div>
                            <p class="title is-6">${agv.name}</p>
                            <p class="subtitle is-7">位置: (${agv.x.toFixed(1)}, ${agv.y.toFixed(1)})</p>
                        </div>
                    </div>
                    <div class="level-right">
                        <span class="tag ${getAGVStatusClass(agv.status)}">
                            ${agv.status}
                        </span>
                    </div>
                </div>
                <div class="buttons">
                    <button class="button is-small is-info" onclick="sendAGVCommand(${agv.id}, 'stop')">
                        停止
                    </button>
                    <button class="button is-small is-success" onclick="sendAGVCommand(${agv.id}, 'resume')">
                        繼續
                    </button>
                </div>
            </div>
        `).join('');
    }

    function getTaskStatusClass(status) {
        const statusMap = {
            'pending': 'is-warning',
            'running': 'is-info',
            'completed': 'is-success',
            'failed': 'is-danger'
        };
        return statusMap[status] || 'is-light';
    }

    function getAGVStatusClass(status) {
        const statusMap = {
            'idle': 'is-light',
            'moving': 'is-info',
            'working': 'is-warning',
            'error': 'is-danger'
        };
        return statusMap[status] || 'is-light';
    }

    // 公開方法
    return {
        setup: () => {
            if (window.location.pathname === '/map') {
                initMap();
            }
        }
    };
})();

// 全域函數
window.createNewTask = function() {
    // 實現任務創建邏輯
    notify.showInfo('任務創建功能開發中...');
};

window.sendAGVCommand = function(agvId, command) {
    if (window.socketManager) {
        window.socketManager.sendAGVCommand(agvId, command);
    }
};
```

## 🧪 測試策略和部署指南

### 測試階段

#### 1. 單元測試
```python
# tests/test_auth.py
import pytest
from agvcui.auth import create_access_token, verify_token, hash_password, verify_password

def test_password_hashing():
    password = "test123"
    hashed = hash_password(password)
    assert verify_password(password, hashed)
    assert not verify_password("wrong", hashed)

def test_token_creation_and_verification():
    data = {"sub": "testuser"}
    token = create_access_token(data)
    verified_data = verify_token(token)
    assert verified_data.username == "testuser"

# tests/test_database.py
import pytest
from agvcui.database.connection import connection_pool
from agvcui.database.agv_ops import get_all_agvs, create_agv

def test_agv_operations():
    # 測試 AGV 創建和查詢
    agv_data = {
        "name": "test_agv",
        "x": 100.0,
        "y": 200.0,
        "heading": 90.0
    }

    agv = create_agv(agv_data)
    assert agv.name == "test_agv"

    agvs = get_all_agvs()
    assert len(agvs) > 0
```

#### 2. 整合測試
```python
# tests/test_socket_integration.py
import pytest
import asyncio
from agvcui.agvc_ui_socket import AgvcUiSocket
import socketio

@pytest.mark.asyncio
async def test_socket_connection():
    sio = socketio.AsyncServer()
    socket_handler = AgvcUiSocket(sio)

    # 模擬連線
    await socket_handler.connect("test_sid", {})
    assert "test_sid" in socket_handler.connected_sids

    # 模擬登入
    login_data = {"username": "admin", "password": "admin123"}
    response = await socket_handler.user_login("test_sid", login_data)
    assert response["success"] == True

# tests/test_api_endpoints.py
import pytest
from fastapi.testclient import TestClient
from agvcui.agvc_ui_server import AgvcUiServer

@pytest.fixture
def client():
    server = AgvcUiServer()
    return TestClient(server.app)

def test_home_page(client):
    response = client.get("/")
    assert response.status_code == 200
    assert "AGVCUI" in response.text

def test_map_page(client):
    response = client.get("/map")
    assert response.status_code == 200
```

#### 3. 前端測試
```javascript
// tests/frontend/test_socket.js
import { socketManager } from '../../static/js/socket.js';
import { agvStore, userStore } from '../../static/store/index.js';

describe('Socket Manager', () => {
    test('should connect successfully', (done) => {
        socketManager.socket.on('connect', () => {
            expect(socketManager.isConnected).toBe(true);
            done();
        });
        socketManager.connect();
    });

    test('should handle AGV updates', () => {
        const testAGVs = [
            { id: 1, name: 'AGV001', x: 100, y: 200, heading: 90 }
        ];

        socketManager.socket.emit('agv_list', { agvs: testAGVs });

        const state = agvStore.getState();
        expect(state.agvs).toEqual(testAGVs);
    });
});
```

### 部署指南

#### 1. 環境準備
```bash
# 系統需求
Ubuntu 20.04/22.04
Python 3.8+
Node.js 16+ (用於前端工具)
PostgreSQL 12+
ROS 2 Humble/Foxy

# 安裝 ROS 2
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install ros-humble-desktop

# 安裝 Python 依賴
pip install fastapi uvicorn python-socketio sqlalchemy psycopg2-binary
```

#### 2. 資料庫設定
```sql
-- 創建資料庫
CREATE DATABASE agvc;
CREATE USER agvc WITH PASSWORD 'password';
GRANT ALL PRIVILEGES ON DATABASE agvc TO agvc;

-- 創建基礎表結構
\c agvc;

CREATE TABLE agv (
    id SERIAL PRIMARY KEY,
    name VARCHAR(50) UNIQUE NOT NULL,
    x FLOAT DEFAULT 0.0,
    y FLOAT DEFAULT 0.0,
    heading FLOAT DEFAULT 0.0,
    status VARCHAR(20) DEFAULT 'idle',
    is_active BOOLEAN DEFAULT true,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE TABLE task (
    id SERIAL PRIMARY KEY,
    agv_id INTEGER REFERENCES agv(id),
    task_type VARCHAR(50),
    status VARCHAR(20) DEFAULT 'pending',
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    completed_at TIMESTAMP
);

CREATE TABLE "user" (
    id SERIAL PRIMARY KEY,
    username VARCHAR(50) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    role VARCHAR(20) DEFAULT 'user',
    full_name VARCHAR(100),
    is_active BOOLEAN DEFAULT true,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
```

#### 3. 專案部署
```bash
# 1. 克隆或創建專案
mkdir -p /opt/agvcui_ws/src
cd /opt/agvcui_ws/src

# 2. 建置 ROS 2 套件
cd /opt/agvcui_ws
colcon build --packages-select agvcui

# 3. 設定環境變數
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source /opt/agvcui_ws/install/setup.bash" >> ~/.bashrc

# 4. 創建系統服務
sudo tee /etc/systemd/system/agvcui.service > /dev/null <<EOF
[Unit]
Description=AGVCUI Web Server
After=network.target postgresql.service

[Service]
Type=simple
User=agvc
WorkingDirectory=/opt/agvcui_ws
Environment=ROS_DOMAIN_ID=0
ExecStart=/bin/bash -c "source install/setup.bash && ros2 run agvcui agvc_ui_server"
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

# 5. 啟動服務
sudo systemctl daemon-reload
sudo systemctl enable agvcui
sudo systemctl start agvcui
```

#### 4. Nginx 反向代理設定
```nginx
# /etc/nginx/sites-available/agvcui
server {
    listen 80;
    server_name your-domain.com;

    location / {
        proxy_pass http://127.0.0.1:8001;
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection 'upgrade';
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;
        proxy_cache_bypass $http_upgrade;
    }

    location /socket.io/ {
        proxy_pass http://127.0.0.1:8001;
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection "upgrade";
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;
    }
}
```

#### 5. 監控和日誌
```bash
# 查看服務狀態
sudo systemctl status agvcui

# 查看日誌
sudo journalctl -u agvcui -f

# 查看 ROS 2 日誌
ros2 topic echo /rosout

# 效能監控
htop
iotop
```

## 🔧 關鍵配置範例

### 環境變數配置
```bash
# .env
DATABASE_URL=postgresql+psycopg2://agvc:password@192.168.100.254/agvc
SECRET_KEY=your-secret-key-change-this-in-production
ROS_DOMAIN_ID=0
AGVCUI_HOST=0.0.0.0
AGVCUI_PORT=8001
```

### ROS 2 Launch 文件
```python
# launch/agvcui.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='agvcui',
            executable='agvc_ui_server',
            name='agvcui_server',
            output='screen',
            parameters=[{
                'host': '0.0.0.0',
                'port': 8001
            }]
        )
    ])
```

## 📋 開發檢查清單

### 後端開發
- [ ] FastAPI 伺服器設定完成
- [ ] Socket.IO 事件處理實現
- [ ] 資料庫連線和 CRUD 操作
- [ ] 認證和權限系統
- [ ] API 路由實現
- [ ] ROS 2 整合

### 前端開發
- [ ] 基礎 HTML 模板
- [ ] JavaScript 模組化架構
- [ ] 狀態管理系統
- [ ] Socket.IO 通訊層
- [ ] 地圖視覺化功能
- [ ] 響應式 UI 設計

### 測試和部署
- [ ] 單元測試覆蓋
- [ ] 整合測試
- [ ] 前端測試
- [ ] 部署腳本
- [ ] 監控設定
- [ ] 文檔完整性

---

**重建完成標準**: 所有核心功能正常運作，通過測試，成功部署並可正常訪問

**預估開發時間**: 4-6 週（包含測試和部署）

**維護建議**: 定期更新依賴、監控系統效能、備份資料庫、收集用戶反饋
