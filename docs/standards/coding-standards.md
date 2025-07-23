# RosAGV 程式碼標準規範

## 📋 概述

本文檔定義 RosAGV 專案的程式碼撰寫標準，基於現有程式碼實作的分析和最佳實踐，確保程式碼品質、一致性和可維護性。

## 🎯 程式碼品質目標

### 核心原則
- **一致性**: 整個專案使用統一的程式碼風格
- **可讀性**: 程式碼清晰易懂，自我文檔化
- **可維護性**: 模組化設計，易於修改和擴展
- **可測試性**: 程式碼結構支援單元測試和整合測試
- **效能**: 在保持可讀性的前提下優化效能

### 品質指標
- **程式碼複雜度**: 單一函數圈複雜度 ≤ 10
- **函數長度**: 單一函數 ≤ 50 行
- **類別大小**: 單一類別 ≤ 500 行
- **測試覆蓋率**: 核心業務邏輯 ≥ 80%

## 🐍 Python 程式碼標準

### 基礎風格規範

#### PEP 8 編碼風格 ✅
```python
# ✅ 正確：遵循 PEP 8 命名規範
class AgvStateMachine:
    def __init__(self, agv_id: str):
        self.agv_id = agv_id
        self.current_state = "idle"
    
    def transition_to_state(self, new_state: str) -> bool:
        """狀態轉換方法"""
        if self._is_valid_transition(new_state):
            self.current_state = new_state
            return True
        return False
    
    def _is_valid_transition(self, state: str) -> bool:
        """私有方法使用單底線前綴"""
        return state in self.VALID_STATES

# ❌ 錯誤：不符合命名規範
class agvStateMachine:  # 類別名稱應使用 PascalCase
    def TransitionToState(self, newState):  # 方法名稱應使用 snake_case
        pass
```

#### Type Hints 使用規範 ✅
```python
# ✅ 正確：完整的 Type Hints
from typing import Optional, List, Dict, Union
from datetime import datetime

class TaskManager:
    def __init__(self, max_tasks: int = 100):
        self.tasks: List[Dict[str, Union[str, int]]] = []
        self.max_tasks = max_tasks
    
    def create_task(
        self, 
        task_id: str, 
        agv_id: str, 
        priority: int = 1,
        deadline: Optional[datetime] = None
    ) -> bool:
        """建立新任務"""
        if len(self.tasks) >= self.max_tasks:
            return False
        
        task: Dict[str, Union[str, int, datetime]] = {
            "task_id": task_id,
            "agv_id": agv_id,
            "priority": priority,
            "created_at": datetime.now()
        }
        
        if deadline:
            task["deadline"] = deadline
        
        self.tasks.append(task)
        return True

# ❌ 錯誤：缺少 Type Hints
def create_task(task_id, agv_id, priority=1):  # 缺少類型註解
    pass
```

#### 文檔字串標準 ✅
```python
# ✅ 正確：完整的 docstring
class LoaderAgv:
    """Loader AGV 狀態機實作
    
    負責處理 Loader AGV 的狀態轉換和業務邏輯，包括：
    - take_transfer 流程控制
    - AGV Port 選擇邏輯
    - 資料庫狀態同步
    
    Attributes:
        agv_id (str): AGV 識別碼
        current_state (str): 當前狀態
        room_id (int): 房間 ID
    """
    
    def __init__(self, agv_id: str, room_id: int):
        """初始化 Loader AGV
        
        Args:
            agv_id: AGV 識別碼
            room_id: 房間 ID，用於計算 port_address
            
        Raises:
            ValueError: 當 room_id 無效時
        """
        if room_id <= 0:
            raise ValueError("room_id 必須大於 0")
        
        self.agv_id = agv_id
        self.room_id = room_id
        self.current_state = "idle"
    
    def calculate_port_address(self) -> int:
        """計算 AGV Port 地址
        
        根據 room_id 計算對應的 port_address：
        - room_id=1 → port_address=1100
        - room_id=2 → port_address=2100
        - room_id=3 → port_address=3100
        
        Returns:
            int: 計算得出的 port_address
            
        Example:
            >>> agv = LoaderAgv("AGV_001", 1)
            >>> agv.calculate_port_address()
            1100
        """
        return self.room_id * 1000 + 100

# ❌ 錯誤：缺少或不完整的 docstring
def calculate_port_address(self):
    """計算地址"""  # 過於簡略
    return self.room_id * 1000 + 100
```

### ROS 2 程式碼標準

#### 節點結構規範 ✅
```python
# ✅ 正確：標準 ROS 2 節點結構
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from typing import Optional

class PlcProxyNode(Node):
    """PLC 代理節點
    
    負責處理 PLC 通訊和 ROS 2 訊息轉換
    """
    
    def __init__(self):
        super().__init__('plc_proxy_node')
        
        # 參數宣告
        self.declare_parameter('plc_host', '192.168.1.100')
        self.declare_parameter('plc_port', 8501)
        
        # 發布者和訂閱者
        self.status_publisher = self.create_publisher(
            String, 
            'plc_status', 
            10
        )
        
        self.command_subscription = self.create_subscription(
            String,
            'plc_command',
            self.command_callback,
            10
        )
        
        # 定時器
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info('PLC Proxy Node 已啟動')
    
    def command_callback(self, msg: String) -> None:
        """處理 PLC 指令"""
        self.get_logger().info(f'收到指令: {msg.data}')
        # 處理邏輯...
    
    def timer_callback(self) -> None:
        """定時器回調"""
        status_msg = String()
        status_msg.data = 'PLC 連線正常'
        self.status_publisher.publish(status_msg)

def main(args: Optional[list] = None) -> None:
    """主函數"""
    rclpy.init(args=args)
    node = PlcProxyNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Launch 檔案標準 ✅
```python
# ✅ 正確：標準 Launch 檔案結構
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """生成 Launch 描述"""
    
    # 宣告啟動參數
    plc_host_arg = DeclareLaunchArgument(
        'plc_host',
        default_value='192.168.1.100',
        description='PLC 主機地址'
    )
    
    plc_port_arg = DeclareLaunchArgument(
        'plc_port',
        default_value='8501',
        description='PLC 端口號'
    )
    
    # 節點定義
    plc_proxy_node = Node(
        package='plc_proxy',
        executable='plc_proxy_node',
        name='plc_proxy_node',
        parameters=[{
            'plc_host': LaunchConfiguration('plc_host'),
            'plc_port': LaunchConfiguration('plc_port'),
        }],
        output='screen'
    )
    
    return LaunchDescription([
        plc_host_arg,
        plc_port_arg,
        plc_proxy_node,
    ])
```

### Web API 程式碼標準

#### FastAPI 應用結構 ✅
```python
# ✅ 正確：標準 FastAPI 應用結構
from fastapi import FastAPI, HTTPException, Depends
from pydantic import BaseModel, Field
from typing import List, Optional
import uvicorn

# Pydantic 模型定義
class TaskCreate(BaseModel):
    """任務建立請求模型"""
    agv_id: str = Field(..., description="AGV 識別碼")
    mission_code: str = Field(..., description="任務代碼")
    priority: int = Field(default=1, ge=1, le=10, description="優先級 (1-10)")
    parameters: Optional[dict] = Field(default=None, description="任務參數")

class TaskResponse(BaseModel):
    """任務回應模型"""
    task_id: int
    agv_id: str
    status: str
    created_at: str

# FastAPI 應用初始化
app = FastAPI(
    title="RosAGV Web API",
    description="RosAGV 系統 Web API 服務",
    version="1.0.0"
)

# 路由定義
@app.post("/tasks/", response_model=TaskResponse)
async def create_task(task: TaskCreate) -> TaskResponse:
    """建立新任務
    
    Args:
        task: 任務建立請求
        
    Returns:
        TaskResponse: 建立的任務資訊
        
    Raises:
        HTTPException: 當任務建立失敗時
    """
    try:
        # 業務邏輯處理
        new_task = await task_service.create_task(task)
        return TaskResponse(
            task_id=new_task.id,
            agv_id=new_task.agv_id,
            status=new_task.status,
            created_at=new_task.created_at.isoformat()
        )
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail="內部伺服器錯誤")

@app.get("/tasks/", response_model=List[TaskResponse])
async def get_tasks(
    skip: int = 0, 
    limit: int = 100,
    agv_id: Optional[str] = None
) -> List[TaskResponse]:
    """取得任務列表"""
    tasks = await task_service.get_tasks(skip=skip, limit=limit, agv_id=agv_id)
    return [
        TaskResponse(
            task_id=task.id,
            agv_id=task.agv_id,
            status=task.status,
            created_at=task.created_at.isoformat()
        )
        for task in tasks
    ]

# 應用啟動
if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
```

#### Socket.IO 事件處理標準 ✅
```python
# ✅ 正確：標準 Socket.IO 事件處理
import socketio
from typing import Dict, Any, Optional

class OpUiSocket:
    """OPUI Socket.IO 事件處理器"""
    
    def __init__(self, sio: socketio.AsyncServer):
        self.sio = sio
        self.user_sid_map: Dict[str, str] = {}
        
    def init_socketio(self) -> None:
        """初始化 Socket.IO 事件處理器"""
        self.sio.on('connect')(self.connect)
        self.sio.on('disconnect')(self.disconnect)
        self.sio.on('login')(self.login)
        self.sio.on('client_update')(self.client_update)
    
    async def connect(self, sid: str, environ: dict, auth: Optional[dict]) -> None:
        """客戶端連線事件"""
        print(f"✅ 客戶端連線: {sid}")
        # 連線處理邏輯...
    
    async def disconnect(self, sid: str) -> None:
        """客戶端斷線事件"""
        print(f"❌ 客戶端斷線: {sid}")
        # 清理邏輯...
    
    async def login(self, sid: str, data: Dict[str, Any]) -> Dict[str, Any]:
        """客戶端登入事件
        
        Args:
            sid: Socket.IO 會話 ID
            data: 登入資料 {"deviceId": str, "machineId": int}
            
        Returns:
            Dict: 登入結果 {"success": bool, "message": str, "client": dict}
        """
        try:
            device_id = data.get("deviceId")
            machine_id = data.get("machineId")
            
            if not device_id or not machine_id:
                return {
                    "success": False,
                    "message": "缺少必要參數"
                }
            
            # 驗證設備授權
            if not await self._check_device_authorization(device_id):
                return {
                    "success": False,
                    "message": "設備授權失敗"
                }
            
            # 建立客戶端映射
            self.user_sid_map[device_id] = sid
            
            return {
                "success": True,
                "message": f"登入成功，clientId: {device_id}",
                "clientId": device_id
            }
            
        except Exception as e:
            return {
                "success": False,
                "message": f"登入失敗: {str(e)}"
            }
    
    async def _check_device_authorization(self, device_id: str) -> bool:
        """檢查設備授權（私有方法）"""
        # 授權檢查邏輯...
        return True
```

### 資料庫程式碼標準

#### SQLModel 模型定義 ✅
```python
# ✅ 正確：標準 SQLModel 模型定義
from sqlmodel import SQLModel, Field, Relationship
from typing import Optional, List
from datetime import datetime

class AgvBase(SQLModel):
    """AGV 基礎模型"""
    name: str = Field(max_length=50, description="AGV 名稱")
    model: str = Field(max_length=20, description="AGV 型號")
    x: float = Field(description="X 座標")
    y: float = Field(description="Y 座標")
    heading: float = Field(description="方向角")
    enable: int = Field(default=1, description="啟用狀態")

class Agv(AgvBase, table=True):
    """AGV 資料表模型"""
    __tablename__ = "agv"
    
    id: Optional[int] = Field(default=None, primary_key=True)
    last_node: Optional[int] = Field(default=None, foreign_key="node.id")
    status_id: Optional[int] = Field(default=None, foreign_key="agv_status.id")
    battery_level: Optional[float] = Field(default=None, description="電池電量")
    current_position: Optional[str] = Field(default=None, description="當前位置")
    description: Optional[str] = Field(default=None, description="描述")
    
    # 關聯關係
    tasks: List["Task"] = Relationship(back_populates="agv")
    status: Optional["AgvStatus"] = Relationship(back_populates="agvs")

class AgvCreate(AgvBase):
    """AGV 建立模型"""
    pass

class AgvUpdate(SQLModel):
    """AGV 更新模型"""
    name: Optional[str] = None
    x: Optional[float] = None
    y: Optional[float] = None
    heading: Optional[float] = None
    battery_level: Optional[float] = None
    current_position: Optional[str] = None
```

#### CRUD 操作標準 ✅
```python
# ✅ 正確：標準 CRUD 操作
from sqlmodel import Session, select
from typing import List, Optional

class AgvCRUD:
    """AGV CRUD 操作類別"""
    
    def __init__(self, model_class=Agv):
        self.model_class = model_class
    
    def create(self, session: Session, obj_in: AgvCreate) -> Agv:
        """建立 AGV 記錄"""
        db_obj = self.model_class.from_orm(obj_in)
        session.add(db_obj)
        session.commit()
        session.refresh(db_obj)
        return db_obj
    
    def get_by_id(self, session: Session, agv_id: int) -> Optional[Agv]:
        """根據 ID 取得 AGV"""
        statement = select(self.model_class).where(self.model_class.id == agv_id)
        return session.exec(statement).first()
    
    def get_all(
        self, 
        session: Session, 
        skip: int = 0, 
        limit: int = 100
    ) -> List[Agv]:
        """取得所有 AGV"""
        statement = select(self.model_class).offset(skip).limit(limit)
        return session.exec(statement).all()
    
    def update(
        self, 
        session: Session, 
        db_obj: Agv, 
        obj_in: AgvUpdate
    ) -> Agv:
        """更新 AGV 記錄"""
        update_data = obj_in.dict(exclude_unset=True)
        for field, value in update_data.items():
            setattr(db_obj, field, value)
        
        session.add(db_obj)
        session.commit()
        session.refresh(db_obj)
        return db_obj
    
    def delete(self, session: Session, agv_id: int) -> bool:
        """刪除 AGV 記錄"""
        db_obj = self.get_by_id(session, agv_id)
        if db_obj:
            session.delete(db_obj)
            session.commit()
            return True
        return False

# 全域 CRUD 實例
agv_crud = AgvCRUD()
```

## 🌐 JavaScript/TypeScript 程式碼標準

### ES6+ 語法使用 ✅
```javascript
// ✅ 正確：使用現代 JavaScript 語法
class SocketAPI {
    constructor(url) {
        this.socket = io(url);
        this.eventHandlers = new Map();
        this.isConnected = false;
    }
    
    // 使用箭頭函數保持 this 綁定
    connect = () => {
        return new Promise((resolve, reject) => {
            this.socket.on('connect', () => {
                this.isConnected = true;
                console.log('✅ Socket.IO 連線成功');
                resolve();
            });
            
            this.socket.on('connect_error', (error) => {
                console.error('❌ Socket.IO 連線失敗:', error);
                reject(error);
            });
        });
    }
    
    // 使用 async/await 處理非同步操作
    async emitWithResponse(event, data) {
        if (!this.isConnected) {
            throw new Error('Socket 未連線');
        }
        
        return new Promise((resolve, reject) => {
            this.socket.emit(event, data, (response) => {
                if (response.success) {
                    resolve(response);
                } else {
                    reject(new Error(response.message));
                }
            });
        });
    }
    
    // 使用解構賦值和預設參數
    addEventListener(event, handler, { once = false } = {}) {
        if (once) {
            this.socket.once(event, handler);
        } else {
            this.socket.on(event, handler);
        }
        
        // 記錄事件處理器以便清理
        if (!this.eventHandlers.has(event)) {
            this.eventHandlers.set(event, []);
        }
        this.eventHandlers.get(event).push(handler);
    }
}

// ❌ 錯誤：使用過時的語法
function SocketAPI(url) {  // 應使用 class
    var self = this;       // 應使用箭頭函數
    this.socket = io(url);
    
    this.connect = function() {  // 應使用 Promise/async-await
        self.socket.on('connect', function() {
            console.log('connected');
        });
    };
}
```

### 模組化架構標準 ✅
```javascript
// ✅ 正確：模組化架構
// store.js - 狀態管理
export class Store {
    constructor(initialState = {}) {
        this.state = { ...initialState };
        this.listeners = [];
    }
    
    setState(newState) {
        this.state = { ...this.state, ...newState };
        this.notifyListeners();
    }
    
    getState() {
        return { ...this.state };
    }
    
    subscribe(listener) {
        this.listeners.push(listener);
        return () => {
            this.listeners = this.listeners.filter(l => l !== listener);
        };
    }
    
    notifyListeners() {
        this.listeners.forEach(listener => listener(this.state));
    }
}

// homePage.js - 頁面專用功能
import { Store } from './store.js';
import { SocketAPI } from './api.js';

export class HomePage {
    constructor() {
        this.store = new Store({
            leftProducts: [],
            rightProducts: [],
            selectedProduct: { left: 0, right: 0 }
        });
        
        this.api = new SocketAPI('/');
        this.initializeEventListeners();
    }
    
    initializeEventListeners() {
        // AGV 操作按鈕事件
        document.getElementById('call-empty-left')?.addEventListener('click', 
            () => this.handleCallEmpty('left')
        );
        
        document.getElementById('dispatch-full-right')?.addEventListener('click', 
            () => this.handleDispatchFull('right')
        );
    }
    
    async handleCallEmpty(side) {
        try {
            const response = await this.api.emitWithResponse('call_empty', { side });
            this.showNotification(response.message, 'success');
        } catch (error) {
            this.showNotification(error.message, 'error');
        }
    }
    
    showNotification(message, type) {
        // 通知顯示邏輯
        console.log(`${type.toUpperCase()}: ${message}`);
    }
}

// index.js - 主入口點
import { HomePage } from './homePage.js';
import { SettingPage } from './settingPage.js';
import { RackPage } from './rackPage.js';

class App {
    constructor() {
        this.currentPage = null;
        this.initializeApp();
    }
    
    initializeApp() {
        const path = window.location.pathname;
        
        switch (path) {
            case '/home':
                this.currentPage = new HomePage();
                break;
            case '/setting':
                this.currentPage = new SettingPage();
                break;
            case '/rack':
                this.currentPage = new RackPage();
                break;
            default:
                console.warn('未知頁面路徑:', path);
        }
    }
}

// 應用初始化
document.addEventListener('DOMContentLoaded', () => {
    new App();
});
```

## 📏 程式碼品質檢查

### 自動化檢查工具

#### Python 品質檢查 ✅
```bash
# flake8 程式碼風格檢查
flake8 --max-line-length=88 --extend-ignore=E203,W503 src/

# black 程式碼格式化
black --line-length=88 src/

# mypy 類型檢查
mypy --strict src/

# pytest 測試覆蓋率
pytest --cov=src --cov-report=html --cov-report=term
```

#### JavaScript 品質檢查 ✅
```bash
# ESLint 程式碼檢查
eslint static/js/ --ext .js

# Prettier 程式碼格式化
prettier --write static/js/**/*.js

# JSDoc 文檔檢查
jsdoc -c jsdoc.conf.json static/js/
```

### 程式碼審查檢查清單

#### 通用檢查項目 ✅
- [ ] 程式碼遵循專案命名規範
- [ ] 函數和類別有適當的文檔字串
- [ ] 複雜邏輯有清晰的註解
- [ ] 錯誤處理機制完整
- [ ] 沒有硬編碼的魔術數字
- [ ] 資源正確釋放（檔案、連線等）
- [ ] 安全性考量（輸入驗證、SQL 注入防護）

#### Python 特定檢查 ✅
- [ ] 使用 Type Hints
- [ ] 遵循 PEP 8 風格指南
- [ ] 適當使用 f-string 格式化
- [ ] 異常處理具體明確
- [ ] 使用 context manager 管理資源

#### JavaScript 特定檢查 ✅
- [ ] 使用 ES6+ 語法
- [ ] 適當的錯誤處理
- [ ] 避免全域變數污染
- [ ] 事件監聽器正確清理
- [ ] 非同步操作使用 Promise/async-await

---

**最後更新**: 2025-01-23  
**維護責任**: 開發團隊、程式碼審查員  
**版本**: v1.0.0 (基於實際程式碼分析)
