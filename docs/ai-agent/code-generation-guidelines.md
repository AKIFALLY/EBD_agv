# AI Agent 程式碼生成指導

## 📋 概述

本文檔提供 AI Agent 在 RosAGV 專案中進行程式碼生成的詳細指導，基於 `docs/standards/coding-standards.md` 中分析的實際程式碼風格和專案架構模式，確保生成的程式碼符合專案技術標準。

## 🎯 程式碼生成目標

### 核心原則
- **風格一致性**: 100% 符合專案程式碼標準
- **架構相容性**: 遵循現有的架構模式和設計原則
- **品質保證**: 生成的程式碼通過所有品質檢查
- **可維護性**: 生成的程式碼易於理解和維護

### 生成範圍
- **ROS 2 節點**: 狀態機、服務節點、通訊代理
- **Web API**: FastAPI 路由、Pydantic 模型、Socket.IO 事件處理
- **資料庫**: SQLModel 模型、CRUD 操作、遷移腳本
- **前端**: JavaScript 模組、HTML 模板、CSS 樣式

## 🐍 Python 程式碼生成規範

### ROS 2 節點生成模板

#### 標準 ROS 2 節點模板
```python
# 模板: ROS 2 節點生成
"""
基於 docs/standards/coding-standards.md 中的 ROS 2 程式碼標準
"""

TEMPLATE_ROS2_NODE = '''
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from typing import Optional

class {node_class_name}(Node):
    """{node_description}
    
    {detailed_description}
    """
    
    def __init__(self):
        super().__init__('{node_name}')
        
        # 參數宣告
        {parameter_declarations}
        
        # 發布者和訂閱者
        {publishers_and_subscribers}
        
        # 定時器
        {timers}
        
        self.get_logger().info('{node_class_name} 已啟動')
    
    {callback_methods}
    
    {utility_methods}

def main(args: Optional[list] = None) -> None:
    """主函數"""
    rclpy.init(args=args)
    node = {node_class_name}()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
'''

# 生成範例
def generate_plc_proxy_node():
    return TEMPLATE_ROS2_NODE.format(
        node_class_name="PlcProxyNode",
        node_description="PLC 代理節點",
        detailed_description="負責處理 PLC 通訊和 ROS 2 訊息轉換",
        node_name="plc_proxy_node",
        parameter_declarations="""
        self.declare_parameter('plc_host', '192.168.1.100')
        self.declare_parameter('plc_port', 8501)""",
        publishers_and_subscribers="""
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
        )""",
        timers="self.timer = self.create_timer(1.0, self.timer_callback)",
        callback_methods="""
    def command_callback(self, msg: String) -> None:
        \"\"\"處理 PLC 指令\"\"\"
        self.get_logger().info(f'收到指令: {msg.data}')
        # 處理邏輯...
    
    def timer_callback(self) -> None:
        \"\"\"定時器回調\"\"\"
        status_msg = String()
        status_msg.data = 'PLC 連線正常'
        self.status_publisher.publish(status_msg)""",
        utility_methods=""
    )
```

#### AGV 狀態機生成模板
```python
# 模板: AGV 狀態機生成
"""
基於 agv_ws 中實際的狀態機實作模式
"""

TEMPLATE_AGV_STATE_MACHINE = '''
from typing import Optional, Dict, Any
import asyncio

class {agv_class_name}:
    """{agv_description}
    
    負責處理 {agv_type} AGV 的狀態轉換和業務邏輯，包括：
    {features_list}
    
    Attributes:
        agv_id (str): AGV 識別碼
        current_state (str): 當前狀態
        {additional_attributes}
    """
    
    # 狀態常數定義
    {state_constants}
    
    def __init__(self, agv_id: str, {init_parameters}):
        """初始化 {agv_type} AGV
        
        Args:
            agv_id: AGV 識別碼
            {init_parameter_docs}
            
        Raises:
            ValueError: 當參數無效時
        """
        {parameter_validation}
        
        self.agv_id = agv_id
        {attribute_initialization}
        self.current_state = "idle"
    
    {business_logic_methods}
    
    {state_transition_methods}
    
    {utility_methods}
'''

# 生成範例
def generate_loader_agv_state_machine():
    return TEMPLATE_AGV_STATE_MACHINE.format(
        agv_class_name="LoaderAgv",
        agv_description="Loader AGV 狀態機實作",
        agv_type="Loader",
        features_list="""- take_transfer 流程控制
    - AGV Port 選擇邏輯
    - 資料庫狀態同步""",
        additional_attributes="""room_id (int): 房間 ID
        port_address (int): Port 地址""",
        state_constants="""
    # Port 選擇狀態
    SELECT_PORT01 = "SELECT_PORT01"
    SELECT_PORT02 = "SELECT_PORT02"
    SELECT_PORT03 = "SELECT_PORT03"
    SELECT_PORT04 = "SELECT_PORT04"
    
    # 有效狀態列表
    VALID_STATES = [SELECT_PORT01, SELECT_PORT02, SELECT_PORT03, SELECT_PORT04]""",
        init_parameters="room_id: int",
        init_parameter_docs="room_id: 房間 ID，用於計算 port_address",
        parameter_validation="""
        if room_id <= 0:
            raise ValueError("room_id 必須大於 0")""",
        attribute_initialization="""
        self.room_id = room_id
        self.port_address = self.calculate_port_address()""",
        business_logic_methods="""
    def calculate_port_address(self) -> int:
        \"\"\"計算 AGV Port 地址
        
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
        \"\"\"
        return self.room_id * 1000 + 100""",
        state_transition_methods="""
    def transition_to_state(self, new_state: str) -> bool:
        \"\"\"狀態轉換方法\"\"\"
        if self._is_valid_transition(new_state):
            self.current_state = new_state
            return True
        return False
    
    def _is_valid_transition(self, state: str) -> bool:
        \"\"\"私有方法：驗證狀態轉換\"\"\"
        return state in self.VALID_STATES""",
        utility_methods=""
    )
```

### FastAPI 應用生成模板

#### API 路由生成模板
```python
# 模板: FastAPI 路由生成
"""
基於 web_api_ws 中實際的 FastAPI 實作模式
"""

TEMPLATE_FASTAPI_ROUTER = '''
from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel, Field
from typing import List, Optional
from sqlmodel import Session

from ..database import get_session
from ..models.{model_name} import {model_class}, {model_class}Create, {model_class}Update
from ..crud.{model_name}_crud import {model_name}_crud

router = APIRouter()

# Pydantic 回應模型
class {model_class}Response(BaseModel):
    """{model_class} 回應模型"""
    {response_fields}

@router.post("/{endpoint_path}/", response_model={model_class}Response)
async def create_{model_name}(
    {model_name}: {model_class}Create,
    session: Session = Depends(get_session)
) -> {model_class}Response:
    """建立新{model_description}
    
    Args:
        {model_name}: {model_description}建立請求
        session: 資料庫會話
        
    Returns:
        {model_class}Response: 建立的{model_description}資訊
        
    Raises:
        HTTPException: 當{model_description}建立失敗時
    """
    try:
        new_{model_name} = {model_name}_crud.create(session, {model_name})
        return {model_class}Response(
            {response_mapping}
        )
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail="內部伺服器錯誤")

@router.get("/{endpoint_path}/", response_model=List[{model_class}Response])
async def get_{model_name}s(
    skip: int = 0,
    limit: int = 100,
    session: Session = Depends(get_session)
) -> List[{model_class}Response]:
    """取得{model_description}列表"""
    {model_name}s = {model_name}_crud.get_all(session, skip=skip, limit=limit)
    return [
        {model_class}Response(
            {response_mapping}
        )
        for {model_name} in {model_name}s
    ]

@router.get("/{endpoint_path}/{{item_id}}", response_model={model_class}Response)
async def get_{model_name}(
    item_id: int,
    session: Session = Depends(get_session)
) -> {model_class}Response:
    """取得特定{model_description}"""
    {model_name} = {model_name}_crud.get_by_id(session, item_id)
    if not {model_name}:
        raise HTTPException(status_code=404, detail="{model_description}不存在")
    
    return {model_class}Response(
        {response_mapping}
    )

@router.put("/{endpoint_path}/{{item_id}}", response_model={model_class}Response)
async def update_{model_name}(
    item_id: int,
    {model_name}_update: {model_class}Update,
    session: Session = Depends(get_session)
) -> {model_class}Response:
    """更新{model_description}"""
    {model_name} = {model_name}_crud.get_by_id(session, item_id)
    if not {model_name}:
        raise HTTPException(status_code=404, detail="{model_description}不存在")
    
    updated_{model_name} = {model_name}_crud.update(session, {model_name}, {model_name}_update)
    return {model_class}Response(
        {response_mapping}
    )

@router.delete("/{endpoint_path}/{{item_id}}")
async def delete_{model_name}(
    item_id: int,
    session: Session = Depends(get_session)
) -> dict:
    """刪除{model_description}"""
    success = {model_name}_crud.delete(session, item_id)
    if not success:
        raise HTTPException(status_code=404, detail="{model_description}不存在")
    
    return {{"message": "{model_description}已成功刪除"}}
'''

# 生成範例
def generate_task_api_router():
    return TEMPLATE_FASTAPI_ROUTER.format(
        model_name="task",
        model_class="Task",
        model_description="任務",
        endpoint_path="tasks",
        response_fields="""
    id: int
    agv_id: str
    status: str
    created_at: str""",
        response_mapping="""
            id=task.id,
            agv_id=task.agv_id,
            status=task.status,
            created_at=task.created_at.isoformat()"""
    )
```

### Socket.IO 事件處理生成模板

#### Socket.IO 事件處理器模板
```python
# 模板: Socket.IO 事件處理生成
"""
基於 OPUI 和 AGVCUI 中實際的 Socket.IO 實作模式
"""

TEMPLATE_SOCKET_EVENT_HANDLER = '''
import socketio
from typing import Dict, Any, Optional

class {socket_class_name}:
    """{socket_description}"""
    
    def __init__(self, sio: socketio.AsyncServer):
        self.sio = sio
        {additional_initialization}
        
    def init_socketio(self) -> None:
        """初始化 Socket.IO 事件處理器"""
        {event_registrations}
    
    {event_handlers}
    
    {utility_methods}
'''

# 生成範例
def generate_agv_socket_handler():
    return TEMPLATE_SOCKET_EVENT_HANDLER.format(
        socket_class_name="AgvSocket",
        socket_description="AGV 操作 Socket.IO 事件處理器",
        additional_initialization="""
        self.user_sid_map: Dict[str, str] = {}
        self.task_monitor = TaskMonitor()""",
        event_registrations="""
        self.sio.on('connect')(self.connect)
        self.sio.on('disconnect')(self.disconnect)
        self.sio.on('call_empty')(self.call_empty)
        self.sio.on('dispatch_full')(self.dispatch_full)""",
        event_handlers="""
    async def connect(self, sid: str, environ: dict, auth: Optional[dict]) -> None:
        \"\"\"客戶端連線事件\"\"\"
        print(f"✅ 客戶端連線: {sid}")
        # 連線處理邏輯...
    
    async def disconnect(self, sid: str) -> None:
        \"\"\"客戶端斷線事件\"\"\"
        print(f"❌ 客戶端斷線: {sid}")
        # 清理邏輯...
    
    async def call_empty(self, sid: str, data: Dict[str, Any]) -> Dict[str, Any]:
        \"\"\"叫空車事件
        
        Args:
            sid: Socket.IO 會話 ID
            data: 叫車資料 {"side": str}
            
        Returns:
            Dict: 叫車結果 {"success": bool, "message": str}
        \"\"\"
        try:
            side = data.get("side")
            if not side:
                return {"success": False, "message": "缺少 side 參數"}
            
            # 叫車邏輯處理
            task_id = await self._create_call_empty_task(side)
            
            return {
                "success": True,
                "message": f"叫車成功，任務 ID: {task_id}"
            }
            
        except Exception as e:
            return {"success": False, "message": f"叫車失敗: {str(e)}"}""",
        utility_methods="""
    async def _create_call_empty_task(self, side: str) -> int:
        \"\"\"建立叫空車任務（私有方法）\"\"\"
        # 任務建立邏輯...
        return 123"""
    )
```

## 🗄️ 資料庫程式碼生成規範

### SQLModel 模型生成模板

#### 資料模型生成模板
```python
# 模板: SQLModel 模型生成
"""
基於 db_proxy_ws 中實際的 SQLModel 實作模式
"""

TEMPLATE_SQLMODEL_CLASS = '''
from sqlmodel import SQLModel, Field, Relationship
from typing import Optional, List
from datetime import datetime

class {model_name}Base(SQLModel):
    """{model_description}基礎模型"""
    {base_fields}

class {model_name}({{model_name}}Base, table=True):
    """{model_description}資料表模型"""
    __tablename__ = "{table_name}"
    
    id: Optional[int] = Field(default=None, primary_key=True)
    {table_fields}
    
    # 關聯關係
    {relationships}

class {model_name}Create({{model_name}}Base):
    """{model_description}建立模型"""
    pass

class {model_name}Update(SQLModel):
    """{model_description}更新模型"""
    {update_fields}
'''

# 生成範例
def generate_agv_model():
    return TEMPLATE_SQLMODEL_CLASS.format(
        model_name="Agv",
        model_description="AGV",
        table_name="agv",
        base_fields="""
    name: str = Field(max_length=50, description="AGV 名稱")
    model: str = Field(max_length=20, description="AGV 型號")
    x: float = Field(description="X 座標")
    y: float = Field(description="Y 座標")
    heading: float = Field(description="方向角")
    enable: int = Field(default=1, description="啟用狀態")""",
        table_fields="""
    last_node: Optional[int] = Field(default=None, foreign_key="node.id")
    status_id: Optional[int] = Field(default=None, foreign_key="agv_status.id")
    battery_level: Optional[float] = Field(default=None, description="電池電量")
    current_position: Optional[str] = Field(default=None, description="當前位置")
    description: Optional[str] = Field(default=None, description="描述")""",
        relationships="""
    tasks: List["Task"] = Relationship(back_populates="agv")
    status: Optional["AgvStatus"] = Relationship(back_populates="agvs")""",
        update_fields="""
    name: Optional[str] = None
    x: Optional[float] = None
    y: Optional[float] = None
    heading: Optional[float] = None
    battery_level: Optional[float] = None
    current_position: Optional[str] = None"""
    )
```

## 🌐 前端程式碼生成規範

### JavaScript 模組生成模板

#### 頁面模組生成模板
```javascript
// 模板: JavaScript 頁面模組生成
/*
基於 OPUI 中實際的 JavaScript 模組化架構
*/

const TEMPLATE_PAGE_MODULE = `
import { Store } from './store.js';
import { SocketAPI } from './api.js';

export class {PageClassName} {
    constructor() {
        this.store = new Store({
            {initial_state}
        });
        
        this.api = new SocketAPI('/');
        this.initializeEventListeners();
    }
    
    initializeEventListeners() {
        {event_listeners}
    }
    
    {page_methods}
    
    {utility_methods}
    
    showNotification(message, type) {
        // 通知顯示邏輯
        console.log(\`\${type.toUpperCase()}: \${message}\`);
    }
}
`;

// 生成範例
function generateHomePage() {
    return TEMPLATE_PAGE_MODULE
        .replace('{PageClassName}', 'HomePage')
        .replace('{initial_state}', `
            leftProducts: [],
            rightProducts: [],
            selectedProduct: { left: 0, right: 0 }`)
        .replace('{event_listeners}', `
        // AGV 操作按鈕事件
        document.getElementById('call-empty-left')?.addEventListener('click', 
            () => this.handleCallEmpty('left')
        );
        
        document.getElementById('dispatch-full-right')?.addEventListener('click', 
            () => this.handleDispatchFull('right')
        );`)
        .replace('{page_methods}', `
    async handleCallEmpty(side) {
        try {
            const response = await this.api.emitWithResponse('call_empty', { side });
            this.showNotification(response.message, 'success');
        } catch (error) {
            this.showNotification(error.message, 'error');
        }
    }
    
    async handleDispatchFull(side) {
        try {
            const rack = this.store.getState().selectedRack[side];
            const response = await this.api.emitWithResponse('dispatch_full', { side, rack });
            this.showNotification(response.message, 'success');
        } catch (error) {
            this.showNotification(error.message, 'error');
        }
    }`)
        .replace('{utility_methods}', '');
}
```

## 📏 程式碼生成品質檢查

### 自動品質檢查流程
```python
# AI Agent 程式碼品質檢查
def validate_generated_code(generated_code: str, code_type: str) -> ValidationResult:
    """驗證生成的程式碼品質"""
    
    checks = []
    
    if code_type == "python":
        # Python 特定檢查
        checks.extend([
            check_pep8_compliance(generated_code),
            check_type_hints_coverage(generated_code),
            check_docstring_coverage(generated_code),
            check_function_complexity(generated_code),
            check_function_length(generated_code)
        ])
    
    elif code_type == "javascript":
        # JavaScript 特定檢查
        checks.extend([
            check_es6_syntax(generated_code),
            check_module_structure(generated_code),
            check_error_handling(generated_code),
            check_event_cleanup(generated_code)
        ])
    
    # 通用檢查
    checks.extend([
        check_naming_conventions(generated_code),
        check_code_duplication(generated_code),
        check_security_issues(generated_code)
    ])
    
    return ValidationResult(
        passed=all(check.passed for check in checks),
        checks=checks,
        suggestions=generate_improvement_suggestions(checks)
    )
```

### 自動修正機制
```python
# AI Agent 程式碼自動修正
def auto_fix_code_issues(generated_code: str, validation_result: ValidationResult) -> str:
    """自動修正程式碼問題"""
    
    fixed_code = generated_code
    
    for check in validation_result.checks:
        if not check.passed and check.auto_fixable:
            if check.issue_type == "formatting":
                fixed_code = apply_code_formatting(fixed_code)
            elif check.issue_type == "imports":
                fixed_code = fix_import_order(fixed_code)
            elif check.issue_type == "naming":
                fixed_code = fix_naming_conventions(fixed_code)
            elif check.issue_type == "docstring":
                fixed_code = add_missing_docstrings(fixed_code)
    
    return fixed_code
```

---

**最後更新**: 2025-01-23  
**維護責任**: AI Agent 開發團隊、程式碼品質工程師  
**版本**: v1.0.0 (基於實際程式碼標準和架構模式)
