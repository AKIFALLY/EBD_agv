# CLAUDE.md

## 系統概述
AGV手動指令服務工作空間，提供AGV遠程手動控制功能，透過ROS 2服務接口控制PLC實現AGV運動和任務管理。

**🚗 運行環境**: AGV車載系統  
**🔗 核心依賴**: plc_proxy_ws (PLC通訊)

## 核心架構
```
agv_cmd_service_ws/
├── agv_cmd_interfaces/          # ROS 2服務接口定義
│   └── srv/
│       ├── ManualCommand.srv    # 手動運動控制服務
│       └── GeneralCommand.srv   # 一般指令控制服務
└── agv_cmd_service/            # 指令服務實作
    └── agv_cmd_service_node.py # 主要服務節點
```

## 主要功能

### 1. 手動運動控制 (ManualCommand服務)
**支援指令**:
- `forward/backward`: 前進/後退控制
- `rotate_left/rotate_right`: 左轉/右轉控制  
- `shift_left/shift_right`: 左移/右移控制
- `break`: 緊急煞車
- `enable`: 啟用AGV

**PLC地址映射**:
```python
forward_address: '3708'     backward_address: '3709'
rotate_left_address: '3712' rotate_right_address: '3713'  
shift_left_address: '3801'  shift_right_address: '3802'
break_address: '3714'       enable_address: '3715'
```

### 2. 一般系統控制 (GeneralCommand服務)  
**支援指令**:
- `auto`: 自動模式開關 (參數: on/off)
- `stop`: 緊急停止
- `reset`: 系統重置
- `send_mission`: 發送任務 (參數: from,to,magic)
- `cancel_mission`: 取消任務
- `traffic_stop`: 交通停止控制 (參數: on/off)

**任務管理PLC地址**:
```python
send_mission_from_address: '2990'   # 任務起點
send_mission_to_address: '2991'     # 任務終點  
send_mission_magic_address: '2993'  # 任務魔法數字
cancel_mission_address: '7001'      # 取消任務
```

## 開發指令

### 環境設定 (AGV容器內)
```bash
source /app/setup.bash && all_source
cd /app/agv_cmd_service_ws
```

### 服務啟動
```bash
# 啟動AGV指令服務
ros2 run agv_cmd_service agv_cmd_service_node

# 測試指令服務連線
ros2 run agv_cmd_service test_command_service
```

### 構建與測試
```bash
build_ws agv_cmd_service_ws
ros2 test agv_cmd_service  # 指令服務測試
```

## 指令系統開發

### 指令基礎架構
```python
# commands/base_command.py
from abc import ABC, abstractmethod

class BaseCommand(ABC):
    def __init__(self, command_id: str, parameters: dict):
        self.command_id = command_id
        self.parameters = parameters
        self.status = "pending"
        self.result = None
        self.error_message = None
        
    @abstractmethod
    async def validate(self) -> tuple[bool, str]:
        """驗證指令參數"""
        pass
        
    @abstractmethod
    async def execute(self) -> tuple[bool, dict]:
        """執行指令"""
        pass
        
    @abstractmethod
    async def rollback(self):
        """指令回滾"""
        pass
```

### 運動控制指令
```python
# commands/motion_commands.py
class MoveToPositionCommand(BaseCommand):
    def __init__(self, target_x: float, target_y: float, target_theta: float = 0.0):
        super().__init__("move_to_position", {
            'target_x': target_x,
            'target_y': target_y, 
            'target_theta': target_theta
        })
        
    async def validate(self) -> tuple[bool, str]:
        """驗證移動指令參數"""
        x, y = self.parameters['target_x'], self.parameters['target_y']
        
        # 檢查座標範圍
        if not self.is_valid_coordinate(x, y):
            return False, f"目標座標超出有效範圍: ({x}, {y})"
            
        # 檢查障礙物
        if await self.check_obstacles(x, y):
            return False, f"目標位置存在障礙物: ({x}, {y})"
            
        return True, "指令驗證通過"
        
    async def execute(self) -> tuple[bool, dict]:
        """執行移動指令"""
        try:
            # 發送移動指令給AGV狀態機
            move_request = MoveRequest(
                target_x=self.parameters['target_x'],
                target_y=self.parameters['target_y'],
                target_theta=self.parameters['target_theta']
            )
            
            result = await self.agv_client.move_to_position(move_request)
            
            if result.success:
                self.status = "completed"
                return True, {"final_position": result.final_position}
            else:
                self.status = "failed"
                self.error_message = result.error_message
                return False, {"error": result.error_message}
                
        except Exception as e:
            self.status = "error"
            self.error_message = str(e)
            return False, {"error": str(e)}
```

### 指令執行器
```python
# executors/command_executor.py
class CommandExecutor:
    def __init__(self):
        self.active_commands = {}
        self.command_history = []
        self.agv_client = self.create_agv_client()
        
    async def execute_command(self, command: BaseCommand) -> dict:
        """執行單一指令"""
        self.active_commands[command.command_id] = command
        
        try:
            # 驗證指令
            is_valid, validation_message = await command.validate()
            if not is_valid:
                return {
                    'success': False,
                    'error': f"指令驗證失敗: {validation_message}"
                }
                
            # 執行指令
            command.status = "executing"
            success, result = await command.execute()
            
            # 記錄結果
            command.result = result
            self.command_history.append(command)
            
            return {
                'success': success,
                'command_id': command.command_id,
                'result': result
            }
            
        except Exception as e:
            command.status = "error"
            command.error_message = str(e)
            return {
                'success': False,
                'error': str(e)
            }
        finally:
            if command.command_id in self.active_commands:
                del self.active_commands[command.command_id]
```

### 指令序列執行
```python
# executors/sequence_executor.py
class SequenceExecutor:
    def __init__(self, command_executor: CommandExecutor):
        self.command_executor = command_executor
        self.sequence_history = []
        
    async def execute_sequence(self, commands: List[BaseCommand]) -> dict:
        """執行指令序列"""
        sequence_id = f"seq_{int(time.time())}"
        results = []
        
        for i, command in enumerate(commands):
            try:
                result = await self.command_executor.execute_command(command)
                results.append(result)
                
                # 如果指令失敗，停止執行序列
                if not result['success']:
                    return {
                        'sequence_id': sequence_id,
                        'success': False,
                        'completed_commands': i,
                        'failed_at': i,
                        'results': results,
                        'error': f"序列在第{i+1}個指令失敗"
                    }
                    
            except Exception as e:
                return {
                    'sequence_id': sequence_id,
                    'success': False,
                    'completed_commands': i,
                    'error': str(e)
                }
                
        return {
            'sequence_id': sequence_id,
            'success': True,
            'completed_commands': len(commands),
            'results': results
        }
```

## 服務介面

### ROS 2服務定義
```python
# agv_cmd_service/command_service.py
class AGVCommandService:
    def __init__(self):
        self.command_executor = CommandExecutor()
        self.sequence_executor = SequenceExecutor(self.command_executor)
        self.create_services()
        
    def create_services(self):
        """創建ROS 2服務"""
        # 單一指令服務
        self.single_command_service = self.create_service(
            ExecuteCommand, '/agv_cmd/execute', self.handle_execute_command
        )
        
        # 指令序列服務
        self.sequence_service = self.create_service(
            ExecuteSequence, '/agv_cmd/execute_sequence', self.handle_execute_sequence
        )
        
        # 狀態查詢服務
        self.status_service = self.create_service(
            GetCommandStatus, '/agv_cmd/get_status', self.handle_get_status
        )
        
    async def handle_execute_command(self, request, response):
        """處理單一指令執行請求"""
        try:
            command = self.create_command_from_request(request)
            result = await self.command_executor.execute_command(command)
            
            response.success = result['success']
            response.command_id = result.get('command_id', '')
            response.result = json.dumps(result.get('result', {}))
            if not result['success']:
                response.error_message = result.get('error', '')
                
        except Exception as e:
            response.success = False
            response.error_message = str(e)
            
        return response
```

## 指令配置

### 指令參數配置
```yaml
# /app/config/agv/cmd_service_config.yaml
agv_cmd_service:
  # 指令執行設定
  execution:
    timeout: 300.0          # 指令超時時間(秒)
    max_concurrent: 5       # 最大並發指令數
    retry_attempts: 3       # 失敗重試次數
    
  # 安全限制
  safety_limits:
    max_linear_velocity: 2.0   # 最大線性速度(m/s)
    max_angular_velocity: 1.57 # 最大角速度(rad/s) 
    min_obstacle_distance: 0.5 # 最小障礙物距離(m)
    
  # 座標範圍限制
  coordinate_limits:
    x_min: -50.0
    x_max: 50.0
    y_min: -25.0  
    y_max: 25.0
    
  # 支援的指令類型
  supported_commands:
    - name: "move_to_position"
      description: "移動到指定位置"
      parameters: ["target_x", "target_y", "target_theta"]
      
    - name: "rotate_to_angle"
      description: "旋轉到指定角度"
      parameters: ["target_angle"]
      
    - name: "stop_immediately"
      description: "立即停止"
      parameters: []
      
    - name: "get_current_status"
      description: "獲取當前狀態"
      parameters: []
```

### 指令模板
```yaml
# 常用指令模板
command_templates:
  # 移動到充電站
  move_to_charging:
    command_type: "move_to_position"
    parameters:
      target_x: 0.0
      target_y: 0.0
      target_theta: 0.0
    description: "移動到充電站"
    
  # 原地旋轉180度
  turn_around:
    command_type: "rotate_to_angle"
    parameters:
      target_angle: 3.14159
    description: "原地掉頭"
```

## 測試與調試

### 指令測試
```bash
# 測試移動指令
ros2 service call /agv_cmd/execute agv_cmd_msgs/srv/ExecuteCommand "{
  command_type: 'move_to_position',
  parameters: '{\"target_x\": 5.0, \"target_y\": 3.0, \"target_theta\": 1.57}'
}"

# 測試狀態查詢
ros2 service call /agv_cmd/get_status agv_cmd_msgs/srv/GetCommandStatus "{command_id: ''}"

# 測試指令序列
ros2 service call /agv_cmd/execute_sequence agv_cmd_msgs/srv/ExecuteSequence "{
  commands: [
    {command_type: 'move_to_position', parameters: '{\"target_x\": 2.0, \"target_y\": 0.0}'},
    {command_type: 'rotate_to_angle', parameters: '{\"target_angle\": 1.57}'}
  ]
}"
```

### 調試工具
```bash
# 查看指令歷史
ros2 topic echo /agv_cmd/command_history

# 監控指令執行狀態
ros2 topic echo /agv_cmd/execution_status

# 指令服務診斷
ros2 run agv_cmd_service command_diagnostics
```

## 安全機制

### 指令驗證
```python
# validators/command_validator.py
class CommandValidator:
    def __init__(self, config):
        self.safety_limits = config['safety_limits']
        self.coordinate_limits = config['coordinate_limits']
        
    def validate_move_command(self, parameters: dict) -> tuple[bool, str]:
        """驗證移動指令安全性"""
        x, y = parameters.get('target_x'), parameters.get('target_y')
        
        # 座標範圍檢查
        if not self.is_coordinate_in_bounds(x, y):
            return False, f"座標超出允許範圍: ({x}, {y})"
            
        # 障礙物檢查
        if self.has_obstacles_at_position(x, y):
            return False, f"目標位置存在障礙物: ({x}, {y})"
            
        return True, "指令驗證通過"
```

### 緊急停止
```python
# 緊急停止機制
class EmergencyStopHandler:
    def __init__(self):
        self.emergency_active = False
        
    def trigger_emergency_stop(self):
        """觸發緊急停止，取消所有執行中的指令"""
        self.emergency_active = True
        
        # 停止所有執行中的指令
        for command_id, command in self.active_commands.items():
            command.status = "cancelled"
            
        # 發送緊急停止指令給AGV
        self.send_emergency_stop_to_agv()
```

## 故障排除

### 常見問題
1. **指令驗證失敗**: 檢查參數範圍與安全限制
2. **指令執行超時**: 調整超時時間或檢查AGV狀態
3. **AGV無回應**: 確認AGV狀態機服務運行正常
4. **座標超出範圍**: 檢查座標限制配置

### 診斷工具
```bash
# 檢查指令服務狀態
ros2 service call /agv_cmd/get_service_status

# 查看活躍指令
ros2 topic echo /agv_cmd/active_commands

# 測試AGV連線
ros2 run agv_cmd_service test_agv_connection
```

## 監控與統計

### 指令統計
```python
# 指令執行統計
class CommandStatistics:
    def __init__(self):
        self.total_commands = 0
        self.successful_commands = 0
        self.failed_commands = 0
        self.average_execution_time = 0.0
        
    def update_statistics(self, command: BaseCommand, execution_time: float):
        """更新指令執行統計"""
        self.total_commands += 1
        
        if command.status == "completed":
            self.successful_commands += 1
        else:
            self.failed_commands += 1
            
        # 更新平均執行時間
        self.average_execution_time = (
            (self.average_execution_time * (self.total_commands - 1) + execution_time) 
            / self.total_commands
        )
```

## 重要提醒
- 指令服務直接控制AGV運動，安全檢查至關重要
- 所有指令必須經過嚴格驗證才能執行
- 緊急停止功能不可禁用或繞過
- 僅限AGV車載系統使用，確保在正確容器環境中運行