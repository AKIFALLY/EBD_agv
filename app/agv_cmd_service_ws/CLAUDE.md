# AGV 手動指令服務 CLAUDE.md

## 📚 Context Loading
@docs-ai/context/system/rosagv-overview.md
@docs-ai/context/system/dual-environment.md
@docs-ai/context/system/technology-stack.md
@docs-ai/context/workspaces/agv-workspaces.md
@docs-ai/context/structure/module-index.md
@docs-ai/operations/development/core-principles.md
@docs-ai/operations/development/ros2-development.md
@docs-ai/operations/development/docker-development.md
@docs-ai/operations/development/plc-communication.md
@docs-ai/operations/maintenance/system-diagnostics.md
@docs-ai/operations/maintenance/troubleshooting.md
@docs-ai/operations/tools/unified-tools.md
@docs-ai/knowledge/protocols/keyence-plc-protocol.md

## 🎯 適用場景
- AGV 遠程手動控制功能開發
- PLC 通訊服務整合
- 手動運動控制和任務管理
- ROS 2 服務接口設計和實作

## 📋 模組概述

AGV 手動指令服務工作空間提供完整的 AGV 遠程手動控制解決方案，透過 ROS 2 服務接口與 PLC 通訊，實現精確的 AGV 運動控制和任務管理功能。

### 核心特色
- **雙服務架構**: ManualCommand (運動控制) + GeneralCommand (系統控制)
- **PLC 整合**: 透過 plc_proxy_ws 實現可靠的 PLC 通訊
- **安全控制**: 提供緊急煞車和啟用/停用功能
- **任務管理**: 支援完整的任務發送和取消機制
- **配置驅動**: 基於 YAML 配置的 PLC 地址映射

### 技術架構
- **運行環境**: AGV 車載系統 (Docker 容器內)
- **核心依賴**: plc_proxy_ws (PLC 通訊), agv_interfaces (共用介面)
- **通訊協定**: ROS 2 服務 + Keyence PLC 協定
- **配置管理**: YAML 配置檔案驅動

## 核心架構
```
agv_cmd_service_ws/
├── agv_cmd_interfaces/          # ROS 2服務接口定義
│   └── srv/
│       ├── ManualCommand.srv    # 手動運動控制服務
│       └── GeneralCommand.srv   # 一般指令控制服務
└── agv_cmd_service/            # 指令服務實作
    ├── agv_cmd_service_node.py # 主要服務節點
    ├── agv_cmd_client_node.py  # 指令客戶端
    ├── agv_cmd_porxy.py        # AGV指令代理類
    └── config/
        └── agv_cmd_service.yaml # PLC地址配置
```

## 主要功能

### 1. 手動運動控制 (ManualCommand服務)
**服務定義**:
```
# Request
string command
bool onoff
---
# Response  
bool success
```

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
**服務定義**:
```
# Request
string command
string parameter
---
# Response
bool success
```

**支援指令**:
- `auto`: 自動模式開關 (參數: "on"/"off")
- `stop`: 緊急停止
- `reset`: 系統重置
- `send_mission`: 發送任務 (參數格式: "on,from,to,magic")
- `cancel_mission`: 取消任務
- `traffic_stop`: 交通停止控制 (參數: "on"/"off")

**任務管理PLC地址**:
```python
auto_address1: '4001'           auto_address2: '0000'
stop_address: '3701'            reset_address: '302'
send_mission_from_address: '2990'   # 任務起點
send_mission_to_address: '2991'     # 任務終點  
send_mission_magic_address: '2993'  # 任務魔法數字
cancel_mission_address: '7001'      # 取消任務
traffic_stop_address: '7002'        # 交通停止
```

## 🚀 快速開始

### 開發環境
@docs-ai/operations/development/docker-development.md
@docs-ai/operations/development/ros2-development.md

```bash
# 標準容器開發流程
cd /app/agv_cmd_service_ws
colcon build --packages-select agv_cmd_interfaces agv_cmd_service
source install/setup.bash

# 啟動服務
ros2 run agv_cmd_service agv_cmd_service_node

# 驗證服務
ros2 service list | grep -E "(ManualCommand|GeneralCommand)"
```

## 核心類別實現

### 1. AgvCommandService (主要服務節點)
位置: `agv_cmd_service/agv_cmd_service_node.py`

```python
class AgvCommandService(Node):
    def __init__(self):
        super().__init__('agv_cmd_service_node')
        
        # 初始化PLC通訊客戶端
        self.plc_comm_client = PlcClient(Node('node'), self.get_namespace())
        
        # 創建ROS 2服務
        self.create_service(ManualCommand, 'ManualCommand', self.manual_command_callback)
        self.create_service(GeneralCommand, 'GeneralCommand', self.general_command_callback)
    
    def manual_command_callback(self, request, response):
        """處理手動控制指令"""
        # 根據指令類型對應到PLC地址並執行
        command_map = {
            "forward": self.forward_address,
            "backward": self.backward_address,
            "rotate_left": self.rotate_left_address,
            "rotate_right": self.rotate_right_address,
            "shift_left": self.shift_left_address,
            "shift_right": self.shift_right_address,
            "break": self.break_address,
            "enable": self.enable_address
        }
        
        address = command_map.get(request.command)
        if address:
            # 透過PLC客戶端發送指令
            if request.onoff:
                self.plc_comm_client.force_on("MR", address)
            else:
                self.plc_comm_client.force_off("MR", address)
            response.success = True
        
        return response
    
    def general_command_callback(self, request, response):
        """處理一般系統指令"""
        # 解析參數 (格式: "on,from,to,magic")
        para = request.parameter.split(',')
        
        if request.command == "auto":
            # 自動模式控制兩個PLC地址
            if para[0] == "on":
                self.plc_comm_client.force_on("MR", self.auto_address1)
                self.plc_comm_client.force_on("MR", self.auto_address2)
            else:
                self.plc_comm_client.force_off("MR", self.auto_address1)
                self.plc_comm_client.force_off("MR", self.auto_address2)
                
        elif request.command == "send_mission":
            # 任務發送需要寫入三個DM地址
            self.plc_comm_client.write_data("DM", self.send_mission_from_address, para[1])
            self.plc_comm_client.write_data("DM", self.send_mission_to_address, para[2])
            self.plc_comm_client.write_data("DM", self.send_mission_magic_address, para[3])
        
        response.success = True
        return response
```

### 2. AgvCommandClient (指令客戶端)
位置: `agv_cmd_service/agv_cmd_client_node.py`

```python
class AgvCommandClient:
    def __init__(self, node: Node, namespace: str = ""):
        self.node = node
        self.namespace = '/' + namespace.lstrip('/')
        
        # 創建服務客戶端
        self.manual_command_client = self.node.create_client(
            ManualCommand, f"{self.namespace}/ManualCommand"
        )
        self.general_command_client = self.node.create_client(
            GeneralCommand, f"{self.namespace}/GeneralCommand"
        )
    
    def send_manual_command(self, command: str, onoff: bool) -> bool:
        """發送手動命令"""
        request = ManualCommand.Request()
        request.command = command
        request.onoff = onoff
        
        future = self.manual_command_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
        
        if future.done() and future.result():
            return future.result().success
        return False
    
    def send_general_command(self, command: str, parameter: str) -> bool:
        """發送一般命令"""
        request = GeneralCommand.Request()
        request.command = command
        request.parameter = parameter
        
        future = self.general_command_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=0.5)
        
        if future.done() and future.result():
            return future.result().success
        return False
```

### 3. AGVCommandProxy (指令代理類)
位置: `agv_cmd_service/agv_cmd_porxy.py`

```python
class AGVCommandProxy:
    def __init__(self, node: Node):
        self.node = node
        
        # 載入PLC地址配置
        config_path = "/app/agv_cmd_service_ws/src/agv_cmd_service/config/agv_cmd_service.yaml"
        self.address_map = self.load_config(config_path)
        
        # 初始化PLC客戶端
        self.plc_client = PlcClient(node)
    
    def send_movement_command(self, direction: str, onoff: bool) -> bool:
        """發送運動指令"""
        addr = str(self.address_map[direction])
        
        def plc_cb(success: bool):
            if success:
                self.node.get_logger().info(f"✅ 指令傳送成功: {direction}")
            else:
                self.node.get_logger().error(f"❌ 指令傳送失敗: {direction}")
        
        if onoff:
            self.plc_client.async_force_on("MR", addr, plc_cb)
        else:
            self.plc_client.async_force_off("MR", addr, plc_cb)
        return True
    
    def send_mission(self, mfrom: int, mto: int, magic: int) -> bool:
        """發送任務指令"""
        # 需要同時寫入三個DM地址
        self.plc_client.async_write_data(
            "DM", int(self.address_map['send_mission_from']), str(mfrom)
        )
        self.plc_client.async_write_data(
            "DM", int(self.address_map['send_mission_to']), str(mto)
        )
        self.plc_client.async_write_data(
            "DM", int(self.address_map['send_mission_magic']), str(magic)
        )
        return True
```

## 配置文件
位置: `config/agv_cmd_service.yaml`

```json
{
    "forward": "3708",
    "backward": "3709", 
    "rotate_left": "3712",
    "rotate_right": "3713",
    "shift_left": "3801",
    "shift_right": "3802",
    "break": "3714",
    "enable": "3715",
    "auto1": "4001",
    "auto2": "0",
    "stop": "3701",
    "reset": "302",
    "send_mission_from": "2990",
    "send_mission_to": "2991",
    "send_mission_magic": "2993",
    "cancel_mission": "7001",
    "traffic_stop": "7002"
}
```

## 測試與調試

### 服務測試
```bash
# 測試手動控制指令
ros2 service call /ManualCommand agv_cmd_interfaces/srv/ManualCommand "{command: 'forward', onoff: true}"

# 測試停止指令
ros2 service call /ManualCommand agv_cmd_interfaces/srv/ManualCommand "{command: 'break', onoff: true}"

# 測試自動模式開啟
ros2 service call /GeneralCommand agv_cmd_interfaces/srv/GeneralCommand "{command: 'auto', parameter: 'on'}"

# 測試任務發送
ros2 service call /GeneralCommand agv_cmd_interfaces/srv/GeneralCommand "{command: 'send_mission', parameter: 'on,1,2,123'}"
```

### 調試工具
```bash
# 檢查服務是否運行
ros2 service list | grep -E "(ManualCommand|GeneralCommand)"

# 查看服務類型定義
ros2 interface show agv_cmd_interfaces/srv/ManualCommand
ros2 interface show agv_cmd_interfaces/srv/GeneralCommand

# 監控節點狀態
ros2 node info /agv_cmd_service_node
```

## 🔍 故障排除

### 通用診斷
@docs-ai/operations/maintenance/system-diagnostics.md
@docs-ai/operations/maintenance/troubleshooting.md
@docs-ai/operations/development/plc-communication.md

### 模組特定問題

#### 服務無回應
```bash
# 檢查服務狀態
ros2 service list | grep -E "(ManualCommand|GeneralCommand)"
ros2 service call /ManualCommand agv_cmd_interfaces/srv/ManualCommand "{command: 'enable', onoff: true}"
```

#### 配置載入失敗
```bash
# 驗證配置檔案
cat /app/agv_cmd_service_ws/src/agv_cmd_service/config/agv_cmd_service.yaml
ros2 run agv_cmd_service agv_cmd_service_node --ros-args --log-level DEBUG
```

#### 指令測試
```bash
# 手動指令測試
ros2 service call /ManualCommand agv_cmd_interfaces/srv/ManualCommand "{command: 'forward', onoff: true}"
ros2 service call /GeneralCommand agv_cmd_interfaces/srv/GeneralCommand "{command: 'auto', parameter: 'on'}"
```

## ⚠️ 重要提醒

### 安全注意事項
- **直接控制**: 本服務直接控制 AGV 運動，使用時需注意安全
- **緊急控制**: break 和 enable 指令會直接觸發 force_on，無論 onoff 參數值
- **運行環境**: 僅適用於 AGV 車載系統，必須在 AGV 容器內運行

### 使用規範
- **參數格式**: send_mission 指令的 parameter 格式為 "on,from,to,magic"
- **依賴服務**: 所有 PLC 通訊都透過 plc_proxy_ws 進行
- **配置管理**: PLC 地址配置透過 YAML 檔案管理，修改需重啟服務

## 📋 使用指導

### 指令格式
- **ManualCommand**: `{command: 'forward/backward/rotate_left/rotate_right/shift_left/shift_right/break/enable', onoff: true/false}`
- **GeneralCommand**: `{command: 'auto/stop/reset/send_mission/cancel_mission/traffic_stop', parameter: 'on/off' 或 'on,from,to,magic'}`

### 安全使用
- **運動控制**: 發送運動指令前確認 AGV 處於安全狀態
- **緊急停止**: break 和 enable 指令會直接觸發，忽略 onoff 參數
- **任務管理**: send_mission 參數格式務必為 "on,from,to,magic"

## 🔗 交叉引用
- AGV 狀態機: `app/agv_ws/src/agv_base/CLAUDE.md`
- PLC 通訊模組: `app/keyence_plc_ws/CLAUDE.md`
- PLC 代理服務: `app/plc_proxy_ws/CLAUDE.md`
- ROS 2 介面定義: `app/agv_ws/src/agv_interfaces/CLAUDE.md`
- ROS 2 開發指導: @docs-ai/operations/development/ros2-development.md
- PLC 通訊最佳實踐: @docs-ai/operations/development/plc-communication.md
- Keyence 協定詳解: @docs-ai/knowledge/protocols/keyence-plc-protocol.md
- 容器開發環境: @docs-ai/operations/development/docker-development.md
- 系統診斷工具: @docs-ai/operations/maintenance/system-diagnostics.md
- 故障排除流程: @docs-ai/operations/maintenance/troubleshooting.md