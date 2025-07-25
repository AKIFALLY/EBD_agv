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

# 檢查服務狀態
ros2 service list | grep -E "(ManualCommand|GeneralCommand)"
```

### 構建與測試
```bash
build_ws agv_cmd_service_ws
test_ws agv_cmd_service_ws
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

## 故障排除

### 常見問題
1. **服務無回應**: 確認 plc_proxy_ws 正常運行
2. **PLC通訊失敗**: 檢查PLC連接狀態和地址配置
3. **指令執行失敗**: 查看節點日誌確認錯誤原因
4. **參數格式錯誤**: 確認GeneralCommand的parameter格式正確

### 診斷步驟
```bash
# 1. 檢查節點運行狀態
ros2 node list | grep agv_cmd_service

# 2. 檢查PLC連接
ros2 topic echo /plc_proxy/status

# 3. 查看詳細日誌
ros2 run agv_cmd_service agv_cmd_service_node --ros-args --log-level DEBUG
```

## 重要提醒
- 本服務直接控制AGV運動，使用時需注意安全
- break和enable指令會直接觸發force_on，無論onoff參數值
- send_mission指令的parameter格式為 "on,from,to,magic"
- 所有PLC通訊都透過plc_proxy_ws進行
- 僅適用於AGV車載系統，需在AGV容器內運行