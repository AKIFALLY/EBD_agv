# CLAUDE.md

## 系統概述
ROS 2 PLC服務代理工作空間，封裝keyence_plc_ws為ROS 2服務，為上層應用提供標準化PLC通訊接口。

**🔗 通訊架構**: 上層應用 → plc_proxy → keyence_plc → PLC硬體

## 核心架構
```
plc_proxy_ws/
├── plc_interfaces/             # ROS 2服務定義
│   └── srv/
│       ├── ForceOn.srv         # 強制設定ON
│       ├── ForceOff.srv        # 強制設定OFF
│       ├── ReadData.srv        # 讀取單一資料
│       ├── WriteData.srv       # 寫入單一資料
│       ├── ReadContinuousData.srv   # 連續讀取
│       ├── WriteContinuousData.srv  # 連續寫入
│       ├── ReadContinuousByte.srv   # 連續位元組讀取
│       └── WriteContinuousByte.srv  # 連續位元組寫入
└── plc_proxy/                  # 代理服務實作
    ├── plc_service.py          # ROS 2服務節點
    ├── plc_client.py           # PLC客戶端封裝
    └── plc_client_node.py      # 客戶端節點
```

## 主要組件

### 1. PlcService節點 (plc_service.py)
**ROS 2服務提供者**，提供8個PLC服務接口:
```python
class PlcService(Node):
    def __init__(self):
        super().__init__("plc_service")
        
        # 初始化PLC連線池和記憶體
        self.pool = KeyencePlcPool(self.ip, self.port)
        self.memory = PlcMemory(65536 * 2)  # 64KB PLC記憶體對應
        
        # 建立8個ROS 2服務 (使用ReentrantCallbackGroup支援並發)
        self.callback_group = ReentrantCallbackGroup()
        
        self.create_service(ForceOn, "force_on", self.force_on_callback, callback_group=self.callback_group)
        self.create_service(ForceOff, "force_off", self.force_off_callback, callback_group=self.callback_group)
        self.create_service(ReadData, "read_data", self.read_data_callback, callback_group=self.callback_group)
        self.create_service(WriteData, "write_data", self.write_data_callback, callback_group=self.callback_group)
        self.create_service(ReadContinuousData, "read_continuous_data", self.read_continuous_data_callback, callback_group=self.callback_group)
        self.create_service(WriteContinuousData, "write_continuous_data", self.write_continuous_data_callback, callback_group=self.callback_group)
        self.create_service(ReadContinuousByte, "read_continuous_byte", self.read_continuous_byte_callback, callback_group=self.callback_group)
        self.create_service(WriteContinuousByte, "write_continuous_byte", self.write_continuous_byte_callback, callback_group=self.callback_group)
```

**核心參數**:
```python
self.declare_parameter("plc_ip", "192.168.2.100")      # PLC IP地址 (預設值)
self.declare_parameter("read_ranges", ["DM,7600,200", "DM,5000,200"])  # 自動讀取範圍
self.port = 8501                                        # PLC端口 (固定值)
```

**記憶體和連線管理**:
```python
self.memory = PlcMemory(65536 * 2)      # PLC記憶體對應 (128KB)
self.pool = KeyencePlcPool(ip, port)    # Keyence PLC連線池
self.callback_group = ReentrantCallbackGroup()  # 支援並發服務調用
```

### 2. PlcClient類別 (plc_client.py) 
**客戶端封裝**，提供簡化的ROS 2服務調用接口:
```python
class PlcClient:
    def __init__(self, node: Node):
        self.node = node
        self.qos = QoSProfile(depth=100)
        self.namespace = node.get_namespace()
        
    def force_on(self, device_type, address):
        """強制設定ON - 調用ForceOn服務"""
        return self._call_sync(self.client_force_on, ForceOn.Request(device_type=device_type, address=address))
        
    def force_off(self, device_type, address):
        """強制設定OFF - 調用ForceOff服務"""
        return self._call_sync(self.client_force_off, ForceOff.Request(device_type=device_type, address=address))
        
    def read_data(self, device_type, address):
        """讀取單一資料 - 調用ReadData服務"""
        return self._call_sync(self.client_read, ReadData.Request(device_type=device_type, address=address))
        
    def write_data(self, device_type, address, value):
        """寫入單一資料 - 調用WriteData服務"""
        return self._call_sync(self.client_write, WriteData.Request(device_type=device_type, address=address, value=value))
        
    def read_continuous_data(self, device_type, start_address, count):
        """連續讀取資料 - 調用ReadContinuousData服務"""
        return self._call_sync(self.client_read_continuous, ReadContinuousData.Request(device_type=device_type, start_address=start_address, count=count))
        
    def write_continuous_data(self, device_type, start_address, values):
        """連續寫入資料 - 調用WriteContinuousData服務"""
        return self._call_sync(self.client_write_continuous, WriteContinuousData.Request(device_type=device_type, start_address=start_address, values=values))
```

### 3. PlcClientNode類別 (plc_client_node.py)
**節點封裝的客戶端**，提供節點層級的PLC客戶端功能:
```python
class PlcClientNode(Node):
    def __init__(self, node_name="plc_client", namespace=""):
        super().__init__(node_name, namespace=namespace)
        self.client = PlcClient(self)
        
    # 同步方法
    def force_on(self, device_type, address):
        return self.client.force_on(device_type, address)
        
    # 異步方法 (非阻塞)
    def async_force_on(self, device_type, address, callback):
        self.client.async_force_on(device_type, address, callback)
```

## ROS 2 服務接口定義

### 基本控制服務
```bash
# ForceOn.srv - 強制設定ON
string device_type    # 設備類型 (如 "MR")
string address        # 地址 (如 "3708")
---
bool success         # 操作是否成功
string message       # 回應訊息

# ForceOff.srv - 強制設定OFF  
string device_type
string address
---
bool success
string message
```

### 資料讀寫服務
```bash
# ReadData.srv - 讀取單一資料
string device_type    # 設備類型 (如 "DM")
string address        # 地址 (如 "2990")
---
bool success         # 操作是否成功
string value         # 讀取到的值
string message       # 回應訊息

# WriteData.srv - 寫入單一資料
string device_type
string address
string value         # 要寫入的值
---
bool success
string message
```

### 連續資料操作服務
```bash
# ReadContinuousData.srv - 連續讀取
string device_type
string start_address
int32 count          # 讀取數量
---
bool success
string[] values      # 讀取到的值陣列
string message

# WriteContinuousData.srv - 連續寫入
string device_type
string start_address
string[] values      # 要寫入的值陣列
---
bool success
string message
```

## 開發指令

### 環境設定 (容器內執行)
```bash
# AGV容器內
source /app/setup.bash && agv_source  # 或使用 all_source (自動檢測)
cd /app/plc_proxy_ws

# AGVC容器內
source /app/setup.bash && agvc_source  # 或使用 all_source (自動檢測)
cd /app/plc_proxy_ws
```

### 構建與測試
```bash
build_ws plc_proxy_ws
test_ws plc_proxy_ws
```

### 服務啟動 (容器內執行)
```bash
# 啟動PLC代理服務
ros2 run plc_proxy plc_service_node

# 使用自定義參數啟動
ros2 run plc_proxy plc_service_node --ros-args -p plc_ip:=192.168.1.100 -p read_ranges:="['DM,7600,200']"

# 啟動PLC客戶端節點
ros2 run plc_proxy plc_client_node
```

## 使用範例

### 1. 在AGV指令服務中使用PlcClient
```python
# agv_cmd_service中的實際使用範例
from plc_proxy.plc_client import PlcClient

class AgvCommandService(Node):
    def __init__(self):
        super().__init__('agv_cmd_service_node')
        
        # 建立PLC客戶端
        self.plc_comm_client = PlcClient(Node('node'), self.get_namespace())
        
    def manual_command_callback(self, request, response):
        """手動控制指令處理"""
        try:
            if request.command == "forward":
                # 使用PlcClient調用force_on服務
                self.plc_comm_client.force_on("MR", "3708")
                response.success = True
            elif request.command == "stop":
                # 使用PlcClient調用force_off服務  
                self.plc_comm_client.force_off("MR", "3708")
                response.success = True
        except Exception as e:
            self.get_logger().error(f"PLC操作失敗: {e}")
            response.success = False
        return response
```

### 2. 直接使用ROS 2服務
```bash
# 強制設定MR3708為ON
ros2 service call /force_on plc_interfaces/srv/ForceOn "{device_type: 'MR', address: '3708'}"

# 強制設定MR3708為OFF
ros2 service call /force_off plc_interfaces/srv/ForceOff "{device_type: 'MR', address: '3708'}"

# 讀取DM2990的值
ros2 service call /read_data plc_interfaces/srv/ReadData "{device_type: 'DM', address: '2990'}"

# 寫入值100到DM2990
ros2 service call /write_data plc_interfaces/srv/WriteData "{device_type: 'DM', address: '2990', value: '100'}"

# 連續讀取DM7600開始的5個值
ros2 service call /read_continuous_data plc_interfaces/srv/ReadContinuousData "{device_type: 'DM', start_address: '7600', count: 5}"
```

### 3. 在Python程式中使用PlcClientNode
```python
import rclpy
from plc_proxy.plc_client_node import PlcClientNode

def main():
    rclpy.init()
    
    # 建立PLC客戶端節點
    plc_node = PlcClientNode("my_plc_client")
    
    try:
        # 同步調用
        response = plc_node.force_on("MR", "3708")
        if response and response.success:
            print("成功設定MR3708為ON")
        
        # 讀取數據
        response = plc_node.read_data("DM", "2990")
        if response and response.success:
            print(f"DM2990的值: {response.value}")
            
        # 異步調用 (非阻塞)
        def callback(response):
            if response.success:
                print("異步操作成功")
                
        plc_node.async_force_off("MR", "3708", callback)
        
    except Exception as e:
        print(f"操作失敗: {e}")
    finally:
        plc_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4. 檢查服務狀態
```bash
# 列出所有PLC相關服務
ros2 service list | grep -E "(force_on|force_off|read_data|write_data)"

# 查看服務接口定義
ros2 interface show plc_interfaces/srv/ForceOn
ros2 interface show plc_interfaces/srv/ReadData

# 檢查PLC服務節點狀態
ros2 node info /plc_service

# 監控服務調用
ros2 topic echo /parameter_events | grep plc
```

## 故障排除

### 常見問題
1. **服務無回應**: 確認plc_service節點正在運行
   ```bash
   ros2 node list | grep plc_service
   ```

2. **PLC連線失敗**: 檢查keyence_plc_ws連線狀態和網路
   ```bash
   ping 192.168.2.100  # 檢查PLC IP連通性
   ```

3. **參數錯誤**: 驗證device_type和address格式
   ```python
   # 正確格式
   device_type: "MR" 或 "DM"
   address: "3708" (字串格式)
   ```

4. **命名空間問題**: 確認ROS 2命名空間設定正確
   ```python
   # PlcClient會自動處理命名空間
   self.namespace = node.get_namespace()
   ```

### 診斷步驟
```bash
# 1. 檢查服務節點狀態
ros2 node list | grep plc
ros2 node info /plc_service

# 2. 檢查服務可用性
ros2 service list | grep -E "(force_on|force_off|read_data|write_data)"

# 3. 測試基本服務調用
ros2 service call /force_on plc_interfaces/srv/ForceOn "{device_type: 'MR', address: '3708'}"

# 4. 檢查PLC連線 (容器內)
python3 -c "
from keyence_plc.keyence_plc_com import KeyencePlcCom
plc = KeyencePlcCom('192.168.2.100', 8501)
try:
    plc.connect()
    print('PLC連線正常')
except Exception as e:
    print(f'PLC連線失敗: {e}')
"
```

### 錯誤處理
```python
# 在客戶端代碼中實現錯誤處理
def safe_plc_operation(self, operation_func, *args):
    try:
        response = operation_func(*args)
        if response and response.success:
            return response
        else:
            self.get_logger().warn(f"PLC操作失敗: {response.message if response else '無回應'}")
    except Exception as e:
        self.get_logger().error(f"PLC服務調用異常: {e}")
    return None

# 使用範例
response = self.safe_plc_operation(self.plc_client.force_on, "MR", "3708")
```

## 性能考量

### 並發處理特性
- **ReentrantCallbackGroup**: 支援多線程並發服務調用
- **連線池管理**: KeyencePlcPool最多5個並發TCP連線
- **QoS設定**: depth=100處理高頻服務請求
- **記憶體映射**: PlcMemory提供本地緩存機制

### 最佳實踐
```python
# 1. 批量操作優於單一操作
# 好的做法：連續讀取
response = plc_client.read_continuous_data("DM", "7600", 10)

# 避免：多次單一讀取
for i in range(10):
    response = plc_client.read_data("DM", str(7600 + i))  # 效率較低

# 2. 適當的異步處理
def handle_response(response):
    if response.success:
        # 處理成功回應
        pass

plc_node.async_force_on("MR", "3708", handle_response)
```

## 系統整合架構

### 在RosAGV系統中的角色
```
AGV應用層 (agv_cmd_service_ws)
    ↓ 調用PlcClient
plc_proxy_ws (ROS 2服務層)  
    ↓ 使用KeyencePlcPool
keyence_plc_ws (純Python庫)
    ↓ TCP通訊
Keyence PLC硬體
```

### 雙環境支援
- **AGV容器**: 控制車載PLC設備 (運動控制、感測器)
- **AGVC容器**: 控制站點PLC設備 (充電站、門控、環境設備)

## 重要提醒
- plc_proxy_ws是keyence_plc_ws的ROS 2封裝層
- 提供8種標準化PLC操作服務接口
- 支援同步和異步兩種調用方式
- 自動管理連線池和錯誤重連機制
- 適用於AGV和AGVC雙環境部署
- 所有ROS 2操作需在對應容器內執行
- PlcClient是最常用的客戶端封裝，被agv_cmd_service_ws廣泛使用