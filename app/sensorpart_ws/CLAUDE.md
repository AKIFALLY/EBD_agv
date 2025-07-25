# CLAUDE.md

## 系統概述
TCP客戶端感測器數據接收工作空間，連接外部感測器服務器接收3D定位和OCR識別數據。

**🔗 TCP連接架構**: TCP客戶端 → 外部感測器服務器 (192.168.2.100:2005)

## 核心架構
```
sensorpart_ws/
└── sensorpart/                   # TCP客戶端核心
    ├── sensorpart.py             # TCP客戶端主程式
    ├── test_sensorpart_node.py   # ROS 2節點封裝
    └── __init__.py
```

## 主要組件

### 1. SensorPart類別 (sensorpart.py)
**TCP客戶端**，連接外部感測器服務器接收數據:
```python
class SensorPart:
    def __init__(self, host='192.168.2.100', port=2005):
        self.host = host
        self.port = port
        self.client_socket = None
        self.is_connected = False
        self.position_data = None  # 3D定位數據存儲
        self.ocr_result = None     # OCR結果存儲
```

**連接管理**:
```python
def connect(self):
    """連接到感測器服務器，支援重連機制"""
    while not self.is_connected and not self.stop_event.is_set():
        try:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((self.host, self.port))
            self.is_connected = True
            print("Connected to server.")
        except socket.error as e:
            print(f"Connection failed: {e}. Retrying in 5 seconds...")
            time.sleep(5)
```

**數據處理**:
```python
def handle_message(self, message):
    """處理接收到的感測器數據"""
    # 3D定位數據格式: (005,P,x,y,z,rx,ry,rz)
    position_pattern = r"\((005),(P|F),(\d+),(\d+),(\d+),([-\d.]+),([-\d.]+),([-\d.]+)\)"
    
    # OCR結果格式: (OCR,text)
    ocr_pattern = r"\((OCR),(.+)\)"
    
    if match := re.match(position_pattern, message):
        _, status, x, y, z, rx, ry, rz = match.groups()
        if status == 'P':  # 成功狀態
            self.position_data = {
                'x': int(x),
                'y': int(y), 
                'z': int(z),
                'rx': float(rx),
                'ry': float(ry),
                'rz': float(rz)
            }
    elif match := re.match(ocr_pattern, message):
        _, ocr_string = match.groups()
        self.ocr_result = ocr_string
```

### 2. TestSensorPartNode類別 (test_sensorpart_node.py)
**ROS 2節點封裝**，將TCP客戶端整合到ROS 2系統:
```python
class TestSensorPartNode(Node):
    def __init__(self):
        super().__init__('sensorpart_node')
        
        # 初始化TCP客戶端
        self.tcp_client = SensorPart()
        self.tcp_client.start()
        
        # 創建定時器定期記錄數據
        self.timer = self.create_timer(1.0, self.timer_callback)
        
    def timer_callback(self):
        """定時回調，記錄當前接收到的數據"""
        self.get_logger().info(f"Position Data: {self.tcp_client.position_data}")
        self.get_logger().info(f"OCR Result: {self.tcp_client.ocr_result}")
```

## 支援的數據格式

### 1. 3D定位數據
**格式**: `(005,P,x,y,z,rx,ry,rz)`
- `005`: 固定標識符
- `P`: 成功狀態 (F表示失敗)
- `x,y,z`: 位置坐標 (整數)
- `rx,ry,rz`: 旋轉角度 (浮點數)

**範例**:
```
(005,P,1250,890,120,0.5,-1.2,2.1)
```

**解析結果**:
```python
{
    'x': 1250,
    'y': 890,
    'z': 120,
    'rx': 0.5,
    'ry': -1.2,
    'rz': 2.1
}
```

### 2. OCR識別結果
**格式**: `(OCR,text)`
- `OCR`: 固定標識符
- `text`: 識別到的文字內容

**範例**:
```
(OCR,AGV001)
```

**解析結果**:
```python
ocr_result = "AGV001"
```

## 開發指令

### 環境設定 (AGV容器內執行)
```bash
# AGV容器內
source /app/setup.bash && agv_source  # 或使用 all_source (自動檢測)
cd /app/sensorpart_ws
```

### 構建與測試
```bash
build_ws sensorpart_ws
```

### 節點啟動 (AGV容器內執行)
```bash
# 啟動ROS 2節點 (包含TCP客戶端)
ros2 run sensorpart test_sensorpart_node

# 或單獨運行TCP客戶端
ros2 run sensorpart sensorpart
```

## 使用範例

### 1. 直接使用TCP客戶端
```python
from sensorpart.sensorpart import SensorPart

# 創建並啟動客戶端
client = SensorPart(host='192.168.2.100', port=2005)
client.start()

# 檢查接收到的數據
if client.position_data:
    print(f"Position: {client.position_data}")
    
if client.ocr_result:
    print(f"OCR: {client.ocr_result}")

# 停止客戶端
client.stop()
```

### 2. 在ROS 2節點中使用
```python
import rclpy
from rclpy.node import Node
from sensorpart.sensorpart import SensorPart

class MySensorNode(Node):
    def __init__(self):
        super().__init__('my_sensor_node')
        
        # 初始化TCP客戶端
        self.sensor_client = SensorPart()
        self.sensor_client.start()
        
        # 創建定時器處理數據
        self.timer = self.create_timer(0.5, self.process_sensor_data)
        
    def process_sensor_data(self):
        """處理感測器數據"""
        if self.sensor_client.position_data:
            pos = self.sensor_client.position_data
            self.get_logger().info(f"AGV位置: ({pos['x']}, {pos['y']}, {pos['z']})")
            
        if self.sensor_client.ocr_result:
            self.get_logger().info(f"識別結果: {self.sensor_client.ocr_result}")
```

### 3. 測試連接
```bash
# 測試TCP連接 (AGV容器內)
telnet 192.168.2.100 2005

# 檢查網路連通性
ping 192.168.2.100

# 查看端口狀態
netstat -tlnp | grep 2005
```

## 配置參數

### 連接參數
```python
# 預設連接參數
DEFAULT_HOST = '192.168.2.100'  # 感測器服務器IP
DEFAULT_PORT = 2005             # 服務器端口

# 重連參數
RECONNECT_INTERVAL = 5          # 重連間隔(秒)
SOCKET_TIMEOUT = None           # Socket超時設定
```

### 自定義連接設定
```python
# 連接到自定義服務器
custom_client = SensorPart(host='192.168.1.100', port=3005)

# 或在環境變數中設定
import os
host = os.getenv('SENSOR_HOST', '192.168.2.100')
port = int(os.getenv('SENSOR_PORT', '2005'))
client = SensorPart(host=host, port=port)
```

## 故障排除

### 常見問題
1. **無法連接服務器**: 檢查網路連通性和服務器狀態
   ```bash
   ping 192.168.2.100
   telnet 192.168.2.100 2005
   ```

2. **連接頻繁斷開**: 檢查網路穩定性或服務器負載
   - 客戶端會自動重連，檢查重連日誌

3. **數據格式錯誤**: 檢查接收到的原始訊息格式
   - 查看 "Unrecognized message format" 警告

4. **ROS 2節點無回應**: 檢查TCP客戶端狀態
   ```bash
   ros2 node info /sensorpart_node
   ```

### 診斷步驟
```bash
# 1. 檢查網路連接
ping 192.168.2.100
nmap -p 2005 192.168.2.100

# 2. 測試TCP連接
telnet 192.168.2.100 2005

# 3. 檢查ROS 2節點狀態
ros2 node list | grep sensor
ros2 node info /sensorpart_node

# 4. 查看日誌輸出
ros2 run sensorpart test_sensorpart_node
```

### 錯誤處理
```python
# 在節點中添加錯誤處理
def timer_callback(self):
    try:
        if not self.tcp_client.is_connected:
            self.get_logger().warn("TCP客戶端未連接")
            return
            
        # 檢查數據時效性
        if self.last_data_time and (time.time() - self.last_data_time) > 10:
            self.get_logger().warn("數據超時，超過10秒未收到新數據")
            
    except Exception as e:
        self.get_logger().error(f"處理感測器數據時發生錯誤: {e}")
```

## 技術特性

### 多線程設計
- **主線程**: ROS 2節點運行
- **TCP線程**: 獨立線程處理TCP連接和數據接收
- **線程安全**: 使用threading.Event進行線程同步

### 自動重連機制
- 連接失敗時自動重試 (5秒間隔)
- 連接中斷時自動重新建立連接
- 優雅的關閉和清理機制

### 數據解析
- 使用正則表達式解析固定格式訊息
- 支援3D定位和OCR兩種數據類型
- 容錯處理，忽略無法識別的訊息格式

## 系統整合

### 在AGV系統中的角色
```
外部感測器設備 (192.168.2.100:2005)
    ↓ TCP通訊
sensorpart_ws (TCP客戶端)
    ↓ ROS 2節點
AGV應用層 (定位和識別數據處理)
```

### 數據流向
1. **外部感測器** → TCP服務器 (192.168.2.100:2005)
2. **sensorpart客戶端** → 接收並解析數據
3. **ROS 2節點** → 定時輸出到日誌
4. **上層應用** → 可擴展為發布ROS 2話題

## 重要提醒
- sensorpart_ws僅提供基本的TCP客戶端功能
- 適用於AGV車載系統接收外部感測器數據
- 當前實現僅支援日誌輸出，可擴展為ROS 2話題發布
- 所有操作必須在AGV容器內執行
- 需確保與感測器服務器的網路連通性