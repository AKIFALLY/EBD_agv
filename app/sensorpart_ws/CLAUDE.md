# sensorpart_ws CLAUDE.md

## 📚 Context Loading
../../CLAUDE.md  # 引用根目錄系統文档
@docs-ai/knowledge/business/eyewear-production-process.md

## 🎯 適用場景
- AGV 車載系統的外部感測器資料接收
- 3D 定位和 OCR 識別資料的 TCP 客戶端整合
- 感測器伺服器連線管理和資料解析
- ROS 2 系統的感測器資料橋接功能

## 📋 模組概述

**sensorpart_ws** 是 RosAGV 系統中的感測器資料接收工作空間，作為 TCP 客戶端連接外部感測器伺服器，接收並解析 3D 定位和 OCR 識別資料，為 AGV 車載系統提供視覺感測器資料支援。

### 核心特色
- **TCP 客戶端**: 連接外部感測器伺服器 (192.168.2.100:2005)
- **自動重連**: 內建重連機制確保連線穩定性
- **多執行緒設計**: 獨立執行緒處理 TCP 通訊，不阻塞 ROS 2 節點
- **資料解析**: 支援 3D 定位和 OCR 兩種資料格式解析
- **ROS 2 整合**: 提供標準 ROS 2 節點封裝

### 業務價值
- **即時定位**: 為 AGV 提供外部視覺定位資料
- **物品識別**: 支援 OCR 文字識別功能
- **系統整合**: 無縫整合到 ROS 2 生態系統
- **穩定通訊**: 自動處理網路中斷和重連

### 感測器資料接收工作空間特有功能
- **📡 TCP 客戶端**: 連接外部感測器伺服器 (192.168.2.100:2005)
- **🔄 自動重連**: 內建重連機制確保連線穩定性
- **🧵 多執行緒設計**: 獨立執行緒處理 TCP 通訊
- **📊 資料解析**: 支援 3D 定位和 OCR 兩種資料格式

### 通訊架構
```
外部感測器設備 
    ↓ TCP 伺服器 (192.168.2.100:2005)
sensorpart_ws (TCP 客戶端)
    ↓ 資料解析和存儲
ROS 2 節點 (TestSensorPartNode)
    ↓ 日誌輸出 / 可擴展為主題發布
AGV 應用層
```

## 🏗️ 系統架構

### 工作空間結構
```
sensorpart_ws/
├── src/
│   └── sensorpart/              # 感測器套件
│       ├── package.xml          # ROS 2 套件定義
│       ├── setup.py             # Python 套件設定
│       ├── setup.cfg            # 套件配置
│       ├── resource/            # 資源檔案
│       └── sensorpart/          # 核心模組
│           ├── __init__.py      # 套件初始化
│           ├── sensorpart.py    # TCP 客戶端主程式
│           └── test_sensorpart_node.py  # ROS 2 節點封裝
└── CLAUDE.md                    # 模組文檔
```

### 核心組件關係
```
SensorPart (TCP 客戶端)
├── 多執行緒 TCP 連線管理
├── 自動重連機制
├── 資料格式解析
└── 執行緒安全的資料存儲

TestSensorPartNode (ROS 2 節點)
├── SensorPart 實例管理
├── 定時器回調
├── ROS 2 日誌輸出
└── 節點生命週期管理
```

## 🔧 核心組件

### 1. SensorPart 類別 (sensorpart.py)
@docs-ai/operations/development/ros2/ros2-development.md

**SensorPart** 是核心的 TCP 客戶端類別，負責與外部感測器伺服器建立連線並接收資料。

#### 核心特性
- **多執行緒設計**: 獨立執行緒處理 TCP 通訊，不影響主程式運行
- **自動重連機制**: 連線失敗或中斷時自動重試 (5秒間隔)
- **資料解析**: 支援 3D 定位和 OCR 兩種固定格式資料
- **執行緒安全**: 使用 threading.Event 進行執行緒同步

#### 實際實作架構 (基於真實代碼)
```python
class SensorPart:
    def __init__(self, host='192.168.2.100', port=2005):
        self.host = host
        self.port = port
        self.client_socket = None
        self.is_connected = False
        self.position_data = None  # 3D定位數據存儲
        self.ocr_result = None     # OCR結果存儲
        self.thread = None
        self.stop_event = threading.Event()
```

#### 連線管理機制 (來自實際代碼)
```python
def connect(self):
    """連接到感測器服務器，支援重連機制 (來自 sensorpart.py 第 18 行)"""
    while not self.is_connected and not self.stop_event.is_set():
        try:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((self.host, self.port))
            self.is_connected = True
            print("Connected to server.")
        except socket.error as e:
            print(f"Connection failed: {e}. Retrying in 5 seconds...")
            time.sleep(5)

def start(self):
    """啟動 TCP 客戶端執行緒 (來自 sensorpart.py 第 84 行)"""
    self.thread = threading.Thread(target=self.run, daemon=True)
    self.thread.start()
    print("TCP Client started in a separate thread.")

def stop(self):
    """停止 TCP 客戶端和執行緒 (來自 sensorpart.py 第 89 行)"""
    self.stop_event.set()
    self.disconnect()
    if self.thread and self.thread.is_alive():
        print("Waiting for TCP Client thread to finish...")
        if threading.current_thread() != self.thread:
            self.thread.join(timeout=1.0)
        else:
            print("⚠️ 無法在 sensopart 執行緒內 join 自己，略過 join()")
    print("TCP Client stopped.")
```

#### 資料解析實作 (來自實際代碼)
```python
def handle_message(self, message):
    """處理接收到的感測器數據 (來自 sensorpart.py 第 36 行)"""
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
            print(f"3D Positioning Data Updated: {self.position_data}")
        else:
            print("3D Positioning Data Invalid.")
    elif match := re.match(ocr_pattern, message):
        _, ocr_string = match.groups()
        self.ocr_result = ocr_string
        print(f"OCR Result Updated: {self.ocr_result}")
    else:
        print("Unrecognized message format. Ignoring.")
```

### 2. TestSensorPartNode 類別 (test_sensorpart_node.py)

**TestSensorPartNode** 是 ROS 2 節點封裝，將 TCP 客戶端整合到 ROS 2 系統中。

#### 核心特性
- **ROS 2 節點**: 標準 ROS 2 節點實作
- **SensorPart 整合**: 內建 TCP 客戶端管理
- **定時回調**: 定期記錄接收到的感測器資料
- **優雅關閉**: 正確處理節點關閉和資源清理

#### 實際實作架構 (來自真實代碼)
```python
class TestSensorPartNode(Node):
    def __init__(self):
        super().__init__('sensorpart_node')
        self.get_logger().info("TestSensorPartNode initialized.")

        # 初始化TCP客戶端 (來自 test_sensorpart_node.py 第 12 行)
        self.tcp_client = SensorPart()
        self.tcp_client.start()

        # 創建定時器定期記錄數據 (來自 test_sensorpart_node.py 第 16 行)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        """定時回調，記錄當前接收到的數據 (來自 test_sensorpart_node.py 第 19 行)"""
        self.get_logger().info(f"Position Data: {self.tcp_client.position_data}")
        self.get_logger().info(f"OCR Result: {self.tcp_client.ocr_result}")

    def destroy_node(self):
        """節點關閉時停止 TCP 客戶端 (來自 test_sensorpart_node.py 第 25 行)"""
        self.tcp_client.stop()
        super().destroy_node()
```

#### 完整的 main 函數 (來自實際代碼)
```python
def main(args=None):
    """主函數 (來自 test_sensorpart_node.py 第 31 行)"""
    rclpy.init(args=args)
    node = TestSensorPartNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down SensorPartNode...")
    finally:
        node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()
```

## 📋 支援的資料格式

### 3D 定位資料格式
**協議格式**: `(005,P,x,y,z,rx,ry,rz)`

| 欄位 | 說明 | 範例值 |
|------|------|--------|
| 005 | 固定標識符 | 005 |
| P/F | 狀態 (P=成功, F=失敗) | P |
| x,y,z | 位置坐標 (整數) | 1250,890,120 |
| rx,ry,rz | 旋轉角度 (浮點數) | 0.5,-1.2,2.1 |

**資料範例**:
```
輸入: (005,P,1250,890,120,0.5,-1.2,2.1)

解析結果:
{
    'x': 1250,
    'y': 890,
    'z': 120,
    'rx': 0.5,
    'ry': -1.2,
    'rz': 2.1
}
```

### OCR 識別結果格式
**協議格式**: `(OCR,text)`

| 欄位 | 說明 | 範例值 |
|------|------|--------|
| OCR | 固定標識符 | OCR |
| text | 識別到的文字內容 | AGV001 |

**資料範例**:
```
輸入: (OCR,AGV001)
解析結果: ocr_result = "AGV001"
```

## 🚀 感測器專用開發

**⚠️ 通用開發環境請參考**: ../../CLAUDE.md 開發指導章節

### 感測器節點特定啟動
```bash
# 【推薦方式】透過根目錄統一工具
# 參考: ../../CLAUDE.md 開發指導

# 【直接啟動】感測器節點
ros2 run sensorpart test_sensorpart_node
```

## 💡 使用範例和實際整合

### 1. 直接使用 TCP 客戶端
```python
from sensorpart.sensorpart import SensorPart

# 建立並啟動客戶端 (來自實際代碼架構)
client = SensorPart(host='192.168.2.100', port=2005)
client.start()

# 檢查接收到的資料
if client.position_data:
    print(f"Position: {client.position_data}")
    
if client.ocr_result:
    print(f"OCR: {client.ocr_result}")

# 停止客戶端
client.stop()
```

### 2. 在 ROS 2 節點中使用
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
        
        # 建立定時器處理資料
        self.timer = self.create_timer(0.5, self.process_sensor_data)
        
    def process_sensor_data(self):
        """處理感測器資料"""
        if self.sensor_client.position_data:
            pos = self.sensor_client.position_data
            self.get_logger().info(f"AGV位置: ({pos['x']}, {pos['y']}, {pos['z']})")
            
        if self.sensor_client.ocr_result:
            self.get_logger().info(f"識別結果: {self.sensor_client.ocr_result}")
    
    def destroy_node(self):
        """節點關閉時正確停止 TCP 客戶端"""
        self.sensor_client.stop()
        super().destroy_node()
```

### 3. 自定義連線設定
```python
# 連接到自定義伺服器 (基於實際代碼構造函數)
custom_client = SensorPart(host='192.168.1.100', port=3005)

# 或使用環境變數設定
import os
host = os.getenv('SENSOR_HOST', '192.168.2.100')
port = int(os.getenv('SENSOR_PORT', '2005'))
client = SensorPart(host=host, port=port)
```

## 🚨 感測器專項故障排除

**⚠️ 通用故障排除請參考**: ../../CLAUDE.md 故障排除章節

### 感測器特有問題診斷
```bash
# TCP 連線診斷
telnet 192.168.2.100 2005  # 測試感測器伺服器連接
netstat -an | grep 2005     # 檢查端口狀態

# 感測器節點診斷
ros2 node info /sensorpart_node
ros2 run sensorpart test_sensorpart_node  # 查看即時日誌
```

### 關鍵依賴檢查
- **感測器伺服器**: 192.168.2.100:2005 需正常運行
- **網路連接**: TCP 連線穩定性
- **資料格式**: 3D定位和OCR資料格式正確性

## ⚡ 效能特性

### sensorpart_ws 特有效能特點
- **多執行緒設計**: TCP 通訊在獨立執行緒中處理，不阻塞 ROS 2 節點
- **自動重連**: 5秒間隔的自動重連機制確保連線穩定性
- **執行緒安全**: 使用 threading.Event 確保資料存取的執行緒安全
- **輕量級設計**: 純 Python 實作，資源使用量低

### 連線參數配置
```python
# 實際代碼中的預設參數 (來自 sensorpart.py 第 8 行)
host='192.168.2.100'    # 感測器伺服器IP (構造函數預設值)
port=2005               # 伺服器端口 (構造函數預設值)
# 重連間隔: 5秒 (來自 sensorpart.py 第 28 行 time.sleep(5))
# Socket 超時: 未設定 (使用系統預設)
```

## 🏗️ 系統整合架構

### sensorpart_ws 在 RosAGV 中的定位
```
外部感測器設備
    ↓ TCP 伺服器 (192.168.2.100:2005)
sensorpart_ws (TCP 客戶端)
    ↓ ROS 2 節點整合
AGV 車載系統 (定位和識別資料)
```

### 資料流向
1. **外部感測器設備** → TCP 伺服器發送資料
2. **sensorpart 客戶端** → 接收並解析 3D 定位和 OCR 資料
3. **ROS 2 節點** → 定時輸出資料到日誌
4. **擴展可能** → 可發布為 ROS 2 主題供其他節點使用

### 核心價值
- **感測器橋接**: 將外部 TCP 感測器資料引入 ROS 2 系統
- **資料解析**: 標準化 3D 定位和 OCR 資料格式
- **穩定通訊**: 自動重連確保資料接收的連續性

## 💡 開發要點

- **TCP 客戶端**: 連接外部感測器伺服器接收資料
- **多執行緒設計**: 獨立執行緒處理網路通訊
- **資料格式固定**: 支援 3D 定位 `(005,P,...)` 和 OCR `(OCR,...)` 格式
- **ROS 2 整合**: 提供節點封裝便於系統整合
- **AGV 車載專用**: 主要用於 AGV 車載系統的感測器資料接收

## 🔗 交叉引用

### 相關模組
- **AGV 狀態機**: `../agv_ws/src/agv_base/CLAUDE.md` - 感測器資料應用於定位
- **路徑規劃**: `../path_algorithm/CLAUDE.md` - 3D 定位資料整合

### 通用支援
詳細指導請參考: ../../CLAUDE.md 交叉引用章節