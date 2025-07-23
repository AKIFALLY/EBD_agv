# PLC 代理工作空間 (plc_proxy_ws)

## 📋 基本資訊

**啟動狀態**: ✅ 自動載入 (容器啟動腳本中自動載入但不執行特定節點)
**運行環境**: 🚗🖥️ 共用 (AGV 車載系統 + AGVC 管理系統)
**主要功能**: PLC 通訊代理服務和 ROS 2 服務介面
**依賴狀態**: 依賴 `keyence_plc_ws` 套件，被 `ecs_ws` 和其他需要 PLC 通訊的模組使用

## 📋 專案概述

PLC 代理工作空間提供與 PLC (可程式邏輯控制器) 通訊的代理服務，實現 ROS 2 系統與工業 PLC 設備之間的橋接功能。該工作空間將 Keyence PLC 的底層通訊功能封裝為標準化的 ROS 2 服務介面，支援單一讀寫、連續讀寫、強制控制等操作。

此工作空間作為 RosAGV 系統中 PLC 通訊的中間層，提供了完整的 ROS 2 服務介面，包括 8 種不同的服務類型，支援同步和非同步操作。它還具備自動讀取功能，可定期從 PLC 讀取指定範圍的資料並快取到本地記憶體中，提高資料存取效率。

## 🔗 依賴關係

### 系統套件依賴
- **ROS 2**: `rclpy`, `rclpy.qos`, `rclpy.executors`, `rclpy.callback_groups`
- **Python 標準庫**: `threading`, `time`, `logging`

### 依賴的工作空間
- **keyence_plc_ws**: 使用 `KeyencePlcPool`、`KeyencePlcCommand`、`PlcMemory`、`PlcBytes`

### 被依賴的工作空間
- **ecs_ws**: 設備控制系統 - 使用 PLC 代理服務進行設備通訊
- **agv_ws**: AGV 核心系統 - 可能使用 PLC 服務進行狀態讀寫

### 外部依賴
- **Keyence PLC 設備**: 透過 TCP/IP (預設 port 8501) 進行通訊

## 🏗️ 專案結構

```
plc_proxy_ws/
├── src/
│   ├── plc_proxy/                 # PLC 代理主套件
│   │   ├── plc_proxy/
│   │   │   ├── plc_service.py    # PLC 服務節點 (主要服務實作)
│   │   │   ├── plc_client.py     # PLC 客戶端 (服務調用封裝)
│   │   │   └── __init__.py
│   │   ├── package.xml           # 套件依賴配置
│   │   └── setup.py              # Python 套件設定
│   └── plc_interfaces/            # PLC 服務介面定義
│       ├── srv/                  # ROS 2 服務定義
│       │   ├── ForceOn.srv       # 強制開啟 MR 位元
│       │   ├── ForceOff.srv      # 強制關閉 MR 位元
│       │   ├── ReadData.srv      # 讀取單一資料
│       │   ├── WriteData.srv     # 寫入單一資料
│       │   ├── ReadContinuousData.srv    # 連續讀取多個資料
│       │   ├── WriteContinuousData.srv   # 連續寫入多個資料
│       │   ├── ReadContinuousByte.srv    # 連續讀取位元組
│       │   └── WriteContinuousByte.srv   # 連續寫入位元組
│       ├── CMakeLists.txt        # CMake 建置配置
│       └── package.xml           # 介面套件配置
├── test/                         # 測試檔案
│   └── ros_batched_service_client.py  # 批次服務客戶端測試
└── README.md                     # 本檔案
```

## ⚙️ 主要功能

### 1. ROS 2 服務介面
- **8 種服務類型**: 涵蓋所有 PLC 操作需求
- **同步服務**: 阻塞式服務調用，適合即時操作
- **非同步服務**: 非阻塞式回調機制，提高系統效能
- **多執行緒支援**: 使用 `ReentrantCallbackGroup` 支援並發請求

### 2. PLC 通訊管理
- **連線池整合**: 使用 `KeyencePlcPool` 管理 PLC 連線
- **自動重連**: 連線失效時自動重新建立連線
- **錯誤處理**: 完整的通訊錯誤處理和回報機制
- **效能最佳化**: 連線復用和資源管理

### 3. 記憶體操作
- **位元操作**: MR (Memory Relay) 位元讀寫和強制控制
- **資料操作**: DM (Data Memory) 字組讀寫
- **連續操作**: 批次讀寫多個記憶體位址
- **位元組操作**: 低階位元組級別操作

### 4. 自動讀取功能
- **定時讀取**: 每秒自動讀取指定範圍的 PLC 資料
- **資料快取**: 將讀取的資料快取到本地 `PlcMemory`
- **可配置範圍**: 支援多個讀取範圍配置
- **效能提升**: 減少重複的 PLC 通訊請求

### 5. 服務客戶端封裝
- **PlcClient 類別**: 提供簡化的 PLC 服務調用介面
- **命名空間支援**: 支援 ROS 2 命名空間配置
- **同步/非同步**: 同時提供同步和非同步調用方法
- **錯誤處理**: 內建超時和錯誤處理機制

## 📡 ROS 2 服務介面

### 服務節點資訊
- **節點名稱**: `plc_service`
- **服務數量**: 8 個
- **QoS 設定**: `depth=100`, `RELIABLE`
- **回調群組**: `ReentrantCallbackGroup` (支援並發)

### 基本讀寫服務

#### 1. ReadData.srv - 讀取單一資料
```yaml
# Request
string device_type    # "DM" (Data Memory) 或 "MR" (Memory Relay)
string address        # PLC 位址，如 "7600"

---
# Response
bool success          # 操作是否成功
string value          # 讀取的值 (字串格式)
string message        # 錯誤訊息或狀態說明
```

#### 2. WriteData.srv - 寫入單一資料
```yaml
# Request
string device_type    # "DM" 或 "MR"
string address        # PLC 位址，如 "1000"
string value          # 要寫入的值 (字串格式)

---
# Response
bool success          # 操作是否成功
string message        # 錯誤訊息或狀態說明
```

### 連續讀寫服務

#### 3. ReadContinuousData.srv - 連續讀取多個資料
```yaml
# Request
string device_type    # "DM" 或 "MR"
string start_address  # 起始位址，如 "7600"
int32 count          # 讀取數量

---
# Response
bool success          # 操作是否成功
string[] values       # 讀取的值陣列
string message        # 錯誤訊息或狀態說明
```

#### 4. WriteContinuousData.srv - 連續寫入多個資料
```yaml
# Request
string device_type    # "DM" 或 "MR"
string start_address  # 起始位址
string[] values       # 要寫入的值陣列

---
# Response
bool success          # 操作是否成功
string message        # 錯誤訊息或狀態說明
```

### 位元組操作服務

#### 5. ReadContinuousByte.srv - 連續讀取位元組
```yaml
# Request
string device_type    # "DM" 或 "MR"
string start_address  # 起始位址
int32 count          # 讀取位元組數量

---
# Response
bool success          # 操作是否成功
uint8[] values        # 讀取的位元組陣列
string message        # 錯誤訊息或狀態說明
```

#### 6. WriteContinuousByte.srv - 連續寫入位元組
```yaml
# Request
string device_type    # "DM" 或 "MR"
string start_address  # 起始位址
uint8[] values        # 要寫入的位元組陣列

---
# Response
bool success          # 操作是否成功
string message        # 錯誤訊息或狀態說明
```

### 強制控制服務

#### 7. ForceOn.srv - 強制開啟 MR 位元
```yaml
# Request
string device_type    # 通常為 "MR"
string address        # MR 位址，如 "100"

---
# Response
bool success          # 操作是否成功
string message        # 錯誤訊息或狀態說明
```

#### 8. ForceOff.srv - 強制關閉 MR 位元
```yaml
# Request
string device_type    # 通常為 "MR"
string address        # MR 位址，如 "100"

---
# Response
bool success          # 操作是否成功
string message        # 錯誤訊息或狀態說明
```

#### ForceOff.srv
```yaml
# Request
string device_type    # 通常為 "MR"
string address        # MR 位址，如 "100"

---
# Response
bool success          # 操作是否成功
string message        # 錯誤訊息或狀態說明
```

### 連續操作服務

#### ReadContinuousData.srv
```yaml
# Request
string device_type    # "DM" 或 "MR"
string start_address  # 起始位址，如 "1000"
int32 count          # 讀取數量

---
# Response
bool success          # 操作是否成功
string[] values       # 讀取的值陣列 (字串格式)
string message        # 錯誤訊息或狀態說明
```

#### WriteContinuousData.srv
```yaml
# Request
string device_type    # "DM" 或 "MR"
string start_address  # 起始位址
string[] values       # 要寫入的值陣列

---
# Response
bool success          # 操作是否成功
string message        # 錯誤訊息或狀態說明
```

### 位元組操作服務

#### ReadContinuousByte.srv
```yaml
# Request
string device_type    # 設備類型
string start_address  # 起始位址
int32 count          # 讀取位元組數量

---
# Response
bool success          # 操作是否成功
uint8[] values        # 讀取的位元組陣列
string message        # 錯誤訊息或狀態說明
```

#### WriteContinuousByte.srv
```yaml
# Request
string device_type    # 設備類型
string start_address  # 起始位址
uint8[] values        # 要寫入的位元組陣列

---
# Response
bool success          # 操作是否成功
string message        # 錯誤訊息或狀態說明
```

## 🚀 使用方法

### 1. 建置工作空間
```bash
# 載入 ROS 2 環境並建置
source /opt/ros/jazzy/setup.bash && source /opt/ws_rmw_zenoh/install/setup.bash && cd /app/plc_proxy_ws && colcon build
source install/setup.bash
```

### 2. 啟動 PLC 服務
```bash
# 使用預設配置啟動 (預設 IP: 192.168.12.224)
ros2 run plc_proxy plc_service

# 指定 PLC IP 位址
ros2 run plc_proxy plc_service --ros-args -p plc_ip:="192.168.12.224"

# 指定讀取範圍參數
ros2 run plc_proxy plc_service --ros-args \
  -p plc_ip:="192.168.12.224" \
  -p read_ranges:="['DM,7600,200','DM,5000,200']"

# 在命名空間中啟動
ros2 run plc_proxy plc_service --ros-args -r __ns:=/agvc
```

### 3. 檢查服務狀態
```bash
# 檢查服務是否啟動
ros2 service list | grep plc

# 檢查服務類型
ros2 service type /read_data

# 測試服務連線
ros2 service call /read_data plc_interfaces/srv/ReadData \
  "{device_type: 'DM', address: '7600'}"
```

### 4. 服務參數配置
```yaml
# 主要參數
plc_ip: "192.168.12.224"          # PLC IP 位址
read_ranges:                      # 自動讀取範圍
  - "DM,7600,200"                # DM7600 開始讀取 200 個 word
  - "DM,5000,200"                # DM5000 開始讀取 200 個 word

# 進階參數
timer_period: 1.0                 # 自動讀取週期 (秒)
connection_timeout: 5.0           # 連線超時 (秒)
max_pool_size: 5                  # 連線池大小
```

## 🔧 核心 API

### PlcService 節點
```python
# 節點名稱: plc_service
# 命名空間: 可配置 (預設為根命名空間)

# 提供的服務:
# - /force_on          (ForceOn)
# - /force_off         (ForceOff)
# - /read_data         (ReadData)
# - /write_data        (WriteData)
# - /read_continuous_data    (ReadContinuousData)
# - /write_continuous_data   (WriteContinuousData)
# - /read_continuous_byte    (ReadContinuousByte)
# - /write_continuous_byte   (WriteContinuousByte)

# 自動功能:
# - 定時讀取 PLC 資料 (每秒)
# - 資料快取到本地記憶體
# - 連線池管理和自動重連
```

### 使用 PlcClient 類別 (推薦)
```python
import rclpy
from rclpy.node import Node
from plc_proxy.plc_client import PlcClient

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')

        # 初始化 PLC 客戶端
        self.plc_client = PlcClient(self)

    def read_agv_status(self):
        """讀取 AGV 狀態資料"""
        # 讀取 AGV ID
        response = self.plc_client.read_data("DM", "7600")
        if response and response.success:
            agv_id = response.value
            self.get_logger().info(f"AGV ID: {agv_id}")

        # 讀取電池電量
        response = self.plc_client.read_data("DM", "7610")
        if response and response.success:
            battery = response.value
            self.get_logger().info(f"電池電量: {battery}")

    def write_agv_command(self, command_value):
        """寫入 AGV 命令"""
        response = self.plc_client.write_data("DM", "1000", str(command_value))
        if response and response.success:
            self.get_logger().info("命令寫入成功")
        else:
            self.get_logger().error(f"命令寫入失敗: {response.message if response else 'No response'}")

    def force_emergency_stop(self):
        """強制緊急停止"""
        response = self.plc_client.force_on("MR", "100")
        if response and response.success:
            self.get_logger().info("緊急停止已啟動")
```

### 直接使用 ROS 2 服務
```python
import rclpy
from rclpy.node import Node
from plc_interfaces.srv import ReadData, WriteData, ReadContinuousData

class DirectServiceNode(Node):
    def __init__(self):
        super().__init__('direct_service_node')

        # 建立服務客戶端
        self.read_client = self.create_client(ReadData, '/read_data')
        self.write_client = self.create_client(WriteData, '/write_data')
        self.read_continuous_client = self.create_client(ReadContinuousData, '/read_continuous_data')

    def read_dm_data(self, address):
        """讀取單一 DM 資料"""
        request = ReadData.Request()
        request.device_type = "DM"
        request.address = str(address)

        future = self.read_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().success:
            return future.result().value
        else:
            error_msg = future.result().message if future.result() else "Service call failed"
            self.get_logger().error(f"讀取失敗: {error_msg}")
            return None

    def read_multiple_dm(self, start_address, count):
        """讀取多個連續 DM 資料"""
        request = ReadContinuousData.Request()
        request.device_type = "DM"
        request.start_address = str(start_address)
        request.count = count

        future = self.read_continuous_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().success:
            return future.result().values
        else:
            error_msg = future.result().message if future.result() else "Service call failed"
            self.get_logger().error(f"連續讀取失敗: {error_msg}")
            return None

    def write_dm_data(self, address, value):
        """寫入 DM 資料"""
        request = WriteData.Request()
        request.device_type = "DM"
        request.address = str(address)
        request.value = str(value)

        future = self.write_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        return future.result().success
```

### 4. 非同步操作範例
```python
def read_callback(response):
    """非同步讀取回調函數"""
    if response and response.success:
        print(f"讀取成功: {response.value}")
    else:
        print(f"讀取失敗: {response.message if response else 'No response'}")

def write_callback(response):
    """非同步寫入回調函數"""
    if response and response.success:
        print("寫入成功")
    else:
        print(f"寫入失敗: {response.message if response else 'No response'}")

# 非同步讀取
self.plc_client.async_read_data("DM", "7600", read_callback)

# 非同步寫入
self.plc_client.async_write_data("DM", "1000", "123", write_callback)

# 非同步連續讀取
self.plc_client.async_read_continuous_data("DM", "7600", 10, read_callback)
```

## 🧪 測試方法

### 1. 建置和測試
```bash
# 載入 ROS 2 環境並建置
source /opt/ros/jazzy/setup.bash && source /opt/ws_rmw_zenoh/install/setup.bash && cd /app/plc_proxy_ws && colcon build
source install/setup.bash

# 執行測試
colcon test
colcon test-result --verbose
```

### 2. 系統套件測試
```bash
# 測試 ROS 2 套件
python3 -c "
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
print('✅ ROS 2 套件可用')
"

# 測試 PLC 介面
python3 -c "
from plc_interfaces.srv import ReadData, WriteData, ForceOn, ForceOff
from plc_interfaces.srv import ReadContinuousData, WriteContinuousData
from plc_interfaces.srv import ReadContinuousByte, WriteContinuousByte
print('✅ PLC 介面套件可用')
"
```

### 3. 模組功能測試
```bash
# 測試 PLC 代理模組載入
python3 -c "
from plc_proxy.plc_service import PlcService
from plc_proxy.plc_client import PlcClient
print('✅ PLC 代理模組載入成功')
"

# 測試 keyence_plc 依賴
python3 -c "
from keyence_plc.keyence_plc_pool import KeyencePlcPool
from keyence_plc.keyence_plc_command import KeyencePlcCommand
from keyence_plc.keyence_plc_memory import PlcMemory
from keyence_plc.keyence_plc_bytes import PlcBytes
print('✅ Keyence PLC 依賴載入成功')
"
```

### 4. 服務功能測試 (需要實際 PLC)
```bash
# 啟動 PLC 服務 (在背景執行)
ros2 run plc_proxy plc_service --ros-args -p plc_ip:="192.168.12.224" &

# 等待服務啟動
sleep 3

# 測試讀取 DM 資料
ros2 service call /read_data plc_interfaces/srv/ReadData \
  "{device_type: 'DM', address: '7600'}"

# 測試寫入 DM 資料
ros2 service call /write_data plc_interfaces/srv/WriteData \
  "{device_type: 'DM', address: '1000', value: '123'}"

# 測試連續讀取
ros2 service call /read_continuous_data plc_interfaces/srv/ReadContinuousData \
  "{device_type: 'DM', start_address: '7600', count: 10}"

# 測試強制控制
ros2 service call /force_on plc_interfaces/srv/ForceOn \
  "{device_type: 'MR', address: '100'}"

# 停止服務
pkill -f plc_service
```

### 5. 批次服務測試
```bash
# 執行批次服務客戶端測試
cd /app/plc_proxy_ws/test
python3 ros_batched_service_client.py
```
## ⚙️ 配置說明

### PLC 連線配置
```python
# 預設配置參數
DEFAULT_PLC_IP = "192.168.12.224"    # PLC IP 位址
DEFAULT_PLC_PORT = 8501              # PLC 通訊埠
DEFAULT_READ_RANGES = [              # 自動讀取範圍
    "DM,7600,200",                   # DM7600 開始讀取 200 個 word
    "DM,5000,200"                    # DM5000 開始讀取 200 個 word
]
TIMER_PERIOD = 1.0                   # 自動讀取週期 (秒)
```

### 服務 QoS 配置
```python
# ROS 2 服務 QoS 設定
QOS_DEPTH = 100                      # 服務佇列深度
QOS_RELIABILITY = "RELIABLE"         # 可靠性政策
CALLBACK_GROUP = "ReentrantCallbackGroup"  # 支援並發
```

### 記憶體配置
```python
# PLC 記憶體配置
MEMORY_SIZE = 65536 * 2              # 記憶體大小 (131072 bytes)
WORD_SIZE = 2                        # Word 大小 (bytes)
```

### 常用服務端點
```bash
# 服務端點列表
/force_on                    # 強制開啟 MR 位元
/force_off                   # 強制關閉 MR 位元
/read_data                   # 讀取單一資料
/write_data                  # 寫入單一資料
/read_continuous_data        # 連續讀取多個資料
/write_continuous_data       # 連續寫入多個資料
/read_continuous_byte        # 連續讀取位元組
/write_continuous_byte       # 連續寫入位元組
```

## 🔧 故障排除

### 1. 服務啟動失敗
**症狀**: `ros2 run plc_proxy plc_service` 失敗
**解決方法**:
```bash
# 檢查工作空間是否正確建置
cd /app/plc_proxy_ws
colcon build

# 確認環境已載入
source install/setup.bash

# 檢查依賴工作空間
source /app/keyence_plc_ws/install/setup.bash

# 檢查 Python 路徑
python3 -c "import sys; print('\\n'.join(sys.path))"
```

### 2. PLC 連線失敗
**症狀**: 服務調用返回 `success: false`
**解決方法**:
```bash
# 檢查 PLC 網路連線
ping 192.168.12.224

# 檢查 PLC 埠是否開啟
telnet 192.168.12.224 8501

# 檢查服務參數
ros2 param list /plc_service
ros2 param get /plc_service plc_ip

# 測試基本連線
python3 -c "
from keyence_plc.keyence_plc_com import KeyencePlcCom
plc = KeyencePlcCom('192.168.12.224', 8501)
try:
    plc.connect()
    print('✅ PLC 連線成功')
    plc.disconnect()
except Exception as e:
    print(f'❌ PLC 連線失敗: {e}')
"
```

### 3. 服務調用超時
**症狀**: 服務調用長時間無回應
**解決方法**:
```bash
# 檢查服務狀態
ros2 service list | grep plc
ros2 service type /read_data

# 檢查節點狀態
ros2 node list | grep plc
ros2 node info /plc_service

# 檢查連線池狀態
ros2 topic echo /rosout | grep plc

# 重啟服務
pkill -f plc_service
ros2 run plc_proxy plc_service &
```

### 4. 記憶體存取錯誤
**症狀**: 讀寫操作返回錯誤
**解決方法**:
```bash
# 檢查位址範圍
python3 -c "
# 確認位址在有效範圍內
# DM: 0-65535, MR: 0-8191
address = 7600
if 0 <= address <= 65535:
    print(f'✅ DM{address} 位址有效')
else:
    print(f'❌ DM{address} 位址無效')
"

# 檢查資料格式
ros2 service call /read_data plc_interfaces/srv/ReadData \
  "{device_type: 'DM', address: '7600'}" --verbose
```

## 🔗 相關文檔

- **keyence_plc_ws**: 提供底層 PLC 通訊功能，本工作空間的核心依賴
- **ecs_ws**: 設備控制系統，使用本工作空間的 PLC 服務進行設備通訊
- **agv_ws**: AGV 核心系統，可能使用本工作空間的 PLC 服務
- **ROS 2 Jazzy 文檔**: [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- **Keyence PLC 官方文檔**: 參考 Keyence PLC 通訊手冊和協定規範

## 📋 ToDo 清單

### 🔴 高優先級 (緊急)
- [ ] 完善錯誤處理機制，提高服務穩定性
- [ ] 新增服務健康檢查和自動恢復功能
- [ ] 最佳化服務回應時間，減少延遲

### 🟡 中優先級 (重要)
- [ ] 新增更多 PLC 協定支援 (Modbus, Ethernet/IP)
- [ ] 實作服務效能監控和統計功能
- [ ] 新增批次操作服務，提高資料傳輸效率
- [ ] 完善單元測試和整合測試覆蓋率

### 🟢 低優先級 (改善)
- [ ] 新增 Web 介面進行 PLC 資料監控
- [ ] 支援動態服務配置和熱重載
- [ ] 新增資料驗證和格式轉換功能
- [ ] 最佳化記憶體使用和快取策略

### 🔧 技術債務
- [ ] 重構服務回調函數，提高程式碼可讀性
- [ ] 統一錯誤訊息格式和多語言支援
- [ ] 改善程式碼文檔和 API 說明完整性

### 📊 完成度追蹤
- ✅ 基礎 ROS 2 服務介面 (100%)
- ✅ PLC 通訊整合 (100%)
- ✅ 自動讀取功能 (100%)
- ✅ 服務客戶端封裝 (100%)
- ✅ 多執行緒支援 (100%)
- ⚠️ 錯誤處理機制 (80% - 需要改善)
- ⚠️ 效能監控功能 (60% - 基礎實作)
- ❌ 多協定支援 (0% - 未開始)

### 🎯 里程碑
- **v1.0.0**: ✅ 基礎服務功能完成 (當前版本)
- **v1.1.0**: 🚧 錯誤處理和監控改善
- **v2.0.0**: 📋 多協定支援和效能最佳化

### 🏆 重要成就
- ✅ 成功整合到 RosAGV 系統
- ✅ 提供完整的 ROS 2 服務介面
- ✅ 實現高效的 PLC 通訊代理
- ✅ 支援同步和非同步操作
