# plc_proxy_ws CLAUDE.md

## 📚 Context Loading
@docs-ai/context/system/rosagv-overview.md
@docs-ai/context/system/dual-environment.md
@docs-ai/context/system/technology-stack.md
@docs-ai/knowledge/protocols/keyence-plc-protocol.md
@docs-ai/operations/development/plc-communication.md
@docs-ai/operations/development/ros2-development.md
@docs-ai/operations/development/docker-development.md

## 🎯 適用場景
- PLC 設備的 ROS 2 服務封裝和標準化接口提供
- 上層應用與 Keyence PLC 之間的通訊代理
- AGV 和 AGVC 雙環境下的 PLC 設備控制
- 解決 PLC 通訊的服務化和並發管理問題

## 📋 模組概述

**plc_proxy_ws** 是 RosAGV 系統中的 PLC 通訊代理工作空間，將底層的 keyence_plc_ws 純 Python 通訊庫封裝為標準化的 ROS 2 服務接口，為上層應用提供統一的 PLC 操作能力。

### 核心特色
- **ROS 2 服務化**: 將 PLC 操作封裝為標準 ROS 2 服務
- **並發支援**: 使用 ReentrantCallbackGroup 支援多線程並發調用
- **連線池管理**: 集成 KeyencePlcPool 實現高效連線管理
- **雙環境支援**: 適用於 AGV 車載和 AGVC 管理雙環境
- **統一接口**: 提供 8 種標準化 PLC 操作服務

### 業務價值
- **標準化**: 統一的 ROS 2 服務接口，簡化上層應用開發
- **高效性**: 連線池和並發處理提升通訊效率
- **可靠性**: 完善的錯誤處理和重連機制
- **擴展性**: 支援多種 PLC 操作類型和設備配置

**⚠️ 重要**: 所有 ROS 2 程式必須在 Docker 容器內執行，宿主機無 ROS 2 環境。

### 通訊架構
```
上層應用 (AGV/AGVC 節點)
    ↓ ROS 2 服務調用
plc_proxy_ws (ROS 2 服務層)  
    ↓ KeyencePlcPool 連線池
keyence_plc_ws (純 Python 通訊庫)
    ↓ TCP Socket 通訊
Keyence PLC 硬體設備
```

## 🏗️ 系統架構

### 工作空間結構
```
plc_proxy_ws/
├── src/
│   ├── plc_interfaces/         # ROS 2 服務介面定義
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── srv/                # 8 個 PLC 服務定義
│   │       ├── ForceOn.srv         # 強制設定 ON
│   │       ├── ForceOff.srv        # 強制設定 OFF
│   │       ├── ReadData.srv        # 讀取單一資料
│   │       ├── WriteData.srv       # 寫入單一資料
│   │       ├── ReadContinuousData.srv   # 連續讀取資料
│   │       ├── WriteContinuousData.srv  # 連續寫入資料
│   │       ├── ReadContinuousByte.srv   # 連續位元組讀取
│   │       └── WriteContinuousByte.srv  # 連續位元組寫入
│   └── plc_proxy/              # 代理服務實作
│       ├── package.xml
│       ├── setup.py
│       ├── setup.cfg
│       ├── plc_proxy/
│       │   ├── __init__.py
│       │   ├── plc_service.py      # ROS 2 服務提供者節點
│       │   ├── plc_client.py       # PLC 客戶端封裝類別
│       │   └── plc_client_node.py  # 客戶端節點封裝
│       ├── resource/
│       └── test/
└── test/                       # 整合測試
    └── ros_batched_service_client.py
```

### 核心組件關係
```
PlcService (服務提供者)
├── KeyencePlcPool (連線池管理)
├── PlcMemory (記憶體映射)
├── ReentrantCallbackGroup (並發支援)
└── 8個 ROS 2 服務接口

PlcClient (客戶端封裝)
├── 懶加載服務客戶端
├── 命名空間自動處理
└── 同步調用封裝

PlcClientNode (節點客戶端)
├── PlcClient 實例
├── 同步方法
└── 異步方法支援
```

## 🔧 核心組件

### 1. PlcService 節點 (plc_service.py)
@docs-ai/operations/development/ros2-development.md

**PlcService** 是核心的 ROS 2 服務提供者節點，負責將 Keyence PLC 操作封裝為標準 ROS 2 服務。

#### 核心特性
- **8 個標準服務**: 提供完整的 PLC 操作接口
- **並發處理**: 使用 ReentrantCallbackGroup 支援多線程並發
- **連線池管理**: 集成 KeyencePlcPool 實現高效連線管理
- **記憶體映射**: 提供 128KB PLC 記憶體映射功能

#### 實際實作架構 (基於真實代碼)
```python
class PlcService(Node):
    def __init__(self):
        super().__init__("plc_service")
        
        # 參數配置 (實際代碼)
        self.declare_parameter("plc_ip", "192.168.2.100")
        para_ip = self.get_parameter("plc_ip").get_parameter_value().string_value
        self.get_logger().info(f"🚀PlcService Start {para_ip} !")
        
        self.declare_parameter("read_ranges", ["DM,7600,200", "DM,5000,200"])
        raw_ranges = self.get_parameter("read_ranges").get_parameter_value().string_array_value
        self.read_ranges = [tuple(r.split(",")) for r in raw_ranges]
        
        # 核心組件初始化 (實際代碼)
        self.ip = para_ip
        self.port = 8501
        self.pool = KeyencePlcPool(self.ip, self.port)
        self.memory = PlcMemory(65536 * 2)  # PLC 記憶體映射
        
        # 並發支援 (實際代碼)
        self.callback_group = ReentrantCallbackGroup()
        
        # 實際的服務註冊 (8 個服務)
        self.create_service(ForceOn, "force_on", self.force_on_callback,
                           qos_profile=rclpy.qos.QoSProfile(depth=100, 
                               reliability=rclpy.qos.ReliabilityPolicy.RELIABLE),
                           callback_group=self.callback_group)
        # ... 其他 7 個服務
```

#### 實際服務註冊模式 (來自真實代碼)
實際代碼中每個服務都是獨立註冊，使用相同的 QoS 配置：
```python
# 實際的服務註冊方式 (從 plc_service.py 第 58-100+ 行)
self.create_service(ForceOn, "force_on", self.force_on_callback,
                   qos_profile=rclpy.qos.QoSProfile(
                       depth=100, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE),
                   callback_group=self.callback_group)

self.create_service(ReadData, "read_data", self.read_data_callback,
                   qos_profile=rclpy.qos.QoSProfile(
                       depth=100, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE),
                   callback_group=self.callback_group)
# ... 總共 8 個服務，格式相同
```

#### 關鍵配置參數 (基於實際代碼)
```python
# 來自實際代碼的預設值
self.declare_parameter("plc_ip", "192.168.2.100")      # PLC IP (可配置)
self.port = 8501                                        # PLC 端口 (固定)
self.declare_parameter("read_ranges", ["DM,7600,200", "DM,5000,200"])  # 讀取範圍

# 實際的元件配置
self.pool = KeyencePlcPool(self.ip, self.port)         # 連線池
self.memory = PlcMemory(65536 * 2)                     # 128KB 記憶體映射
self.callback_group = ReentrantCallbackGroup()         # 並發支援
```

### 2. PlcClient 類別 (plc_client.py)

**PlcClient** 是 PLC 操作的客戶端封裝類別，為上層應用提供簡化的 ROS 2 服務調用接口。

#### 核心特性
- **懶加載**: 服務客戶端在首次使用時才建立
- **命名空間支援**: 自動處理 ROS 2 命名空間
- **統一介面**: 提供 8 種 PLC 操作的統一調用方法
- **同步調用**: 封裝複雜的 ROS 2 服務調用邏輯

#### 實際實作架構 (來自真實代碼)
```python
class PlcClient:
    def __init__(self, node: Node):
        self.node = node
        
        # 設定 QoS (實際代碼)
        self.qos = QoSProfile(depth=100)
        
        # 命名空間處理 (實際代碼)
        ns = node.get_namespace()
        self.namespace = '' if ns == '/' else ns
        
        # 日誌輸出 (實際代碼)
        node._logger.info(f"🧭Using namespace: {self.namespace}")
        
        # 懶加載服務客戶端，初始化為 None (實際代碼)
        self.client_force_on = None
        self.client_force_off = None
        self.client_read = None
        self.client_write = None
        self.client_read_continuous = None
        self.client_write_continuous = None
        self.client_read_continuous_byte = None
        self.client_write_continuous_byte = None
```

#### 核心方法實作 (來自真實代碼)
```python
def force_on(self, device_type, address):
    """來自 plc_client.py 第 40 行"""
    if self.client_force_on is None:
        self.client_force_on = self.node.create_client(
            ForceOn, f"{self.namespace}/force_on" if self.namespace else "force_on", 
            qos_profile=self.qos)
    return self._call_sync(self.client_force_on, 
                          ForceOn.Request(device_type=device_type, address=address))

def read_data(self, device_type, address) -> ReadData.Response | None:
    """來自 plc_client.py 第 52 行"""
    if self.client_read is None:
        self.client_read = self.node.create_client(
            ReadData, f"{self.namespace}/read_data" if self.namespace else "read_data", 
            qos_profile=self.qos)
    return self._call_sync(self.client_read, 
                          ReadData.Request(device_type=device_type, address=address))

def read_continuous_data(self, device_type, start_address, count) -> ReadContinuousData.Response | None:
    """來自 plc_client.py 第 64 行"""
    if self.client_read_continuous is None:
        self.client_read_continuous = self.node.create_client(
            ReadContinuousData, f"{self.namespace}/read_continuous_data" if self.namespace else "read_continuous_data", 
            qos_profile=self.qos)
    return self._call_sync(self.client_read_continuous, 
                          ReadContinuousData.Request(device_type=device_type, 
                                                    start_address=start_address, count=count))
```

#### 同步調用機制 (實際實作)
```python
def _call_sync(self, client, request, timeout_sec=1.0):
    """同步服務調用封裝 (來自實際代碼)"""
    wait_for_service = client.wait_for_service(timeout_sec=1.0)
    if not wait_for_service:
        self.node.get_logger().warn(
            f"Service {client.srv_type.__name__} not available")
        return None
    future = client.call_async(request)
    rclpy.spin_until_future_complete(
        self.node, future, timeout_sec=timeout_sec)
    if future.done():
        return future.result()
    else:
        self.node.get_logger().warn("Service call timed out")
        return None
```

### 3. PlcClientNode 類別 (plc_client_node.py)

**PlcClientNode** 是節點層級的 PLC 客戶端封裝，提供完整的 ROS 2 節點功能和 PLC 操作接口。

#### 核心特性
- **獨立節點**: 可作為獨立的 ROS 2 節點運行
- **PlcClient 整合**: 內建 PlcClient 實例
- **同步異步支援**: 提供同步和異步兩種調用方式
- **自動初始化**: 自動處理 rclpy 初始化

#### 實際實作架構
```python
class PlcClientNode(Node):
    def __init__(self, node_name="plc_client", namespace=""):
        # rclpy 自動初始化處理
        if not rclpy.ok():
            rclpy.init()
            
        super().__init__(node_name, namespace=namespace)
        
        # 集成 PlcClient
        self.client = PlcClient(self)
        
        self.get_logger().info(f"PlcClientNode 已初始化: {node_name}")
```

#### 同步方法封裝
```python
def force_on(self, device_type, address):
    """強制設定 ON (同步)"""
    return self.client.force_on(device_type, address)

def force_off(self, device_type, address):
    """強制設定 OFF (同步)"""
    return self.client.force_off(device_type, address)

def read_data(self, device_type, address):
    """讀取單一資料 (同步)"""
    return self.client.read_data(device_type, address)

def write_data(self, device_type, address, value):
    """寫入單一資料 (同步)"""
    return self.client.write_data(device_type, address, value)

def read_continuous_data(self, device_type, start_address, count):
    """連續讀取資料 (同步)"""
    return self.client.read_continuous_data(device_type, start_address, count)
```

#### 異步方法支援
```python
def async_force_on(self, device_type, address, callback):
    """強制設定 ON (異步)"""
    self.client.async_force_on(device_type, address, callback)

def async_read_data(self, device_type, address, callback):
    """讀取單一資料 (異步)"""
    self.client.async_read_data(device_type, address, callback)

# 異步調用範例
def callback_handler(response):
    if response and response.success:
        print(f"異步操作成功: {response.value}")
    else:
        print("異步操作失敗")

# 使用方式
plc_node.async_read_data("DM", "2990", callback_handler)
```

## 📋 ROS 2 服務介面定義
@docs-ai/knowledge/protocols/keyence-plc-protocol.md

plc_proxy_ws 提供 8 個標準化的 ROS 2 服務介面，將 Keyence PLC 原始指令封裝為結構化的服務調用：

### 服務介面對應
| ROS 2 服務 | 底層 PLC 指令 | 功能說明 |
|-----------|--------------|---------|
| ForceOn.srv | ST 指令 | 強制設定 ON |
| ForceOff.srv | RS 指令 | 強制設定 OFF |
| ReadData.srv | RD 指令 | 讀取單一資料 |
| WriteData.srv | WR 指令 | 寫入單一資料 |
| ReadContinuousData.srv | RDS 指令 | 連續讀取資料 |
| WriteContinuousData.srv | WRS 指令 | 連續寫入資料 |
| ReadContinuousByte.srv | 位元組讀取 | 連續位元組讀取 |
| WriteContinuousByte.srv | 位元組寫入 | 連續位元組寫入 |

### 統一服務介面格式
所有服務都遵循統一的請求/回應格式：
- **Request**: 包含 `device_type`、`address`、操作參數
- **Response**: 包含 `success` (bool)、`message` (string)、結果資料

詳細的協議規範和指令格式請參考：@docs-ai/knowledge/protocols/keyence-plc-protocol.md

## 🚀 開發環境設定和服務啟動
@docs-ai/operations/development/docker-development.md
@docs-ai/operations/development/ros2-development.md

### 快速啟動 (容器內執行)
```bash
# 基本環境設定 (詳細步驟請參考上方連結)
all_source && cd /app/plc_proxy_ws

# 建置 plc_proxy_ws
colcon build --packages-select plc_interfaces plc_proxy && all_source

# 啟動 PLC 代理服務
ros2 run plc_proxy plc_service_node

# 自定義參數啟動
ros2 run plc_proxy plc_service_node --ros-args \
  -p plc_ip:=192.168.2.101 \
  -p read_ranges:="['DM,7600,200','DM,5000,200']"
```

### 服務驗證
```bash
# 檢查服務狀態
ros2 service list | grep -E "(force_on|force_off|read_data|write_data)"
ros2 node info /plc_service

# 快速功能測試
ros2 service call /read_data plc_interfaces/srv/ReadData \
  "{device_type: 'DM', address: '2990'}"
```

## 💡 使用範例和實際整合

### 1. 在 AGV 指令服務中使用 PlcClient
以下是在 `agv_cmd_service_ws` 中實際使用 PlcClient 的範例：

```python
from plc_proxy.plc_client import PlcClient
from rclpy.node import Node

class AgvCommandService(Node):
    def __init__(self):
        super().__init__('agv_cmd_service_node')
        
        # 建立 PLC 客戶端 (自動處理命名空間)
        self.plc_comm_client = PlcClient(self)
        
        self.get_logger().info("AGV 指令服務已初始化")
        
    def manual_command_callback(self, request, response):
        """手動控制指令處理"""
        try:
            if request.command == "forward":
                # 強制設定前進控制位為 ON
                result = self.plc_comm_client.force_on("MR", "3708")
                if result and result.success:
                    response.success = True
                    response.message = "前進指令執行成功"
                else:
                    response.success = False
                    response.message = f"前進指令失敗: {result.message if result else '無回應'}"
                    
            elif request.command == "stop":
                # 強制設定前進控制位為 OFF
                result = self.plc_comm_client.force_off("MR", "3708")
                if result and result.success:
                    response.success = True
                    response.message = "停止指令執行成功"
                else:
                    response.success = False
                    response.message = f"停止指令失敗: {result.message if result else '無回應'}"
                    
        except Exception as e:
            self.get_logger().error(f"PLC 操作異常: {e}")
            response.success = False
            response.message = f"系統異常: {str(e)}"
            
        return response
        
    def read_agv_status(self):
        """讀取 AGV 狀態資料"""
        try:
            # 讀取 AGV 狀態記憶體
            result = self.plc_comm_client.read_data("DM", "2990")
            if result and result.success:
                return int(result.value)
            else:
                self.get_logger().warn(f"讀取狀態失敗: {result.message if result else '無回應'}")
                return None
        except Exception as e:
            self.get_logger().error(f"讀取狀態異常: {e}")
            return None
```

### 2. 直接使用 ROS 2 服務調用

```bash
# 強制設定 MR3708 為 ON
ros2 service call /force_on plc_interfaces/srv/ForceOn \
  "{device_type: 'MR', address: '3708'}"

# 強制設定 MR3708 為 OFF  
ros2 service call /force_off plc_interfaces/srv/ForceOff \
  "{device_type: 'MR', address: '3708'}"

# 讀取 DM2990 的值
ros2 service call /read_data plc_interfaces/srv/ReadData \
  "{device_type: 'DM', address: '2990'}"

# 寫入值 100 到 DM2990
ros2 service call /write_data plc_interfaces/srv/WriteData \
  "{device_type: 'DM', address: '2990', value: '100'}"

# 連續讀取 DM7600 開始的 5 個值
ros2 service call /read_continuous_data plc_interfaces/srv/ReadContinuousData \
  "{device_type: 'DM', start_address: '7600', count: 5}"

# 連續寫入多個值到 DM7600
ros2 service call /write_continuous_data plc_interfaces/srv/WriteContinuousData \
  "{device_type: 'DM', start_address: '7600', values: ['100', '200', '300']}"
```

### 3. 在 Python 程式中使用 PlcClientNode

```python
import rclpy
from plc_proxy.plc_client_node import PlcClientNode
import time

def main():
    rclpy.init()
    
    # 建立 PLC 客戶端節點
    plc_node = PlcClientNode("my_plc_client")
    
    try:
        # 1. 同步調用範例
        print("=== 同步調用範例 ===")
        
        # 強制設定 MR3708 為 ON
        response = plc_node.force_on("MR", "3708")
        if response and response.success:
            print("✅ 成功設定 MR3708 為 ON")
        else:
            print(f"❌ 設定失敗: {response.message if response else '無回應'}")
        
        # 讀取 DM2990 的值  
        response = plc_node.read_data("DM", "2990")
        if response and response.success:
            print(f"📊 DM2990 的值: {response.value}")
        else:
            print(f"❌ 讀取失敗: {response.message if response else '無回應'}")
            
        # 連續讀取資料
        response = plc_node.read_continuous_data("DM", "7600", 5)
        if response and response.success:
            print(f"📊 DM7600-7604 的值: {response.values}")
        else:
            print(f"❌ 連續讀取失敗: {response.message if response else '無回應'}")
            
        # 2. 異步調用範例 (如果支援)
        print("\n=== 異步調用範例 ===")
        
        def callback_handler(response):
            if response and response.success:
                print("✅ 異步操作成功")
            else:
                print(f"❌ 異步操作失敗: {response.message if response else '無回應'}")
                
        # 異步強制設定 MR3708 為 OFF
        if hasattr(plc_node, 'async_force_off'):
            plc_node.async_force_off("MR", "3708", callback_handler)
            time.sleep(1)  # 等待異步回應
        
        # 3. 批量操作範例
        print("\n=== 批量操作範例 ===")
        
        # 連續寫入多個值
        values_to_write = ["100", "200", "300", "400", "500"]
        response = plc_node.write_continuous_data("DM", "7700", values_to_write)
        if response and response.success:
            print("✅ 批量寫入成功")
        else:
            print(f"❌ 批量寫入失敗: {response.message if response else '無回應'}")
        
    except Exception as e:
        print(f"💥 程式異常: {e}")
    finally:
        plc_node.destroy_node()
        rclpy.shutdown()
        print("🔚 程式結束")

if __name__ == '__main__':
    main()
```

### 4. 檢查服務狀態和診斷

```bash
# 列出所有 PLC 相關服務
ros2 service list | grep -E "(force_on|force_off|read_data|write_data|continuous)"

# 查看服務介面定義
ros2 interface show plc_interfaces/srv/ForceOn
ros2 interface show plc_interfaces/srv/ReadData
ros2 interface show plc_interfaces/srv/ReadContinuousData

# 檢查 PLC 服務節點狀態
ros2 node info /plc_service

# 檢查服務可用性
ros2 service type /force_on
ros2 service type /read_data

# 監控節點活動
ros2 node list | grep plc
ros2 topic list | grep plc

# 檢查參數配置
ros2 param list /plc_service
ros2 param get /plc_service plc_ip
ros2 param get /plc_service read_ranges
```

## 🚨 故障排除
@docs-ai/operations/maintenance/troubleshooting.md
@docs-ai/operations/maintenance/system-diagnostics.md
@docs-ai/operations/tools/unified-tools.md

### plc_proxy_ws 特定問題

#### 服務無回應快速診斷
```bash
# 快速檢查
ros2 node list | grep plc_service
ros2 service list | grep force_on

# 重啟服務
ros2 run plc_proxy plc_service_node
```

#### PLC 連線診斷
```bash
# 檢查 PLC IP 設定
ros2 param get /plc_service plc_ip

# 測試底層連線 (容器內)
python3 -c "
from keyence_plc.keyence_plc_com import KeyencePlcCom
plc = KeyencePlcCom('192.168.2.100', 8501)
try:
    plc.connect()
    print('✅ PLC 連線正常')
    plc.disconnect()
except Exception as e:
    print(f'❌ PLC 連線失敗: {e}')
"
```

#### 參數格式提醒
```python
# 正確的參數格式
device_type: "MR" 或 "DM"    # 字串，不是數字
address: "3708"              # 字串格式
values: ["100", "200"]       # 字串陣列
```

### 統一診斷工具
```bash
r quick-diag                 # 系統綜合診斷
r agvc-check                 # AGVC 健康檢查
r network-check              # 網路連接檢查
```

通用的故障排除流程和系統診斷方法請參考上方的 docs-ai 連結。

## ⚡ 效能最佳化
@docs-ai/operations/development/plc-communication.md

### plc_proxy_ws 特有效能特性
- **ReentrantCallbackGroup**: 支援多線程並發服務調用
- **懶加載客戶端**: PlcClient 服務客戶端按需建立
- **QoS 最佳化**: depth=100 處理高頻服務請求
- **統一錯誤處理**: 標準化的錯誤回應機制

### plc_proxy_ws 使用建議
```python
# ✅ 使用連續操作服務
response = plc_client.read_continuous_data("DM", "7600", 10)

# ✅ 重用 PlcClient 實例  
class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.plc_client = PlcClient(self)  # 重用，不要重複建立

# ✅ 適當的錯誤處理
response = self.plc_client.read_data("DM", "2990")
if response and response.success:
    value = response.value
else:
    self.get_logger().warn(f"讀取失敗: {response.message if response else '無回應'}")
```

更多 PLC 通訊效能最佳化技巧請參考：@docs-ai/operations/development/plc-communication.md

## 🏗️ 系統整合架構

### plc_proxy_ws 在 RosAGV 中的定位
```
上層應用 (agv_cmd_service_ws, agv_base 等)
    ↓ 調用 PlcClient
plc_proxy_ws (ROS 2 服務封裝層)  
    ↓ 使用 KeyencePlcPool
keyence_plc_ws (底層通訊庫)
    ↓ TCP Socket
Keyence PLC 硬體
```

### 核心價值
- **服務化**: 將 PLC 操作標準化為 ROS 2 服務
- **並發支援**: 多線程處理提升系統響應性
- **統一介面**: 為所有上層應用提供一致的 PLC 操作接口

## 💡 開發要點

- **ROS 2 封裝**: 將 keyence_plc_ws 封裝為標準 ROS 2 服務
- **8 個標準服務**: 涵蓋所有基本 PLC 操作需求
- **PlcClient 廣泛使用**: 是 agv_cmd_service_ws 等模組的核心依賴
- **雙環境支援**: 在 AGV 和 AGVC 容器中都可使用

## 🔗 交叉引用

### 相關模組
- **Keyence PLC 通訊庫**: `app/keyence_plc_ws/CLAUDE.md` - 底層 Python 通訊庫
- **手動控制服務**: `app/agv_cmd_service_ws/CLAUDE.md` - PlcClient 主要使用者

### 通用指導
- **Keyence 協議詳解**: @docs-ai/knowledge/protocols/keyence-plc-protocol.md
- **PLC ROS 2 介面規範**: @docs-ai/knowledge/protocols/plc-ros2-interfaces.md
- **PLC 開發最佳實踐**: @docs-ai/operations/development/plc-communication.md
- **ROS 2 開發指導**: @docs-ai/operations/development/ros2-development.md
- **容器開發環境**: @docs-ai/operations/development/docker-development.md

### 運維支援
- **系統診斷工具**: @docs-ai/operations/maintenance/system-diagnostics.md
- **故障排除流程**: @docs-ai/operations/maintenance/troubleshooting.md
- **統一工具系統**: @docs-ai/operations/tools/unified-tools.md

### 系統架構
- **雙環境架構**: @docs-ai/context/system/dual-environment.md
- **技術棧說明**: @docs-ai/context/system/technology-stack.md
- **模組索引導航**: @docs-ai/context/structure/module-index.md