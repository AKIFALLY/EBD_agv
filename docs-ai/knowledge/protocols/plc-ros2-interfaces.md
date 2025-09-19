# PLC ROS 2 服務介面規範

## 🎯 適用場景
- PLC 設備的 ROS 2 服務介面設計
- 工業控制系統的標準化服務定義
- PLC 客戶端設計模式和最佳實踐
- 跨模組的 PLC 服務整合

## 📋 PLC 服務介面架構

### 標準 PLC 服務分類
```
PLC ROS 2 服務架構
├── 🎛️ 基本控制服務
│   ├── ForceOn.srv      # 強制設定 ON
│   └── ForceOff.srv     # 強制設定 OFF
├── 📊 資料讀寫服務
│   ├── ReadData.srv     # 讀取單一資料
│   └── WriteData.srv    # 寫入單一資料
├── 🔄 批量操作服務
│   ├── ReadContinuousData.srv   # 連續讀取資料
│   ├── WriteContinuousData.srv  # 連續寫入資料
│   ├── ReadContinuousByte.srv   # 連續位元組讀取
│   └── WriteContinuousByte.srv  # 連續位元組寫入
```

### PLC 指令映射原則
| ROS 2 服務 | PLC 原始指令 | 使用場景 |
|-----------|-------------|---------|
| ForceOn | ST 指令 | 設備強制啟動、測試 |
| ForceOff | RS 指令 | 設備強制停止、安全 |
| ReadData | RD 指令 | 狀態查詢、參數讀取 |
| WriteData | WR 指令 | 參數設定、控制指令 |
| ReadContinuousData | RDS 指令 | 批量狀態監控 |
| WriteContinuousData | WRS 指令 | 批量參數配置 |

## 🔧 服務介面設計規範

### 統一請求格式
所有 PLC 服務都遵循統一的請求格式：
```srv
# 基本參數
string device_type       # 設備類型 (如 "MR", "DM")
string address          # 設備地址 (字串格式)

# 操作參數 (根據服務類型)
string value            # 寫入值 (寫入類服務)
int32 count            # 讀取數量 (連續操作)
string start_address    # 起始地址 (連續操作)
string[] values        # 值陣列 (批量寫入)
uint8[] byte_values    # 位元組陣列 (位元組操作)
```

### 統一回應格式
所有 PLC 服務都遵循統一的回應格式：
```srv
---
# 標準回應欄位
bool success           # 操作是否成功
string message         # 回應訊息或錯誤描述

# 資料回應欄位 (根據服務類型)
string value           # 單一讀取值
string[] values        # 陣列讀取值
uint8[] byte_values    # 位元組陣列值
```

### 錯誤處理標準
```python
# PLC 服務錯誤分類
PLC_ERROR_CODES = {
    "SUCCESS": "操作成功",
    "CONNECTION_FAILED": "PLC 連線失敗",
    "INVALID_DEVICE_TYPE": "無效的設備類型",
    "INVALID_ADDRESS": "無效的設備地址",
    "READ_TIMEOUT": "讀取超時",
    "WRITE_TIMEOUT": "寫入超時",
    "PROTOCOL_ERROR": "協議錯誤",
    "HARDWARE_ERROR": "硬體錯誤"
}

# 標準錯誤回應格式
def create_error_response(error_code: str, details: str = ""):
    response = PLCServiceResponse()
    response.success = False
    response.message = f"{PLC_ERROR_CODES.get(error_code, 'Unknown Error')}: {details}"
    return response
```

## 🏗️ PLC 客戶端設計模式

### 懶加載服務客戶端模式
```python
class PLCClientPattern:
    """PLC 客戶端設計模式"""
    
    def __init__(self, node: Node):
        self.node = node
        self.qos = QoSProfile(depth=100)
        
        # 命名空間處理
        ns = node.get_namespace()
        self.namespace = '' if ns == '/' else ns
        
        # 懶加載客戶端 (初始化為 None)
        self.service_clients = {}
    
    def get_or_create_client(self, service_type, service_name):
        """懶加載服務客戶端"""
        if service_name not in self.service_clients:
            full_service_name = f"{self.namespace}/{service_name}" if self.namespace else service_name
            self.service_clients[service_name] = self.node.create_client(
                service_type, full_service_name, qos_profile=self.qos)
        return self.service_clients[service_name]
    
    def call_service_sync(self, client, request, timeout_sec=1.0):
        """統一的同步服務調用"""
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.node.get_logger().warn(f"Service {client.srv_type.__name__} not available")
            return None
            
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
        
        if future.done():
            return future.result()
        else:
            self.node.get_logger().warn("Service call timed out")
            return None
```

### 批量操作最佳化模式
```python
class PLCBatchOperations:
    """PLC 批量操作最佳化"""
    
    @staticmethod
    def batch_read_optimization(device_type: str, addresses: list):
        """批量讀取最佳化建議"""
        if len(addresses) <= 1:
            return "single_read", addresses
        
        # 檢查地址連續性
        sorted_addrs = sorted([int(addr) for addr in addresses])
        is_continuous = all(sorted_addrs[i] + 1 == sorted_addrs[i + 1] 
                           for i in range(len(sorted_addrs) - 1))
        
        if is_continuous:
            return "continuous_read", {
                "start_address": str(sorted_addrs[0]),
                "count": len(addresses)
            }
        else:
            return "multiple_single_reads", addresses
    
    @staticmethod
    def estimate_performance_gain(operation_type: str, count: int):
        """估算效能提升"""
        if operation_type == "continuous_read":
            # 連續讀取比多次單一讀取快 60-80%
            return f"約節省 {min(80, count * 15)}% 的通訊時間"
        return "無效能提升"
```

### 連線池整合模式
```python
class PLCConnectionPoolPattern:
    """PLC 連線池整合模式"""
    
    def __init__(self, connection_pool):
        self.pool = connection_pool
        self.retry_count = 3
        self.retry_delay = 0.5
    
    def execute_with_retry(self, operation_func, *args, **kwargs):
        """帶重試機制的操作執行"""
        for attempt in range(self.retry_count):
            try:
                return operation_func(*args, **kwargs)
            except Exception as e:
                if attempt < self.retry_count - 1:
                    time.sleep(self.retry_delay)
                    continue
                else:
                    raise e
    
    def health_check(self):
        """連線池健康檢查"""
        try:
            # 執行簡單的 PLC 查詢測試
            test_result = self.execute_with_retry(
                lambda: self.pool.get_connection().send_command("?K\r\n"))
            return test_result is not None
        except Exception:
            return False
```

## 📊 效能最佳化指導

### 服務調用效能
```python
# ✅ 推薦：使用連續操作
def efficient_data_read(plc_client, device_type, start_addr, count):
    response = plc_client.read_continuous_data(device_type, start_addr, count)
    return response.values if response and response.success else []

# ❌ 避免：多次單一操作
def inefficient_data_read(plc_client, device_type, addresses):
    results = []
    for addr in addresses:  # 每次調用都有網路延遲
        response = plc_client.read_data(device_type, addr)
        if response and response.success:
            results.append(response.value)
    return results

# ✅ 推薦：重用客戶端實例
class EfficientPLCNode(Node):
    def __init__(self):
        super().__init__('efficient_plc_node')
        self.plc_client = PLCClient(self)  # 一次建立，重複使用
    
    def read_multiple_data(self, operations):
        # 使用同一個客戶端實例進行多次操作
        results = []
        for op in operations:
            result = self.plc_client.read_data(op['device'], op['address'])
            results.append(result)
        return results
```

### QoS 配置建議
```python
# PLC 服務專用 QoS 配置
PLC_SERVICE_QOS = rclpy.qos.QoSProfile(
    depth=100,                                    # 緩衝區大小
    reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,  # 可靠傳輸
    durability=rclpy.qos.DurabilityPolicy.VOLATILE,    # 不需持久化
    history=rclpy.qos.HistoryPolicy.KEEP_LAST          # 保留最新資料
)

# 高頻率 PLC 監控 QoS
PLC_MONITORING_QOS = rclpy.qos.QoSProfile(
    depth=10,                                     # 較小緩衝區
    reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,  # 最佳努力
    durability=rclpy.qos.DurabilityPolicy.VOLATILE,
    history=rclpy.qos.HistoryPolicy.KEEP_LAST
)
```

## 🔄 跨模組整合指導

### 服務命名空間策略
```python
# 按功能分組的命名空間
PLC_NAMESPACE_STRATEGY = {
    "agv_control": "/agv1/plc",      # AGV 控制相關
    "station_control": "/station/plc", # 站點控制相關
    "safety_system": "/safety/plc",   # 安全系統相關
    "diagnostic": "/diag/plc"         # 診斷系統相關
}

# 命名空間自動路由
class PLCNamespaceRouter:
    def route_service_call(self, service_type, target_system):
        namespace = PLC_NAMESPACE_STRATEGY.get(target_system, "/plc")
        return f"{namespace}/{service_type}"
```

### 服務發現和註冊
```python
class PLCServiceRegistry:
    """PLC 服務註冊和發現"""
    
    def __init__(self):
        self.available_services = {}
    
    def register_plc_services(self, node_name, services_list):
        """註冊 PLC 服務"""
        self.available_services[node_name] = {
            'services': services_list,
            'timestamp': time.time(),
            'status': 'active'
        }
    
    def discover_plc_services(self, service_type):
        """發現可用的 PLC 服務"""
        available = []
        for node, info in self.available_services.items():
            if service_type in info['services'] and info['status'] == 'active':
                available.append(node)
        return available
```

## 🧪 測試和驗證

### 服務介面測試
```python
import unittest
from plc_interfaces.srv import ReadData, ForceOn

class TestPLCInterfaces(unittest.TestCase):
    def test_read_data_request_format(self):
        """測試讀取資料請求格式"""
        request = ReadData.Request()
        request.device_type = "DM"
        request.address = "2990"
        
        # 驗證必要欄位
        self.assertIsInstance(request.device_type, str)
        self.assertIsInstance(request.address, str)
        self.assertNotEqual(request.device_type, "")
        self.assertNotEqual(request.address, "")
    
    def test_force_on_response_format(self):
        """測試強制 ON 回應格式"""
        response = ForceOn.Response()
        response.success = True
        response.message = "操作成功"
        
        # 驗證回應格式
        self.assertIsInstance(response.success, bool)
        self.assertIsInstance(response.message, str)

class TestPLCClientPattern(unittest.TestCase):
    def test_lazy_loading_pattern(self):
        """測試懶加載模式"""
        # 模擬測試懶加載客戶端建立
        pass
    
    def test_namespace_handling(self):
        """測試命名空間處理"""
        # 測試命名空間正確處理
        pass
```

## 🔗 交叉引用
- **ROS 2 介面設計**: docs-ai/knowledge/protocols/ros2-interfaces.md - 通用介面設計原則
- **Keyence PLC 協議**: docs-ai/knowledge/protocols/keyence-plc-protocol.md - 底層協議規範
- **PLC 通訊開發**: docs-ai/operations/development/ros2/plc-communication.md - 開發實踐指導
- **ROS 2 開發**: docs-ai/operations/development/ros2/ros2-development.md - ROS 2 開發環境
- **技術棧**: docs-ai/context/system/technology-stack.md - 系統技術架構