# PLC 通訊開發最佳實踐

## 🎯 適用場景
- PLC 通訊模組的開發和整合
- 工業控制系統的穩定性設計
- PLC 相關問題的診斷和解決

## 📋 RosAGV PLC 通訊架構

### 架構層次
```
RosAGV PLC 通訊架構
├── 應用層: AGV 狀態機、手動控制
├── 服務層: plc_proxy_ws (ROS 2 服務)
├── 通訊層: keyence_plc_ws (TCP Socket)
└── 硬體層: Keyence PLC 設備
```

### 模組關係
docs-ai/context/structure/module-index.md

- **keyence_plc_ws**: 純 Python 通訊庫
- **plc_proxy_ws**: ROS 2 服務封裝
- **agv_cmd_service_ws**: 上層應用介面

## 🔧 開發環境設定

### 容器內開發
docs-ai/operations/development/docker-development.md

```bash
# 進入對應容器
docker compose -f docker-compose.yml exec rosagv bash      # AGV 環境
docker compose -f docker-compose.agvc.yml exec agvc_server bash  # AGVC 環境

# 載入工作空間
all_source

# 切換到 PLC 相關目錄
cd /app/keyence_plc_ws  # 或 /app/plc_proxy_ws
```

## 🏗️ 架構設計原則

### 分層設計
1. **通訊層分離**: keyence_plc_ws 專注純通訊
2. **服務層封裝**: plc_proxy_ws 提供 ROS 2 介面
3. **應用層抽象**: 上層不直接操作 TCP Socket
4. **錯誤處理隔離**: 各層獨立的錯誤處理機制

### 連接管理
```python
# 連接池設計模式
class KeyencePlcPool:
    def __init__(self, ip, port, max_connections=5):
        self.max_connections = max_connections
        self.connections = []
        self.lost_connections = []
        self.semaphore = threading.Semaphore(max_connections)
    
    def get_connection(self):
        """取得可用連接"""
        self.semaphore.acquire()
        # 實現連接分配邏輯
        
    def return_connection(self, connection):
        """歸還連接到池中"""
        # 實現連接歸還邏輯
        self.semaphore.release()
```

### 錯誤處理策略
```python
# 多層錯誤處理
class PlcErrorHandler:
    @staticmethod
    def handle_connection_error(error):
        """處理連接錯誤"""
        # 重連邏輯
        
    @staticmethod  
    def handle_protocol_error(response):
        """處理協議錯誤"""
        if response[:2] in ERROR_MESSAGES:
            raise PlcProtocolError(ERROR_MESSAGES[response[:2]])
            
    @staticmethod
    def handle_timeout_error(error):
        """處理超時錯誤"""
        # 超時處理邏輯
```

## 🚀 開發工作流程

### 1. 通訊層開發 (keyence_plc_ws)
```python
# 基本通訊類別開發
from keyence_plc.keyence_plc_com import KeyencePlcCom
from keyence_plc.keyence_plc_command import KeyencePlcCommand

class MyPlcService:
    def __init__(self, ip, port):
        self.plc = KeyencePlcCom(ip, port)
        
    def read_device(self, device_type, address):
        """讀取設備資料"""
        cmd = KeyencePlcCommand.read_data(device_type, address)
        return self.plc.send_command(cmd)
```

### 2. 服務層開發 (plc_proxy_ws)
```python
# ROS 2 服務節點開發
import rclpy
from rclpy.node import Node
from plc_interfaces.srv import PlcRead

class PlcProxyNode(Node):
    def __init__(self):
        super().__init__('plc_proxy_node')
        self.service = self.create_service(
            PlcRead, 'plc_read', self.plc_read_callback)
            
    def plc_read_callback(self, request, response):
        """PLC 讀取服務回調"""
        # 使用 keyence_plc_ws 執行實際通訊
        result = self.plc_service.read_device(
            request.device_type, request.address)
        response.data = result
        return response
```

### 3. 應用層整合
```python
# 在 AGV 狀態機中使用 PLC 服務
class AgvStateMachine:
    def __init__(self):
        self.plc_client = self.create_client(PlcRead, 'plc_read')
        
    async def read_plc_status(self):
        """讀取 PLC 狀態"""
        request = PlcRead.Request()
        request.device_type = "DM"
        request.address = "2990"
        
        future = self.plc_client.call_async(request)
        response = await future
        return response.data
```

## ⚡ 效能最佳化

### 連接池最佳化
```python
# 連接池配置建議
PLC_POOL_CONFIG = {
    'min_connections': 1,      # 最小連接數
    'max_connections': 5,      # 最大連接數
    'reconnect_interval': 5,   # 重連間隔 (秒)
    'connection_timeout': 10,  # 連接超時 (秒)
    'idle_timeout': 300,       # 閒置超時 (秒)
}
```

### 批量操作最佳化
```python
# 使用連續讀寫提高效能
def read_multiple_data(self, device_type, start_address, count):
    """批量讀取資料"""
    cmd = KeyencePlcCommand.read_continuous_data(
        device_type, start_address, count)
    return self.plc.send_command(cmd)

def write_multiple_data(self, device_type, start_address, data_list):
    """批量寫入資料"""
    cmd = KeyencePlcCommand.write_continuous_data(
        device_type, start_address, data_list)
    return self.plc.send_command(cmd)
```

### 快取策略
```python
# 實施適當的快取機制
class PlcDataCache:
    def __init__(self, cache_timeout=1.0):
        self.cache = {}
        self.cache_timeout = cache_timeout
        
    def get_cached_data(self, key):
        """取得快取資料"""
        if key in self.cache:
            data, timestamp = self.cache[key]
            if time.time() - timestamp < self.cache_timeout:
                return data
        return None
        
    def set_cached_data(self, key, data):
        """設定快取資料"""
        self.cache[key] = (data, time.time())
```

## 🔍 測試和驗證

### 單元測試
docs-ai/operations/development/testing/testing-procedures.md

```python
# PLC 通訊測試
import unittest
from keyence_plc.keyence_plc_com import KeyencePlcCom
from keyence_plc.mock_keyence_plc_com import MockKeyencePlcCom

class TestPlcCommunication(unittest.TestCase):
    def setUp(self):
        # 使用模擬 PLC 進行測試
        self.plc = MockKeyencePlcCom("localhost", 8501)
        
    def test_connection(self):
        """測試 PLC 連接"""
        result = self.plc.connect()
        self.assertTrue(result)
        
    def test_read_command(self):
        """測試讀取指令"""
        response = self.plc.send_command("RD DM2990\r\n")
        self.assertEqual(response, "OK\r\n")
```

### 整合測試
```python
# ROS 2 服務整合測試
class TestPlcProxyService(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = PlcProxyNode()
        
    def test_plc_read_service(self):
        """測試 PLC 讀取服務"""
        request = PlcRead.Request()
        request.device_type = "DM"
        request.address = "2990"
        
        # 測試服務回調
        response = PlcRead.Response()
        self.node.plc_read_callback(request, response)
        
        self.assertIsNotNone(response.data)
```

### 壓力測試
```python
# 連接池壓力測試
async def stress_test_connection_pool():
    """測試連接池在高併發下的表現"""
    pool = KeyencePlcPool("192.168.2.101", 8501, max_connections=5)
    
    async def worker():
        plc = pool.get_connection()
        try:
            result = plc.send_command("RD DM2990\r\n")
            return result
        finally:
            pool.return_connection(plc)
    
    # 建立多個並行任務
    tasks = [worker() for _ in range(20)]
    results = await asyncio.gather(*tasks)
    
    print(f"成功完成 {len(results)} 次請求")
```

## 🚨 故障排除

### 診斷工具
docs-ai/operations/guides/system-diagnostics.md

```bash
# 使用統一診斷工具
r network-check           # 檢查網路連接
r quick-diag             # 快速系統診斷

# PLC 專用診斷
ping 192.168.2.101       # 測試網路連通性
telnet 192.168.2.101 8501  # 測試端口連通性
```

### 常見問題解決
```python
# 連接問題診斷
def diagnose_connection_issue(ip, port):
    """診斷連接問題"""
    try:
        # 測試網路連通性
        import subprocess
        result = subprocess.run(['ping', '-c', '1', ip], 
                              capture_output=True, text=True)
        if result.returncode != 0:
            return "網路不通"
            
        # 測試端口連通性
        import socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5)
        result = sock.connect_ex((ip, port))
        sock.close()
        
        if result != 0:
            return "端口無法連接"
            
        return "連接正常"
    except Exception as e:
        return f"診斷錯誤: {e}"
```

### 錯誤日誌分析
```python
# 結構化錯誤日誌
import logging

class PlcLogger:
    def __init__(self):
        self.logger = logging.getLogger('plc_communication')
        
    def log_connection_error(self, ip, port, error):
        """記錄連接錯誤"""
        self.logger.error(f"PLC連接失敗 - IP:{ip}, Port:{port}, Error:{error}")
        
    def log_protocol_error(self, command, response):
        """記錄協議錯誤"""
        self.logger.error(f"PLC協議錯誤 - Command:{command}, Response:{response}")
        
    def log_performance_warning(self, operation, duration):
        """記錄效能警告"""
        if duration > 1.0:  # 超過1秒
            self.logger.warning(f"PLC操作緩慢 - Operation:{operation}, Duration:{duration}s")
```

## 💡 部署注意事項

### 網路配置
```yaml
# Docker 網路配置建議
networks:
  plc_network:
    driver: bridge
    ipam:
      config:
        - subnet: 192.168.100.0/24
```

### 環境變數配置
```bash
# PLC 相關環境變數
export PLC_IP="192.168.2.101"
export PLC_PORT="8501"
export PLC_TIMEOUT="10"
export PLC_POOL_SIZE="5"
```

### 監控和告警
```python
# PLC 健康檢查
class PlcHealthChecker:
    def __init__(self, plc_pool):
        self.plc_pool = plc_pool
        
    def check_health(self):
        """檢查 PLC 健康狀態"""
        try:
            plc = self.plc_pool.get_connection()
            response = plc.send_command("?K\r\n")  # 查詢機型
            self.plc_pool.return_connection(plc)
            return "健康" if response else "異常"
        except Exception as e:
            return f"錯誤: {e}"
```

## 🔗 交叉引用
- Keyence 協議詳解: docs-ai/knowledge/protocols/keyence-plc-protocol.md
- ROS 2 開發指導: docs-ai/operations/development/ros2/ros2-development.md
- 容器開發環境: docs-ai/operations/development/docker-development.md
- 系統診斷工具: docs-ai/operations/guides/system-diagnostics.md
- 模組索引: docs-ai/context/structure/module-index.md