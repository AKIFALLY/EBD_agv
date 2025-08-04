# PLC 整合方案

## 🎯 PLC 系統整合架構

本文檔詳細說明 RosAGV 與 Keyence PLC 系統的整合方案，包括通訊協定、硬體介面、軟體架構和實作細節。

## 📋 PLC 整合概覽

### PLC 在 RosAGV 中的角色
```
PLC 整合架構
├── 🏭 工業控制層
│   ├── Keyence PLC 控制器
│   ├── 機械臂控制
│   ├── 感測器整合
│   └── 執行器控制
├── 🔗 通訊介面層
│   ├── keyence_plc_ws (純通訊庫)
│   ├── plc_proxy_ws (ROS 2 封裝)
│   └── TCP/IP 通訊協定
└── 🤖 應用整合層
    ├── AGV 狀態機整合
    ├── 機械臂操作整合
    └── 安全系統整合
```

### 雙工作空間設計
```
PLC 通訊雙工作空間架構
keyence_plc_ws (通訊層)
├── 純 Python 通訊庫
├── Keyence 協定實作
├── 連接池管理
└── 錯誤處理機制

plc_proxy_ws (服務層)
├── ROS 2 服務封裝
├── 統一 API 介面
├── 狀態監控
└── 日誌和診斷
```

## 🔧 硬體整合架構

### PLC 硬體配置
```
Keyence PLC 系統配置
├── 主控制器
│   ├── 型號: Keyence KV-8000 系列
│   ├── IP 地址: 192.168.2.101
│   ├── 通訊端口: 8501
│   └── 協定: Keyence 專有協定
├── I/O 模組
│   ├── 數位輸入: 32 點
│   ├── 數位輸出: 32 點
│   ├── 類比輸入: 8 點
│   └── 類比輸出: 4 點
└── 擴展模組
    ├── 高速計數器
    ├── 脈衝輸出
    └── 通訊模組
```

### 機械臂整合
```python
# 機械臂 PLC 整合架構
class RobotPLCIntegration:
    """機械臂與 PLC 的整合控制"""
    
    def __init__(self):
        self.plc_client = KeyencePlcCom("192.168.2.101", 8501)
        self.robot_parameters = RobotParameter()
        
    # PGNO 指令系統 (Program Number)
    PGNO_COMMANDS = {
        "CHG_PARA": "40000",      # 參數變更
        "PHOTO_RACK_UP": "40001", # 架台上層拍照
        "PHOTO_RACK_DOWN": "40002", # 架台下層拍照
        "PHOTO_BOX_IN": "40003",  # 入口箱拍照
        "PHOTO_BOX_OUT": "40004", # 出口箱拍照
        "IDLE": "50000"           # 機械臂閒置
    }
    
    def execute_robot_command(self, command, parameters=None):
        """執行機械臂指令"""
        if parameters:
            self.update_parameters(parameters)
        
        pgno = self.PGNO_COMMANDS.get(command)
        if pgno:
            return self.plc_client.write_device("DM", "100", pgno)
```

## 🌐 通訊協定實作

### Keyence 協定規範
```python
# Keyence PLC 通訊協定
class KeyencePlcProtocol:
    """Keyence PLC 通訊協定實作"""
    
    # 指令格式
    COMMAND_FORMAT = {
        'read_single': "RD {device}{address}\r\n",
        'write_single': "WR {device}{address} {value}\r\n",
        'read_continuous': "RDS {device}{address} {length}\r\n",
        'write_continuous': "WRS {device}{address} {length} {values}\r\n",
        'force_on': "ST {device}{address}\r\n",
        'force_off': "RS {device}{address}\r\n"
    }
    
    # 設備類型
    DEVICE_TYPES = {
        'DM': 'Data Memory',      # 資料記憶體
        'MR': 'Memory Relay',     # 內部繼電器
        'R': 'Input Relay',       # 輸入繼電器
        'Y': 'Output Relay',      # 輸出繼電器
        'B': 'Link Relay',        # 連結繼電器
        'SM': 'Special Memory'    # 特殊記憶體
    }
    
    # 錯誤碼
    ERROR_CODES = {
        "E0": "元件編號異常",
        "E1": "指令異常", 
        "E4": "禁止寫入"
    }
```

### 連接池管理
```python
# PLC 連接池管理
class PLCConnectionPool:
    """PLC 連接池，提高併發性能和穩定性"""
    
    def __init__(self, ip, port, max_connections=5):
        self.ip = ip
        self.port = port
        self.max_connections = max_connections
        self.connections = []
        self.lost_connections = []
        self.semaphore = threading.Semaphore(max_connections)
        self.lock = threading.Lock()
    
    def get_connection(self):
        """獲取可用連接"""
        self.semaphore.acquire()
        
        with self.lock:
            if self.connections:
                return self.connections.pop()
            else:
                # 創建新連接
                conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                conn.connect((self.ip, self.port))
                return conn
    
    def return_connection(self, connection):
        """歸還連接到池中"""
        with self.lock:
            if len(self.connections) < self.max_connections:
                self.connections.append(connection)
            else:
                connection.close()
        
        self.semaphore.release()
    
    def check_connection_health(self):
        """檢查連接健康狀態"""
        healthy_connections = []
        
        with self.lock:
            for conn in self.connections:
                try:
                    # 發送心跳檢查
                    conn.send(b"?K\r\n")
                    response = conn.recv(1024)
                    if response:
                        healthy_connections.append(conn)
                    else:
                        conn.close()
                except:
                    conn.close()
            
            self.connections = healthy_connections
```

## 🔄 ROS 2 服務整合

### PLC Proxy 服務
```python
# PLC Proxy ROS 2 節點
import rclpy
from rclpy.node import Node
from plc_interfaces.srv import PLCRead, PLCWrite
from plc_interfaces.msg import PLCStatus

class PLCProxyNode(Node):
    """PLC 代理服務節點"""
    
    def __init__(self):
        super().__init__('plc_proxy_node')
        
        # 初始化 PLC 連接
        self.plc_pool = PLCConnectionPool("192.168.2.101", 8501)
        
        # 創建服務
        self.read_service = self.create_service(
            PLCRead, 'plc_read', self.plc_read_callback)
        
        self.write_service = self.create_service(
            PLCWrite, 'plc_write', self.plc_write_callback)
        
        # 狀態發布者
        self.status_publisher = self.create_publisher(
            PLCStatus, 'plc_status', 10)
        
        # 狀態監控定時器
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('PLC Proxy Node 已啟動')
    
    def plc_read_callback(self, request, response):
        """PLC 讀取服務回調"""
        try:
            conn = self.plc_pool.get_connection()
            
            # 構造讀取指令
            command = f"RD {request.device}{request.address}\r\n"
            conn.send(command.encode())
            
            # 接收回應
            data = conn.recv(1024).decode().strip()
            
            self.plc_pool.return_connection(conn)
            
            # 檢查錯誤
            if data.startswith('E'):
                response.success = False
                response.error_message = self.get_error_message(data)
            else:
                response.success = True
                response.data = data
                
        except Exception as e:
            response.success = False
            response.error_message = str(e)
            self.get_logger().error(f'PLC 讀取失敗: {e}')
        
        return response
    
    def plc_write_callback(self, request, response):
        """PLC 寫入服務回調"""
        try:
            conn = self.plc_pool.get_connection()
            
            # 構造寫入指令
            command = f"WR {request.device}{request.address} {request.value}\r\n"
            conn.send(command.encode())
            
            # 接收回應
            data = conn.recv(1024).decode().strip()
            
            self.plc_pool.return_connection(conn)
            
            # 檢查回應
            if data == "OK":
                response.success = True
            else:
                response.success = False
                response.error_message = self.get_error_message(data)
                
        except Exception as e:
            response.success = False
            response.error_message = str(e)
            self.get_logger().error(f'PLC 寫入失敗: {e}')
        
        return response
```

### 服務介面定義
```python
# plc_interfaces/srv/PLCRead.srv
string device      # 設備類型 (DM, MR, R, Y, B, SM)
string address     # 設備地址
---
bool success       # 操作是否成功
string data        # 讀取的資料
string error_message  # 錯誤訊息

# plc_interfaces/srv/PLCWrite.srv  
string device      # 設備類型
string address     # 設備地址
string value       # 寫入的值
---
bool success       # 操作是否成功
string error_message  # 錯誤訊息

# plc_interfaces/msg/PLCStatus.msg
string plc_ip          # PLC IP 地址
bool connected         # 連接狀態
int32 active_connections  # 活躍連接數
float64 response_time  # 平均響應時間
string last_error      # 最後錯誤訊息
```

## 🏭 應用場景整合

### AGV 狀態機整合
```python
# AGV 狀態機中的 PLC 整合
class AGVStateMachine:
    def __init__(self):
        self.plc_client = self.create_client(PLCRead, 'plc_read')
        self.plc_write_client = self.create_client(PLCWrite, 'plc_write')
        
    async def check_safety_status(self):
        """檢查安全狀態"""
        request = PLCRead.Request()
        request.device = "R"
        request.address = "100"  # 安全輸入點
        
        future = self.plc_client.call_async(request)
        response = await future
        
        if response.success:
            return response.data == "1"  # 1 表示安全
        else:
            self.get_logger().error(f'安全狀態檢查失敗: {response.error_message}')
            return False
    
    async def control_robot_arm(self, command, parameters=None):
        """控制機械臂"""
        if parameters:
            # 先更新參數
            await self.update_robot_parameters(parameters)
        
        # 發送 PGNO 指令
        request = PLCWrite.Request()
        request.device = "DM"
        request.address = "100"
        request.value = self.PGNO_COMMANDS.get(command, "50000")
        
        future = self.plc_write_client.call_async(request)
        response = await future
        
        return response.success
```

### 機械臂參數管理
```python
# 機械臂參數管理系統
class RobotParameterManager:
    """機械臂參數管理和同步"""
    
    def __init__(self, plc_client):
        self.plc_client = plc_client
        self.parameter_map = {
            'rack_port': 'DM200',      # 架台端口
            'boxin_port': 'DM201',     # 入口箱端口  
            'boxout_port': 'DM202',    # 出口箱端口
            'speed': 'DM210',          # 運動速度
            'acceleration': 'DM211',   # 加速度
            'force_limit': 'DM212'     # 力限制
        }
    
    async def update_parameters(self, parameters):
        """批量更新參數到 PLC"""
        update_tasks = []
        
        for param_name, value in parameters.items():
            if param_name in self.parameter_map:
                plc_address = self.parameter_map[param_name]
                device, address = plc_address[:2], plc_address[2:]
                
                request = PLCWrite.Request()
                request.device = device
                request.address = address
                request.value = str(value)
                
                task = self.plc_client.call_async(request)
                update_tasks.append(task)
        
        # 並行執行所有更新
        results = await asyncio.gather(*update_tasks)
        
        # 檢查更新結果
        success_count = sum(1 for result in results if result.success)
        total_count = len(results)
        
        self.get_logger().info(f'參數更新完成: {success_count}/{total_count}')
        
        return success_count == total_count
```

## 📊 監控和診斷

### PLC 連接監控
```python
# PLC 連接監控和診斷
class PLCMonitor:
    def __init__(self, plc_pool):
        self.plc_pool = plc_pool
        self.metrics = {
            'total_requests': 0,
            'successful_requests': 0,
            'failed_requests': 0,
            'average_response_time': 0.0,
            'connection_errors': 0
        }
    
    def record_request(self, success, response_time):
        """記錄請求統計"""
        self.metrics['total_requests'] += 1
        
        if success:
            self.metrics['successful_requests'] += 1
        else:
            self.metrics['failed_requests'] += 1
        
        # 更新平均響應時間
        current_avg = self.metrics['average_response_time']
        total_requests = self.metrics['total_requests']
        
        self.metrics['average_response_time'] = (
            (current_avg * (total_requests - 1) + response_time) / total_requests
        )
    
    def get_health_status(self):
        """獲取 PLC 健康狀態"""
        total = self.metrics['total_requests']
        if total == 0:
            return {'status': 'unknown', 'reason': 'no requests'}
        
        success_rate = self.metrics['successful_requests'] / total
        avg_response = self.metrics['average_response_time']
        
        if success_rate >= 0.95 and avg_response < 100:  # 100ms
            return {'status': 'healthy'}
        elif success_rate >= 0.8:
            return {'status': 'degraded', 'reason': 'high error rate'}
        else:
            return {'status': 'unhealthy', 'reason': 'system failure'}
```

### 診斷工具
```bash
# PLC 診斷命令工具
#!/bin/bash
# plc-diagnostic.sh

PLC_IP="192.168.2.101"
PLC_PORT="8501"

echo "=== PLC 連接診斷 ==="

# 1. 網路連通性測試
echo "1. 測試網路連通性..."
if ping -c 3 $PLC_IP > /dev/null 2>&1; then
    echo "✅ 網路連通正常"
else
    echo "❌ 網路連通失敗"
    exit 1
fi

# 2. 端口連接測試
echo "2. 測試端口連接..."
if timeout 5 bash -c "echo > /dev/tcp/$PLC_IP/$PLC_PORT" 2>/dev/null; then
    echo "✅ 端口連接正常"
else
    echo "❌ 端口連接失敗"
    exit 1
fi

# 3. PLC 通訊測試
echo "3. 測試 PLC 通訊..."
response=$(echo -e "?K\r\n" | nc -w 5 $PLC_IP $PLC_PORT)
if [ -n "$response" ]; then
    echo "✅ PLC 通訊正常: $response"
else
    echo "❌ PLC 通訊失敗"
    exit 1
fi

# 4. ROS 2 服務測試
echo "4. 測試 ROS 2 PLC 服務..."
if ros2 service call /plc_read plc_interfaces/PLCRead "device: 'DM' address: '100'" > /dev/null 2>&1; then
    echo "✅ ROS 2 PLC 服務正常"
else
    echo "❌ ROS 2 PLC 服務異常"
fi

echo "=== 診斷完成 ==="
```

## 🔧 故障排除

### 常見問題和解決方案

#### 1. PLC 連接失敗
```bash
# 診斷步驟
ping 192.168.2.101
telnet 192.168.2.101 8501

# 可能原因和解決方法
# - 網路問題：檢查網路配置和路由
# - PLC 設定問題：檢查 PLC IP 和端口設定
# - 防火牆問題：開放必要端口
```

#### 2. 指令執行失敗
```python
# 錯誤處理範例
def safe_plc_command(self, command):
    max_retries = 3
    retry_delay = 1.0
    
    for attempt in range(max_retries):
        try:
            result = self.execute_command(command)
            if result.success:
                return result
            else:
                self.get_logger().warning(f'PLC 指令失敗 (嘗試 {attempt + 1}): {result.error_message}')
        except Exception as e:
            self.get_logger().error(f'PLC 通訊異常 (嘗試 {attempt + 1}): {e}')
        
        if attempt < max_retries - 1:
            time.sleep(retry_delay)
            retry_delay *= 2  # 指數退避
    
    return None  # 所有嘗試都失敗
```

#### 3. 效能問題
```python
# 效能最佳化建議
class OptimizedPLCClient:
    def __init__(self):
        # 使用連接池
        self.connection_pool = PLCConnectionPool()
        
        # 批量操作
        self.batch_queue = []
        self.batch_timer = None
    
    def batch_write(self, operations):
        """批量寫入操作"""
        # 將多個寫入操作合併為一個 WRS 指令
        if len(operations) > 1:
            return self.write_continuous(operations)
        else:
            return self.write_single(operations[0])
```

## 📚 開發指導

### 最佳實踐
1. **連接管理**: 使用連接池管理 PLC 連接
2. **錯誤處理**: 實作完整的錯誤處理和重試機制
3. **批量操作**: 使用批量指令提高效率
4. **狀態監控**: 實作連接和操作狀態監控
5. **日誌記錄**: 詳細記錄所有 PLC 操作用於除錯

### 安全考量
1. **權限控制**: 限制 PLC 寫入操作的權限
2. **輸入驗證**: 驗證所有 PLC 指令參數
3. **異常處理**: 防止異常導致系統崩潰
4. **監控告警**: 實作 PLC 通訊狀態監控和告警

---

**相關文檔：**
- [Keyence 協定詳解](../knowledge/protocols/keyence-plc-protocol.md) - 協定規範
- [故障排除](../operations/troubleshooting.md) - PLC 問題診斷
- [開發環境](../operations/development.md) - PLC 開發指導
- [AGV 車型](../agv-vehicles/vehicle-types.md) - 機械臂整合應用