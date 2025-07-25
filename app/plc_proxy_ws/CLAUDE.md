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
**ROS 2服務提供者**:
```python
class PlcService(Node):
    def __init__(self)
    # 提供8個ROS 2服務
    self.srv_force_on = self.create_service(ForceOn, 'force_on', self.force_on_callback)
    self.srv_force_off = self.create_service(ForceOff, 'force_off', self.force_off_callback)
    self.srv_read_data = self.create_service(ReadData, 'read_data', self.read_data_callback)
    # ... 其他服務
```

**核心參數**:
```python
self.declare_parameter("plc_ip", "192.168.12.224")      # PLC IP地址
self.declare_parameter("read_ranges", ["DM,7600,200"])  # 自動讀取範圍
self.port = 8501                                        # PLC端口
```

**記憶體管理**:
```python
self.memory = PlcMemory(65536 * 2)  # PLC記憶體對應(64KB)
self.pool = KeyencePlcPool(ip, port) # 連線池管理
```

### 2. PlcClient類別 (plc_client.py) 
**客戶端封裝**，提供簡化的調用接口:
```python
class PlcClient:
    def force_on(self, device_type, address)          # MR強制ON
    def force_off(self, device_type, address)         # MR強制OFF  
    def read_data(self, device_type, address)         # 讀取資料
    def write_data(self, device_type, address, data)  # 寫入資料
    def read_continuous_data(...)                     # 連續讀取
    def write_continuous_data(...)                    # 連續寫入
```

**使用範例**:
```python
plc_client = PlcClient(node, namespace)
response = plc_client.force_on("MR", "3708")  # 強制MR3708 ON
```

## 開發指令

### 環境設定
```bash
# AGV容器內
source /app/setup.bash && all_source
cd /app/plc_proxy_ws

# AGVC容器內
source /app/setup.bash && agvc_source  
cd /app/plc_proxy_ws
```

### 服務啟動 (容器內執行)
```bash
# 必須先進入容器並載入環境
docker compose -f <compose-file> exec <container> bash
source /app/setup.bash && all_source

# 啟動PLC代理服務 (實際檔案: plc_service.py)
python3 src/plc_proxy/plc_proxy/plc_service.py

# 或使用ROS 2啟動 (如果已建置)
ros2 run plc_proxy plc_service

# PLC客戶端節點
ros2 run plc_proxy plc_client_node
```

### 構建與測試
```bash
build_ws plc_proxy_ws
ros2 test plc_proxy  # 代理服務測試
```

## 代理服務開發

### PLC代理客戶端
```python
# clients/plc_client.py
class PLCProxyClient:
    def __init__(self):
        self.keyence_client = self.create_keyence_client()
        self.service_registry = {}
        
    async def read_plc_register(self, address: str):
        """讀取PLC暫存器數據"""
        request = PLCReadRequest(address=address)
        response = await self.keyence_client.read_register(request)
        return self.convert_plc_response(response)
        
    async def write_plc_register(self, address: str, value, data_type: str):
        """寫入PLC暫存器數據"""
        converted_value = self.convert_to_plc_format(value, data_type)
        request = PLCWriteRequest(address=address, value=converted_value)
        response = await self.keyence_client.write_register(request)
        return response.success
```

### 服務介面實現
```python
# services/plc_service_interface.py
class PLCServiceInterface:
    def __init__(self, plc_client: PLCProxyClient):
        self.plc_client = plc_client
        self.create_ros_services()
        
    def create_ros_services(self):
        """創建ROS 2服務端點"""
        self.read_service = self.create_service(
            PLCRead, '/plc_proxy/read', self.handle_read_request
        )
        self.write_service = self.create_service(
            PLCWrite, '/plc_proxy/write', self.handle_write_request
        )
        
    async def handle_read_request(self, request, response):
        """處理PLC讀取請求"""
        try:
            result = await self.plc_client.read_plc_register(request.address)
            response.success = True
            response.value = result.value
            response.data_type = result.data_type
        except Exception as e:
            response.success = False
            response.error_message = str(e)
        return response
```

### 數據轉換層
```python
# utils/data_converter.py
class DataConverter:
    @staticmethod
    def ros_to_plc_data(ros_value, target_type: str):
        """ROS 2數據轉換為PLC格式"""
        converters = {
            'int16': lambda x: struct.pack('>h', int(x)),
            'uint16': lambda x: struct.pack('>H', int(x)),
            'float32': lambda x: struct.pack('>f', float(x)),
            'bool': lambda x: struct.pack('?', bool(x))
        }
        return converters.get(target_type, lambda x: x)(ros_value)
        
    @staticmethod
    def plc_to_ros_data(plc_bytes: bytes, source_type: str):
        """PLC數據轉換為ROS 2格式"""
        parsers = {
            'int16': lambda x: struct.unpack('>h', x)[0],
            'uint16': lambda x: struct.unpack('>H', x)[0], 
            'float32': lambda x: struct.unpack('>f', x)[0],
            'bool': lambda x: struct.unpack('?', x)[0]
        }
        return parsers.get(source_type, lambda x: x)(plc_bytes)
```

## 服務註冊與發現

### 服務註冊
```python
# services/service_registry.py
class ServiceRegistry:
    def __init__(self):
        self.registered_services = {}
        self.service_health = {}
        
    def register_plc_service(self, service_name: str, config: dict):
        """註冊PLC服務"""
        service_config = PLCServiceConfig(
            name=service_name,
            plc_address=config['plc_address'],
            register_map=config['register_map'],
            update_rate=config.get('update_rate', 10.0)
        )
        self.registered_services[service_name] = service_config
        
    async def health_check_services(self):
        """服務健康檢查"""
        for service_name, config in self.registered_services.items():
            try:
                await self.test_service_connectivity(config)
                self.service_health[service_name] = 'healthy'
            except Exception as e:
                self.service_health[service_name] = f'unhealthy: {e}'
```

### 動態服務創建
```python
# 根據配置動態創建PLC服務
class DynamicServiceCreator:
    def create_services_from_config(self, config_file: str):
        """從配置文件創建服務"""
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
            
        for service_def in config['plc_services']:
            service = self.create_plc_service(service_def)
            self.register_service(service)
```

## 配置管理

### 代理配置
```yaml
# /app/config/agv/plc_proxy.yaml (AGV環境)
plc_proxy:
  keyence_plc_node: "keyence_plc_node"
  timeout: 5.0
  retry_attempts: 3
  
  services:
    - name: "agv_control"
      description: "AGV運動控制"
      registers:
        read:
          - {address: "D0", name: "current_x", type: "float32"}
          - {address: "D2", name: "current_y", type: "float32"}
        write:
          - {address: "D100", name: "target_x", type: "float32"}
          - {address: "D102", name: "target_y", type: "float32"}
          
    - name: "robot_arm"
      description: "機械臂控制"
      registers:
        read:
          - {address: "D10", name: "joint_angles", type: "float32", count: 6}
        write:
          - {address: "D110", name: "target_joints", type: "float32", count: 6}
```

```yaml
# /app/config/agvc/plc_proxy.yaml (AGVC環境)
plc_proxy:
  keyence_plc_node: "keyence_plc_node"
  
  services:
    - name: "charging_station"
      description: "充電站控制"
      registers:
        read:
          - {address: "D200", name: "charge_status", type: "int16"}
          - {address: "D201", name: "voltage", type: "float32"}
        write:
          - {address: "D300", name: "charge_enable", type: "bool"}
```

## 錯誤處理與重試

### 重試機制
```python
# utils/retry_handler.py
class RetryHandler:
    def __init__(self, max_retries=3, backoff_factor=1.5):
        self.max_retries = max_retries
        self.backoff_factor = backoff_factor
        
    async def execute_with_retry(self, operation, *args, **kwargs):
        """帶重試的操作執行"""
        last_exception = None
        
        for attempt in range(self.max_retries + 1):
            try:
                return await operation(*args, **kwargs)
            except Exception as e:
                last_exception = e
                if attempt < self.max_retries:
                    delay = self.backoff_factor ** attempt
                    await asyncio.sleep(delay)
                    
        raise last_exception
```

### 錯誤分類處理
```python
# 根據錯誤類型採取不同處理策略
class ErrorHandler:
    def handle_plc_error(self, error_type: str, error_data):
        error_handlers = {
            'CONNECTION_TIMEOUT': self.handle_timeout_error,
            'INVALID_ADDRESS': self.handle_address_error,
            'DATA_FORMAT_ERROR': self.handle_format_error,
            'PLC_OFFLINE': self.handle_offline_error
        }
        
        handler = error_handlers.get(error_type, self.handle_generic_error)
        return handler(error_data)
```

## 測試與調試

### 代理服務測試
```bash
# 測試PLC讀取服務
ros2 service call /plc_proxy/read plc_proxy_msgs/srv/PLCRead "{address: 'D0', data_type: 'float32'}"

# 測試PLC寫入服務
ros2 service call /plc_proxy/write plc_proxy_msgs/srv/PLCWrite "{address: 'D100', value: '123.45', data_type: 'float32'}"

# 檢查服務健康狀態
ros2 service call /plc_proxy/health_check
```

### 整合測試
```python
# test/test_integration.py
@pytest.mark.asyncio
async def test_plc_proxy_integration():
    """測試代理服務與keyence_plc整合"""
    proxy_client = PLCProxyClient()
    
    # 測試寫入後讀取
    write_result = await proxy_client.write_plc_register("D100", 42.0, "float32")
    assert write_result == True
    
    read_result = await proxy_client.read_plc_register("D100")
    assert abs(read_result.value - 42.0) < 0.001
```

## 監控與診斷

### 服務監控
```python
# utils/service_monitor.py
class ServiceMonitor:
    def __init__(self):
        self.metrics = {
            'request_count': 0,
            'success_count': 0,
            'error_count': 0,
            'average_response_time': 0.0
        }
        
    def log_request(self, service_name: str, response_time: float, success: bool):
        """記錄服務請求統計"""
        self.metrics['request_count'] += 1
        if success:
            self.metrics['success_count'] += 1
        else:
            self.metrics['error_count'] += 1
            
        # 更新平均響應時間
        self.update_average_response_time(response_time)
```

### 診斷工具
```bash
# 代理服務診斷
ros2 run plc_proxy service_diagnostics

# 檢查代理統計
ros2 topic echo /plc_proxy/statistics

# 服務連線狀態
ros2 service call /plc_proxy/get_connection_status
```

## 故障排除

### 常見問題
1. **keyence_plc連線失敗**: 確認keyence_plc_ws服務運行狀態
2. **數據轉換錯誤**: 檢查數據類型映射配置
3. **服務響應超時**: 調整timeout參數或檢查PLC響應
4. **服務註冊失敗**: 驗證服務配置文件格式

### 診斷步驟
```bash
# 1. 檢查keyence_plc服務狀態
ros2 service list | grep keyence

# 2. 測試底層PLC通訊
ros2 service call /keyence_plc/test_connection

# 3. 檢查代理服務註冊
ros2 service call /plc_proxy/list_services

# 4. 查看錯誤日誌
ros2 topic echo /plc_proxy/error_log
```

## 性能最佳化

### 連線池管理
- 維持與keyence_plc的持久連線
- 連線復用減少建立連線開銷
- 連線健康檢查與自動恢復

### 快取機制
- 頻繁讀取數據的本地快取
- 快取失效策略與數據一致性
- 減少不必要的PLC通訊

## 重要提醒
- 作為關鍵通訊代理，穩定性至關重要
- 數據轉換必須保證精確性
- 支援AGV與AGVC雙環境部署
- 與keyence_plc_ws版本相容性需維護