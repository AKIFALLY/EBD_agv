# kuka_fleet_ws CLAUDE.md

## 模組概述
KUKA Fleet整合系統，提供與KUKA Fleet Manager的通訊介面，實現AGV車隊與KUKA系統的無縫整合

## 專案結構
```
src/
└── kuka_fleet/         # KUKA Fleet整合核心
    ├── kuka_fleet/     # 主要整合邏輯
    ├── api_client/     # KUKA Fleet API客戶端
    ├── message_converter/# 訊息格式轉換
    └── sync_manager/   # 狀態同步管理
```

## 核心功能

### KUKA整合
- **API通訊**: 與KUKA Fleet Manager REST API通訊
- **任務同步**: WCS任務與KUKA訂單雙向同步
- **狀態同步**: AGV狀態與KUKA系統即時同步
- **錯誤處理**: KUKA系統錯誤自動處理與恢復

### 數據轉換
- **任務轉換**: WCS任務格式轉換為KUKA訂單格式
- **狀態映射**: AGV狀態映射到KUKA車輛狀態
- **座標轉換**: 不同座標系統間的轉換
- **時間同步**: 時間戳格式標準化

## 開發指令

### 環境設定 (AGVC容器內)
```bash
source /app/setup.bash
agvc_source  # 載入AGVC工作空間
cd /app/kuka_fleet_ws
```

### 服務啟動
```bash
# 啟動KUKA Fleet整合節點
ros2 run kuka_fleet kuka_fleet_node

# 測試KUKA連線
ros2 run kuka_fleet test_kuka_connection

# 同步狀態服務
ros2 run kuka_fleet sync_manager_node
```

### 構建與測試
```bash
build_ws kuka_fleet_ws
ros2 test kuka_fleet  # KUKA整合測試
```

## KUKA API整合

### API客戶端
```python
# api_client/kuka_api_client.py
class KukaAPIClient:
    def __init__(self, base_url: str, api_key: str):
        self.base_url = base_url
        self.api_key = api_key
        self.session = aiohttp.ClientSession()
        self.headers = {
            'Authorization': f'Bearer {api_key}',
            'Content-Type': 'application/json'
        }
        
    async def submit_order(self, order_data: dict) -> dict:
        """提交訂單到KUKA Fleet Manager"""
        try:
            url = f"{self.base_url}/api/v1/orders"
            async with self.session.post(url, json=order_data, headers=self.headers) as response:
                if response.status == 200:
                    result = await response.json()
                    return {
                        'success': True,
                        'order_id': result['orderId'],
                        'status': result['status']
                    }
                else:
                    error_text = await response.text()
                    return {
                        'success': False,
                        'error': f"HTTP {response.status}: {error_text}"
                    }
        except Exception as e:
            return {
                'success': False,
                'error': str(e)
            }
            
    async def get_vehicle_status(self, vehicle_id: str) -> dict:
        """獲取車輛狀態"""
        try:
            url = f"{self.base_url}/api/v1/vehicles/{vehicle_id}/status"
            async with self.session.get(url, headers=self.headers) as response:
                if response.status == 200:
                    return await response.json()
                else:
                    raise Exception(f"Failed to get vehicle status: {response.status}")
        except Exception as e:
            self.get_logger().error(f"獲取車輛狀態錯誤: {e}")
            return None
            
    async def cancel_order(self, order_id: str) -> bool:
        """取消訂單"""
        try:
            url = f"{self.base_url}/api/v1/orders/{order_id}/cancel"
            async with self.session.post(url, headers=self.headers) as response:
                return response.status == 200
        except Exception as e:
            self.get_logger().error(f"取消訂單錯誤: {e}")
            return False
```

### 訊息轉換器
```python
# message_converter/wcs_to_kuka_converter.py
class WCSToKukaConverter:
    def __init__(self):
        self.coordinate_transformer = CoordinateTransformer()
        
    def convert_transport_task(self, wcs_task: dict) -> dict:
        """將WCS運輸任務轉換為KUKA訂單"""
        kuka_order = {
            'orderId': self.generate_order_id(wcs_task['task_id']),
            'priority': self.map_priority(wcs_task['priority']),
            'vehicleId': wcs_task['assigned_agv_id'],
            'orderType': 'TRANSPORT',
            'created': self.format_timestamp(wcs_task['created_time']),
            'actions': []
        }
        
        # 轉換起始位置動作
        pickup_action = {
            'actionType': 'PICK',
            'locationId': self.convert_location_id(wcs_task['start_location']),
            'position': self.coordinate_transformer.wcs_to_kuka(
                wcs_task['start_position']
            ),
            'parameters': {
                'loadType': wcs_task.get('cargo_type', 'DEFAULT'),
                'loadWeight': wcs_task.get('cargo_weight', 0)
            }
        }
        kuka_order['actions'].append(pickup_action)
        
        # 轉換目標位置動作
        drop_action = {
            'actionType': 'DROP',
            'locationId': self.convert_location_id(wcs_task['end_location']),
            'position': self.coordinate_transformer.wcs_to_kuka(
                wcs_task['end_position']
            ),
            'parameters': {}
        }
        kuka_order['actions'].append(drop_action)
        
        return kuka_order
        
    def convert_status_update(self, agv_status: dict) -> dict:
        """將AGV狀態轉換為KUKA車輛狀態"""
        kuka_status = {
            'vehicleId': agv_status['agv_id'],
            'timestamp': self.format_timestamp(agv_status['timestamp']),
            'position': self.coordinate_transformer.wcs_to_kuka(
                agv_status['position']
            ),
            'batteryLevel': agv_status.get('battery_level', 0),
            'operationalState': self.map_operational_state(agv_status['state']),
            'currentOrder': agv_status.get('current_task_id'),
            'errors': self.convert_error_list(agv_status.get('errors', []))
        }
        
        return kuka_status
```

### 狀態同步管理器
```python
# sync_manager/status_sync_manager.py
class StatusSyncManager:
    def __init__(self, kuka_client: KukaAPIClient):
        self.kuka_client = kuka_client
        self.agv_status_cache = {}
        self.sync_interval = 5.0  # 5秒同步一次
        self.running = False
        
    async def start_sync_loop(self):
        """開始狀態同步循環"""
        self.running = True
        while self.running:
            try:
                await self.sync_agv_status()
                await asyncio.sleep(self.sync_interval)
            except Exception as e:
                self.get_logger().error(f"狀態同步錯誤: {e}")
                await asyncio.sleep(self.sync_interval)
                
    async def sync_agv_status(self):
        """同步所有AGV狀態到KUKA系統"""
        # 獲取當前所有AGV狀態
        current_agv_status = await self.get_all_agv_status()
        
        for agv_id, status in current_agv_status.items():
            # 檢查狀態是否有變化
            if self.has_status_changed(agv_id, status):
                # 轉換並發送到KUKA
                kuka_status = self.converter.convert_status_update(status)
                await self.kuka_client.update_vehicle_status(kuka_status)
                
                # 更新本地快取
                self.agv_status_cache[agv_id] = status.copy()
                
    def has_status_changed(self, agv_id: str, current_status: dict) -> bool:
        """檢查AGV狀態是否有變化"""
        if agv_id not in self.agv_status_cache:
            return True
            
        cached_status = self.agv_status_cache[agv_id]
        
        # 比較關鍵狀態欄位
        key_fields = ['state', 'position', 'battery_level', 'current_task_id']
        for field in key_fields:
            if current_status.get(field) != cached_status.get(field):
                return True
                
        return False
```

## 任務生命週期管理

### 訂單管理
```python
# kuka_fleet/order_manager.py
class OrderManager:
    def __init__(self, kuka_client: KukaAPIClient):
        self.kuka_client = kuka_client
        self.active_orders = {}
        self.order_history = []
        
    async def create_order_from_task(self, wcs_task: dict) -> dict:
        """從WCS任務創建KUKA訂單"""
        try:
            # 轉換任務格式
            kuka_order = self.converter.convert_transport_task(wcs_task)
            
            # 提交到KUKA系統
            result = await self.kuka_client.submit_order(kuka_order)
            
            if result['success']:
                # 記錄活躍訂單
                self.active_orders[result['order_id']] = {
                    'wcs_task_id': wcs_task['task_id'],
                    'kuka_order_id': result['order_id'],
                    'status': result['status'],
                    'created_time': time.time(),
                    'agv_id': wcs_task['assigned_agv_id']
                }
                
                return {
                    'success': True,
                    'order_id': result['order_id']
                }
            else:
                return {
                    'success': False,
                    'error': result['error']
                }
                
        except Exception as e:
            return {
                'success': False,
                'error': str(e)
            }
            
    async def monitor_order_status(self):
        """監控訂單狀態"""
        for order_id, order_info in list(self.active_orders.items()):
            try:
                status = await self.kuka_client.get_order_status(order_id)
                
                if status and status['status'] != order_info['status']:
                    # 狀態已變更
                    order_info['status'] = status['status']
                    
                    # 通知WCS狀態變更
                    await self.notify_wcs_status_change(order_info, status)
                    
                    # 如果訂單完成，移至歷史記錄
                    if status['status'] in ['COMPLETED', 'CANCELLED', 'FAILED']:
                        self.order_history.append(order_info)
                        del self.active_orders[order_id]
                        
            except Exception as e:
                self.get_logger().error(f"監控訂單 {order_id} 狀態錯誤: {e}")
```

## 配置管理

### KUKA系統配置
```yaml
# /app/config/agvc/kuka_fleet_config.yaml
kuka_fleet:
  # API連線設定
  api:
    base_url: "https://kuka-fleet-manager.example.com"
    api_key: "${KUKA_API_KEY}"  # 環境變數
    timeout: 30.0               # 連線超時(秒)
    retry_attempts: 3           # 重試次數
    
  # 同步設定
  sync:
    status_sync_interval: 5.0   # 狀態同步間隔(秒)
    order_monitor_interval: 10.0# 訂單監控間隔(秒)
    batch_size: 10             # 批次處理大小
    
  # 座標系統轉換
  coordinate_transform:
    # WCS座標系到KUKA座標系的轉換參數
    translation: [0.0, 0.0, 0.0]  # x, y, z偏移(m)
    rotation: [0.0, 0.0, 0.0]     # roll, pitch, yaw(rad)
    scale: [1.0, 1.0, 1.0]        # x, y, z縮放
    
  # 狀態映射
  status_mapping:
    # WCS狀態到KUKA狀態的映射
    IDLE: "AVAILABLE" 
    MOVING: "DRIVING"
    LOADING: "LOADING"
    UNLOADING: "UNLOADING"
    CHARGING: "CHARGING"
    ERROR: "ERROR"
    
  # 支援的任務類型
  supported_order_types:
    - "TRANSPORT"
    - "PICK"
    - "DROP"
    - "CHARGE"
```

### 網路設定
```yaml
# 網路連線配置
network:
  connection_pool:
    max_connections: 20
    max_idle_time: 300      # 5分鐘
    
  ssl:
    verify_certificates: true
    ca_bundle_path: "/app/config/certs/ca-bundle.pem"
    
  proxy:
    enabled: false
    http_proxy: ""
    https_proxy: ""
```

## 測試與調試

### API測試
```bash
# 測試KUKA API連線
ros2 service call /kuka_fleet/test_connection kuka_fleet_msgs/srv/TestConnection

# 提交測試訂單
ros2 service call /kuka_fleet/submit_order kuka_fleet_msgs/srv/SubmitOrder "{
  order_data: '{\"orderId\": \"test_001\", \"vehicleId\": \"AGV001\", \"orderType\": \"TRANSPORT\"}'
}"

# 查看活躍訂單
ros2 topic echo /kuka_fleet/active_orders

# 檢查同步狀態
ros2 service call /kuka_fleet/get_sync_status
```

### 整合測試
```bash
# 端到端任務測試
ros2 run kuka_fleet integration_test --task-type transport

# 狀態同步測試
ros2 run kuka_fleet sync_test --duration 60

# 錯誤恢復測試
ros2 run kuka_fleet error_recovery_test
```

## 錯誤處理與恢復

### 連線錯誤處理
```python
# error_handling/connection_handler.py
class ConnectionErrorHandler:
    def __init__(self, max_retries=3, backoff_factor=2.0):
        self.max_retries = max_retries
        self.backoff_factor = backoff_factor
        
    async def handle_connection_error(self, error, operation):
        """處理連線錯誤並重試"""
        for attempt in range(self.max_retries):
            try:
                delay = self.backoff_factor ** attempt
                await asyncio.sleep(delay)
                
                # 重試操作
                result = await operation()
                return result
                
            except Exception as e:
                if attempt == self.max_retries - 1:
                    # 最後一次重試失敗
                    await self.escalate_error(error, operation)
                    raise e
                    
    async def escalate_error(self, error, operation):
        """錯誤升級處理"""
        # 記錄錯誤
        self.get_logger().error(f"KUKA連線錯誤，已達最大重試次數: {error}")
        
        # 通知WCS系統KUKA不可用
        await self.notify_wcs_kuka_unavailable()
        
        # 啟動降級模式
        await self.activate_degraded_mode()
```

### 數據一致性保證
```python
# 確保數據一致性的機制
class DataConsistencyManager:
    def __init__(self):
        self.pending_updates = {}
        self.consistency_check_interval = 30.0
        
    async def ensure_consistency(self):
        """定期檢查數據一致性"""
        while True:
            try:
                await self.check_order_consistency()
                await self.check_status_consistency()
                await asyncio.sleep(self.consistency_check_interval)
            except Exception as e:
                self.get_logger().error(f"一致性檢查錯誤: {e}")
```

## 監控與統計

### 性能監控
```python
# monitoring/performance_monitor.py
class PerformanceMonitor:
    def __init__(self):
        self.metrics = {
            'api_calls_total': 0,
            'api_calls_success': 0,
            'api_calls_failed': 0,
            'average_response_time': 0.0,
            'orders_submitted': 0,
            'orders_completed': 0
        }
        
    def record_api_call(self, success: bool, response_time: float):
        """記錄API調用統計"""
        self.metrics['api_calls_total'] += 1
        if success:
            self.metrics['api_calls_success'] += 1
        else:
            self.metrics['api_calls_failed'] += 1
            
        # 更新平均響應時間
        total_calls = self.metrics['api_calls_total']
        current_avg = self.metrics['average_response_time']
        self.metrics['average_response_time'] = (
            (current_avg * (total_calls - 1) + response_time) / total_calls
        )
```

## 故障排除

### 常見問題
1. **API連線失敗**: 檢查網路連線與API憑證
2. **訂單提交失敗**: 驗證訂單格式與KUKA系統狀態
3. **狀態同步延遲**: 檢查同步間隔設定與系統負載
4. **座標轉換錯誤**: 驗證座標轉換參數配置

### 診斷工具
```bash
# KUKA系統連線診斷
ros2 run kuka_fleet connection_diagnostics

# API呼叫統計
ros2 topic echo /kuka_fleet/api_statistics

# 訂單狀態追蹤
ros2 service call /kuka_fleet/trace_order "{order_id: 'ORDER_123'}"
```

## 安全與權限

### API安全
- **API金鑰管理**: 安全存儲與輪換API憑證
- **HTTPS通訊**: 強制使用加密通訊
- **憑證驗證**: 驗證KUKA伺服器憑證
- **訪問控制**: 限制API訪問權限範圍

### 數據保護
- **敏感資料加密**: 加密存儲敏感配置資料
- **通訊加密**: 端到端通訊加密
- **審計日誌**: 詳細的API調用審計記錄

## 重要提醒
- KUKA整合影響整個車隊操作，變更需謹慎測試
- API憑證需定期更新，避免過期導致服務中斷
- 座標系統轉換參數需精確校準
- 必須在AGVC容器內運行，確保網路連線可達KUKA系統