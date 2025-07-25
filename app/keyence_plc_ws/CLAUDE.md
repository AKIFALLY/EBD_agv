# CLAUDE.md

## 系統概述
Keyence PLC通訊庫，提供低層TCP Socket連線與Keyence專用協議實現，為AGV/AGVC系統提供PLC控制基礎服務。

**🔗 重要**: 這是純Python庫(非節點)，被plc_proxy_ws封裝為ROS 2服務使用。

## 核心架構
```
keyence_plc_ws/
└── src/keyence_plc/
    ├── keyence_plc_com.py      # TCP通訊核心類別
    ├── keyence_plc_pool.py     # 連線池管理
    ├── keyence_plc_command.py  # Keyence協議指令
    ├── keyence_plc_memory.py   # 記憶體操作工具
    ├── keyence_plc_bytes.py    # 位元組處理工具
    └── mock_keyence_plc_com.py # 模擬PLC(測試用)
```

## 主要類別

### 1. KeyencePlcCom (keyence_plc_com.py)
**核心TCP通訊類別**:
```python
class KeyencePlcCom:
    def __init__(self, ip, port)
    def connect(self, test=False)           # TCP連線建立
    def send_command(self, command)         # 發送PLC指令
    def force_on(self, device_type, device) # 強制設定ON
    def force_off(self, device_type, device) # 強制設定OFF
    def read_data(self, device_type, device) # 讀取資料
    def write_data(self, device_type, device, data) # 寫入資料
```

**錯誤處理機制**:
```python
ERROR_MESSAGES = {
    "E0": "E0:元件編號異常",
    "E1": "E1:指令異常", 
    "E4": "E4:禁止寫入"
}
```

### 2. KeyencePlcPool (keyence_plc_pool.py)
**連線池管理**:
```python
class KeyencePlcPool:
    def __init__(self, ip, port, max_connections=5)
    def get_connection()    # 取得可用連線
    def return_connection() # 歸還連線
    def _pool_daemon()      # 背景重連程序
```

**特性**:
- 最大連線數: 5個並發連線
- 自動重連機制: 5秒間隔重試
- 線程安全的連線池管理

### 3. KeyencePlcCommand (keyence_plc_command.py)
**Keyence協議指令定義**:
```python
class KeyencePlcCommand:
    @staticmethod
    def model()         # 查詢機型: "?K\r\n"
    def get_run_mode()  # 查詢運行模式: "?M\r\n"  
    def force_on(device_type, device_number)  # ForceOn: "ST MR3708\r\n"
    def force_off(device_type, device_number) # ForceOff: "RS MR3708\r\n"
    def read_data(device_type, device_number) # 讀取: "RD DM2990\r\n"
    def write_data(device_type, device_number, data) # 寫入: "WR DM2990 100\r\n"
```

**協議特性**:
- 終止符號: `\r\n` (PLC_END_MARKER)
- 連線超時: 5秒 (CONNECT_TIMEOUT)
- 支援設備類型: MR(繼電器), DM(資料記憶體)

## 開發指令

### 環境設定
```bash
# AGV容器內
source /app/setup.bash && all_source
cd /app/keyence_plc_ws

# AGVC容器內  
source /app/setup.bash && agvc_source
cd /app/keyence_plc_ws
```

### 服務啟動
```bash
# 啟動Keyence PLC驅動
ros2 run keyence_plc keyence_plc_node

# 指定配置啟動
ros2 run keyence_plc keyence_plc_node --ros-args -p config_file:=/app/config/agv/plc_config.yaml

# 測試PLC連線
ros2 run keyence_plc test_connection
```

### 構建與測試
```bash
build_ws keyence_plc_ws
ros2 test keyence_plc  # PLC通訊測試
```

## PLC通訊開發

### 連線配置
```yaml
# /app/config/agv/plc_config.yaml
keyence_plc:
  host: "192.168.1.100"
  port: 8501
  timeout: 5.0
  retry_count: 3
  reconnect_interval: 10.0
  
  # 數據映射
  input_registers:
    - {address: "D0", name: "agv_status", type: "int16"}
    - {address: "D1", name: "robot_position", type: "float32"}
    
  output_registers:
    - {address: "D100", name: "move_command", type: "int16"}
    - {address: "D101", name: "target_position", type: "float32"}
```

### 協議實現
```python
# protocols/keyence_protocol.py
class KeyenceProtocol:
    def read_register(self, address: str) -> bytes:
        """讀取PLC暫存器數據"""
        command = self._build_read_command(address)
        response = self._send_command(command)
        return self._parse_response(response)
        
    def write_register(self, address: str, value: bytes) -> bool:
        """寫入PLC暫存器數據"""
        command = self._build_write_command(address, value)
        response = self._send_command(command)
        return self._verify_write_success(response)
```

### 數據轉換
```python
# utils/data_converter.py
def plc_to_ros(plc_data: bytes, data_type: str):
    """PLC數據轉換為ROS 2訊息格式"""
    if data_type == "int16":
        return struct.unpack(">h", plc_data)[0]
    elif data_type == "float32":
        return struct.unpack(">f", plc_data)[0]
        
def ros_to_plc(ros_value, data_type: str) -> bytes:
    """ROS 2數據轉換為PLC格式"""
    if data_type == "int16":
        return struct.pack(">h", int(ros_value))
    elif data_type == "float32":
        return struct.pack(">f", float(ros_value))
```

## PLC整合模式

### AGV車載整合
- **機械臂控制**: 位置指令與狀態回饋
- **感測器數據**: 安全感測器狀態讀取
- **運動控制**: 馬達使能與速度控制
- **安全系統**: 緊急停止與安全檢查

### AGVC站點整合
- **充電站控制**: 充電狀態監控與控制
- **緩衝區管理**: 料架位置檢測
- **環境監控**: 溫濕度、煙霧感測器
- **設備狀態**: 站點設備健康監控

## 錯誤處理

### 連線錯誤
```python
class PLCConnectionManager:
    def handle_connection_error(self, error):
        self.logger.error(f"PLC連線錯誤: {error}")
        self.attempt_reconnect()
        
    def attempt_reconnect(self):
        for attempt in range(self.max_retries):
            try:
                self.connect()
                break
            except Exception as e:
                time.sleep(self.reconnect_interval)
```

### 數據驗證
- 檢查PLC回應完整性
- 驗證數據格式正確性
- 實施數據範圍檢查
- 記錄異常數據事件

## 測試與調試

### 單元測試
```python
# test/test_keyence_protocol.py
def test_read_register():
    protocol = KeyenceProtocol()
    mock_plc_response = b'\x01\x02\x03\x04'
    result = protocol.parse_response(mock_plc_response)
    assert result == expected_value
```

### 調試工具
```bash
# PLC連線測試
ros2 run keyence_plc debug_connection --host 192.168.1.100

# 數據讀取測試
ros2 topic echo /plc/input_data

# 數據寫入測試  
ros2 topic pub /plc/output_command keyence_plc_msgs/PLCCommand "{address: 'D100', value: 123}"
```

## 硬體配置

### 網路設定
- **PLC IP**: 根據硬體映射配置
- **埠號**: 通常為8501(Keyence預設)
- **網路延遲**: 考慮工業網路特性
- **防火牆**: 確保通訊埠開放

### PLC程式配置
- 確認PLC端通訊設定正確
- 驗證暫存器地址映射
- 檢查數據格式設定
- 測試通訊協議版本相容性

## 故障排除

### 常見問題
1. **連線超時**: 檢查網路連線與PLC狀態
2. **數據格式錯誤**: 驗證PLC端數據類型設定
3. **地址錯誤**: 確認暫存器地址正確性
4. **協議不相容**: 檢查Keyence PLC型號與協議版本

### 診斷指令
```bash
# 網路連通性測試
ping 192.168.1.100

# PLC服務狀態
ros2 service call /plc/get_status keyence_plc_msgs/srv/GetStatus

# 檢查PLC主題
ros2 topic list | grep plc
```

### 日誌分析
- PLC連線日誌: ROS 2節點輸出
- 網路通訊日誌: tcpdump分析
- 錯誤統計: 透過監控系統查看

## 安全注意事項

### 工業安全
- 實施適當的安全檢查
- 緊急停止信號處理
- 防止誤操作保護
- 設備狀態監控

### 通訊安全
- 使用專用工業網路
- 限制PLC訪問權限
- 監控異常通訊活動
- 實施通訊加密(如需要)

## 重要提醒
- PLC通訊影響系統安全，變更需謹慎
- 數據地址映射需與PLC程式一致
- 支援AGV與AGVC雙環境，注意配置差異
- 網路延遲影響即時性，需最佳化通訊頻率