# CLAUDE.md

## 系統概述
Keyence PLC通訊庫，提供低層TCP Socket連線與Keyence專用協議實現，為AGV/AGVC系統提供PLC控制基礎服務。

**🔗 重要**: 這是純Python庫(非ROS節點)，被plc_proxy_ws封裝為ROS 2服務使用。

## 核心架構
```
keyence_plc_ws/
└── src/keyence_plc/
    ├── keyence_plc_com.py      # TCP通訊核心類別
    ├── keyence_plc_pool.py     # 連線池管理
    ├── keyence_plc_command.py  # Keyence協議指令生成器
    ├── keyence_plc_memory.py   # PLC記憶體模擬
    ├── keyence_plc_bytes.py    # 位元組處理工具
    └── mock_keyence_plc_com.py # 模擬PLC(測試用)
```

## 主要類別

### 1. KeyencePlcCom (keyence_plc_com.py)
**核心TCP通訊類別**:
```python
class KeyencePlcCom:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.sock = None
        self.timeout = CONNECT_TIMEOUT  # 5秒
        
    def connect(self, test=False):
        """建立TCP連線到PLC"""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(self.timeout)
        self.sock.connect((self.ip, self.port))
        
    def send_command(self, command):
        """發送指令並接收回應，包含錯誤檢查"""
        self.sock.sendall(command.encode("utf-8"))
        response = self.receive_until()
        
        # 錯誤檢查
        if response[:2] in self.ERROR_MESSAGES:
            raise Exception(self.ERROR_MESSAGES[response[:2]])
        return response
        
    def receive_until(self, end_marker=b"\r\n"):
        """接收資料直到收到結束標記"""
        # 實現接收邏輯
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
    def __init__(self, ip, port, max_connections=MAX_POOL_SIZE):
        self.max_connections = max_connections  # 預設5個
        self.connections = [KeyencePlcCom(ip, port) for _ in range(max_connections)]
        self.lost_connections = []
        self.semaphore = threading.Semaphore(max_connections)
        
    def _pool_daemon(self):
        """背景執行緒持續重連失效的連線"""
        while self._running:
            time.sleep(RECONNECT_INTERVAL)  # 5秒重試間隔
            # 嘗試重連 lost_connections 中的連線
```

**特性**:
- 最大連線數: 5個並發連線 (MAX_POOL_SIZE)
- 自動重連機制: 5秒間隔重試 (RECONNECT_INTERVAL)
- 線程安全的 Semaphore 控制

### 3. KeyencePlcCommand (keyence_plc_command.py)
**Keyence協議指令靜態生成器**:
```python
class KeyencePlcCommand:
    @staticmethod
    def model():
        """查詢機型指令"""
        return f"?K{PLC_END_MARKER}"
        
    @staticmethod
    def get_run_mode():
        """查詢運行模式指令"""
        return f"?M{PLC_END_MARKER}"
        
    @staticmethod
    def force_on(device_type, device_number):
        """ForceOn指令: ST MR3708\r\n"""
        return f"ST {device_type}{device_number}{PLC_END_MARKER}"
        
    @staticmethod
    def force_off(device_type, device_number):
        """ForceOff指令: RS MR3708\r\n"""
        return f"RS {device_type}{device_number}{PLC_END_MARKER}"
        
    @staticmethod
    def read_data(device_type, device_number):
        """讀取PLC資料指令: RD DM2990\r\n"""
        return f"RD {device_type}{device_number}{PLC_END_MARKER}"
        
    @staticmethod
    def write_data(device_type, device_number, write_data):
        """寫入PLC資料指令: WR DM2990 100\r\n"""
        return f"WR {device_type}{device_number} {write_data}{PLC_END_MARKER}"
        
    @staticmethod
    def read_continuous_data(device_type, device_number, device_length):
        """連續讀取指令: RDS DM2990 5\r\n"""
        return f"RDS {device_type}{device_number} {device_length}{PLC_END_MARKER}"
        
    @staticmethod
    def write_continuous_data(device_type, device_number, write_data):
        """連續寫入指令: WRS DM2990 3 100 200 300\r\n"""
        data_str = " ".join(str(x) for x in write_data)
        return f"WRS {device_type}{device_number} {len(write_data)} {data_str}{PLC_END_MARKER}"
```

### 4. PlcBytes (keyence_plc_bytes.py)
**位元組處理工具類**:
```python
class PlcBytes(bytearray):
    def to_int(self) -> int:
        """轉換為整數，支援2/4/8位元組"""
        length = len(self)
        if length == 2:
            fmt = "<h"  # 2 bytes (short)
        elif length == 4:
            fmt = "<i"  # 4 bytes (int)
        elif length == 8:
            fmt = "<q"  # 8 bytes (long long)
        return struct.unpack(fmt, bytes(self))[0]
        
    def to_float(self) -> float:
        """轉換為浮點數(4位元組)"""
        return struct.unpack("<f", bytes(self))[0]
        
    @classmethod
    def from_int(cls, value: int, length: int = 4):
        """從整數創建PlcBytes"""
        fmt = "<H" if length == 2 else "<I" if length == 4 else "<Q"
        return cls(struct.pack(fmt, value))
        
    @classmethod
    def from_float(cls, value: float):
        """從浮點數創建PlcBytes"""
        return cls(struct.pack("<f", value))
```

### 5. PlcMemory (keyence_plc_memory.py)
**PLC記憶體模擬類別**:
```python
class PlcMemory:
    def __init__(self, size: int = 131072):  # 65535*2 bytes
        self.memory = PlcBytes(size)
        
    def address_to_index(self, address: int) -> int:
        """PLC地址轉換為記憶體索引"""
        return address * 2
        
    def set_int(self, address: int, value: int, length: int = 2):
        """設置整數值到記憶體"""
        self.set_memory(address, PlcBytes.from_int(value, length))
        
    def get_int(self, address: int, length: int = 2) -> int:
        """從記憶體讀取整數值"""
        return self.get_bytes(address, length).to_int()
        
    def set_float(self, address: int, value: float):
        """設置浮點數值到記憶體"""
        self.set_memory(address, PlcBytes.from_float(value))
        
    def get_float(self, address: int, length: int = 4) -> float:
        """從記憶體讀取浮點數值"""
        return self.get_bytes(address, length).to_float()
```

### 6. MockKeyencePlcCom (mock_keyence_plc_com.py)
**測試用模擬PLC**:
```python
class MockKeyencePlcCom:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.sock = None
        
    def connect(self):
        """模擬連線成功"""
        return True
        
    def send_command(self, command):
        """模擬PLC回應"""
        return "OK\r\n"
```

## 協議常數定義
```python
# keyence_plc_com.py 和 keyence_plc_command.py
PLC_END_MARKER = "\r\n"  # PLC協議結束標記
CONNECT_TIMEOUT = 5      # TCP連線超時(秒)

# keyence_plc_pool.py
MIN_POOL_SIZE = 1        # 最小連線池大小
MAX_POOL_SIZE = 5        # 最大連線池大小
RECONNECT_INTERVAL = 5   # 重連間隔(秒)
```

## 開發指令

### 環境設定 (容器內執行)
```bash
# AGV容器內
source /app/setup.bash && agv_source  # 或使用 all_source (自動檢測)
cd /app/keyence_plc_ws

# AGVC容器內  
source /app/setup.bash && agvc_source  # 或使用 all_source (自動檢測)
cd /app/keyence_plc_ws
```

### 構建與測試
```bash
build_ws keyence_plc_ws
test_ws keyence_plc_ws
```

## 使用範例

### 1. 基本 PLC 通訊
```python
from keyence_plc.keyence_plc_com import KeyencePlcCom
from keyence_plc.keyence_plc_command import KeyencePlcCommand

# 建立PLC連線
plc = KeyencePlcCom("192.168.1.100", 8501)
plc.connect()

# 查詢PLC機型
model_cmd = KeyencePlcCommand.model()
response = plc.send_command(model_cmd)
print(f"PLC機型: {response}")

# 強制設定MR3708為ON
force_on_cmd = KeyencePlcCommand.force_on("MR", "3708")
response = plc.send_command(force_on_cmd)

# 寫入數據到DM2990
write_cmd = KeyencePlcCommand.write_data("DM", "2990", "100")
response = plc.send_command(write_cmd)

# 讀取DM2990的數據
read_cmd = KeyencePlcCommand.read_data("DM", "2990")
response = plc.send_command(read_cmd)
print(f"DM2990值: {response}")

plc.disconnect()
```

### 2. 使用連線池
```python
from keyence_plc.keyence_plc_pool import KeyencePlcPool

# 建立連線池
pool = KeyencePlcPool("192.168.1.100", 8501, max_connections=3)

# 取得連線
plc = pool.get_connection()

try:
    # 執行PLC操作
    cmd = KeyencePlcCommand.read_data("DM", "2990")
    response = plc.send_command(cmd)
    print(f"讀取結果: {response}")
finally:
    # 歸還連線到池中
    pool.return_connection(plc)
```

### 3. 數據類型轉換
```python
from keyence_plc.keyence_plc_bytes import PlcBytes

# 整數轉換
int_bytes = PlcBytes.from_int(12345, length=2)  # 2位元組整數
value = int_bytes.to_int()

# 浮點數轉換
float_bytes = PlcBytes.from_float(3.14159)
float_value = float_bytes.to_float()

# 位元組陣列處理
data = PlcBytes(b'\x01\x02\x03\x04')
int_value = data.to_int()  # 轉換為整數
```

### 4. PLC記憶體模擬
```python
from keyence_plc.keyence_plc_memory import PlcMemory

# 建立PLC記憶體模擬
memory = PlcMemory(size=1024)  # 1KB記憶體

# 寫入整數到地址100
memory.set_int(100, 12345)

# 讀取地址100的整數
value = memory.get_int(100)
print(f"地址100的值: {value}")

# 寫入浮點數到地址200
memory.set_float(200, 3.14159)

# 讀取地址200的浮點數
float_value = memory.get_float(200)
print(f"地址200的值: {float_value}")
```

## 測試與調試

### 1. 測試文件結構
```
keyence_plc_ws/
├── test/                    # 測試相關文件 (在工作空間根目錄)
├── keyence_plc_com_async.py    # 異步通訊測試
├── keyence_plc_com_patch.py    # 修補版本測試  
└── test/
    ├── keyence_plc_com_test.py     # 基本通訊測試
    ├── plc_memory_test.py          # 記憶體操作測試
    └── read_write_test.py          # 讀寫功能測試
```

### 2. 基本測試範例
```python
# test/keyence_plc_com_test.py
from keyence_plc.keyence_plc_com import KeyencePlcCom
from keyence_plc.keyence_plc_command import KeyencePlcCommand

# 測試PLC連線
def test_plc_connection():
    plc = KeyencePlcCom("192.168.1.100", 8501)
    try:
        success = plc.connect()
        assert success, "PLC連線失敗"
        print("✅ PLC連線測試通過")
    except Exception as e:
        print(f"❌ PLC連線測試失敗: {e}")
    finally:
        plc.disconnect()

# 測試指令生成
def test_command_generation():
    # 測試各種指令格式
    assert KeyencePlcCommand.model() == "?K\r\n"
    assert KeyencePlcCommand.force_on("MR", "3708") == "ST MR3708\r\n"
    assert KeyencePlcCommand.read_data("DM", "2990") == "RD DM2990\r\n"
    print("✅ 指令生成測試通過")
```

### 3. 模擬PLC測試
```python
from keyence_plc.mock_keyence_plc_com import MockKeyencePlcCom

# 使用模擬PLC進行測試
mock_plc = MockKeyencePlcCom("localhost", 8501)
mock_plc.connect()
response = mock_plc.send_command("?K\r\n")
print(f"模擬PLC回應: {response}")  # 輸出: OK\r\n
```

## 故障排除

### 常見問題
1. **連線超時**: 檢查網路連線與PLC狀態
   ```python
   # 調整超時時間
   plc = KeyencePlcCom("192.168.1.100", 8501)
   plc.timeout = 10  # 設定為10秒
   ```

2. **協議錯誤**: 確認指令格式正確
   ```python
   # 檢查錯誤回應
   try:
       response = plc.send_command(command)
   except Exception as e:
       if "E0" in str(e):
           print("元件編號異常")
       elif "E1" in str(e):
           print("指令異常")
   ```

3. **連線池問題**: 連線數量超過限制
   ```python
   # 監控連線池狀態
   pool = KeyencePlcPool("192.168.1.100", 8501)
   print(f"可用連線: {len(pool.connections)}")
   print(f"失效連線: {len(pool.lost_connections)}")
   ```

### 調試技巧
```bash
# 1. 網路連通性測試  
ping 192.168.1.100
telnet 192.168.1.100 8501

# 2. 查看Python導入
python3 -c "from keyence_plc.keyence_plc_com import KeyencePlcCom; print('導入成功')"

# 3. 容器內測試
# AGV/AGVC容器內
cd /app/keyence_plc_ws
python3 test/keyence_plc_com_test.py
```

## 硬體配置注意事項

### PLC網路設定
- **預設端口**: Keyence PLC通常使用8501端口
- **IP配置**: 確保PLC與系統在同一網段
- **通訊協議**: 支援Keyence專用TCP協議
- **延遲考量**: 工業網路可能有較高延遲

### 實際部署建議
- 在plc_proxy_ws中封裝此庫為ROS 2服務
- 透過agv_cmd_service_ws使用PLC功能
- 配置正確的PLC IP地址和端口
- 實施適當的錯誤處理和重連機制

## 重要提醒
- 這是純Python庫，不直接提供ROS 2接口
- 通過plc_proxy_ws封裝後供ROS 2系統使用
- 支援AGV與AGVC雙環境，需正確配置網路
- PLC通訊影響系統安全，變更需謹慎測試
- 連線池可提高並發性能，適合高頻操作場景