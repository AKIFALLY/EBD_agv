# keyence_plc_ws CLAUDE.md

## 📚 Context Loading
@docs-ai/context/system/rosagv-overview.md
@docs-ai/context/system/dual-environment.md
@docs-ai/knowledge/protocols/keyence-plc-protocol.md
@docs-ai/operations/development/plc-communication.md
@docs-ai/knowledge/protocols/ros2-interfaces.md
@docs-ai/operations/development/ros2-development.md
@docs-ai/operations/development/docker-development.md

## 系統概述
Keyence PLC通訊庫，提供低層TCP Socket連線與Keyence專用協議實現，為AGV/AGVC系統提供PLC控制基礎服務。

**⚠️ 重要**: 所有 ROS 2 程式必須在 Docker 容器內執行，宿主機無 ROS 2 環境。

**🔗 重要**: 這是純Python庫(非ROS節點)，被plc_proxy_ws封裝為ROS 2服務使用。

## 核心架構
```
keyence_plc_ws/
└── src/keyence_plc/
    ├── keyence_plc_com.py      # TCP通訊核心類別
    ├── keyence_plc_pool.py     # 連線池管理
    ├── keyence_plc_command.py  # 協議指令生成器
    ├── keyence_plc_memory.py   # PLC記憶體模擬
    ├── keyence_plc_bytes.py    # 位元組處理工具
    └── mock_keyence_plc_com.py # 模擬PLC(測試用)
```

## API 概覽
| 類別 | 用途 | 主要方法 |
|------|------|----------|
| `KeyencePlcCom` | TCP 通訊 | `connect()`, `send_command()`, `disconnect()` |
| `KeyencePlcPool` | 連線池管理 | `get_connection()`, `return_connection()` |
| `KeyencePlcCommand` | 指令生成 | `read_data()`, `write_data()`, `force_on()` |
| `PlcBytes` | 資料轉換 | `to_int()`, `to_float()`, `from_int()` |
| `PlcMemory` | 記憶體模擬 | `set_int()`, `get_int()`, `set_float()` |
| `MockKeyencePlcCom` | 測試模擬 | `connect()`, `send_command()` |

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
靜態指令生成器，封裝 Keyence 協議指令格式。
**詳細協議說明**: @docs-ai/knowledge/protocols/keyence-plc-protocol.md

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

## 配置參數
**詳細配置說明**: @docs-ai/operations/development/plc-communication.md

主要參數: 連接超時(5秒)、連線池大小(1-5個)、重連間隔(5秒)

## 🔧 開發環境設定

### 容器內開發環境
@docs-ai/operations/development/docker-development.md

```bash
# 進入 AGV 容器
docker compose -f docker-compose.yml exec rosagv bash

# 進入 AGVC 容器  
docker compose -f docker-compose.agvc.yml exec agvc_server bash

# 智能載入工作空間
all_source  # 自動檢測並載入對應環境的工作空間

cd /app/keyence_plc_ws
```

### 建置與測試
@docs-ai/operations/development/build-and-test.md

```bash
# 建置工作空間
colcon build --packages-select keyence_plc

# 執行測試
colcon test --packages-select keyence_plc
```

## 快速開始

**詳細使用範例**: @docs-ai/operations/development/plc-communication.md

```python
# 基本使用
from keyence_plc.keyence_plc_com import KeyencePlcCom
from keyence_plc.keyence_plc_command import KeyencePlcCommand

plc = KeyencePlcCom("192.168.2.101", 8501)
plc.connect()
response = plc.send_command(KeyencePlcCommand.read_data("DM", "2990"))
plc.disconnect()
```

## 測試

**詳細測試指導**: @docs-ai/operations/development/plc-communication.md

### 測試結構
- 基本通訊測試: `test/keyence_plc_com_test.py`
- 記憶體操作測試: `test/plc_memory_test.py`
- 模擬PLC: `mock_keyence_plc_com.py`

## 故障排除

**完整故障排除指導**: @docs-ai/operations/development/plc-communication.md

### 模組特定問題
- **連線池耗盡**: 檢查 `pool.lost_connections` 狀態
- **Python 導入錯誤**: 確認虛擬環境載入
- **Mock PLC 測試**: 使用 `MockKeyencePlcCom` 進行離線測試

### 快速診斷
```bash
# 模組導入測試
python3 -c "from keyence_plc.keyence_plc_com import KeyencePlcCom; print('導入成功')"

# 執行基本測試
python3 test/keyence_plc_com_test.py
```

## 💡 重要提醒

### 架構設計
- 這是純Python庫，不直接提供ROS 2接口
- 通過plc_proxy_ws封裝後供ROS 2系統使用
- 支援AGV與AGVC雙環境，需正確配置網路
- PLC通訊影響系統安全，變更需謹慎測試
- 連線池可提高並發性能，適合高頻操作場景

### 雙環境支援
@docs-ai/context/system/dual-environment.md
- AGV 車載環境和 AGVC 管理環境都可使用此模組
- 透過 Zenoh RMW 實現跨容器通訊
- 需要正確配置 PLC 網路連接

## 🔗 交叉引用
- **協議和實踐**: @docs-ai/knowledge/protocols/keyence-plc-protocol.md - Keyence 協議詳解
- **開發最佳實踐**: @docs-ai/operations/development/plc-communication.md - PLC 通訊開發指導
- **PLC 代理服務**: `app/plc_proxy_ws/CLAUDE.md` - ROS 2 PLC 服務封裝
- **手動控制服務**: `app/agv_cmd_service_ws/CLAUDE.md` - 上層應用整合
- **ROS 2 開發指導**: @docs-ai/operations/development/ros2-development.md
- **容器開發環境**: @docs-ai/operations/development/docker-development.md
- **系統診斷工具**: @docs-ai/operations/maintenance/system-diagnostics.md
- **Zenoh 通訊機制**: @docs-ai/knowledge/protocols/zenoh-rmw.md