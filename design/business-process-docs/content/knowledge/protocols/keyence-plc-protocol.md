# Keyence PLC 協定詳解

## 概述

Keyence PLC 通訊協定是 RosAGV 系統中關鍵的工業通訊標準，透過 TCP/IP Socket 連接實現與 Keyence PLC 設備的可靠通訊。本協定支援資料讀寫、強制控制和狀態監控等核心功能。

## 協定基礎

### 通訊特性

#### 基本參數
```
Keyence PLC 通訊協定規格
├── 通訊方式: TCP/IP Socket 連接
├── 預設端口: 8501
├── 編碼格式: UTF-8
├── 指令終止符: \r\n (CRLF)
├── 超時設定: 5秒 (可調整)
└── 連接模式: 持久連接 + 連接池管理
```

#### 協定格式標準
```
指令格式規範
├── 查詢指令: [指令碼]\r\n
├── 讀取指令: [指令碼] [設備][地址]\r\n
├── 寫入指令: [指令碼] [設備][地址] [數值]\r\n
└── 連續操作: [指令碼] [設備][地址] [長度] [數值...]\r\n

回應格式規範
├── 正常回應: [狀態/資料]\r\n
├── 錯誤回應: [錯誤碼]\r\n
└── 資料回應: [資料值1] [資料值2] ...\r\n
```

## 指令集詳解

### 系統查詢指令

#### 基本系統資訊
```bash
# PLC 機型查詢
?K\r\n
# 回應範例: "KV-7500\r\n"

# 運行模式查詢
?M\r\n  
# 回應: "01\r\n" (01=運行模式, 00=停止模式)

# 系統狀態查詢
?D\r\n
# 回應: 系統診斷資訊
```

#### 進階系統查詢
```bash
# CPU 模組資訊
?KS\r\n
# 回應: CPU 模組型號和版本

# 通訊設定查詢
?T\r\n
# 回應: 通訊協定設定資訊

# 錯誤狀態查詢
?E\r\n
# 回應: 當前錯誤狀態碼
```

### 資料讀寫指令

#### 單一資料操作
```bash
# 讀取單一資料
RD [設備][地址]\r\n
# 範例: RD DM2990\r\n
# 回應: "12345\r\n" (DM2990 的數值)

# 寫入單一資料  
WR [設備][地址] [數值]\r\n
# 範例: WR DM2990 54321\r\n
# 回應: "OK\r\n" (寫入成功)

# 讀取位元資料
RD [設備][地址]\r\n
# 範例: RD MR3708\r\n
# 回應: "1\r\n" (ON狀態) 或 "0\r\n" (OFF狀態)
```

#### 連續資料操作
```bash
# 連續讀取資料
RDS [設備][地址] [長度]\r\n
# 範例: RDS DM2990 10\r\n
# 回應: "12345 23456 34567 ... (10個數值)\r\n"

# 連續寫入資料
WRS [設備][地址] [長度] [數值1] [數值2] ...\r\n
# 範例: WRS DM2990 3 100 200 300\r\n
# 回應: "OK\r\n"

# 位元組資料讀取
RDB [設備][地址] [長度]\r\n
# 範例: RDB DM2990 4\r\n (讀取4個位元組)
```

### 強制控制指令

#### 位元強制操作
```bash
# 強制 ON (ForceOn)
ST [設備][地址]\r\n
# 範例: ST MR3708\r\n
# 回應: "OK\r\n"

# 強制 OFF (ForceOff)  
RS [設備][地址]\r\n
# 範例: RS MR3708\r\n
# 回應: "OK\r\n"

# 批量強制操作
STS [設備][地址] [長度] [位元組資料]\r\n
# 範例: STS MR3700 8 FF\r\n (設定8個位元為ON)
```

## 設備地址系統

### 設備類型對照

#### 記憶體設備
```python
MEMORY_DEVICES = {
    "DM": {
        "name": "Data Memory",
        "description": "資料記憶體",
        "range": "DM0-DM32767",
        "data_type": "16位元整數",
        "example": "DM2990"
    },
    "EM": {
        "name": "Extended Memory", 
        "description": "擴展記憶體",
        "range": "EM0-EM32767",
        "data_type": "16位元整數",
        "example": "EM1000"
    }
}
```

#### 繼電器設備
```python
RELAY_DEVICES = {
    "MR": {
        "name": "Memory Relay",
        "description": "內部繼電器",
        "range": "MR0-MR32767", 
        "data_type": "位元",
        "example": "MR3708"
    },
    "R": {
        "name": "Input Relay",
        "description": "輸入繼電器", 
        "range": "R000-R3FF (十六進制)",
        "data_type": "位元",
        "example": "R100"
    },
    "Y": {
        "name": "Output Relay",
        "description": "輸出繼電器",
        "range": "Y000-Y3FF (十六進制)", 
        "data_type": "位元",
        "example": "Y200"
    }
}
```

#### 特殊設備
```python
SPECIAL_DEVICES = {
    "SM": {
        "name": "Special Memory",
        "description": "特殊記憶體",
        "range": "SM400-SM2047",
        "data_type": "位元", 
        "example": "SM400"
    },
    "B": {
        "name": "Link Relay", 
        "description": "連結繼電器",
        "range": "B000-B1FFF",
        "data_type": "位元",
        "example": "B300"
    }
}
```

### 地址格式規範

#### 數值格式
```python
# 十進制地址 (標準格式)
DECIMAL_FORMAT = {
    "pattern": r"^[A-Z]+\d+$",
    "examples": ["DM2990", "MR3708", "EM1000"],
    "description": "直接使用數字表示地址"
}

# 十六進制地址
HEX_FORMAT = {
    "pattern": r"^[A-Z]+H[0-9A-F]+$", 
    "examples": ["DMH1234", "MRH0A0B", "EMH2000"],
    "description": "使用 H 前綴表示十六進制"
}

# 位元地址 (針對字組設備)
BIT_FORMAT = {
    "pattern": r"^[A-Z]+\d+\.\d+$",
    "examples": ["DM2990.0", "DM2990.15"],
    "description": "字組設備的特定位元"
}
```

#### 地址驗證函數
```python
import re
from typing import Tuple, Optional

def validate_device_address(device_type: str, address: str) -> Tuple[bool, Optional[str]]:
    """驗證設備地址格式"""
    
    # 設備類型驗證
    valid_devices = ["DM", "EM", "MR", "R", "Y", "SM", "B"]
    if device_type not in valid_devices:
        return False, f"無效的設備類型: {device_type}"
    
    # 地址格式驗證
    decimal_pattern = r"^\d+$"
    hex_pattern = r"^H[0-9A-F]+$"
    bit_pattern = r"^\d+\.\d+$"
    
    if re.match(decimal_pattern, address):
        # 十進制地址範圍檢查
        addr_num = int(address)
        if device_type in ["DM", "EM"] and (addr_num < 0 or addr_num > 32767):
            return False, f"{device_type} 地址範圍錯誤: 0-32767"
        return True, None
        
    elif re.match(hex_pattern, address):
        # 十六進制地址檢查
        try:
            addr_num = int(address[1:], 16)  # 移除 H 前綴
            return True, None
        except ValueError:
            return False, f"無效的十六進制地址: {address}"
    
    elif re.match(bit_pattern, address):
        # 位元地址檢查
        parts = address.split('.')
        word_addr = int(parts[0])
        bit_addr = int(parts[1])
        if bit_addr > 15:
            return False, f"位元地址超出範圍: {bit_addr} (0-15)"
        return True, None
    
    return False, f"無效的地址格式: {address}"
```

## 錯誤處理機制

### 標準錯誤碼

#### 基本錯誤類型
```python
KEYENCE_ERROR_CODES = {
    "E0": {
        "code": "E0",
        "name": "元件編號異常", 
        "description": "設備地址錯誤或不存在",
        "solutions": [
            "檢查設備類型是否正確 (DM, MR, R, Y, etc.)",
            "確認地址範圍是否在有效範圍內",
            "驗證地址格式是否符合規範"
        ]
    },
    "E1": {
        "code": "E1", 
        "name": "指令異常",
        "description": "指令格式錯誤或不支援的指令",
        "solutions": [
            "檢查指令格式是否正確",
            "確認指令是否以 \\r\\n 結尾", 
            "驗證指令參數數量和類型"
        ]
    },
    "E4": {
        "code": "E4",
        "name": "禁止寫入",
        "description": "寫入權限錯誤或設備處於保護狀態", 
        "solutions": [
            "檢查 PLC 是否處於運行模式",
            "確認設備是否為只讀類型",
            "驗證寫入權限設定"
        ]
    }
}
```

#### 進階錯誤碼
```python
EXTENDED_ERROR_CODES = {
    "E2": {
        "code": "E2",
        "name": "資料格式異常",
        "description": "寫入資料格式不正確",
        "solutions": [
            "檢查數值是否在有效範圍內",
            "確認資料類型是否匹配設備要求"
        ]
    },
    "E3": {
        "code": "E3", 
        "name": "通訊逾時",
        "description": "PLC 響應超時",
        "solutions": [
            "檢查網路連接狀態",
            "調整通訊超時參數",
            "確認 PLC 運行狀態"
        ]
    },
    "E5": {
        "code": "E5",
        "name": "系統忙碌",
        "description": "PLC 系統正在執行其他操作",
        "solutions": [
            "等待片刻後重試",
            "檢查是否有其他程式同時存取",
            "調整請求頻率"
        ]
    }
}
```

### 錯誤處理策略

#### 分層錯誤處理
```python
class KeyencePlcErrorHandler:
    def __init__(self, logger=None):
        self.logger = logger or logging.getLogger(__name__)
        self.error_statistics = {}
        
    def handle_response_error(self, response: str, command: str) -> Exception:
        """處理回應錯誤"""
        if not response:
            return PlcCommunicationError("PLC 無回應")
            
        # 檢查錯誤碼
        error_code = response.strip()[:2] 
        if error_code in KEYENCE_ERROR_CODES:
            error_info = KEYENCE_ERROR_CODES[error_code]
            self._log_error(error_code, command, error_info)
            return PlcProtocolError(
                f"{error_info['name']}: {error_info['description']}", 
                error_code=error_code,
                command=command
            )
        
        return None
    
    def handle_connection_error(self, error: Exception, retry_count: int = 0) -> bool:
        """處理連接錯誤"""
        self.logger.error(f"PLC 連接錯誤 (重試 {retry_count}): {error}")
        
        # 更新錯誤統計
        error_type = type(error).__name__
        self.error_statistics[error_type] = self.error_statistics.get(error_type, 0) + 1
        
        # 決定是否重試
        if retry_count < 3:
            time.sleep(min(2 ** retry_count, 10))  # 指數退避
            return True
        return False
    
    def _log_error(self, error_code: str, command: str, error_info: dict):
        """記錄錯誤詳情"""
        self.logger.error(
            f"PLC 協定錯誤 - 錯誤碼: {error_code}, "
            f"指令: {command}, 說明: {error_info['description']}"
        )
        
        # 記錄解決建議
        for i, solution in enumerate(error_info['solutions'], 1):
            self.logger.info(f"建議 {i}: {solution}")
```

#### 自動重試機制
```python
import asyncio
from functools import wraps

def with_retry(max_retries: int = 3, base_delay: float = 1.0):
    """PLC 指令重試裝飾器"""
    def decorator(func):
        @wraps(func)
        async def wrapper(self, *args, **kwargs):
            last_exception = None
            
            for attempt in range(max_retries + 1):
                try:
                    return await func(self, *args, **kwargs)
                    
                except PlcCommunicationError as e:
                    last_exception = e
                    if attempt < max_retries:
                        delay = base_delay * (2 ** attempt)
                        self.logger.warning(
                            f"PLC 通訊失敗，{delay}秒後重試 (嘗試 {attempt + 1}/{max_retries + 1})"
                        )
                        await asyncio.sleep(delay)
                    
                except PlcProtocolError as e:
                    # 協定錯誤通常不需要重試
                    raise e
                    
            raise last_exception
        return wrapper
    return decorator

# 使用範例
class KeyencePlcCommunicator:
    @with_retry(max_retries=3, base_delay=1.0)
    async def read_data(self, device_type: str, address: str) -> str:
        """讀取 PLC 資料 (含重試)"""
        command = f"RD {device_type}{address}\r\n"
        return await self._send_command(command)
```

## 連接管理最佳化

### 連接池設計

#### 高效連接池實作
```python
import asyncio
import threading
from typing import Optional, List
from dataclasses import dataclass

@dataclass
class PlcConnection:
    id: str
    reader: asyncio.StreamReader
    writer: asyncio.StreamWriter
    last_used: float
    is_busy: bool = False

class KeyencePlcConnectionPool:
    def __init__(
        self, 
        host: str, 
        port: int, 
        min_connections: int = 1,
        max_connections: int = 5,
        connection_timeout: float = 10.0,
        idle_timeout: float = 300.0
    ):
        self.host = host
        self.port = port
        self.min_connections = min_connections
        self.max_connections = max_connections
        self.connection_timeout = connection_timeout
        self.idle_timeout = idle_timeout
        
        self.connections: List[PlcConnection] = []
        self.semaphore = asyncio.Semaphore(max_connections)
        self.lock = asyncio.Lock()
        
    async def initialize(self):
        """初始化連接池"""
        async with self.lock:
            for i in range(self.min_connections):
                conn = await self._create_connection(f"conn_{i}")
                if conn:
                    self.connections.append(conn)
    
    async def get_connection(self) -> Optional[PlcConnection]:
        """取得可用連接"""
        await self.semaphore.acquire()
        
        async with self.lock:
            # 尋找閒置連接
            for conn in self.connections:
                if not conn.is_busy and self._is_connection_alive(conn):
                    conn.is_busy = True
                    conn.last_used = time.time()
                    return conn
            
            # 建立新連接
            if len(self.connections) < self.max_connections:
                conn = await self._create_connection(f"conn_{len(self.connections)}")
                if conn:
                    conn.is_busy = True
                    self.connections.append(conn)
                    return conn
        
        # 等待連接可用
        await self._wait_for_available_connection()
        return await self.get_connection()
    
    async def return_connection(self, connection: PlcConnection):
        """歸還連接"""
        async with self.lock:
            connection.is_busy = False
            connection.last_used = time.time()
        
        self.semaphore.release()
    
    async def _create_connection(self, conn_id: str) -> Optional[PlcConnection]:
        """建立新 PLC 連接"""
        try:
            reader, writer = await asyncio.wait_for(
                asyncio.open_connection(self.host, self.port),
                timeout=self.connection_timeout
            )
            
            return PlcConnection(
                id=conn_id,
                reader=reader,
                writer=writer,
                last_used=time.time()
            )
            
        except Exception as e:
            logging.error(f"建立 PLC 連接失敗: {e}")
            return None
    
    def _is_connection_alive(self, connection: PlcConnection) -> bool:
        """檢查連接是否仍然有效"""
        if connection.writer.is_closing():
            return False
            
        # 檢查閒置超時
        if time.time() - connection.last_used > self.idle_timeout:
            connection.writer.close()
            return False
            
        return True
```

### 連接監控

#### 健康檢查機制
```python
class PlcConnectionMonitor:
    def __init__(self, connection_pool: KeyencePlcConnectionPool):
        self.pool = connection_pool
        self.monitoring = False
        
    async def start_monitoring(self):
        """開始連接監控"""
        self.monitoring = True
        while self.monitoring:
            await self._check_connection_health()
            await asyncio.sleep(30)  # 每30秒檢查一次
    
    async def _check_connection_health(self):
        """檢查連接健康狀態"""
        dead_connections = []
        
        async with self.pool.lock:
            for conn in self.pool.connections:
                if not self.pool._is_connection_alive(conn):
                    dead_connections.append(conn)
        
        # 清理死連接
        for dead_conn in dead_connections:
            await self._remove_dead_connection(dead_conn)
        
        # 確保最小連接數
        await self._ensure_min_connections()
    
    async def _remove_dead_connection(self, connection: PlcConnection):
        """移除失效連接"""
        async with self.pool.lock:
            if connection in self.pool.connections:
                self.pool.connections.remove(connection)
                try:
                    connection.writer.close()
                    await connection.writer.wait_closed()
                except:
                    pass
    
    async def _ensure_min_connections(self):
        """確保最小連接數"""
        async with self.pool.lock:
            current_count = len(self.pool.connections)
            if current_count < self.pool.min_connections:
                for i in range(self.pool.min_connections - current_count):
                    conn = await self.pool._create_connection(f"conn_recover_{i}")
                    if conn:
                        self.pool.connections.append(conn)
```

## 效能最佳化

### 批量操作最佳化

#### 智能批量讀寫
```python
class BatchOperationOptimizer:
    def __init__(self, plc_communicator):
        self.plc = plc_communicator
        self.batch_size = 50  # 每批最大操作數
        
    async def batch_read_optimized(
        self, 
        read_requests: List[Tuple[str, str]]  # [(device_type, address), ...]
    ) -> List[str]:
        """最佳化批量讀取"""
        
        # 按設備類型分組
        grouped_requests = self._group_by_device_type(read_requests)
        results = []
        
        for device_type, addresses in grouped_requests.items():
            # 檢查是否可以使用連續讀取
            continuous_groups = self._find_continuous_addresses(addresses)
            
            for group in continuous_groups:
                if len(group) > 1:
                    # 使用連續讀取 (RDS)
                    start_addr = min(group)
                    length = max(group) - start_addr + 1
                    batch_result = await self.plc.read_continuous_data(
                        device_type, start_addr, length
                    )
                    results.extend(batch_result)
                else:
                    # 單一讀取 (RD)
                    result = await self.plc.read_data(device_type, group[0])
                    results.append(result)
        
        return results
    
    def _group_by_device_type(self, requests: List[Tuple[str, str]]) -> Dict[str, List[str]]:
        """按設備類型分組"""
        groups = {}
        for device_type, address in requests:
            if device_type not in groups:
                groups[device_type] = []
            groups[device_type].append(address)
        return groups
    
    def _find_continuous_addresses(self, addresses: List[str]) -> List[List[int]]:
        """尋找連續地址群組"""
        # 轉換為數字並排序
        numeric_addresses = []
        for addr in addresses:
            if addr.isdigit():
                numeric_addresses.append(int(addr))
        
        numeric_addresses.sort()
        
        # 尋找連續群組
        groups = []
        current_group = [numeric_addresses[0]] if numeric_addresses else []
        
        for i in range(1, len(numeric_addresses)):
            if numeric_addresses[i] == numeric_addresses[i-1] + 1:
                current_group.append(numeric_addresses[i])
            else:
                if current_group:
                    groups.append(current_group)
                current_group = [numeric_addresses[i]]
        
        if current_group:
            groups.append(current_group)
        
        return groups
```

### 快取機制

#### 智能快取策略
```python
import time
from typing import Dict, Tuple, Optional

class PlcDataCache:
    def __init__(self, default_ttl: float = 1.0):
        self.cache: Dict[str, Tuple[str, float]] = {}  # key: (value, timestamp)
        self.default_ttl = default_ttl
        self.access_count: Dict[str, int] = {}
        self.last_cleanup = time.time()
        
    def get(self, device_type: str, address: str, ttl: Optional[float] = None) -> Optional[str]:
        """取得快取資料"""
        key = f"{device_type}{address}"
        current_time = time.time()
        
        if key in self.cache:
            value, timestamp = self.cache[key]
            cache_ttl = ttl or self.default_ttl
            
            if current_time - timestamp < cache_ttl:
                # 更新存取統計
                self.access_count[key] = self.access_count.get(key, 0) + 1
                return value
            else:
                # 快取過期，移除
                del self.cache[key]
        
        return None
    
    def set(self, device_type: str, address: str, value: str):
        """設定快取資料"""
        key = f"{device_type}{address}"
        self.cache[key] = (value, time.time())
        
        # 定期清理過期快取
        if time.time() - self.last_cleanup > 60:  # 每分鐘清理一次
            self._cleanup_expired()
    
    def _cleanup_expired(self):
        """清理過期快取"""
        current_time = time.time()
        expired_keys = []
        
        for key, (value, timestamp) in self.cache.items():
            if current_time - timestamp > self.default_ttl * 2:  # 雙倍TTL後清理
                expired_keys.append(key)
        
        for key in expired_keys:
            del self.cache[key]
            if key in self.access_count:
                del self.access_count[key]
        
        self.last_cleanup = current_time
    
    def get_cache_stats(self) -> Dict:
        """取得快取統計"""
        return {
            "total_entries": len(self.cache),
            "total_accesses": sum(self.access_count.values()),
            "most_accessed": max(self.access_count.items(), key=lambda x: x[1]) if self.access_count else None
        }
```

## 診斷和測試工具

### 通訊診斷

#### 連接測試工具
```bash
#!/bin/bash
# keyence-plc-diagnostic.sh

PLC_IP="192.168.2.101"
PLC_PORT="8501"

echo "🔍 Keyence PLC 通訊診斷開始..."

# 1. 網路連通性測試
echo "📡 測試網路連通性:"
if ping -c 3 $PLC_IP > /dev/null 2>&1; then
    echo "✅ 網路連通正常"
else
    echo "❌ 網路連通失敗"
    exit 1
fi

# 2. 端口連接測試
echo "🔌 測試端口連接:"
if timeout 5 bash -c "echo > /dev/tcp/$PLC_IP/$PLC_PORT" 2>/dev/null; then
    echo "✅ 端口連接正常"
else
    echo "❌ 端口連接失敗"
    exit 1
fi

# 3. 基本指令測試
echo "📋 測試基本指令:"
response=$(echo -e "?K\r\n" | nc -w 5 $PLC_IP $PLC_PORT)
if [ -n "$response" ]; then
    echo "✅ PLC 回應正常: $response"
else
    echo "❌ PLC 無回應"
fi

# 4. 讀取測試
echo "📖 測試資料讀取:"
test_response=$(echo -e "RD DM0\r\n" | nc -w 5 $PLC_IP $PLC_PORT)
if [[ $test_response =~ ^[0-9]+$ ]]; then
    echo "✅ 資料讀取正常: $test_response"
elif [[ $test_response =~ ^E[0-9]+ ]]; then
    echo "⚠️ 回應錯誤碼: $test_response"
else
    echo "❓ 未知回應: $test_response"
fi

echo "🎉 診斷完成"
```

#### Python 診斷工具
```python
import asyncio
import socket
import time
from typing import Dict, List

class KeyencePlcDiagnostic:
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        
    async def run_full_diagnostic(self) -> Dict:
        """執行完整診斷"""
        results = {
            "timestamp": time.time(),
            "network_test": await self._test_network_connectivity(),
            "port_test": await self._test_port_connection(),
            "protocol_test": await self._test_protocol_communication(),
            "performance_test": await self._test_communication_performance()
        }
        
        return results
    
    async def _test_network_connectivity(self) -> Dict:
        """測試網路連通性"""
        start_time = time.time()
        try:
            # 使用 socket 測試連通性
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5)
            result = sock.connect_ex((self.host, self.port))
            sock.close()
            
            success = result == 0
            latency = (time.time() - start_time) * 1000  # ms
            
            return {
                "success": success,
                "latency_ms": latency,
                "error": None if success else f"Connection failed: {result}"
            }
        except Exception as e:
            return {
                "success": False,
                "latency_ms": 0,
                "error": str(e)
            }
    
    async def _test_protocol_communication(self) -> Dict:
        """測試協定通訊"""
        test_commands = [
            ("?K", "查詢機型"),
            ("?M", "查詢模式"), 
            ("RD DM0", "讀取DM0")
        ]
        
        results = []
        for command, description in test_commands:
            try:
                reader, writer = await asyncio.open_connection(self.host, self.port)
                
                # 發送指令
                start_time = time.time()
                writer.write(f"{command}\r\n".encode())
                await writer.drain()
                
                # 讀取回應
                response = await asyncio.wait_for(reader.read(1024), timeout=5)
                response_time = (time.time() - start_time) * 1000
                
                writer.close()
                await writer.wait_closed()
                
                results.append({
                    "command": command,
                    "description": description,
                    "success": True,
                    "response": response.decode().strip(),
                    "response_time_ms": response_time
                })
                
            except Exception as e:
                results.append({
                    "command": command,
                    "description": description,
                    "success": False,
                    "error": str(e),
                    "response_time_ms": 0
                })
        
        return results
    
    async def _test_communication_performance(self) -> Dict:
        """測試通訊效能"""
        test_iterations = 100
        response_times = []
        
        try:
            reader, writer = await asyncio.open_connection(self.host, self.port)
            
            for i in range(test_iterations):
                start_time = time.time()
                
                writer.write(b"RD DM0\r\n")
                await writer.drain()
                
                response = await asyncio.wait_for(reader.read(1024), timeout=1)
                response_time = (time.time() - start_time) * 1000
                response_times.append(response_time)
                
                # 小延遲避免過載
                await asyncio.sleep(0.01)
            
            writer.close()
            await writer.wait_closed()
            
            return {
                "iterations": test_iterations,
                "avg_response_time_ms": sum(response_times) / len(response_times),
                "min_response_time_ms": min(response_times),
                "max_response_time_ms": max(response_times),
                "success_rate": 100.0
            }
            
        except Exception as e:
            return {
                "iterations": 0,
                "error": str(e),
                "success_rate": 0.0
            }

# 使用範例
async def main():
    diagnostic = KeyencePlcDiagnostic("192.168.2.101", 8501)
    results = await diagnostic.run_full_diagnostic()
    
    print("🔍 Keyence PLC 診斷結果:")
    print(f"網路測試: {'✅' if results['network_test']['success'] else '❌'}")
    print(f"協定測試: {len([r for r in results['protocol_test'] if r['success']])}/{len(results['protocol_test'])} 成功")
    
    if results['performance_test'].get('avg_response_time_ms'):
        print(f"平均回應時間: {results['performance_test']['avg_response_time_ms']:.2f}ms")

if __name__ == "__main__":
    asyncio.run(main())
```

## 相關文檔

- [PLC 整合](../../technical-details/plc-integration.md) - PLC 系統整合
- [系統架構](../../system-architecture/dual-environment.md) - 雙環境架構
- [故障排除](../../operations/troubleshooting.md) - 問題診斷和解決
- [效能調優](../../operations/performance-tuning.md) - 系統效能最佳化
- [監控設定](../../technical-details/monitoring-setup.md) - 系統監控配置