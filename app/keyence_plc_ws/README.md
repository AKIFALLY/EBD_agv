# Keyence PLC 工作空間 (keyence_plc_ws)

## 📋 基本資訊

**啟動狀態**: ✅ 自動載入 (容器啟動腳本中自動載入但不執行特定節點)
**運行環境**: 🚗🖥️ 共用 (AGV 車載系統 + AGVC 管理系統)
**主要功能**: Keyence PLC 專用通訊和記憶體管理
**依賴狀態**: 純系統套件，被 `plc_proxy_ws` 和 `agv_ws` 依賴

## 📋 專案概述

Keyence PLC 工作空間提供與 Keyence PLC 設備的專用通訊功能，實現高效能的資料交換和控制操作。該工作空間針對 Keyence PLC 協定進行最佳化，提供記憶體管理、非同步通訊和錯誤處理功能。

此工作空間是 RosAGV 系統中 PLC 通訊的核心基礎設施，採用純 Python 標準庫實作，無需額外的虛擬環境套件。它提供了完整的 Keyence PLC 通訊協定支援，包括連線池管理、記憶體映射、位元組操作和錯誤恢復機制。

## 🔗 依賴關係

### 系統套件依賴
- **Python 標準庫**: `struct`, `threading`, `time`, `socket`
- **ROS 2**: `rclpy.logging` (用於日誌記錄)

### 被依賴的工作空間
- **plc_proxy_ws**: PLC 代理服務 - 使用 `KeyencePlcPool`、`KeyencePlcCommand`、`PlcMemory`、`PlcBytes`
- **agv_ws**: AGV 核心系統 - 使用 `PlcMemory` 進行記憶體管理

### 外部依賴
- **Keyence PLC 設備**: 透過 TCP/IP (預設 port 8501) 進行通訊

## 🏗️ 專案結構

```
keyence_plc_ws/
├── src/keyence_plc/keyence_plc/   # 核心 Keyence PLC 套件 (完整實作)
│   ├── keyence_plc_memory.py      # PLC 記憶體管理類別 (PlcMemory)
│   ├── keyence_plc_com.py         # 基礎 PLC 通訊類別 (KeyencePlcCom)
│   ├── keyence_plc_pool.py        # PLC 連線池管理 (KeyencePlcPool)
│   ├── keyence_plc_command.py     # PLC 指令封裝 (KeyencePlcCommand)
│   ├── keyence_plc_bytes.py       # 位元組資料處理 (PlcBytes)
│   └── __init__.py                # 套件初始化
├── keyence_plc_com_async.py       # 非同步通訊實作 (ThreadPoolExecutor)
├── keyence_plc_com_async_asyncio.py # 非同步通訊實作 (AsyncIO)
├── keyence_plc_com_patch.py       # 通訊修補版本 (實驗性)
├── keyence_plc_com_patch_asyncio.py # AsyncIO 修補版本 (實驗性)
├── test/                          # 測試和範例檔案
│   ├── keyence_plc_com_test.py    # 基礎通訊測試
│   ├── plc_memory_test.py         # 記憶體管理測試
│   └── read_write_test.py         # 讀寫功能測試
├── package.xml                    # ROS 2 套件配置
└── setup.py                       # Python 套件設定
```

## ⚙️ 主要功能

### 1. PLC 記憶體管理 (PlcMemory)
- **記憶體映射**: 高效的記憶體位址映射 (預設 131072 bytes = 65536 words)
- **資料快取**: 本地記憶體快取機制
- **位元組操作**: 低階位元組級別操作 (PlcBytes)
- **資料型別轉換**: 自動資料型別轉換 (int, float, string, bool)
- **位址轉換**: PLC 位址到記憶體索引的自動轉換

### 2. PLC 通訊管理 (KeyencePlcCom)
- **TCP/IP 通訊**: 基於 socket 的 TCP/IP 連線 (預設 port 8501)
- **指令封裝**: 完整的 Keyence PLC 指令集支援
- **錯誤處理**: 完整的通訊錯誤處理和重連機制
- **超時控制**: 可配置的連線和讀取超時設定

### 3. 連線池管理 (KeyencePlcPool)
- **連線池**: 基於 Semaphore 的連線池管理 (預設最大 5 個連線)
- **自動重連**: 失效連線的自動重連機制
- **並發控制**: 支援多執行緒並發存取
- **資源管理**: 自動連線資源釋放和清理

### 4. 指令封裝 (KeyencePlcCommand)
- **標準指令**: 支援 RD/WR (單一讀寫)、RDS/WRS (連續讀寫)
- **強制控制**: 支援 ST/RS (強制開啟/關閉) MR 位元
- **模式查詢**: 支援 ?K (機型查詢)、?M (運行模式查詢)
- **協定相容**: 完全符合 Keyence PLC 通訊協定規範

### 5. 位元組處理 (PlcBytes)
- **資料轉換**: 支援 int、float、string、bool 與 bytes 的雙向轉換
- **位元操作**: 布林陣列與位元組的轉換
- **小端序處理**: 符合 PLC 記憶體格式的小端序處理

## 🔧 核心 API

### PlcMemory 類別
```python
from keyence_plc.keyence_plc_memory import PlcMemory

# 初始化 PLC 記憶體 (預設 131072 bytes = 65536 words)
memory = PlcMemory(131072)

# 基本資料操作
memory.set_int(7600, 1234, length=2)      # 設定 16-bit 整數
value = memory.get_int(7600, length=2)    # 讀取 16-bit 整數

memory.set_float(7610, 3.14)             # 設定 32-bit 浮點數
battery = memory.get_float(7610, 4)      # 讀取 32-bit 浮點數

memory.set_string(7620, "AGV001")        # 設定字串
agv_id = memory.get_string(7620, 20)     # 讀取字串

# 位元操作
memory.set_bit(7636, 9, True)            # 設定 DM7636.9 為 True
low_battery = memory.get_bit(7636, 9)    # 讀取 DM7636.9

# 通用方法
memory.set_value(7600, 1234, format="int", length=2)
value = memory.get_value("7600", format="int", length=2)
```

### PlcBytes 類別
```python
from keyence_plc.keyence_plc_bytes import PlcBytes

# 資料型別轉換
data = PlcBytes.from_int(1234, length=2)  # 整數轉 bytes
value = data.to_int()                     # bytes 轉整數

data = PlcBytes.from_float(3.14)          # 浮點數轉 bytes
value = data.to_float()                   # bytes 轉浮點數

# 位元操作
bools = [True, False, True, False]
data = PlcBytes.from_bools(bools)         # 布林陣列轉 bytes
result = data.to_bools()                  # bytes 轉布林陣列
```

## 🚀 使用方法

### 1. 建置工作空間
```bash
# 載入 ROS 2 環境並建置
source /opt/ros/jazzy/setup.bash && source /opt/ws_rmw_zenoh/install/setup.bash && cd /app/keyence_plc_ws && colcon build
source install/setup.bash
```

### 2. 基本通訊使用
```python
from keyence_plc.keyence_plc_com import KeyencePlcCom
from keyence_plc.keyence_plc_command import KeyencePlcCommand

# 初始化通訊
plc_com = KeyencePlcCom("192.168.12.224", 8501)

# 連線到 PLC
plc_com.connect()

# 讀取單一 DM 資料
command = KeyencePlcCommand.read_data("DM", "7600")
response = plc_com.send_command(command)
print(f"DM7600 值: {response}")

# 寫入單一 DM 資料
command = KeyencePlcCommand.write_data("DM", "1000", "123")
plc_com.send_command(command)

# 連續讀取多個 DM
command = KeyencePlcCommand.read_continuous_data("DM", "7600", "10")
response = plc_com.send_command(command)
print(f"DM7600-7609 值: {response}")

# 斷線
plc_com.disconnect()
```

### 3. 記憶體管理使用
```python
from keyence_plc.keyence_plc_memory import PlcMemory
from keyence_plc.keyence_plc_bytes import PlcBytes

# 初始化記憶體 (65536 words = 131072 bytes)
memory = PlcMemory(131072)

# 寫入整數資料
memory.set_int(7600, 1234, length=2)  # 寫入 16-bit 整數到 DM7600

# 讀取整數資料
value = memory.get_int(7600, length=2)
print(f"DM7600 值: {value}")

# 寫入字串資料
memory.set_string(7610, "AGV001", length=20)

# 讀取字串資料
agv_id = memory.get_string(7610, length=20)
print(f"AGV ID: {agv_id}")
```

### 4. 連線池使用
```python
from keyence_plc.keyence_plc_pool import KeyencePlcPool
from keyence_plc.keyence_plc_command import KeyencePlcCommand

# 初始化連線池 (最大 5 個連線)
pool = KeyencePlcPool("192.168.12.224", 8501, max_connections=5)

# 執行指令 (自動管理連線)
command = KeyencePlcCommand.read_data("DM", "7600")
try:
    response = pool.execute(command)
    print(f"讀取成功: {response}")
except Exception as e:
    print(f"讀取失敗: {e}")

# 關閉連線池
pool.close_connection()
```

# 執行非同步操作
asyncio.run(async_operations())
```

## ⚙️ 配置說明

### PLC 連線設定
```python
# 基本連線設定
PLC_IP = "192.168.1.100"      # PLC IP 位址
PLC_PORT = 8501               # Keyence PLC 預設端口
TIMEOUT = 5.0                 # 連線超時 (秒)

# 記憶體配置
MEMORY_SIZE = 131072          # 記憶體大小 (bytes) = 65536 words
DM_START = 0                  # DM 區域起始位址
DM_SIZE = 32768               # DM 區域大小 (words)
MR_START = 0                  # MR 區域起始位址
MR_SIZE = 8192                # MR 區域大小 (bits)
```

### 連線池參數
```python
# 連線池設定
MIN_POOL_SIZE = 1             # 最小連線數
MAX_POOL_SIZE = 5             # 最大連線數
RECONNECT_INTERVAL = 5        # 重連間隔 (秒)

# 效能參數
MAX_CONCURRENT = 10           # 最大並發請求數
RETRY_COUNT = 3               # 重試次數
RETRY_DELAY = 0.1             # 重試延遲 (秒)
BUFFER_SIZE = 4096            # 緩衝區大小
```

### 常用記憶體位址
```python
# AGV 狀態資料 (範例)
AGV_ID_ADDR = 7600            # AGV ID (20 bytes 字串)
BATTERY_ADDR = 7610           # 電池電量 (4 bytes float)
VELOCITY_X_ADDR = 7612        # X 軸速度 (4 bytes int)
VELOCITY_Y_ADDR = 7614        # Y 軸速度 (4 bytes int)
VELOCITY_A_ADDR = 7616        # 角速度 (4 bytes int)
STATUS_BITS_ADDR = 7636       # 狀態位元 (DM7636.0-15)
```

## 🧪 測試方法

### 1. 建置和測試
```bash
# 載入 ROS 2 環境並建置
source /opt/ros/jazzy/setup.bash && source /opt/ws_rmw_zenoh/install/setup.bash && cd /app/keyence_plc_ws && colcon build
source install/setup.bash

# 執行測試
colcon test
colcon test-result --verbose
```

### 2. 系統套件測試
```bash
# 測試 Python 標準庫
python3 -c "
import struct, threading, time, socket
print('✅ Python 標準庫可用')
"

# 測試 ROS 2 日誌功能
python3 -c "
import rclpy.logging
logger = rclpy.logging.get_logger('test')
print('✅ ROS 2 日誌功能可用')
"
```

### 3. 模組功能測試
```bash
# 測試核心模組載入
python3 -c "
from keyence_plc.keyence_plc_memory import PlcMemory
from keyence_plc.keyence_plc_com import KeyencePlcCom
from keyence_plc.keyence_plc_pool import KeyencePlcPool
from keyence_plc.keyence_plc_command import KeyencePlcCommand
from keyence_plc.keyence_plc_bytes import PlcBytes
print('✅ 所有核心模組載入成功')
"

# 測試記憶體管理
python3 -c "
from keyence_plc.keyence_plc_memory import PlcMemory
memory = PlcMemory(1024)
memory.set_int(100, 1234, length=2)
value = memory.get_int(100, length=2)
assert value == 1234
print('✅ 記憶體管理測試通過')
"

# 測試位元組處理
python3 -c "
from keyence_plc.keyence_plc_bytes import PlcBytes
data = PlcBytes.from_int(1234, 2)
value = data.to_int()
assert value == 1234
print('✅ 位元組處理測試通過')
"
```

### 4. 通訊功能測試 (需要實際 PLC)
```bash
# 基礎通訊測試 (需要修改 IP 位址)
cd /app/keyence_plc_ws/test
python3 keyence_plc_com_test.py

# 記憶體管理測試
python3 plc_memory_test.py

# 讀寫功能測試
python3 read_write_test.py

# 連線池測試
python3 keyence_plc_com_pool.py
```

### 5. 連線池測試
```bash
# 測試連線池功能 (無需實際 PLC)
python3 -c "
from keyence_plc.keyence_plc_pool import KeyencePlcPool
pool = KeyencePlcPool('192.168.1.100', 8501, max_connections=3)
print('✅ 連線池初始化成功')
pool.close_connection()
print('✅ 連線池關閉成功')
"
```

### 6. 效能測試
```bash
# 測試記憶體操作效能
python3 -c "
import time
from keyence_plc.keyence_plc_memory import PlcMemory

memory = PlcMemory(65536 * 2)
start_time = time.time()

# 執行 1000 次寫入操作
for i in range(1000):
    memory.set_int(i, i * 2, length=2)

end_time = time.time()
print(f'✅ 1000 次記憶體寫入耗時: {end_time - start_time:.3f} 秒')
"
```

### 7. 手動驗證 (需要實際 PLC)
```python
# 測試 PLC 連線
from keyence_plc.keyence_plc_com import KeyencePlcCom

plc = KeyencePlcCom("192.168.12.224", 8501)
if plc.connect():
    response = plc.send_command("?K\r\n")  # 查詢機型
    print(f"PLC 機型: {response}")
    plc.disconnect()
```

## 🔧 故障排除

### 1. 模組載入問題
**症狀**: `ModuleNotFoundError: No module named 'keyence_plc'`
**解決方法**:
```bash
# 檢查工作空間是否正確建置
cd /app/keyence_plc_ws
colcon build

# 確認環境已載入
source install/setup.bash

# 檢查 Python 路徑
python3 -c "import sys; print('\\n'.join(sys.path))"
```

### 2. PLC 連線失敗
**症狀**: `ConnectionError` 或連線超時
**解決方法**:
```bash
# 檢查網路連線
ping 192.168.12.224

# 檢查 PLC 埠是否開啟
telnet 192.168.12.224 8501

# 檢查防火牆設定
sudo ufw status

# 測試基本連線
python3 -c "
import socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.settimeout(5)
try:
    sock.connect(('192.168.12.224', 8501))
    print('✅ PLC 連線成功')
except Exception as e:
    print(f'❌ PLC 連線失敗: {e}')
finally:
    sock.close()
"
```

### 3. 記憶體操作錯誤
**症狀**: 記憶體讀寫異常或資料錯誤
**解決方法**:
```bash
# 檢查記憶體大小設定
python3 -c "
from keyence_plc.keyence_plc_memory import PlcMemory
memory = PlcMemory(1024)
print(f'記憶體大小: {len(memory.memory)} bytes')
"

# 檢查位址範圍
python3 -c "
from keyence_plc.keyence_plc_memory import PlcMemory
memory = PlcMemory(1024)
try:
    memory.set_int(1000, 123, length=2)  # 超出範圍
except IndexError as e:
    print(f'✅ 位址範圍檢查正常: {e}')
"
```

### 4. 連線池資源耗盡
**症狀**: 連線池無法取得新連線
**解決方法**:
```bash
# 檢查連線池狀態
python3 -c "
from keyence_plc.keyence_plc_pool import KeyencePlcPool
pool = KeyencePlcPool('192.168.12.224', 8501, max_connections=2)
print(f'可用連線數: {len(pool.connections)}')
print(f'重連池大小: {len(pool.lost_connections)}')
pool.close_connection()
"

# 強制關閉所有連線
python3 -c "
from keyence_plc.keyence_plc_pool import KeyencePlcPool
pool = KeyencePlcPool('192.168.12.224', 8501)
pool.close_connection()
print('✅ 所有連線已關閉')
"
```

## ⚙️ 配置說明

### 連線參數
```python
# 預設連線配置
DEFAULT_IP = "192.168.12.224"     # PLC IP 位址
DEFAULT_PORT = 8501               # PLC 通訊埠
CONNECT_TIMEOUT = 5               # 連線超時 (秒)
MAX_POOL_SIZE = 5                 # 連線池最大連線數
```

### 記憶體配置
```python
# 記憶體配置參數
DEFAULT_MEMORY_SIZE = 131072      # 預設記憶體大小 (bytes)
WORD_SIZE = 2                     # PLC Word 大小 (bytes)
MAX_MEMORY_SIZE = 65536 * 2       # 最大記憶體大小 (65536 words)
```

### 通訊協定
```python
# Keyence PLC 協定參數
PLC_END_MARKER = "\r\n"           # 指令結束標記
BUFFER_SIZE = 1024                # 接收緩衝區大小
MAX_EMPTY_COUNT = 10              # 最大空資料計數
```

### 常用記憶體位址
```python
# AGV 狀態資料 (範例)
AGV_ID_ADDR = 7600            # AGV ID (20 bytes 字串)
BATTERY_ADDR = 7610           # 電池電量 (4 bytes float)
VELOCITY_X_ADDR = 7612        # X 軸速度 (4 bytes int)
VELOCITY_Y_ADDR = 7614        # Y 軸速度 (4 bytes int)
VELOCITY_A_ADDR = 7616        # 角速度 (4 bytes int)
STATUS_BITS_ADDR = 7636       # 狀態位元 (DM7636.0-15)
```

## 🔗 相關文檔

- **plc_proxy_ws**: PLC 代理服務，使用本工作空間的 `KeyencePlcPool`、`KeyencePlcCommand`、`PlcMemory`、`PlcBytes`
- **agv_ws**: AGV 核心系統，使用本工作空間的 `PlcMemory` 進行記憶體管理
- **Keyence PLC 官方文檔**: 參考 Keyence PLC 通訊手冊和協定規範
- **ROS 2 Jazzy 文檔**: [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)

## 📋 ToDo 清單

### 🔴 高優先級 (緊急)
- [ ] 完善錯誤處理機制，提高系統穩定性
- [ ] 新增連線狀態監控和自動恢復功能
- [ ] 最佳化連線池效能，減少連線建立時間

### 🟡 中優先級 (重要)
- [ ] 新增更多 PLC 指令支援 (如批次操作)
- [ ] 實作非同步通訊版本，提高並發效能
- [ ] 新增詳細的效能監控和統計功能
- [ ] 完善單元測試覆蓋率

### 🟢 低優先級 (改善)
- [ ] 新增 GUI 工具進行 PLC 資料監控
- [ ] 支援更多 PLC 品牌和協定
- [ ] 新增資料加密和安全傳輸功能
- [ ] 最佳化記憶體使用效率

### � 技術債務
- [ ] 重構部分舊程式碼，提高可維護性
- [ ] 統一錯誤訊息格式和多語言支援
- [ ] 改善程式碼文檔和註解完整性

### 📊 完成度追蹤
- ✅ 基礎 PLC 通訊功能 (100%)
- ✅ 記憶體管理系統 (100%)
- ✅ 連線池管理 (100%)
- ✅ 指令封裝系統 (100%)
- ✅ 位元組處理工具 (100%)
- ⚠️ 非同步通訊支援 (70% - 實驗性功能)
- ⚠️ 錯誤恢復機制 (80% - 需要改善)
- ❌ 效能監控工具 (0% - 未開始)

### 🎯 里程碑
- **v1.0.0**: ✅ 基礎功能完成 (當前版本)
- **v1.1.0**: 🚧 非同步通訊穩定版本
- **v2.0.0**: 📋 多協定支援和效能最佳化

### 🏆 重要成就
- ✅ 成功整合到 RosAGV 系統
- ✅ 穩定支援 Keyence PLC 通訊
- ✅ 提供完整的記憶體管理功能
- ✅ 實現高效的連線池管理
