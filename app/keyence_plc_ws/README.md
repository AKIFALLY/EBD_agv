# Keyence PLC 工作空間 (keyence_plc_ws)

## 📋 基本資訊

**啟動狀態**: ✅ 實際啟動 (容器啟動時自動載入)
**運行環境**: 🚗🖥️ 共用 (AGV 車載系統 + AGVC 管理系統)
**主要功能**: Keyence PLC 專用通訊和記憶體管理
**依賴狀態**: 被 `ecs_ws` 和 `plc_proxy_ws` 依賴

## 📋 專案概述

Keyence PLC 工作空間提供與 Keyence PLC 設備的專用通訊功能，實現高效能的資料交換和控制操作。該工作空間針對 Keyence PLC 協定進行最佳化，提供記憶體管理、非同步通訊和錯誤處理功能。此工作空間是 RosAGV 系統中 PLC 通訊的核心基礎設施。

## 🔗 依賴關係

### 被依賴的工作空間
- **ecs_ws**: 設備控制系統 - 使用 `PlcMemory` 和 `KeyencePlcCom`
- **plc_proxy_ws**: PLC 代理服務 - 可能使用 Keyence 特定功能

### 外部依賴
- **Python 標準庫**: `struct`, `threading`, `time`, `socket`
- **ROS 2**: `rclpy.logging` (用於日誌記錄)

## 🏗️ 專案結構

```
keyence_plc_ws/
├── src/keyence_plc/keyence_plc/   # 核心 Keyence PLC 套件
│   ├── keyence_plc_memory.py      # PLC 記憶體管理類別
│   ├── keyence_plc_com.py         # 基礎 PLC 通訊類別
│   ├── keyence_plc_pool.py        # PLC 連線池管理
│   ├── keyence_plc_command.py     # PLC 指令封裝
│   ├── keyence_plc_bytes.py       # 位元組資料處理
│   └── __init__.py
├── keyence_plc_com_async.py       # 非同步通訊實作 (ThreadPoolExecutor)
├── keyence_plc_com_async_asyncio.py # 非同步通訊實作 (AsyncIO)
├── keyence_plc_com_patch.py       # 通訊修補版本
├── keyence_plc_com_patch_asyncio.py # AsyncIO 修補版本
├── test/                          # 測試和範例檔案
│   ├── keyence_plc_com_test.py    # 基礎通訊測試
│   ├── plc_memory_test.py         # 記憶體管理測試
│   └── read_write_test.py         # 讀寫功能測試
├── package.xml                    # ROS 2 套件配置
└── setup.py                       # Python 套件設定
```

## ⚙️ 主要功能

### 1. PLC 記憶體管理
- **記憶體映射**: 高效的記憶體位址映射
- **資料快取**: 本地記憶體快取機制
- **位元組操作**: 低階位元組級別操作
- **資料型別轉換**: 自動資料型別轉換

### 2. 非同步通訊
- **高效能通訊**: 非阻塞式 PLC 通訊
- **並發處理**: 支援多個同時請求
- **回調機制**: 靈活的回調處理
- **錯誤恢復**: 自動錯誤恢復機制

### 3. Keyence 協定支援
- **原生協定**: 支援 Keyence 原生通訊協定
- **最佳化效能**: 針對 Keyence PLC 最佳化
- **完整功能**: 支援所有 Keyence PLC 功能
- **版本相容**: 支援多個 Keyence PLC 版本

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
cd /app/keyence_plc_ws
colcon build
source install/setup.bash
```

### 2. 基本使用
```python
from keyence_plc.keyence_plc_memory import PlcMemory
from keyence_plc.keyence_plc_com import KeyencePlcCom

# 初始化記憶體
memory = PlcMemory(65536 * 2)

# 初始化通訊
plc_com = KeyencePlcCom("192.168.1.100", 8501)

# 讀取資料
data = plc_com.read_dm(1000, 10)

# 寫入資料
plc_com.write_dm(1000, [1, 2, 3, 4, 5])
```

### 3. 非同步操作
```python
import asyncio

async def async_operations():
    # 非同步讀取
    data = await plc_com.async_read_dm(1000, 10)
    
    # 非同步寫入
    await plc_com.async_write_dm(1000, [1, 2, 3])

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
# 建置工作空間
cd /app/keyence_plc_ws
colcon build

# 執行測試
colcon test
colcon test-result --verbose
```

### 2. 功能測試
```bash
# 基礎通訊測試
cd /app/keyence_plc_ws/test
python3 keyence_plc_com_test.py

# 記憶體管理測試
python3 plc_memory_test.py

# 讀寫功能測試
python3 read_write_test.py

# 連線池測試
python3 keyence_plc_com_pool.py
```

### 3. 手動驗證
```python
# 測試 PLC 連線
from keyence_plc.keyence_plc_com import KeyencePlcCom

plc = KeyencePlcCom("192.168.1.100", 8501)
if plc.connect():
    response = plc.send_command("?K\r\n")  # 查詢機型
    print(f"PLC 機型: {response}")
    plc.disconnect()
```

## 🔧 故障排除

### 常見問題

#### 1. 連線失敗
**症狀**: `ConnectionError: 無法連接到 PLC`
**解決方法**:
```bash
# 檢查網路連線
ping 192.168.1.100

# 檢查端口是否開放
telnet 192.168.1.100 8501

# 檢查 PLC 設定
# 確認 PLC 的 Ethernet 設定正確
```

#### 2. 記憶體存取錯誤
**症狀**: `ValueError: Unsupported length` 或資料格式錯誤
**解決方法**:
```python
# 檢查資料長度
memory = PlcMemory()
# 確保長度參數正確：int(2/4/8 bytes), float(4 bytes)
value = memory.get_int(7600, length=2)  # 正確
# value = memory.get_int(7600, length=3)  # 錯誤
```

#### 3. 連線池耗盡
**症狀**: 長時間等待連線或連線超時
**解決方法**:
```python
# 檢查連線池狀態
from keyence_plc.keyence_plc_pool import KeyencePlcPool

pool = KeyencePlcPool("192.168.1.100", 8501)
# 確保正確釋放連線，避免連線洩漏
```

### 日誌和診斷

#### 啟用詳細日誌
```python
import rclpy.logging

logger = rclpy.logging.get_logger('keyence_plc')
logger.set_level(rclpy.logging.LoggingSeverity.DEBUG)
```

#### 檢查系統狀態
```bash
# 檢查 PLC 連線狀態
netstat -an | grep 8501

# 檢查記憶體使用
ps aux | grep python

# 檢查錯誤日誌
tail -f /tmp/keyence_plc.log
```

## 🔗 相關文檔

- **plc_proxy_ws**: PLC 代理服務，使用本工作空間的功能
- **ecs_ws**: 設備控制系統，依賴本工作空間
- **Keyence PLC 手冊**: 官方通訊協定文檔

## 📋 ToDo 清單

### 🔴 高優先級 (緊急)
- [ ] **完善非同步通訊** (2 週)
  - 修復 AsyncIO 版本問題
  - 最佳化並發處理效能
  - 新增回調機制完善

### 🟡 中優先級 (重要)
- [ ] **新增 ROS 2 整合** (2 週)
  - 建立 ROS 2 套件結構
  - 實現服務介面
  - 新增訊息定義
- [ ] **完善測試覆蓋** (1 週)
  - 新增單元測試
  - 實現整合測試
  - 建立效能測試

### 🟢 低優先級 (改善)
- [ ] **效能最佳化** (2 週)
  - 最佳化記憶體管理
  - 改善通訊效率
  - 新增效能監控
- [ ] **新增監控功能** (1 週)
  - 實現連線狀態監控
  - 新增效能指標收集
  - 建立警報機制

### 📊 完成度追蹤
- **核心功能**: 85% ✅
- **非同步通訊**: 70% 🔄
- **ROS 2 整合**: 30% ⏳
- **測試覆蓋**: 40% 🔄
