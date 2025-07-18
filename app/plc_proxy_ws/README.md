# PLC 代理工作空間 (plc_proxy_ws)

## 📋 基本資訊

**啟動狀態**: ✅ 實際啟動 (容器啟動時自動載入)
**運行環境**: 🚗🖥️ 共用 (AGV 車載系統 + AGVC 管理系統)
**主要功能**: PLC 通訊代理服務和 ROS 2 服務介面
**依賴狀態**: 依賴 `keyence_plc` 套件，被 `ecs_ws` 使用

## 📋 專案概述

PLC 代理工作空間提供與 PLC (可程式邏輯控制器) 通訊的代理服務，實現 ROS 2 系統與工業 PLC 設備之間的橋接功能。該工作空間將 Keyence PLC 的底層通訊功能封裝為標準化的 ROS 2 服務介面，支援單一讀寫、連續讀寫、強制控制等操作，為上層應用提供可靠的 PLC 資料存取服務。

## 🔗 依賴關係

### 依賴的工作空間
- **keyence_plc**: 使用 `KeyencePlcPool`、`KeyencePlcCommand`、`PlcMemory`、`PlcBytes`

### 被依賴的工作空間
- **ecs_ws**: 設備控制系統 - 使用 PLC 代理服務進行設備通訊

### 外部依賴
- **ROS 2**: `rclpy`, `rclpy.qos`, `rclpy.executors`
- **Python 標準庫**: `threading`, `time`, `logging`

## 🏗️ 專案結構

```
plc_proxy_ws/
├── src/
│   ├── plc_proxy/                 # PLC 代理主套件
│   │   ├── plc_proxy/
│   │   │   ├── plc_service.py    # PLC 服務節點 (主要服務實作)
│   │   │   ├── plc_client.py     # PLC 客戶端 (服務調用封裝)
│   │   │   └── __init__.py
│   │   ├── package.xml           # 套件依賴配置
│   │   └── setup.py              # Python 套件設定
│   └── plc_interfaces/            # PLC 服務介面定義
│       ├── srv/                  # ROS 2 服務定義
│       │   ├── ForceOn.srv       # 強制開啟 MR 位元
│       │   ├── ForceOff.srv      # 強制關閉 MR 位元
│       │   ├── ReadData.srv      # 讀取單一資料
│       │   ├── WriteData.srv     # 寫入單一資料
│       │   ├── ReadContinuousData.srv    # 連續讀取多個資料
│       │   ├── WriteContinuousData.srv   # 連續寫入多個資料
│       │   ├── ReadContinuousByte.srv    # 連續讀取位元組
│       │   └── WriteContinuousByte.srv   # 連續寫入位元組
│       ├── CMakeLists.txt        # CMake 建置配置
│       └── package.xml           # 介面套件配置
├── test/                         # 測試檔案
│   └── ros_batched_service_client.py  # 批次服務客戶端測試
└── README.md                     # 本檔案
```

## ⚙️ 主要功能

### 1. PLC 通訊服務
- **多協定支援**: 支援 Modbus、Ethernet/IP、Keyence 等協定
- **即時通訊**: 低延遲的 PLC 資料交換
- **連線管理**: 自動重連和連線狀態監控
- **錯誤處理**: 完整的通訊錯誤處理機制

### 2. 記憶體操作
- **位元操作**: MR (Memory Relay) 位元讀寫
- **資料操作**: DM (Data Memory) 字組讀寫
- **連續操作**: 批次讀寫多個記憶體位址
- **位元組操作**: 低階位元組級別操作

### 3. 服務介面
- **同步服務**: 阻塞式服務調用
- **非同步服務**: 非阻塞式回調機制
- **批次操作**: 一次操作多個記憶體位址
- **狀態查詢**: PLC 連線和運行狀態查詢

## 📡 ROS 2 服務介面

### 基本讀寫服務

#### ReadData.srv
```yaml
# Request
string device_type    # "DM" (Data Memory) 或 "MR" (Memory Relay)
string address        # PLC 位址，如 "1000"

---
# Response
bool success          # 操作是否成功
string value          # 讀取的值 (字串格式)
string message        # 錯誤訊息或狀態說明
```

#### WriteData.srv
```yaml
# Request
string device_type    # "DM" 或 "MR"
string address        # PLC 位址
string value          # 要寫入的值 (字串格式)

---
# Response
bool success          # 操作是否成功
string message        # 錯誤訊息或狀態說明
```

### 強制控制服務

#### ForceOn.srv
```yaml
# Request
string device_type    # 通常為 "MR"
string address        # MR 位址，如 "100"

---
# Response
bool success          # 操作是否成功
string message        # 錯誤訊息或狀態說明
```

#### ForceOff.srv
```yaml
# Request
string device_type    # 通常為 "MR"
string address        # MR 位址，如 "100"

---
# Response
bool success          # 操作是否成功
string message        # 錯誤訊息或狀態說明
```

### 連續操作服務

#### ReadContinuousData.srv
```yaml
# Request
string device_type    # "DM" 或 "MR"
string start_address  # 起始位址，如 "1000"
int32 count          # 讀取數量

---
# Response
bool success          # 操作是否成功
string[] values       # 讀取的值陣列 (字串格式)
string message        # 錯誤訊息或狀態說明
```

#### WriteContinuousData.srv
```yaml
# Request
string device_type    # "DM" 或 "MR"
string start_address  # 起始位址
string[] values       # 要寫入的值陣列

---
# Response
bool success          # 操作是否成功
string message        # 錯誤訊息或狀態說明
```

### 位元組操作服務

#### ReadContinuousByte.srv
```yaml
# Request
string device_type    # 設備類型
string start_address  # 起始位址
int32 count          # 讀取位元組數量

---
# Response
bool success          # 操作是否成功
uint8[] values        # 讀取的位元組陣列
string message        # 錯誤訊息或狀態說明
```

#### WriteContinuousByte.srv
```yaml
# Request
string device_type    # 設備類型
string start_address  # 起始位址
uint8[] values        # 要寫入的位元組陣列

---
# Response
bool success          # 操作是否成功
string message        # 錯誤訊息或狀態說明
```

## 🚀 使用方法

### 1. 建置工作空間
```bash
# 載入 ROS 2 環境並建置
source /opt/ros/jazzy/setup.bash && source /opt/ws_rmw_zenoh/install/setup.bash && cd /app/plc_proxy_ws && colcon build
source install/setup.bash
```

### 2. 啟動 PLC 服務
```bash
# 使用預設配置啟動 (預設 IP: 192.168.12.224)
ros2 run plc_proxy plc_service

# 指定 PLC IP 位址
ros2 run plc_proxy plc_service --ros-args -p plc_ip:="192.168.1.100"

# 指定讀取範圍參數
ros2 run plc_proxy plc_service --ros-args \
  -p plc_ip:="192.168.1.100" \
  -p read_ranges:="['DM,7600,200','DM,5000,200']"

# 在命名空間中啟動
ros2 run plc_proxy plc_service --ros-args -r __ns:=/agvc
```

### 3. 服務參數配置
```yaml
# 主要參數
plc_ip: "192.168.12.224"          # PLC IP 位址
read_ranges:                      # 自動讀取範圍
  - "DM,7600,200"                # DM7600 開始讀取 200 個 word
  - "DM,5000,200"                # DM5000 開始讀取 200 個 word
```

## 🔧 核心 API

### PlcService 節點
```python
# 節點名稱: plc_service
# 命名空間: 可配置 (預設為根命名空間)

# 提供的服務:
# - /plc_service/force_on
# - /plc_service/force_off
# - /plc_service/read_data
# - /plc_service/write_data
# - /plc_service/read_continuous_data
# - /plc_service/write_continuous_data
# - /plc_service/read_continuous_byte
# - /plc_service/write_continuous_byte
```

### 使用 PLC 客戶端
```python
import rclpy
from rclpy.node import Node
from plc_interfaces.srv import ReadData, WriteData, ForceOn

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')

        # 建立服務客戶端
        self.read_client = self.create_client(ReadData, '/plc_service/read_data')
        self.write_client = self.create_client(WriteData, '/plc_service/write_data')
        self.force_on_client = self.create_client(ForceOn, '/plc_service/force_on')

    def read_dm_data(self, address):
        """讀取 DM 資料"""
        request = ReadData.Request()
        request.device_type = "DM"
        request.address = str(address)

        future = self.read_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            return future.result().value
        else:
            self.get_logger().error(f"讀取失敗: {future.result().message}")
            return None

    def write_dm_data(self, address, value):
        """寫入 DM 資料"""
        request = WriteData.Request()
        request.device_type = "DM"
        request.address = str(address)
        request.value = str(value)

        future = self.write_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        return future.result().success
```

### 4. 非同步操作
```python
def read_callback(response):
    if response.success:
        print(f"讀取成功: {response.values}")

# 非同步連續讀取
self.plc_client.async_read_continuous_data(
    "DM", "1000", 10, read_callback
)
```

## 🔧 配置說明

### PLC 連線配置
```yaml
plc_service:
  ros__parameters:
    # PLC 連線設定
    plc_ip: "192.168.1.100"
    plc_port: 502
    connection_timeout: 5.0
    read_timeout: 1.0
    
    # 通訊協定
    protocol: "modbus_tcp"  # modbus_tcp, keyence, ethernet_ip
    
    # 記憶體映射
    dm_start_address: 0
    dm_size: 65536
    mr_start_address: 0
    mr_size: 8192
```

### 效能參數
```yaml
# 通訊參數
max_concurrent_requests: 10
request_queue_size: 100
retry_count: 3
retry_delay: 0.1

# 快取設定
enable_cache: true
cache_timeout: 0.1
cache_size: 1000
```

## 🧪 測試方法

### 1. 建置和測試
```bash
# 建置工作空間
source /opt/ros/jazzy/setup.bash && source /opt/ws_rmw_zenoh/install/setup.bash && cd /app/plc_proxy_ws && colcon build

# 執行測試
source install/setup.bash && colcon test
colcon test-result --verbose
```

### 2. 服務功能測試
```bash
# 測試讀取 DM 資料
ros2 service call /plc_service/read_data plc_interfaces/srv/ReadData \
  "{device_type: 'DM', address: '7600'}"

# 測試寫入 DM 資料
ros2 service call /plc_service/write_data plc_interfaces/srv/WriteData \
  "{device_type: 'DM', address: '1000', value: '123'}"

# 測試強制開啟 MR
ros2 service call /plc_service/force_on plc_interfaces/srv/ForceOn \
  "{device_type: 'MR', address: '100'}"

# 測試連續讀取
ros2 service call /plc_service/read_continuous_data plc_interfaces/srv/ReadContinuousData \
  "{device_type: 'DM', start_address: '7600', count: 10}"
```

### 3. 批次測試
```bash
# 執行批次服務客戶端測試
cd /app/plc_proxy_ws/test
python3 ros_batched_service_client.py
```

### 連線狀態檢查
```bash
# 檢查服務狀態
ros2 service list | grep plc_service

# 檢查節點資訊
ros2 node info /plc_service

# 監控服務調用
ros2 topic echo /rosout | grep plc_service
```

### 效能監控
```bash
# 監控服務回應時間
ros2 topic echo /plc_service/performance

# 檢查錯誤率
ros2 topic echo /plc_service/errors

# 監控連線狀態
ros2 topic echo /plc_service/connection_status
```

## 🔗 依賴項目

- **ROS 2 Jazzy**: 機器人作業系統框架
- **pymodbus**: Modbus 通訊庫
- **socket**: TCP/UDP 通訊
- **threading**: 多執行緒支援
- **rclpy**: ROS 2 Python 客戶端庫
- **ament_python**: Python 套件建置工具

## 📝 開發指南

### 新增 PLC 協定支援
1. 在 `plc_proxy/` 中新增協定實現類
2. 實現標準的讀寫介面
3. 更新服務節點以支援新協定
4. 新增對應的配置參數
5. 測試新協定功能

### 新增服務介面
1. 在 `plc_interfaces/srv/` 中定義新服務
2. 更新 `CMakeLists.txt`
3. 在服務節點中實現新服務
4. 更新客戶端類別
5. 新增測試案例

### 效能最佳化
1. 實施連線池管理
2. 新增資料快取機制
3. 最佳化批次操作
4. 實施非同步處理
5. 監控和調整參數

## 🔧 故障排除

### 常見問題

1. **PLC 連線失敗**
   ```bash
   # 檢查網路連線
   ping 192.168.1.100
   
   # 檢查埠號
   telnet 192.168.1.100 502
   
   # 檢查防火牆
   sudo ufw status
   ```

2. **讀寫超時**
   ```yaml
   # 調整超時設定
   read_timeout: 2.0
   connection_timeout: 10.0
   ```

3. **記憶體位址錯誤**
   ```bash
   # 檢查 PLC 記憶體配置
   # 確認位址範圍正確
   # 檢查資料型別匹配
   ```

4. **高錯誤率**
   ```yaml
   # 增加重試次數
   retry_count: 5
   retry_delay: 0.2
   
   # 降低並發請求
   max_concurrent_requests: 5
   ```

## 🔧 維護注意事項

1. **連線監控**: 定期檢查 PLC 連線狀態和通訊品質
2. **效能調整**: 根據實際負載調整通訊參數
3. **錯誤分析**: 定期分析通訊錯誤和失敗原因
4. **版本相容**: 確保與 PLC 韌體版本相容
5. **安全性**: 實施適當的網路安全措施

## 📋 ToDo 清單

### 🔴 高優先級 (緊急)
- [ ] **修復連線穩定性問題** (1 週)
  - 解決 PLC 連線中斷問題
  - 實現智能重連機制
  - 新增連線品質監控
- [ ] **完善錯誤處理** (1 週)
  - 統一錯誤回應格式
  - 新增詳細錯誤分類
  - 實現錯誤恢復機制
- [ ] **優化通訊效能** (1 週)
  - 減少通訊延遲
  - 實現批次操作最佳化
  - 新增通訊快取機制

### 🟡 中優先級 (重要)
- [ ] **新增協定支援** (3 週)
  - 完善 Keyence 協定支援
  - 新增 Ethernet/IP 協定
  - 實現協定自動檢測
- [ ] **完善測試覆蓋** (2 週)
  - 新增服務介面測試
  - 實現 PLC 模擬器
  - 建立壓力測試
- [ ] **實現監控功能** (2 週)
  - 新增效能指標收集
  - 實現通訊品質監控
  - 建立警報機制
- [ ] **改善配置管理** (1 週)
  - 統一配置文件格式
  - 新增配置驗證
  - 實現動態配置更新

### 🟢 低優先級 (改善)
- [ ] **新增 Web 管理介面** (3 週)
  - 建立 PLC 管理頁面
  - 實現即時監控顯示
  - 新增遠端診斷功能
- [ ] **實現資料分析** (2 週)
  - 新增通訊統計分析
  - 實現效能趨勢分析
  - 建立診斷報表
- [ ] **新增安全功能** (2 週)
  - 實現通訊加密
  - 新增存取控制
  - 建立稽核日誌

### 🔧 技術債務
- [ ] **重構服務架構** (3 週)
  - 統一服務介面設計
  - 改善程式碼模組化
  - 新增詳細註解
- [ ] **更新依賴套件** (1 週)
  - 升級 pymodbus 版本
  - 更新 ROS 2 套件
  - 檢查安全漏洞
- [ ] **改善記憶體管理** (1 週)
  - 最佳化記憶體使用
  - 修復記憶體洩漏
  - 實現資源監控

### 📊 完成度追蹤
- **核心功能**: 95% ✅
- **協定支援**: 80% ✅
- **錯誤處理**: 70% 🔄
- **監控功能**: 40% 🔄
- **測試覆蓋**: 60% 🔄

### 🎯 里程碑
1. **v1.1.0** (3 週後) - 修復穩定性和效能問題
2. **v1.2.0** (6 週後) - 新協定支援和監控功能
3. **v2.0.0** (10 週後) - Web 介面和完整重構
