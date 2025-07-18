# ECS 工作空間 (ecs_ws)

## 📋 基本資訊

**啟動狀態**: ⚠️ 手動啟動 (未在容器啟動腳本中自動啟動)
**運行環境**: 🖥️ AGVC 管理系統 (主要)
**主要功能**: 設備控制系統 (ECS) - 設備信號監控和門控制邏輯
**依賴狀態**: 依賴 plc_proxy_ws 和 db_proxy_ws，~~paho-mqtt 已棄用~~
**手動啟動**: 可使用 `start_ecs` 指令或 launch 檔案啟動
**重要變更**: ❌ MQTT 門控制功能已棄用，改用 Web API 整合

## 📋 專案概述

ECS (Equipment Control System) 工作空間提供設備控制系統功能，主要負責監控和控制工廠設備，包括門控制系統、設備信號管理、以及與 PLC 的即時通訊。該系統整合了 MQTT 通訊協定，支援遠端設備控制和狀態監控。作為 AGVC 管理系統的重要組件，它提供了工廠設備的自動化控制和監控能力。

## 🔗 依賴關係

### 虛擬環境套件依賴
- ~~**paho-mqtt**: MQTT 客戶端庫，用於 MQTT 通訊協定~~ ❌ **已棄用**

### 系統套件依賴
- **plc_proxy**: PLC 通訊代理服務 (PlcClient)
- **keyence_plc**: Keyence PLC 通訊套件 (PlcMemory)
- **db_proxy**: 資料庫代理服務 (ConnectionPoolManager)

### 依賴的工作空間
- **plc_proxy_ws**: 提供 PLC 通訊客戶端 (✅ 已在容器啟動時載入)
- **keyence_plc_ws**: 提供 PLC 記憶體管理 (✅ 已在容器啟動時載入)
- **db_proxy_ws**: 提供資料庫連線池管理 (❌ 需要手動啟動或確保已載入)

### 被依賴的工作空間
- **launch_ws**: 使用 ecs_launch 啟動 ECS 服務
- **web_api_ws**: 使用 door_logic.py 模組進行門控制 (新架構)
- ~~**外部系統**: KUKA ECS 系統透過 MQTT 進行門控制通訊~~ ❌ **已棄用**

### 外部依賴
- **ROS 2**: `rclpy`, `rclpy.node`
- ~~**MQTT Broker**: 192.168.11.206:2883 (用於門控制通訊)~~ ❌ **已棄用**
- **PostgreSQL**: 資料庫系統 (192.168.100.254:5432)

## 🏗️ 專案結構

```
ecs_ws/
├── src/
│   └── ecs/                       # ECS 主要套件
│       ├── ecs/
│       │   ├── __init__.py               # 套件初始化
│       │   ├── ecs_core.py               # ECS 核心節點 (依賴 db_proxy, plc_proxy)
│       │   ├── ~~door_controller_node_mqtt.py~~ # ❌ 已棄用：MQTT 門控制節點
│       │   ├── door_logic.py             # 門控制邏輯 (供 web_api_ws 使用)
│       │   └── ~~door_controller_config.py~~ # ❌ 已棄用：MQTT 門控制配置
│       ├── package.xml                   # 套件描述文件 (依賴 plc_proxy, keyence_plc)
│       └── setup.py                      # Python 套件設定 (entry_points: ecs_core, door_controller_node_mqtt)
├── build/                        # 建置輸出目錄
├── install/                      # 安裝目錄
└── log/                         # 日誌目錄
```

## ⚙️ 主要功能

### 1. ECS 核心系統 (ecs_core.py)
- **設備信號監控**: 即時讀取和監控設備信號狀態
- **PLC 通訊**: 與 PLC 進行高頻率資料交換（100ms 週期）
- **資料庫整合**: 將設備信號資料同步到資料庫
- **記憶體管理**: 管理 PLC 記憶體映射和資料緩存

### 2. ~~MQTT 門控制系統~~ ❌ **已棄用**
~~**door_controller_node_mqtt.py** - 使用虛擬環境 paho-mqtt~~

**棄用原因**: MQTT 門控制已全面改用 Web API 架構
- ~~MQTT 通訊: 透過 paho-mqtt 客戶端接收門控制指令~~
- ~~即時門控制: 支援門的開啟和關閉控制~~
- ~~狀態回饋: 即時回報門的狀態變化到 MQTT 主題~~
- ~~多門管理: 支援多個門的同時控制和監控~~

**新架構**: 門控制功能已移至 **web_api_ws** 專案
- **door_logic.py**: 門控制邏輯模組，供 Web API 使用
- **HTTP API**: KUKA ECS 系統透過 Web API 介面控制門
- **統一管理**: 與其他 AGVC 功能整合在同一 Web 介面

### 3. 門控制邏輯 (door_logic.py)
- **同步/非同步控制**: 提供同步和非同步的門控制介面
- **狀態查詢**: 即時查詢門的開關狀態
- **錯誤處理**: 完整的錯誤處理和異常管理
- **配置管理**: 靈活的門控制配置系統

## 🔧 核心組件

### ECS 核心節點
```python
class EcsCore(Node):
    def __init__(self):
        # 初始化 PLC 客戶端
        self.plc_client = PlcClient(self)
        
        # 記憶體管理
        self.memory = PlcMemory(65536 * 2)
        
        # 資料庫連線
        self.pool_agvc = ConnectionPoolManager(db_url_agvc)
        
        # 定時器 - 100ms 週期
        self.timer = self.create_timer(0.1, self.main_loop_timer)
```

### MQTT 門控制節點
```python
class DoorControllerNodeMQTT(Node):
    def __init__(self):
        # MQTT 客戶端設定
        self.mqtt_client = mqtt.Client()
        
        # 門控制邏輯
        self.door_logic = DoorLogic(plc_client, config)
        
        # 狀態監控定時器
        self.timer = self.create_timer(1.0, self.check_door_status)
```

## 📡 通訊協定

### MQTT 訊息格式

#### 門控制請求 (request/to/agvc/door)
```json
{
    "doorId": "1",
    "isOpen": "true"
}
```

#### 門狀態回應 (response/to/kukaecs/door)
```json
{
    "doorId": "1",
    "state": "OPEN"
}
```

### PLC 通訊範圍
- **DM 7600-7620**: 設備控制信號
- **DM 5000-5200**: 設備狀態資料
- **讀取頻率**: 100ms
- **通訊協定**: Keyence PLC 協定

## � 核心 API

### ECS 核心節點啟動
```python
import rclpy
from ecs.ecs_core import EcsCore

# 初始化 ROS 2
rclpy.init()

# 建立 ECS 核心節點
ecs_node = EcsCore()

# 執行節點
try:
    rclpy.spin(ecs_node)
except KeyboardInterrupt:
    pass
finally:
    ecs_node.destroy_node()
    rclpy.shutdown()
```

### ~~MQTT 門控制節點啟動~~ ❌ **已棄用**
~~**door_controller_node_mqtt.py** 已棄用，改用 Web API 整合~~

**新架構**: 門控制功能已整合到 **web_api_ws** 專案
```python
# web_api_ws/src/web_api/web_api/api_server.py
from ecs.door_logic import DoorLogic
from ecs.door_controller_config import DoorControllerConfig

# 在 Web API 伺服器中使用門控制邏輯
door_config = DoorControllerConfig()
door_config.load_config_yaml("/app/config/door_config.yaml")
door_controller = DoorLogic(plc_client, door_config)

# 透過 FastAPI 路由器提供 HTTP API
app.include_router(create_door_router(door_controller))
```

### 門控制邏輯使用 (供 web_api_ws 使用)
```python
from ecs.door_logic import DoorLogic
from ecs.door_controller_config import DoorControllerConfig
from plc_proxy.plc_client_node import PlcClientNode

# 建立 PLC 客戶端節點
plc_client = PlcClientNode('plc_client', 'agvc')

# 載入門控制配置
door_config = DoorControllerConfig()
door_config.load_config_yaml("/app/config/door_config.yaml")

# 建立門控制邏輯
door_logic = DoorLogic(plc_client, door_config)

# 非同步門控制 (用於 Web API)
result = await door_logic.async_control_door(door_id=1, is_open=True)
print(f"門控制結果: {result}")

# 非同步查詢門狀態 (用於 Web API)
def handle_state(state_info):
    print(f"門狀態: {state_info}")

door_logic.async_state_door(door_id=1, callback=handle_state)

# 同步查詢門狀態
state = door_logic.state_door(door_id=1)
print(f"門狀態: {state}")
```

### Web API 格式 (在 web_api_ws 中實作)
```json
// 門控制請求 (POST /door/control)
{
    "doorId": "1",
    "isOpen": true  // true 開門, false 關門
}

// 門控制回應
{
    "doorId": 1,
    "isOpen": true,
    "success": true,
    "message": "Door operation completed"
}

// 門狀態查詢請求 (POST /door/state)
{
    "doorId": "1"
}

// 門狀態查詢回應
{
    "doorId": 1,
    "state": "OPEN",  // "OPEN" 或 "CLOSE"
    "isOpen": true,
    "success": true
}
```

**Web API 端點** (在 web_api_ws 中提供):
- `POST /door/control` - 門控制指令
- `POST /door/state` - 查詢門狀態
- **基礎 URL**: http://localhost:8000 (web_api_ws 伺服器)

## �🚀 使用方法

### 1. ~~虛擬環境套件檢查~~ ❌ **已棄用**
~~**paho-mqtt** 套件已不再使用，門控制功能已移至 web_api_ws~~

```bash
# ❌ 已棄用：檢查 paho-mqtt 套件
# /opt/pyvenv_env/bin/pip3 list | grep paho-mqtt

# ✅ 新架構：門控制功能在 web_api_ws 中
# 請參考 web_api_ws 的虛擬環境套件 (FastAPI, SQLAlchemy 等)
```

**注意**: ECS 工作空間現在主要提供 door_logic.py 模組供其他工作空間使用，不再直接使用虛擬環境套件。

### 2. 依賴檢查
```bash
# 檢查 PLC 相關依賴
python3 -c "
from plc_proxy.plc_client import PlcClient
from keyence_plc.keyence_plc_memory import PlcMemory
print('✅ PLC 依賴可用')
"

# 檢查資料庫依賴
python3 -c "
from db_proxy.connection_pool_manager import ConnectionPoolManager
print('✅ 資料庫依賴可用')
"
```

### 3. 建置工作空間
```bash
# 載入 ROS 2 環境並建置
source /opt/ros/jazzy/setup.bash && source /opt/ws_rmw_zenoh/install/setup.bash && cd /app/ecs_ws && colcon build
source install/setup.bash
```

### 4. 手動啟動 ECS 服務 (推薦)
```bash
# 方法 1: 使用 setup.bash 中的便利函數
start_ecs

# 方法 2: 使用 launch 文件啟動
ros2 launch ecs_launch launch.py

# 方法 3: 直接啟動核心節點
ros2 run ecs ecs_core --ros-args -p db_url_agvc:="postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
```

### 5. 檢查 ECS 服務狀態
```bash
# 檢查 ECS 進程
ps aux | grep ecs_core

# 檢查 PID 檔案
cat /tmp/ecs.pid

# 檢查日誌
tail -f /tmp/ecs.log
```

### 6. ~~啟動 MQTT 門控制服務~~ ❌ **已棄用**
~~**door_controller_node_mqtt** 已棄用，門控制功能已移至 web_api_ws~~

```bash
# ❌ 已棄用：MQTT 門控制服務
# ros2 run ecs door_controller_node_mqtt

# ✅ 新架構：門控制透過 Web API 提供
# 請啟動 web_api_ws 專案來使用門控制功能
cd /app/web_api_ws
python3 -m web_api.api_server

# 測試門控制 Web API
curl -X POST http://localhost:8000/door/control \
  -H "Content-Type: application/json" \
  -d '{"doorId": "1", "isOpen": true}'
```

**重要**: 門控制功能現在透過 **web_api_ws** 專案的 HTTP API 提供，不再使用 MQTT 通訊。

### 4. 門控制配置
```bash
# 門配置格式: door_id,mr_type,mr_address,dm_type,dm_address
ros2 run ecs door_controller_node_mqtt --ros-args \
  -p doors:="['1,MR,100,DM,5000', '2,MR,101,DM,5001']"
```

## 🔧 配置說明

### ECS 核心配置
```yaml
# PLC 讀取範圍配置
read_ranges:
  - "DM,7600,20"    # 設備控制信號
  - "DM,5000,200"   # 設備狀態資料

# 資料庫連線
db_url_agvc: "postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
```

### MQTT 門控制配置
```yaml
# MQTT 伺服器設定
broker_host: "192.168.11.206"
broker_port: 2883
username: "DsH8vSx2uhTao1hlc9vx"

# MQTT 主題
sub_topic: "request/to/agvc/door"
pub_topic: "response/to/kukaecs/door"

# 門配置
doors:
  - "1,MR,100,DM,5000"  # 門1: MR100控制, DM5000狀態
  - "2,MR,101,DM,5001"  # 門2: MR101控制, DM5001狀態
```

## 📊 監控和除錯

### 系統狀態監控
```bash
# 檢查 ECS 核心狀態
ros2 topic echo /ecs_core/status

# 檢查門控制狀態
ros2 topic echo /door_controller/status

# 檢查 PLC 通訊狀態
ros2 service call /plc_service/status
```

### MQTT 除錯
```bash
# 監聽 MQTT 訊息
mosquitto_sub -h 192.168.11.206 -p 2883 -u DsH8vSx2uhTao1hlc9vx -t "response/to/kukaecs/door"

# 發送測試訊息
mosquitto_pub -h 192.168.11.206 -p 2883 -u DsH8vSx2uhTao1hlc9vx -t "request/to/agvc/door" -m '{"doorId":"1","isOpen":"true"}'
```

### 日誌檢查
```bash
# 檢查 ROS 2 日誌
ros2 topic echo /rosout

# 檢查特定節點日誌
ros2 run rqt_console rqt_console
```

## 🧪 測試方法

### 1. 建置和測試
```bash
# 建置工作空間
source /opt/ros/jazzy/setup.bash && source /opt/ws_rmw_zenoh/install/setup.bash && cd /app/ecs_ws && colcon build

# 執行測試
source install/setup.bash && colcon test
colcon test-result --verbose
```

### 2. ~~虛擬環境套件測試~~ ❌ **已棄用**
~~**paho-mqtt** 測試已棄用，門控制功能已移至 web_api_ws~~

```bash
# ❌ 已棄用：paho-mqtt 連線測試
# 門控制功能現在透過 Web API 提供

# ✅ 新架構：測試門控制邏輯模組
python3 -c "
from ecs.door_logic import DoorLogic
from ecs.door_controller_config import DoorControllerConfig
print('✅ 門控制邏輯模組載入成功')
print('門控制功能現在透過 web_api_ws 的 HTTP API 提供')
"
```

### 3. ECS 服務測試
```bash
# 啟動 ECS 核心節點
start_ecs

# 測試 ROS 2 節點狀態
ros2 node list | grep ecs

# 測試 PLC 通訊
ros2 service call /agvc/plc_service/read_memory plc_proxy_interfaces/srv/ReadMemory "{address: 'DM5000', count: 10}"

# 測試資料庫連線
ros2 service call /agvc/sql_query db_proxy_interfaces/srv/SqlQuery "{query_string: 'SELECT COUNT(*) FROM eqp_signal'}"
```

### 4. ~~MQTT 門控制測試~~ ❌ **已棄用**
~~**MQTT 門控制** 已棄用，改用 Web API 測試~~

```bash
# ❌ 已棄用：MQTT 門控制測試
# ros2 run ecs door_controller_node_mqtt
# mosquitto_pub -h 192.168.11.206 -p 2883 ...

# ✅ 新架構：Web API 門控制測試
# 確保 web_api_ws 服務運行
curl -X GET http://localhost:8000/health

# 測試開門 API
curl -X POST http://localhost:8000/door/control \
  -H "Content-Type: application/json" \
  -d '{"doorId": "1", "isOpen": true}'

# 測試關門 API
curl -X POST http://localhost:8000/door/control \
  -H "Content-Type: application/json" \
  -d '{"doorId": "1", "isOpen": false}'

# 查詢門狀態 API
curl -X POST http://localhost:8000/door/state \
  -H "Content-Type: application/json" \
  -d '{"doorId": "1"}'
```

### 5. 整合測試
```bash
# 完整的 ECS 系統測試
# 1. 確保依賴服務運行
ros2 service list | grep -E "(plc_service|sql_query)"

# 2. 啟動 ECS 服務
start_ecs

# 3. 檢查設備信號讀取
ros2 topic echo /agvc/eqp_signals --once

# 4. 測試門控制功能 (透過 Web API)
# 確保 web_api_ws 服務運行
curl -X GET http://localhost:8000/health
curl -X POST http://localhost:8000/door/control -H "Content-Type: application/json" -d '{"doorId":"1","isOpen":true}'
```

## 🔧 故障排除

### 常見問題

#### 1. ~~虛擬環境套件問題~~ ❌ **已棄用**
~~**paho-mqtt** 相關問題已不適用，門控制功能已移至 web_api_ws~~

**新架構問題**: 如果遇到門控制相關問題，請檢查：
```bash
# 檢查門控制邏輯模組
python3 -c "
from ecs.door_logic import DoorLogic
from ecs.door_controller_config import DoorControllerConfig
print('✅ 門控制邏輯模組可用')
"

# 檢查 web_api_ws 服務狀態
curl -X GET http://localhost:8000/health

# 如果 web_api_ws 未運行，請啟動它
cd /app/web_api_ws
python3 -m web_api.api_server
```

#### 2. 依賴工作空間問題
**症狀**: `ModuleNotFoundError: No module named 'plc_proxy'` 或類似錯誤
**解決方法**:
```bash
# 檢查依賴工作空間是否載入
python3 -c "
try:
    from plc_proxy.plc_client import PlcClient
    print('✅ plc_proxy 可用')
except ImportError:
    print('❌ plc_proxy 不可用，請確保 plc_proxy_ws 已載入')

try:
    from keyence_plc.keyence_plc_memory import PlcMemory
    print('✅ keyence_plc 可用')
except ImportError:
    print('❌ keyence_plc 不可用，請確保 keyence_plc_ws 已載入')

try:
    from db_proxy.connection_pool_manager import ConnectionPoolManager
    print('✅ db_proxy 可用')
except ImportError:
    print('❌ db_proxy 不可用，請確保 db_proxy_ws 已載入')
"

# 手動載入依賴工作空間
cd /app/plc_proxy_ws && source install/setup.bash
cd /app/keyence_plc_ws && source install/setup.bash
cd /app/db_proxy_ws && source install/setup.bash
```

#### 3. ECS 服務啟動失敗
**症狀**: `start_ecs` 指令失敗或 ECS 節點無法啟動
**解決方法**:
```bash
# 檢查 ECS 建置狀態
ls -la /app/ecs_ws/install/

# 重新建置 ECS
cd /app/ecs_ws
rm -rf build install log
colcon build

# 檢查依賴是否可用
python3 -c "
from plc_proxy.plc_client import PlcClient
from keyence_plc.keyence_plc_memory import PlcMemory
print('✅ ECS 依賴檢查通過')
"

# 手動啟動並檢查錯誤
ros2 run ecs ecs_core --ros-args -p db_url_agvc:="postgresql+psycopg2://agvc:password@192.168.100.254/agvc"
```

#### 4. ~~MQTT 連線問題~~ ❌ **已棄用**
~~**MQTT 門控制** 已棄用，改用 Web API 架構~~

**新架構問題**: Web API 門控制連線問題
**解決方法**:
```bash
# 檢查 web_api_ws 服務狀態
curl -X GET http://localhost:8000/health

# 檢查 web_api_ws 服務日誌
cd /app/web_api_ws
python3 -m web_api.api_server --log-level DEBUG

# 測試門控制 API 連線
curl -X POST http://localhost:8000/door/state \
  -H "Content-Type: application/json" \
  -d '{"doorId": "1"}'

# 檢查網路連線 (如果 web_api_ws 在遠端)
ping <web_api_server_ip>
```

#### 5. 資料庫連線問題
**症狀**: ECS 核心節點無法連線到資料庫
**解決方法**:
```bash
# 檢查資料庫連線
pg_isready -h 192.168.100.254 -p 5432

# 測試資料庫認證
PGPASSWORD=password psql -h 192.168.100.254 -p 5432 -U agvc -d agvc -c "SELECT 1;"

# 檢查資料庫相關表格
PGPASSWORD=password psql -h 192.168.100.254 -p 5432 -U agvc -d agvc -c "SELECT table_name FROM information_schema.tables WHERE table_name LIKE '%eqp%';"
```

### 除錯工具
```bash
# 檢查所有 ECS 相關節點
ros2 node list | grep ecs

# 監控 ECS 效能
top | grep ecs

# 檢查 ROS 2 日誌
ros2 topic echo /rosout | grep ecs

# 檢查 ECS 日誌檔案
tail -f /tmp/ecs.log

# 檢查 PLC 通訊狀態
ros2 service call /agvc/plc_service/status plc_proxy_interfaces/srv/GetStatus "{}"

# 檢查門控制 Web API 狀態 (新架構)
curl -X GET http://localhost:8000/health
curl -X GET http://localhost:8000/docs  # FastAPI 文檔
```

## 🧪 測試與驗證

### ~~門控制測試~~ ❌ **已棄用**
~~**MQTT 門控制測試** 已棄用，改用 Web API 測試~~

```bash
# ❌ 已棄用：MQTT 門控制測試
# import paho.mqtt.client as mqtt
# client.connect("192.168.11.206", 2883, 60)

# ✅ 新架構：Web API 門控制測試
# 開門測試
curl -X POST http://localhost:8000/door/control \
  -H "Content-Type: application/json" \
  -d '{"doorId": "1", "isOpen": true}'

# 關門測試
curl -X POST http://localhost:8000/door/control \
  -H "Content-Type: application/json" \
  -d '{"doorId": "1", "isOpen": false}'

# 查詢門狀態
curl -X POST http://localhost:8000/door/state \
  -H "Content-Type: application/json" \
  -d '{"doorId": "1"}'
```

### PLC 通訊測試
```bash
# 測試 PLC 讀取
ros2 service call /plc_service/read_continuous_byte plc_interfaces/srv/ReadContinuousByte "{device_type: 'DM', start_address: '5000', count: 1}"

# 測試 PLC 寫入
ros2 service call /plc_service/force_on plc_interfaces/srv/ForceOn "{device_type: 'MR', address: '100'}"
```

## 📝 開發指南

### 新增設備控制
1. 在資料庫中新增設備信號定義
2. 更新 `read_ranges` 配置包含新的 PLC 位址
3. 在 `ecs_core.py` 中新增對應的處理邏輯
4. 測試設備控制功能

### ~~新增門控制~~ ❌ **已棄用**
~~門控制功能已移至 web_api_ws 專案~~

**新架構**: 門控制開發請參考 **web_api_ws** 專案
1. 在 web_api_ws 中使用 ecs.door_logic 模組
2. 透過 HTTP API 提供門控制介面
3. 在 door_config.yaml 中配置門的 PLC 位址
4. 測試 Web API 門控制功能

### 效能最佳化
1. 調整 PLC 讀取頻率
2. 最佳化記憶體使用
3. 實施資料快取機制
4. 監控系統效能指標

## 🔧 維護注意事項

1. **PLC 通訊**: 定期檢查 PLC 連線狀態和通訊品質
2. ~~**MQTT 連線**: 監控 MQTT 連線穩定性和訊息延遲~~ ❌ **已棄用**
3. **資料庫同步**: 確保設備狀態資料正確同步到資料庫
4. **錯誤處理**: 完善異常處理機制，確保系統穩定性
5. ~~**安全性**: 實施適當的 MQTT 認證和加密機制~~ ❌ **已棄用**
6. **Web API 整合**: 確保 door_logic.py 模組與 web_api_ws 的正確整合

## � 相關文檔

- **plc_proxy_ws**: PLC 通訊代理服務，提供 PlcClient 和 PLC 通訊功能
- **keyence_plc_ws**: Keyence PLC 通訊套件，提供 PlcMemory 記憶體管理
- **db_proxy_ws**: 資料庫代理服務，提供 ConnectionPoolManager 和資料庫操作
- **web_api_ws**: Web API 服務，使用 ecs.door_logic 模組提供門控制 HTTP API
- **launch_ws**: 啟動配置工作空間，包含 ecs_launch 啟動檔案
- ~~**paho-mqtt 官方文檔**: [Eclipse Paho MQTT Python Client](https://pypi.org/project/paho-mqtt/)~~ ❌ **已棄用**
- **ROS 2 官方文檔**: [ROS 2 Documentation](https://docs.ros.org/en/jazzy/)

## 🔄 架構變更說明

### MQTT → Web API 遷移
**舊架構** (已棄用):
- KUKA ECS 系統 → MQTT Broker (192.168.11.206:2883) → door_controller_node_mqtt.py → PLC

**新架構** (目前):
- KUKA ECS 系統 → HTTP API (web_api_ws) → door_logic.py → PLC

**變更原因**:
1. **統一管理**: 門控制功能與其他 AGVC 功能整合在同一 Web 介面
2. **簡化架構**: 減少 MQTT Broker 依賴，使用標準 HTTP 協定
3. **更好維護**: Web API 提供更好的錯誤處理和日誌記錄
4. **安全性**: HTTP API 更容易實現認證和授權機制

## 📋 ToDo 清單

### 🔴 高優先級 (緊急)
- [ ] **完善錯誤處理** (1 週)
  - 統一錯誤回應格式
  - 新增詳細錯誤日誌
  - 實現自動重試機制
- [ ] ~~**MQTT 連線穩定性**~~ ❌ **已棄用**
- [ ] **Web API 整合最佳化** (3 天)
  - 確保 door_logic.py 與 web_api_ws 的穩定整合
  - 新增 Web API 錯誤處理
  - 優化門控制回應時間

### 🟡 中優先級 (重要)
- [ ] **效能最佳化** (2 週)
  - 優化 PLC 通訊頻率
  - 實現資料快取機制
  - 新增效能監控指標
- [ ] **完善測試覆蓋** (3 週)
  - ~~新增 MQTT 通訊測試~~ ❌ **已棄用**
  - 新增 Web API 整合測試
  - 實現門控制邏輯測試
  - 建立效能測試
- [ ] **新增監控功能** (1 週)
  - 實現設備狀態監控
  - ~~新增 MQTT 連線監控~~ ❌ **已棄用**
  - 新增 Web API 整合監控
  - 建立警報機制

### 🟢 低優先級 (改善)
- [ ] **新增配置管理** (2 週)
  - 建立動態配置系統
  - 實現配置熱重載
  - 新增配置驗證
- [ ] **改善 API 文檔** (1 週)
  - 更新 MQTT 訊息格式文檔
  - 新增使用範例
  - 建立 API 參考手冊
- [ ] **實現多門控制** (3 週)
  - 新增門群組管理 (在 web_api_ws 中實作)
  - 實現門聯動控制 (透過 Web API)
  - 建立門狀態同步

### 🔧 技術債務
- [ ] **重構門控制架構** (2 週)
  - 簡化門控制邏輯
  - 統一配置管理
  - 改善程式碼可維護性
- [ ] ~~**更新 MQTT 協定**~~ ❌ **已棄用**
- [ ] **改善安全性** (2 週)
  - ~~實現 MQTT 加密~~ ❌ **已棄用**
  - 新增 Web API 存取控制 (在 web_api_ws 中實作)
  - 建立稽核日誌

### 📊 完成度追蹤
- **核心功能**: 85% ✅
- ~~**MQTT 通訊**: 80% ✅~~ ❌ **已棄用**
- **Web API 整合**: 90% ✅ (在 web_api_ws 中實作)
- **測試覆蓋**: 40% 🔄
- **效能最佳化**: 50% 🔄
- **監控功能**: 30% ⏳

### 🎯 里程碑
1. **v1.1.0** (3 週後) - 完成錯誤處理和 Web API 整合最佳化
2. **v1.2.0** (6 週後) - 效能最佳化和監控功能
3. **v2.0.0** (10 週後) - 完整重構和新功能 (移除所有 MQTT 相關程式碼)
