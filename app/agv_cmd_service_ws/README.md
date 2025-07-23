# AGV 命令服務工作空間 (agv_cmd_service_ws)

## 📋 基本資訊

**啟動狀態**: ✅ 自動載入 (容器啟動腳本中自動載入但不執行特定節點)
**運行環境**: 🚗🖥️ 共用 (AGV 車載系統 + AGVC 管理系統)
**主要功能**: AGV 手動控制和一般命令服務
**依賴狀態**: 依賴 `plc_proxy` 套件，提供 AGV 控制介面
**手動啟動**: 可使用 `ros2 run agv_cmd_service agv_cmd_service_node` 啟動

## 📋 專案概述

AGV 命令服務工作空間提供 AGV 手動控制和一般命令服務，透過 ROS 2 服務介面與 PLC 通訊，實現對 AGV 的遠端控制功能。支援手動移動控制、系統控制、任務管理等功能，是 RosAGV 系統中 AGV 控制的核心介面。

## 🔗 依賴關係

### 依賴的工作空間
- **plc_proxy**: 使用 `PlcClient` 進行 PLC 通訊

### 被依賴的工作空間
- **agv_ws**: AGV 核心系統可能使用本工作空間的命令服務
- **外部系統**: Web UI、操作員介面等可能調用本服務

### 外部依賴
- **ROS 2**: `rclpy`, `rclpy.executors`
- **Python 標準庫**: 無特殊依賴

## 🏗️ 專案結構

```
agv_cmd_service_ws/
├── src/
│   ├── agv_cmd_service/           # 主要服務套件
│   │   ├── agv_cmd_service/
│   │   │   ├── agv_cmd_service_node.py    # 主要服務節點
│   │   │   ├── agv_cmd_client_node.py     # 客戶端封裝類別
│   │   │   └── cleint_test.py             # 客戶端測試腳本
│   │   ├── config/
│   │   │   └── agv_cmd_service.yaml       # PLC 位址配置文件
│   │   ├── launch/
│   │   │   └── launchtest.py              # 啟動文件
│   │   ├── package.xml                    # 套件描述文件
│   │   └── setup.py                       # Python 套件設定
│   └── agv_cmd_interfaces/        # 服務介面定義
│       ├── srv/
│       │   ├── ManualCommand.srv          # 手動命令服務介面
│       │   └── GeneralCommand.srv         # 一般命令服務介面
│       ├── CMakeLists.txt                 # CMake 建置配置
│       └── package.xml                    # 介面套件配置
└── README.md                      # 本檔案
```

## ⚙️ 主要功能

### 1. 手動控制命令 (ManualCommand)
支援以下手動控制指令：
- **移動控制**：前進 (forward)、後退 (backward)
- **旋轉控制**：左轉 (rotate_left)、右轉 (rotate_right)
- **平移控制**：左移 (shift_left)、右移 (shift_right)
- **系統控制**：煞車 (break)、啟用 (enable)

### 2. 一般命令 (GeneralCommand)
支援以下一般控制指令：
- **自動模式**：auto - 切換自動/手動模式
- **緊急停止**：stop - 緊急停止 AGV
- **系統重置**：reset - 重置 AGV 系統
- **任務派發**：send_mission - 發送任務到指定位置
- **任務取消**：cancel_mission - 取消當前任務
- **交通控制**：traffic_stop - 交通停止控制

### 3. PLC 通訊整合
- 透過 `plc_proxy` 套件與 PLC 進行通訊
- 支援 MR (Memory Relay) 和 DM (Data Memory) 操作
- 完整的錯誤處理和日誌記錄

## 🔧 配置說明

### PLC 位址配置 (agv_cmd_service.yaml)
```yaml
forward_address: "3708"          # 前進控制位址
backward_address: "3709"         # 後退控制位址
rotate_left_address: "3712"      # 左轉控制位址
rotate_right_address: "3713"     # 右轉控制位址
shift_left_address: "3801"       # 左移控制位址
shift_right_address: "3802"      # 右移控制位址
break_address: "3714"            # 煞車控制位址
enable_address: "3715"           # 啟用控制位址
auto_address1: "4001"            # 自動模式位址1
auto_address2: "0000"            # 自動模式位址2
stop_address: "3701"             # 停止控制位址
reset_address: "302"             # 重置控制位址
send_mission_from_address: "2990" # 任務起點位址
send_mission_to_address: "2991"   # 任務終點位址
send_mission_magic_address: "2993" # 任務魔術數位址
cancel_mission_address: "7001"    # 取消任務位址
traffic_stop_address: "7002"      # 交通停止位址
```

## 🚀 使用方法

### 1. 建置工作空間
```bash
# 載入 ROS 2 環境並建置
source /opt/ros/jazzy/setup.bash && source /opt/ws_rmw_zenoh/install/setup.bash && cd /app/agv_cmd_service_ws && colcon build
source install/setup.bash
```

### 2. 啟動服務
```bash
# 使用 launch 文件啟動
ros2 launch agv_cmd_service launchtest.py

# 或直接啟動節點
ros2 run agv_cmd_service agv_cmd_service_node
```

### 3. 使用客戶端
```python
import rclpy
from agv_cmd_service.agv_cmd_client_node import AgvCommandClient
from rclpy.node import Node

# 初始化 ROS 2
rclpy.init()
client = AgvCommandClient(Node())

# 發送手動命令
success = client.send_manual_command("forward", True)
print(f"前進命令結果: {success}")

# 發送一般命令
success = client.send_general_command("auto", "on,100,200,300")
print(f"自動模式命令結果: {success}")

# 清理
client.destroy_node()
rclpy.shutdown()
```

## 📡 ROS 2 服務介面

### ManualCommand.srv
```yaml
# Request
string command    # 命令名稱：forward, backward, rotate_left, rotate_right,
                 #          shift_left, shift_right, break, enable
bool onoff       # 開關狀態：true=開啟, false=關閉

---
# Response
bool success     # 執行結果：true=成功, false=失敗
```

**支援的手動命令**:
- `forward`: 前進 (MR 位址: 3708)
- `backward`: 後退 (MR 位址: 3709)
- `rotate_left`: 左轉 (MR 位址: 3712)
- `rotate_right`: 右轉 (MR 位址: 3713)
- `shift_left`: 左移 (MR 位址: 3801)
- `shift_right`: 右移 (MR 位址: 3802)
- `break`: 煞車 (MR 位址: 3714)
- `enable`: 啟用 (MR 位址: 3715)

### GeneralCommand.srv
```yaml
# Request
string command    # 命令名稱：auto, stop, reset, send_mission,
                 #          cancel_mission, traffic_stop
string parameter  # 命令參數：依命令而定，用逗號分隔

---
# Response
bool success     # 執行結果：true=成功, false=失敗
```

**支援的一般命令**:
- `auto`: 自動模式切換
  - 參數格式: `"on"` 或 `"off"`
  - PLC 位址: MR4001, MR0000
- `stop`: 緊急停止
  - 參數: 無
  - PLC 位址: MR3701
- `reset`: 系統重置
  - 參數: 無
  - PLC 位址: MR302
- `send_mission`: 發送任務
  - 參數格式: `"on,起點,終點,魔術數字"`
  - PLC 位址: DM2990, DM2991, DM2993
- `cancel_mission`: 取消任務
  - 參數: 無
  - PLC 位址: MR7001
- `traffic_stop`: 交通控制
  - 參數格式: `"on"` 或 `"off"`
  - PLC 位址: MR7002

## 🔧 核心 API

### AgvCommandService 節點
```python
# 節點名稱: agv_cmd_service_node
# 命名空間: 可配置 (預設為根命名空間)

# 提供的服務:
# - /ManualCommand (agv_cmd_interfaces/srv/ManualCommand)
# - /GeneralCommand (agv_cmd_interfaces/srv/GeneralCommand)

# 可配置參數:
# - forward_address: "3708"
# - backward_address: "3709"
# - rotate_left_address: "3712"
# - rotate_right_address: "3713"
# - shift_left_address: "3801"
# - shift_right_address: "3802"
# - break_address: "3714"
# - enable_address: "3715"
# - auto_address1: "4001"
# - auto_address2: "0000"
# - stop_address: "3701"
# - reset_address: "302"
# - send_mission_from_address: "2990"
# - send_mission_to_address: "2991"
# - send_mission_magic_address: "2993"
# - cancel_mission_address: "7001"
# - traffic_stop_address: "7002"
```

### AgvCommandClient 類別
```python
import rclpy
from rclpy.node import Node
from agv_cmd_service.agv_cmd_client_node import AgvCommandClient

# 初始化客戶端
rclpy.init()
node = Node('test_node')
client = AgvCommandClient(node, namespace="")  # 可指定命名空間

# 發送手動命令
success = client.send_manual_command("forward", True)
if success:
    print("前進命令執行成功")

# 發送一般命令
success = client.send_general_command("auto", "on")
if success:
    print("自動模式開啟成功")

# 發送任務命令
success = client.send_general_command("send_mission", "on,100,200,300")
if success:
    print("任務發送成功")

# 清理
client.destroy_node()
rclpy.shutdown()
```

## 🧪 測試方法

### 1. 建置和測試
```bash
# 建置工作空間
source /opt/ros/jazzy/setup.bash && source /opt/ws_rmw_zenoh/install/setup.bash && cd /app/agv_cmd_service_ws && colcon build

# 執行測試
source install/setup.bash && colcon test
colcon test-result --verbose
```

### 2. 服務功能測試
```bash
# 測試手動前進命令
ros2 service call /ManualCommand agv_cmd_interfaces/srv/ManualCommand \
  "{command: 'forward', onoff: true}"

# 測試手動後退命令
ros2 service call /ManualCommand agv_cmd_interfaces/srv/ManualCommand \
  "{command: 'backward', onoff: false}"

# 測試自動模式命令
ros2 service call /GeneralCommand agv_cmd_interfaces/srv/GeneralCommand \
  "{command: 'auto', parameter: 'on'}"

# 測試任務發送命令
ros2 service call /GeneralCommand agv_cmd_interfaces/srv/GeneralCommand \
  "{command: 'send_mission', parameter: 'on,100,200,300'}"
```

### 3. 效能測試
```bash
# 執行客戶端效能測試
cd /app/agv_cmd_service_ws/src/agv_cmd_service/agv_cmd_service
python3 cleint_test.py
```

**測試結果**:
- 發送 100 次指令的平均耗時約 0.01-0.02 秒
- 支援高頻率指令發送
- 客戶端具備超時保護機制

### 4. 手動驗證
```python
# 測試服務連線和基本功能
import rclpy
from rclpy.node import Node
from agv_cmd_interfaces.srv import ManualCommand

rclpy.init()
node = Node('test_node')
client = node.create_client(ManualCommand, '/ManualCommand')

# 等待服務可用
if client.wait_for_service(timeout_sec=5.0):
    request = ManualCommand.Request()
    request.command = "forward"
    request.onoff = True

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result().success:
        print("手動命令測試成功")
    else:
        print("手動命令測試失敗")
else:
    print("AGV 命令服務不可用")

node.destroy_node()
rclpy.shutdown()
```

## � 故障排除

### 常見問題

#### 1. 服務啟動失敗
**症狀**: `Failed to create service` 或節點無法啟動
**解決方法**:
```bash
# 檢查 plc_proxy 服務是否運行
ros2 service list | grep plc_service

# 檢查依賴是否正確載入
python3 -c "from plc_proxy.plc_client import PlcClient"

# 檢查 ROS 2 環境
ros2 node list
```

#### 2. 命令執行失敗
**症狀**: `response.success = False` 或 PLC 通訊錯誤
**解決方法**:
```bash
# 檢查 PLC 連線狀態
ros2 service call /plc_service/read_data plc_interfaces/srv/ReadData \
  "{device_type: 'MR', address: '3708'}"

# 檢查命令參數是否正確
ros2 service call /ManualCommand agv_cmd_interfaces/srv/ManualCommand \
  "{command: 'forward', onoff: true}"

# 檢查服務日誌
ros2 topic echo /rosout | grep agv_cmd_service
```

#### 3. 客戶端連線超時
**症狀**: `wait_for_service timeout` 或客戶端初始化失敗
**解決方法**:
```bash
# 檢查服務是否存在
ros2 service list | grep -E "(ManualCommand|GeneralCommand)"

# 檢查服務類型
ros2 service type /ManualCommand
ros2 service type /GeneralCommand

# 增加超時時間
# 在客戶端程式碼中調整 timeout_sec 參數
```

### 除錯工具
```bash
# 檢查所有 AGV 命令相關服務
ros2 service list | grep -E "(Manual|General)"

# 監控服務呼叫
ros2 topic echo /rosout | grep agv_cmd_service

# 檢查節點詳細資訊
ros2 node info /agv_cmd_service_node

# 檢查服務介面定義
ros2 interface show agv_cmd_interfaces/srv/ManualCommand
ros2 interface show agv_cmd_interfaces/srv/GeneralCommand
```

### 日誌和診斷
```bash
# 啟用詳細日誌
export RCUTILS_LOGGING_SEVERITY=DEBUG

# 檢查系統資源
ps aux | grep agv_cmd_service

# 檢查錯誤日誌
tail -f /tmp/agv_cmd_service.log
```

## ⚙️ 配置說明

### PLC 位址配置
所有 PLC 位址都可以透過 ROS 2 參數進行配置：

```yaml
# 手動控制位址
forward_address: "3708"           # 前進
backward_address: "3709"          # 後退
rotate_left_address: "3712"       # 左轉
rotate_right_address: "3713"      # 右轉
shift_left_address: "3801"        # 左移
shift_right_address: "3802"       # 右移
break_address: "3714"             # 煞車
enable_address: "3715"            # 啟用

# 一般控制位址
auto_address1: "4001"             # 自動模式位址1
auto_address2: "0000"             # 自動模式位址2
stop_address: "3701"              # 緊急停止
reset_address: "302"              # 系統重置

# 任務控制位址
send_mission_from_address: "2990" # 任務起點
send_mission_to_address: "2991"   # 任務終點
send_mission_magic_address: "2993" # 任務魔術數字
cancel_mission_address: "7001"    # 取消任務
traffic_stop_address: "7002"      # 交通控制
```

### 啟動參數範例
```bash
# 使用自訂參數啟動服務
ros2 run agv_cmd_service agv_cmd_service_node --ros-args \
  -p forward_address:="3800" \
  -p backward_address:="3801" \
  -p stop_address:="3700"

# 使用配置檔案啟動
ros2 run agv_cmd_service agv_cmd_service_node --ros-args \
  --params-file /app/agv_cmd_service_ws/src/agv_cmd_service/config/agv_cmd_service.yaml
```

## 🔗 相關文檔

- **plc_proxy_ws**: PLC 代理服務，本工作空間的底層依賴
- **agv_ws**: AGV 核心系統，可能使用本工作空間的服務
- **ROS 2 服務文檔**: [Understanding ROS 2 Services](https://docs.ros.org/en/jazzy/Tutorials/Services/Understanding-ROS2-Services.html)


