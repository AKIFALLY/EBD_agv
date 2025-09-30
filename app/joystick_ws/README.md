# 搖桿控制工作空間 (joystick_ws)

## 📋 基本資訊

**啟動狀態**: ✅ 實際啟動 (透過 loader_agv launch 檔案自動啟動 joy_linux_node)
**運行環境**: 🚗🖥️ 共用 (AGV 車載系統 + AGVC 管理系統)
**主要功能**: USB 搖桿和遊戲手把控制介面 (ROS 2 joy_linux 版本)
**依賴狀態**: 不使用虛擬環境套件，完全基於 ROS 2 系統套件
**實作狀態**: 完整實作，已遷移至 ROS 2 joy_linux 標準版本

## 📋 專案概述

搖桿控制工作空間提供 AGV 的手動控制功能，透過 USB 搖桿或遊戲手把實現對 AGV 的即時操控。該系統完全基於 ROS 2 的 joy_linux 套件，提供標準化的搖桿輸入處理和直觀的手動控制介面，適用於 AGV 的測試、調試和緊急操作。

此工作空間採用 ROS 2 標準的 joy_linux 實作，確保與所有 Linux HID 相容設備的完美整合。系統支援死人開關安全機制、多種控制器類型（Xbox、PlayStation、工業搖桿）、50Hz 即時控制回饋，以及完整的按鈕和軸映射配置。工作空間已全面遷移至 ROS 2 標準版本，棄用的 pygame 程式碼僅為向後相容性保留。

**⚠️ 重要說明**: 本工作空間已全面遷移至 ROS 2 joy_linux 版本，pygame 版本的 JoystickHandler 已棄用。新開發請使用 `joy_handler.py` 中的 JoyHandler 類別。

## 🔗 依賴關係

### 系統套件依賴
- **ROS 2 Jazzy**: `rclpy`, `rclpy.node`, `rclpy.timer`
- **ROS 2 套件**: `ros-jazzy-joy-linux` (Linux 搖桿驅動)
- **訊息定義**: `sensor_msgs.msg.Joy`, `geometry_msgs.msg.Twist`
- **Python 標準庫**: `time`, `threading`, `abc`

### 被依賴的工作空間
- **agv_ws**: AGV 核心系統 - 接收搖桿控制命令進行手動控制
- **agv_cmd_service_ws**: AGV 命令服務 - 可能使用搖桿輸入進行控制操作
- **loader_agv**: Launch 檔案中自動啟動 joy_linux_node

### 外部依賴
- **硬體設備**: USB 搖桿設備 (`/dev/input/js*`)
- **系統工具**: `jstest` (搖桿測試工具，可選)
- **已棄用**: pygame (雖然仍在虛擬環境中安裝，但已不再使用)

## 🏗️ 專案結構

```
joystick_ws/
├── src/
│   └── joystick/                   # 搖桿控制套件 (完整實作)
│       ├── joystick/               # 核心功能模組
│       │   ├── __init__.py         # 套件初始化
│       │   ├── joy_handler.py     # ROS 2 Joy 訊息處理器 ✅ (主要使用)
│       │   ├── joystick_handler.py# ⚠️ 已棄用 - Pygame 搖桿處理器
│       │   └── joystick_test_node.py # ⚠️ 已棄用 - Pygame 搖桿測試節點
│       ├── test/                   # 測試檔案目錄
│       │   ├── test_copyright.py  # 版權資訊測試
│       │   ├── test_flake8.py     # 程式碼風格檢查
│       │   └── test_pep257.py     # 文檔字串檢查
│       ├── package.xml             # ROS 2 套件描述文件
│       ├── setup.py                # Python 套件設定
│       └── setup.cfg               # 套件安裝配置
├── CLAUDE.md                       # AI Agent 專用指導文件
└── README.md                       # 本檔案 (專案說明文件)
```

**遷移說明**: 本工作空間已全面遷移至 ROS 2 joy_linux 版本，所有新開發都應使用 `joy_handler.py` 中的 JoyHandler 類別。棄用的程式碼暫時保留以維持向後相容性，但不建議在新開發中使用。

## ⚙️ 主要功能

### 1. ROS 2 搖桿輸入處理 (JoyHandler)
**標準化 ROS 2 整合**:
- **ROS 2 訊息**: 處理標準 `sensor_msgs/Joy` 訊息
- **命名空間支援**: 支援多 AGV 環境部署
- **回調機制**: 按鈕和軸事件回調系統
- **狀態追蹤**: 按鈕和軸狀態變化檢測
- **設備相容性**: 支援所有 Linux 相容的 USB 搖桿

### ⚠️ 已棄用功能 (JoystickHandler - pygame)
**不建議使用的功能**:
- ~~pygame 基礎搖桿處理~~ (已棄用)
- ~~直接硬體設備檢測~~ (已棄用)
- ~~自訂 D-Pad 處理~~ (已棄用)

### 2. AGV 控制整合
- **速度控制**: 透過搖桿軸控制 AGV 移動速度
- **方向控制**: 支援前進、後退、左轉、右轉、左移、右移
- **緊急停止**: 快速停止按鈕 (B 按鈕)
- **啟用控制**: 死人開關 (A 按鈕)
- **模式切換**: 手動/自動模式切換 (X 按鈕)

### 3. 安全機制
- **死人開關**: 需要持續按住啟用按鈕 (A 按鈕)
- **速度限制**: 可配置的最大速度限制
- **超時保護**: 設備超時自動停止 (預設 1.0 秒)
- **狀態指示**: 即時狀態回饋和日誌記錄
- **錯誤恢復**: 搖桿斷線自動重新初始化

## 🎮 支援的搖桿類型和配置

### USB 遊戲手把支援
- **Xbox 控制器**: Xbox One/Series 控制器
- **PlayStation 控制器**: PS4/PS5 DualShock 控制器
- **通用 USB 搖桿**: 標準 HID 相容搖桿
- **工業搖桿**: 專業級工業控制搖桿

### JoystickHandler 按鈕配置 (pygame)
```python
# 軸配置
L_X_AXIS = 0    # 左搖桿 X 軸
L_Y_AXIS = 1    # 左搖桿 Y 軸
R_X_AXIS = 2    # 右搖桿 X 軸
R_Y_AXIS = 3    # 右搖桿 Y 軸

# D-Pad 配置
DPAD_UP = (0, 1)
DPAD_DOWN = (0, -1)
DPAD_LEFT = (-1, 0)
DPAD_RIGHT = (1, 0)
DPAD_CENTER = (0, 0)

# 按鈕索引 (依搖桿類型而定)
# 通常：0=A, 1=B, 2=X, 3=Y, 4=L1, 5=R1, 6=L2, 7=R2, 8=SELECT, 9=START
```

### JoyHandler 按鈕配置 (ROS 2)
```python
# 按鈕常量
A_BUTTON = 0        # 啟用控制
B_BUTTON = 1        # 緊急停止
X_BUTTON = 2        # 模式切換
Y_BUTTON = 3        # 功能按鈕
L1_BUTTON = 4       # L1 按鈕
R1_BUTTON = 5       # R1 按鈕
L2_BUTTON = 6       # L2 按鈕
R2_BUTTON = 7       # R2 按鈕
SELECT_BUTTON = 8   # 選擇按鈕
START_BUTTON = 9    # 開始按鈕

# 軸配置
L_X_AXIS = 0        # 左搖桿 X 軸
L_Y_AXIS = 1        # 左搖桿 Y 軸
R_X_AXIS = 2        # 右搖桿 X 軸
R_Y_AXIS = 3        # 右搖桿 Y 軸
D_PAD_X_AXIS = 4    # D-Pad X 軸
D_PAD_Y_AXIS = 5    # D-Pad Y 軸
```

## 🔧 核心 API

### ⚠️ 棄用警告
**JoystickHandler (pygame 版本) 已棄用**，請使用 JoyHandler (ROS 2 版本)。棄用的 API 暫時保留但不建議使用。

### JoyHandler 類別 (ROS 2 標準版本) - 推薦使用
```python
import rclpy
from rclpy.node import Node
from joystick.joy_handler import JoyHandler

# 初始化 ROS 2 節點
rclpy.init()
node = Node('joy_test_node')
joy_handler = JoyHandler(node)

# 註冊按鈕回調
def button_callback(index, action):
    print(f"ROS 2 按鈕 {index} {action}")

joy_handler.register_button_callback(JoyHandler.A_BUTTON, button_callback)

# 註冊軸回調
def axis_callback(index, value):
    print(f"ROS 2 軸 {index} 值: {value}")

joy_handler.register_axis_callback(JoyHandler.L_X_AXIS, axis_callback)

# 啟動 Joy 處理
joy_handler.start()

# 執行 ROS 2 節點
rclpy.spin(node)

# 清理
joy_handler.stop()
node.destroy_node()
rclpy.shutdown()
```

## 🚀 使用方法

### 1. 建置工作空間
```bash
# 載入 ROS 2 環境並建置
source /opt/ros/jazzy/setup.bash && source /opt/ws_rmw_zenoh/install/setup.bash && cd /app/joystick_ws && colcon build
source install/setup.bash
```

### 2. 依賴檢查
```bash
# 檢查 ROS 2 joy_linux 套件
ros2 pkg list | grep joy

# 檢查 sensor_msgs 套件
python3 -c "from sensor_msgs.msg import Joy; print('✅ sensor_msgs 可用')"

# ⚠️ 注意：pygame 雖然在虛擬環境中，但本工作空間已不再使用
```

### 3. 檢查搖桿設備
```bash
# 檢查 USB 設備
lsusb | grep -i joystick

# 檢查輸入設備
ls /dev/input/js*

# 設定設備權限
sudo chmod 666 /dev/input/js0

# 測試搖桿輸入 (需要安裝 joystick 工具)
jstest /dev/input/js0
```

### 4. 啟動搖桿節點
```bash
# 使用標準 ROS 2 joy_linux 節點 (推薦)
ros2 run joy_linux joy_linux_node

# 指定搖桿設備
ros2 run joy_linux joy_linux_node --ros-args -p dev:="/dev/input/js0"

# 在命名空間中啟動
ros2 run joy_linux joy_linux_node --ros-args -r __ns:=/cargo02 -p dev:="/dev/input/js0"

# ⚠️ 已棄用：不建議使用 pygame 版本的測試節點
# ros2 run joystick joystick_test_node  # 已棄用
```

### 5. 監控搖桿輸入
```bash
# 監聽搖桿訊息
ros2 topic echo /joy

# 檢查搖桿狀態
ros2 topic hz /joy
ros2 topic bw /joy

# 檢查節點資訊
ros2 node info /joy_linux_node
```

## 🔧 配置說明

### 搖桿參數配置
```yaml
# 搖桿設備路徑
dev: "/dev/input/js0"

# 發布頻率 (Hz)
publish_rate: 50.0

# 死區設定
deadzone: 0.1

# 自動重複間隔
autorepeat_rate: 20.0

# 設備超時 (秒)
device_timeout: 1.0
```

### AGV 控制參數
```yaml
# 最大線性速度 (m/s)
max_linear_velocity: 1.0

# 最大角速度 (rad/s)
max_angular_velocity: 1.0

# 速度縮放因子
velocity_scale: 0.5

# 啟用按鈕索引
enable_button: 0

# 緊急停止按鈕索引
emergency_stop_button: 1
```

## 📡 ROS 2 介面

### 發布的主題
```bash
# 搖桿狀態訊息
/joy (sensor_msgs/Joy)

# AGV 控制命令
/cmd_vel (geometry_msgs/Twist)

# 控制狀態
/joystick_status (std_msgs/String)
```

### 訂閱的主題
```bash
# AGV 狀態回饋
/agv_status (agv_interfaces/AgvStatus)

# 系統模式
/system_mode (std_msgs/String)
```

### 服務介面
```bash
# 搖桿校準
/calibrate_joystick (std_srvs/Trigger)

# 重設搖桿
/reset_joystick (std_srvs/Trigger)
```

## 🧪 測試方法

### 1. 建置和測試
```bash
# 建置工作空間
source /opt/ros/jazzy/setup.bash && source /opt/ws_rmw_zenoh/install/setup.bash && cd /app/joystick_ws && colcon build

# 執行測試
source install/setup.bash && colcon test
colcon test-result --verbose
```

### 2. ROS 2 搖桿功能測試
```bash
# ⚠️ 已棄用：pygame 搖桿處理器測試
# cd /app/joystick_ws/src/joystick/joystick
# python3 joystick_handler.py  # 不建議使用

# 使用標準 ROS 2 joy_linux 節點進行測試 (推薦)
ros2 run joy_linux joy_linux_node --ros-args -p dev:="/dev/input/js0"
```

### 3. ROS 2 Joy 功能測試
```bash
# 測試系統 joy_linux 節點
ros2 run joy_linux joy_linux_node --ros-args -p dev:="/dev/input/js0"

# 檢查搖桿訊息
ros2 topic echo /joy

# 測試按鈕和軸 (使用 rqt_plot)
ros2 run rqt_plot rqt_plot /joy/axes[0] /joy/axes[1]
```

### 4. 手動驗證
```bash
# 檢查搖桿設備 (系統層級)
ls /dev/input/js*
lsusb | grep -i joystick

# 測試 ROS 2 Joy 訊息
ros2 topic echo /joy --once

# 檢查搖桿節點狀態
ros2 node info /joy_linux_node
```

```python
# ⚠️ 已棄用：pygame 搖桿檢測 (不建議使用)
# 以下程式碼已棄用，僅供參考
"""
import pygame
import os

os.environ["SDL_AUDIODRIVER"] = "dummy"
pygame.init()
pygame.joystick.init()
# ... pygame 相關程式碼已棄用
"""
```

## � 故障排除

### 常見問題

#### 1. ⚠️ 已棄用問題：pygame 相關錯誤
**如果遇到 pygame 相關錯誤**，請遷移至 ROS 2 joy_linux 版本：
```bash
# 不要嘗試修復 pygame 問題，請使用 ROS 2 版本
ros2 run joy_linux joy_linux_node --ros-args -p dev:="/dev/input/js0"
```

#### 2. 搖桿設備無法檢測
**症狀**: ROS 2 joy_linux 節點無法檢測搖桿設備
**解決方法**:
```bash
# 檢查設備權限
ls -l /dev/input/js*
sudo chmod 666 /dev/input/js0

# 檢查 USB 設備
lsusb | grep -i joystick

# 檢查驅動
dmesg | grep -i joystick

# 重新插拔 USB 搖桿
```

#### 3. ROS 2 Joy 節點啟動失敗
**症狀**: `Failed to open joystick` 或節點無法啟動
**解決方法**:
```bash
# 檢查 joy_linux 套件是否安裝
ros2 pkg list | grep joy

# 檢查設備路徑
ls /dev/input/js*

# 使用正確的設備路徑啟動
ros2 run joy_linux joy_linux_node --ros-args -p dev:="/dev/input/js0"
```

#### 4. 搖桿輸入延遲或無回應
**症狀**: 搖桿輸入延遲過高或按鈕無回應
**解決方法**:
```bash
# 檢查系統負載
top

# 調整 ROS 2 joy_linux 發布頻率
ros2 param set /joy_linux_node publish_rate 100.0

# 檢查搖桿訊息頻率
ros2 topic hz /joy
```

### 除錯工具
```bash
# 檢查所有搖桿相關節點
ros2 node list | grep joy

# 監控搖桿訊息
ros2 topic echo /joy

# 檢查節點詳細資訊
ros2 node info /joy_linux_node

# 檢查參數
ros2 param list /joy_linux_node
ros2 param get /joy_linux_node dev

# 使用 rqt 工具
ros2 run rqt_graph rqt_graph
ros2 run rqt_topic rqt_topic
```

### 日誌和診斷
```bash
# 啟用詳細日誌
export RCUTILS_LOGGING_SEVERITY=DEBUG

# 檢查系統資源
ps aux | grep joy

# 檢查錯誤日誌
tail -f /tmp/joystick.log
```

## 📝 開發指南

### 新增搖桿支援
1. 檢查搖桿的 USB VID/PID
2. 測試搖桿的軸和按鈕配置
3. 更新按鈕映射配置
4. 測試控制功能

### 自訂控制邏輯
1. 修改 `joy_linux_node.py` 中的回調函數
2. 新增自訂的控制算法
3. 實施安全檢查機制
4. 測試控制回應

### 整合其他控制器
1. 新增對應的設備驅動
2. 實施統一的控制介面
3. 更新配置文件
4. 測試相容性

## 🔧 故障排除

### 常見問題

1. **搖桿無法檢測**
   ```bash
   # 檢查設備權限
   ls -l /dev/input/js*
   sudo chmod 666 /dev/input/js0
   
   # 檢查驅動
   dmesg | grep -i joystick
   ```

2. **控制延遲過高**
   ```bash
   # 調整發布頻率
   ros2 param set /joy_linux_node publish_rate 100.0
   
   # 檢查系統負載
   top
   ```

3. **按鈕映射錯誤**
   ```bash
   # 測試按鈕索引
   ros2 topic echo /joy
   
   # 重新校準
   ros2 service call /calibrate_joystick std_srvs/Trigger
   ```

4. **AGV 無回應**
   ```bash
   # 檢查控制命令
   ros2 topic echo /cmd_vel
   
   # 檢查 AGV 狀態
   ros2 topic echo /agv_status
   ```

## 🔧 維護注意事項

1. **設備維護**: 定期檢查搖桿硬體狀態和連接
2. **校準檢查**: 定期校準搖桿軸和按鈕
3. **安全測試**: 定期測試緊急停止功能
4. **效能監控**: 監控控制延遲和回應時間
5. **備用設備**: 準備備用搖桿以防硬體故障

## ⚙️ 配置說明

### ⚠️ 已棄用配置：pygame 搖桿參數
```python
# 以下配置已棄用，不建議使用
# JoystickHandler 配置 (已棄用)
# DEADZONE = 0.05
# POLLING_RATE = 50
# DEVICE_TIMEOUT = 1.0
```

### ROS 2 Joy 節點參數 (推薦使用)
```yaml
# joy_linux_node 參數
dev: "/dev/input/js0"              # 搖桿設備路徑
publish_rate: 50.0                 # 發布頻率 (Hz)
deadzone: 0.1                      # 死區設定
autorepeat_rate: 20.0              # 自動重複間隔
device_timeout: 1.0                # 設備超時 (秒)
```

### AGV 控制參數 (範例)
```yaml
# AGV 控制配置
max_linear_velocity: 1.0           # 最大線性速度 (m/s)
max_angular_velocity: 1.0          # 最大角速度 (rad/s)
velocity_scale: 0.5                # 速度縮放因子
enable_button: 0                   # 啟用按鈕索引 (A 按鈕)
emergency_stop_button: 1           # 緊急停止按鈕索引 (B 按鈕)
mode_switch_button: 2              # 模式切換按鈕索引 (X 按鈕)
```

### 啟動參數範例
```bash
# 使用自訂參數啟動 joy_linux 節點
ros2 run joy_linux joy_linux_node --ros-args \
  -p dev:="/dev/input/js0" \
  -p publish_rate:=100.0 \
  -p deadzone:=0.05

# 在命名空間中啟動
ros2 run joy_linux joy_linux_node --ros-args \
  -r __ns:=/cargo02 \
  -p dev:="/dev/input/js0"
```

## 🔗 相關文檔

- **agv_cmd_service_ws**: AGV 命令服務，可能使用搖桿輸入進行控制
- **agv_ws**: AGV 核心系統，接收搖桿控制命令
- **pygame 官方文檔**: [Pygame Joystick Documentation](https://www.pygame.org/docs/ref/joystick.html)
- **ROS 2 Joy 套件**: [joy_linux Package](https://github.com/ros2/joystick_drivers)
- **ROS 2 感測器訊息**: [sensor_msgs/Joy](https://docs.ros2.org/latest/api/sensor_msgs/msg/Joy.html)
