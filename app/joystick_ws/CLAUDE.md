# joystick_ws - USB 搖桿輸入處理工作空間

## 📚 Context Loading
../../CLAUDE.md  # 引用根目錄系統文档

## 📋 工作空間概述

**USB 搖桿輸入處理工作空間** 專注於為 RosAGV AGV 車載系統提供 USB 搖桿輸入處理和手動控制功能。

### USB 搖桿工作空間特有功能
- **🎮 ROS 2 Joy 訊息處理**: 透過 `JoyHandler` 類別處理 `sensor_msgs/Joy` 訊息 ✅ **主要使用**
- **🚫 Pygame 直接處理**: 透過 `JoystickHandler` 類別直接讀取 USB 搖桿設備 ⚠️ **已棄用**
- **🚫 測試節點整合**: 提供 `JoystickTestNode` 整合測試功能 ⚠️ **已棄用**
- **🔌 USB 設備支援**: 需要 `/dev/input` 設備掛載支援

**⚠️ 重要**: 此模組專為 AGV 車載系統設計，必須在 AGV 容器內執行。

## 📂 工作空間結構

### 目錄架構
```
joystick_ws/
├── src/joystick/                      # ROS 2 Python 套件
│   ├── joystick/                      # 主要模組目錄
│   │   ├── __init__.py               # 套件初始化
│   │   ├── joystick_handler.py       # Pygame 直接處理搖桿
│   │   ├── joy_handler.py            # ROS 2 Joy 訊息處理
│   │   └── joystick_test_node.py     # ROS 2 測試節點
│   ├── package.xml                   # ROS 2 套件描述
│   ├── setup.py                      # Python 套件安裝
│   ├── setup.cfg                     # 套件配置
│   └── resource/joystick             # 套件資源
├── CLAUDE.md                         # 模組文檔
└── README.md                         # 基本說明

## 🔧 核心特色

### 搖桿處理架構

#### ~~🎮 JoystickHandler (Pygame 直接處理)~~ ⚠️ **已棄用**
- **~~直接設備存取~~**: ~~使用 Pygame 直接讀取 `/dev/input/js*` USB 搖桿設備~~
- **~~完整按鍵支援~~**: ~~A/B/X/Y、L1/R1/L2/R2、Select/Start (10個按鍵)~~
- **~~軸控制支援~~**: ~~左右搖桿軸 (L_X/L_Y/R_X/R_Y) 4軸控制~~
- **~~D-Pad 檢測~~**: ~~8方向 D-Pad 檢測，包含對角方向~~
- **~~回調機制~~**: ~~支援按鍵、軸、D-Pad 獨立回調註冊~~
- **~~自動重連~~**: ~~搖桿拔插自動檢測和重新初始化~~
- **~~單例模式~~**: ~~類別級別的單例設計，避免多重初始化~~

**⚠️ 棄用原因**: Pygame 直接處理模式已不再使用，建議使用標準 ROS 2 Joy 訊息處理。

#### 🤖 JoyHandler (ROS 2 訊息處理) ✅ **主要使用**
- **ROS 2 整合**: 處理標準 `sensor_msgs/Joy` 訊息
- **命名空間支援**: 支援 ROS 2 命名空間動態配置
- **軸映射**: 6軸支援 (4個搖桿軸 + 2個 D-Pad 軸)
- **按鍵常量**: 標準化按鍵索引定義 (0-9)
- **狀態追蹤**: 按鍵和軸狀態變化檢測
- **動態適配**: 根據軸數量自動調整 D-Pad 軸映射

#### ~~🧪 JoystickTestNode (整合測試)~~ ⚠️ **已棄用**
- **~~ROS 2 節點~~**: ~~完整的 ROS 2 節點實作~~
- **~~定時器輪詢~~**: ~~使用 ROS 2 定時器進行搖桿狀態輪詢~~
- **~~事件列印~~**: ~~所有搖桿事件的控制台輸出~~
- **~~優雅關閉~~**: ~~支援 Ctrl+C 優雅關閉~~

**⚠️ 棄用原因**: 測試節點整合了已棄用的 Pygame 處理模式，建議直接使用標準的 ROS 2 joy_node。

## 🚀 USB 搖桿處理專用開發

**⚠️ 通用開發環境請參考**: ../../CLAUDE.md 開發指導章節

**⚠️ 環境要求**: 必須在 AGV 車載容器內開發，需要 USB 設備存取權限。

### 環境準備
```bash
# 1. 進入 AGV 容器
docker compose -f docker-compose.yml exec rosagv bash

# 2. 載入 AGV 工作空間
all_source  # 智能載入，或使用 agv_source

# 3. 進入工作空間目錄
cd /app/joystick_ws
```

### 建置和測試
```bash
# 建置 joystick 套件
colcon build --packages-select joystick

# 檢查 USB 搖桿設備
ls -la /dev/input/js*
lsusb | grep -i joystick

# ✅ 執行 ROS 2 Joy 測試 (推薦)
python3 src/joystick/joystick/joy_handler.py

# ⚠️ 已棄用的測試方法
# python3 src/joystick/joystick/joystick_handler.py
# python3 src/joystick/joystick/joystick_test_node.py

# ✅ 使用標準 ROS 2 joy_node (推薦)
ros2 run joy joy_node --ros-args --remap joy:=/agv/joy
```

### Docker 設備掛載確認
```bash
# 確認 docker-compose.yml 包含設備掛載
# devices:
#   - "/dev/input:/dev/input"
```

## 📊 技術實作詳解

### ~~JoystickHandler 類別設計 (Pygame)~~ ⚠️ **已棄用**

#### ~~常量定義~~ ⚠️ **已棄用**
```python
# ⚠️ 以下為已棄用的 Pygame 常量定義
# D-Pad 方向常量 (Hat 值)
# DPAD_UP = (0, 1)         # 上
# DPAD_DOWN = (0, -1)      # 下
# DPAD_LEFT = (-1, 0)      # 左
# DPAD_RIGHT = (1, 0)      # 右
# DPAD_UP_LEFT = (-1, 1)   # 左上
# DPAD_UP_RIGHT = (1, 1)   # 右上
# DPAD_DOWN_LEFT = (-1, -1) # 左下
# DPAD_DOWN_RIGHT = (1, -1) # 右下
# DPAD_CENTER = (0, 0)     # 中心

# 軸常量
# L_X_AXIS = 0    # 左搖桿 X 軸
# L_Y_AXIS = 1    # 左搖桿 Y 軸 
# R_X_AXIS = 2    # 右搖桿 X 軸
# R_Y_AXIS = 3    # 右搖桿 Y 軸
```

#### ~~按鍵映射 (基於實際硬體)~~ ⚠️ **已棄用**
```python
# ⚠️ 以下為已棄用的 Pygame 按鍵映射
# 搖桿按鍵映射 (get_button 索引)
# button_state = {
#     "a_button": joystick.get_button(0),      # A 按鍵
#     "b_button": joystick.get_button(1),      # B 按鍵
#     "x_button": joystick.get_button(2),      # X 按鍵
#     "y_button": joystick.get_button(3),      # Y 按鍵
#     "l1_button": joystick.get_button(4),     # L1 按鍵
#     "r1_button": joystick.get_button(5),     # R1 按鍵
#     "l2_button": joystick.get_button(6),     # L2 按鍵
#     "r2_button": joystick.get_button(7),     # R2 按鍵
#     "select_button": joystick.get_button(8), # Select 按鍵
#     "start_button": joystick.get_button(9),  # Start 按鍵
# }
```

### JoyHandler 類別設計 (ROS 2) ✅ **主要使用**

#### 按鍵常量 (sensor_msgs/Joy)
```python
# 按鍵索引常量
A_BUTTON = 0      # A 按鍵
B_BUTTON = 1      # B 按鍵
X_BUTTON = 2      # X 按鍵
Y_BUTTON = 3      # Y 按鍵
L1_BUTTON = 4     # L1 按鍵
R1_BUTTON = 5     # R1 按鍵
L2_BUTTON = 6     # L2 按鍵
R2_BUTTON = 7     # R2 按鍵
SELECT_BUTTON = 8 # Select 按鍵
START_BUTTON = 9  # Start 按鍵
```

#### 軸常量和 D-Pad 映射
```python
# 軸索引常量
L_X_AXIS = 0      # 左搖桿 X 軸
L_Y_AXIS = 1      # 左搖桿 Y 軸
R_X_AXIS = 2      # 右搖桿 X 軸
R_Y_AXIS = 3      # 右搖桿 Y 軸

# D-Pad 軸 (動態適配)
D_PAD_X_AXIS = 4  # D-Pad X 軸 (6軸模式)
D_PAD_Y_AXIS = 5  # D-Pad Y 軸 (6軸模式)
# 或
D_PAD_X_AXIS = 6  # D-Pad X 軸 (8軸模式)
D_PAD_Y_AXIS = 7  # D-Pad Y 軸 (8軸模式)
```

## 🛠️ 實際使用範例

### ~~Pygame 直接處理範例~~ ⚠️ **已棄用**
```python
# ⚠️ 以下為已棄用的 Pygame 使用範例
# from joystick.joystick_handler import JoystickHandler

# # 初始化搖桿
# JoystickHandler.init()

# # 註冊回調函數
# def on_button_press(button, action):
#     print(f"按鍵 {button} {action}")

# def on_axis_change(axis, value):
#     print(f"軸 {axis} 值: {value}")

# def on_dpad_move(hat_state):
#     print(f"D-Pad: x={hat_state[0]}, y={hat_state[1]}")

# # 註冊事件
# JoystickHandler.register_button_callback("a_button", on_button_press)
# JoystickHandler.register_axis_callback(JoystickHandler.L_X_AXIS, on_axis_change)
# JoystickHandler.register_dpad_callback(on_dpad_move)

# # 手動輪詢模式 (推薦)
# while True:
#     JoystickHandler._joystick_loop(0)  # 執行一次輪詢
#     time.sleep(0.05)  # 20Hz 輪詢率
```

### ROS 2 Joy 處理範例 ✅ **推薦使用**
```python
import rclpy
from rclpy.node import Node
from joystick.joy_handler import JoyHandler

class MyJoyNode(Node):
    def __init__(self):
        super().__init__('my_joy_node', namespace='agv')
        self.joy_handler = JoyHandler(self)
        
        # 註冊回調
        self.joy_handler.register_button_callback(
            JoyHandler.A_BUTTON, self.on_button)
        self.joy_handler.register_axis_callback(
            JoyHandler.L_X_AXIS, self.on_axis)
        
        # 開始訂閱
        self.joy_handler.start()
    
    def on_button(self, index, action):
        self.get_logger().info(f"按鍵 {index} {action}")
    
    def on_axis(self, index, value):
        self.get_logger().info(f"軸 {index} = {value}")
```

## 🚨 USB 搖桿處理專項故障排除

**⚠️ 通用故障排除請參考**: ../../CLAUDE.md 故障排除章節

### 搖桿設備問題
```bash
# 檢查搖桿設備
ls -la /dev/input/js*        # 檢查 js 設備節點
lsusb | grep -i joystick     # 檢查 USB 搖桿
dmesg | grep -i joystick     # 檢查內核訊息

# 權限檢查
groups                       # 檢查用戶群組
ls -la /dev/input/           # 檢查設備權限
```

### 容器設備掛載問題
```bash
# 檢查 docker-compose.yml 設備掛載
# AGV 容器需要:
# devices:
#   - "/dev/input:/dev/input"

# 重啟容器以載入設備
docker compose -f docker-compose.yml restart rosagv
```

### ROS 2 Joy 訊息問題
```bash
# 檢查 Joy 主題
ros2 topic list | grep joy
ros2 topic echo /agv/joy     # 檢查 Joy 訊息
ros2 node list | grep joy    # 檢查 joy_node

# 啟動標準 joy_node
ros2 run joy joy_node --ros-args --remap joy:=/agv/joy
```

### ~~Pygame 相關問題~~ ⚠️ **已棄用**
```bash
# ⚠️ 以下為已棄用的 Pygame 相關診斷
# 檢查 Pygame 安裝
# python3 -c "import pygame; print(pygame.version.ver)"

# 音頻驅動問題 (容器內常見)
# export SDL_AUDIODRIVER=dummy

# 顯示驅動問題
# export SDL_VIDEODRIVER=dummy
```

### 推薦使用標準 ROS 2 Joy
```bash
# ✅ 使用標準 ROS 2 joy_node
ros2 run joy joy_node --ros-args --remap joy:=/agv/joy

# 檢查 Joy 訊息
ros2 topic echo /agv/joy

# 檢查 Joy 節點狀態
ros2 node info /joy_node
```

## 📋 技術限制和注意事項

### 安裝配置限制
- **無 entry_points**: `setup.py` 中未定義 console_scripts，需直接執行 Python 檔案
- **開發測試專用**: 僅供開發測試使用，未包含生產環境安全機制
- **容器依賴**: 必須在 AGV 容器內執行，需要 `/dev/input` 設備掛載
- **⚠️ Pygame 已棄用**: Pygame 相關功能不再維護，建議使用標準 ROS 2 Joy

### 硬體相容性
- **USB 搖桿**: 支援標準 USB HID 搖桿設備 (透過 ROS 2 Joy)
- **按鍵映射**: 基於 ROS 2 Joy 標準按鍵配置
- **軸數量**: 支援 4-8 軸的搖桿設備 (透過 ROS 2 Joy)

### 效能考量
- **訊息頻率**: ROS 2 Joy 標準訊息頻率 (通常 10-50Hz)
- **CPU 使用**: ROS 2 訊息處理提供更好的系統整合
- **記憶體佔用**: 標準 ROS 2 節點記憶體使用
- **⚠️ 已棄用**: Pygame 直接處理的效能優勢不再適用

## 🔗 交叉引用

### 相關模組
- **AGV 手動指令**: `../agv_cmd_service_ws/CLAUDE.md` - 搖桿指令整合

### 通用支援
詳細指導請參考: ../../CLAUDE.md 交叉引用章節
- 技術棧: @docs-ai/context/system/technology-stack.md