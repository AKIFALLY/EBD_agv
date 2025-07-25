# CLAUDE.md

## 系統概述
USB搖桿輸入處理工作空間，提供兩種搖桿處理方式：pygame直接處理和ROS 2 Joy訊息處理。

**🎮 雙重處理架構**: Pygame直接讀取 + ROS 2 Joy訊息訂閱

## 核心架構
```
joystick_ws/
└── joystick/                    # 搖桿處理核心
    ├── joystick_handler.py      # Pygame搖桿直接處理
    ├── joy_handler.py           # ROS 2 Joy訊息處理
    ├── joystick_test_node.py    # 測試節點
    └── __init__.py
```

## 主要組件

### 1. JoystickHandler類別 (joystick_handler.py)
**Pygame基礎搖桿處理**，直接讀取USB搖桿設備:
```python
class JoystickHandler:
    # 軸常量定義
    L_X_AXIS = 0    # 左搖桿X軸
    L_Y_AXIS = 1    # 左搖桿Y軸  
    R_X_AXIS = 2    # 右搖桿X軸
    R_Y_AXIS = 3    # 右搖桿Y軸
    
    # D-Pad常量定義
    DPAD_UP = (0, 1)
    DPAD_DOWN = (0, -1)
    DPAD_LEFT = (-1, 0)
    DPAD_RIGHT = (1, 0)
    DPAD_CENTER = (0, 0)
```

**核心功能**:
```python
@classmethod
def init(cls):
    """初始化 Pygame 和搖桿"""
    os.environ["SDL_AUDIODRIVER"] = "dummy"  # 避免音訊驅動問題
    pygame.init()
    pygame.joystick.init()
    
@classmethod
def register_button_callback(cls, button, callback):
    """註冊按鈕事件回調"""
    cls._callbacks[button] = callback
    
@classmethod  
def register_axis_callback(cls, axis, callback):
    """註冊軸事件回調"""
    cls._axis_callbacks[axis] = callback
```

**按鈕映射**:
```python
button_state = {
    "a_button": cls._joystick.get_button(0),      # A按鍵
    "b_button": cls._joystick.get_button(1),      # B按鍵  
    "x_button": cls._joystick.get_button(2),      # X按鍵
    "y_button": cls._joystick.get_button(3),      # Y按鍵
    "l1_button": cls._joystick.get_button(4),     # L1按鍵
    "r1_button": cls._joystick.get_button(5),     # R1按鍵
    "l2_button": cls._joystick.get_button(6),     # L2按鍵
    "r2_button": cls._joystick.get_button(7),     # R2按鍵
    "select_button": cls._joystick.get_button(8), # Select按鍵
    "start_button": cls._joystick.get_button(9),  # Start按鍵
}
```

### 2. JoyHandler類別 (joy_handler.py)  
**ROS 2 Joy訊息處理**，訂閱標準Joy訊息:
```python
class JoyHandler:
    # 按鈕常量
    A_BUTTON = 0
    B_BUTTON = 1
    X_BUTTON = 2
    Y_BUTTON = 3
    L1_BUTTON = 4
    R1_BUTTON = 5
    L2_BUTTON = 6
    R2_BUTTON = 7
    SELECT_BUTTON = 8
    START_BUTTON = 9
    
    # 軸常量
    L_X_AXIS = 0
    L_Y_AXIS = 1
    R_X_AXIS = 2
    R_Y_AXIS = 3
    D_PAD_X_AXIS = 4  # D-Pad X軸
    D_PAD_Y_AXIS = 5  # D-Pad Y軸
```

**ROS 2整合**:
```python
def __init__(self, node: Node):
    self.node = node
    self.namespace = node.get_namespace()
    self._button_callbacks = {}
    self._axis_callbacks = {}
    
def start(self):
    topic = f"{self.namespace}/joy"
    self.subscription = self.node.create_subscription(
        Joy, topic, self.joy_callback, 10)
```

**Joy訊息處理**:
```python
def joy_callback(self, msg: Joy):
    # 按鈕狀態變化檢測
    for i, state in enumerate(msg.buttons):
        if state != self._prev_buttons[i]:
            action = 'pressed' if state else 'released'
            if i in self._button_callbacks:
                self._button_callbacks[i](i, action)
    
    # 軸值變化檢測 (閾值0.95)
    for i, value in enumerate(msg.axes):
        value = round(value, 2)
        if abs(value - self._prev_axes[i]) > 0.95:
            if i in self._axis_callbacks:
                self._axis_callbacks[i](i, value)
```

### 3. JoystickTestNode類別 (joystick_test_node.py)
**搖桿測試節點**，整合pygame處理器:
```python
class JoystickTestNode(Node):
    def __init__(self):
        super().__init__('joystick_test_node')
        
        # 初始化搖桿處理器
        JoystickHandler.init()
        
        # 註冊所有按鍵和軸的回調
        JoystickHandler.register_button_callback("a_button", self.button_event)
        JoystickHandler.register_axis_callback(JoystickHandler.L_X_AXIS, self.axis_event)
        
        # 使用定時器代替線程處理
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz
        
    def timer_callback(self):
        JoystickHandler._joystick_loop(0)  # 呼叫搖桿讀取循環
```

## 開發指令

### 環境設定 (AGV容器內執行)
```bash
# AGV容器內
source /app/setup.bash && agv_source  # 或使用 all_source (自動檢測)
cd /app/joystick_ws
```

### 構建與測試
```bash
build_ws joystick_ws
```

### 設備檢查 (AGV容器內)
```bash
# 檢查USB搖桿設備
ls -la /dev/input/js*

# 測試搖桿輸入 (如果有jstest工具)
jstest /dev/input/js0

# 查看輸入設備
cat /proc/bus/input/devices | grep -A 5 -B 5 joystick
```

### 節點啟動 (AGV容器內執行)
```bash
# 注意：setup.py 未定義 console_scripts，需要直接執行
python3 /app/joystick_ws/src/joystick/joystick/joystick_test_node.py

# 或在joystick目錄下執行
cd /app/joystick_ws/src/joystick/joystick
python3 joystick_test_node.py
```

## 使用範例

### 1. 使用Pygame搖桿處理器
```python
from joystick.joystick_handler import JoystickHandler

def button_callback(button, action):
    print(f"🎮 {button} {action}")

def axis_callback(axis, value):
    print(f"🕹️ 軸 {axis}: {value}")

# 初始化搖桿
JoystickHandler.init()

# 註冊回調
JoystickHandler.register_button_callback("a_button", button_callback)
JoystickHandler.register_axis_callback(JoystickHandler.L_X_AXIS, axis_callback)

# 手動調用讀取循環 (通常在ROS 2 timer中)
JoystickHandler._joystick_loop(0)
```

### 2. 使用ROS 2 Joy處理器
```python
import rclpy
from rclpy.node import Node
from joystick.joy_handler import JoyHandler

class MyJoyNode(Node):
    def __init__(self):
        super().__init__('my_joy_node')
        self.joy_handler = JoyHandler(self)
        
        # 註冊回調
        self.joy_handler.register_button_callback(0, self.on_button)  # A按鍵
        self.joy_handler.register_axis_callback(0, self.on_axis)      # 左搖桿X軸
        
        # 開始訂閱Joy訊息
        self.joy_handler.start()
        
    def on_button(self, index, action):
        self.get_logger().info(f"Button {index} {action}")
        
    def on_axis(self, index, value):
        self.get_logger().info(f"Axis {index} = {value}")
```

### 3. 測試搖桿功能
```bash
# 執行測試節點 (AGV容器內)
cd /app/joystick_ws/src/joystick/joystick
python3 joystick_test_node.py

# 或使用ROS 2啟動 (需要安裝joy套件)
ros2 run joy joy_node --ros-args --remap __ns:=/agv
```

## 故障排除

### 常見問題
1. **搖桿無法檢測**: 檢查USB連線與設備權限
   ```bash
   ls -la /dev/input/js*
   # 如果沒有設備，檢查USB連接
   lsusb | grep -i joystick
   ```

2. **權限錯誤**: 確保容器有USB設備存取權限
   ```bash
   # 檢查docker-compose.yml是否掛載 /dev/input
   # devices:
   #   - "/dev/input:/dev/input"
   ```

3. **Pygame初始化失敗**: 音訊驅動問題
   ```python
   # 程式碼已設定
   os.environ["SDL_AUDIODRIVER"] = "dummy"
   ```

4. **Joy訊息未收到**: 檢查joy_node是否運行
   ```bash
   ros2 topic list | grep joy
   ros2 topic echo /agv/joy  # 檢查Joy訊息
   ```

### 診斷步驟
```bash
# 1. 檢查搖桿硬體
lsusb | grep -i joystick
ls -la /dev/input/js*

# 2. 測試pygame讀取
cd /app/joystick_ws/src/joystick/joystick
python3 joystick_handler.py  # 直接執行測試

# 3. 檢查ROS 2 Joy節點
ros2 node list | grep joy
ros2 topic list | grep joy

# 4. 測試ROS 2節點
python3 joystick_test_node.py
```

## 技術限制

### 當前實現限制
- **無entry_points**: setup.py未定義console_scripts，無法使用`ros2 run`
- **無配置文件**: 沒有yaml配置檔案支援
- **無安全機制**: 沒有死人開關或緊急停止功能
- **無校準功能**: 沒有搖桿校準服務
- **簡單回調**: 基本的按鍵和軸值回調系統

### 使用建議
- **開發測試**: 適合基本搖桿輸入測試
- **原型驗證**: 驗證搖桿硬體和基本功能
- **學習範例**: 了解pygame搖桿處理和ROS 2整合

## 重要提醒
- joystick_ws僅提供基本搖桿輸入處理
- 適用於AGV車載系統的簡單手動控制測試
- 無複雜安全機制，僅供開發和測試使用
- 實際的安全控制需要在上層應用中實現
- 所有測試必須在AGV容器內執行 (需要/dev/input設備掛載)