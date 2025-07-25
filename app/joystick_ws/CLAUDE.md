# joystick_ws CLAUDE.md

## 模組概述
USB搖桿控制系統，為AGV提供手動遙控功能，支援緊急操作與調試模式

## 專案結構
```
src/
└── joystick/           # USB搖桿控制核心
    ├── joystick/       # 搖桿驅動邏輯
    ├── controllers/    # 控制器映射
    ├── safety/         # 安全機制
    └── calibration/    # 搖桿校準
```

## 核心功能

### 搖桿控制
- **設備檢測**: 自動檢測USB搖桿設備
- **按鍵映射**: 可配置的按鍵功能映射
- **類比搖桿**: 方向控制與速度調節
- **安全機制**: 死人開關與緊急停止

### 運動控制
- **手動駕駛**: 直接控制AGV運動
- **精確定位**: 慢速精確移動模式
- **旋轉控制**: 原地旋轉調整
- **速度限制**: 安全速度限制機制

## 開發指令

### 環境設定 (AGV容器內)
```bash
source /app/setup.bash && all_source
cd /app/joystick_ws
```

### 設備檢查
```bash
# 檢查USB搖桿設備
ls -la /dev/input/js*

# 測試搖桿輸入
jstest /dev/input/js0

# 查看搖桿設備信息
cat /proc/bus/input/devices | grep -A 5 -B 5 joystick
```

### 服務啟動
```bash
# 啟動搖桿控制節點
ros2 run joystick joystick_node

# 啟動搖桿校準工具
ros2 run joystick joystick_calibration

# 測試搖桿連線
ros2 run joystick test_joystick_connection
```

### 構建與測試
```bash
build_ws joystick_ws
ros2 test joystick  # 搖桿系統測試
```

## 搖桿驅動開發

### 設備驅動
```python
# joystick/joystick_driver.py
class JoystickDriver:
    def __init__(self, device_path="/dev/input/js0"):
        self.device_path = device_path
        self.device_fd = None
        self.axis_states = {}
        self.button_states = {}
        
    def open_device(self):
        """開啟搖桿設備"""
        try:
            self.device_fd = open(self.device_path, 'rb')
            self.get_device_info()
            return True
        except Exception as e:
            self.get_logger().error(f"無法開啟搖桿設備: {e}")
            return False
            
    def read_joystick_event(self):
        """讀取搖桿事件"""
        if not self.device_fd:
            return None
            
        try:
            event_data = self.device_fd.read(8)
            if len(event_data) == 8:
                return self.parse_joystick_event(event_data)
        except Exception as e:
            self.get_logger().warn(f"讀取搖桿事件錯誤: {e}")
        return None
```

### 控制器映射
```python
# controllers/controller_mapping.py
class ControllerMapping:
    def __init__(self, config_file):
        self.config = self.load_config(config_file)
        self.axis_mapping = self.config['axis_mapping']
        self.button_mapping = self.config['button_mapping']
        
    def map_axis_to_command(self, axis_id: int, value: float):
        """軸輸入映射到控制指令"""
        if axis_id in self.axis_mapping:
            mapping = self.axis_mapping[axis_id]
            command_type = mapping['command']
            
            # 應用死區和縮放
            processed_value = self.apply_deadzone(value, mapping['deadzone'])
            scaled_value = processed_value * mapping['scale']
            
            return {
                'command_type': command_type,
                'value': scaled_value
            }
        return None
        
    def map_button_to_action(self, button_id: int, pressed: bool):
        """按鍵映射到動作"""
        if button_id in self.button_mapping:
            mapping = self.button_mapping[button_id]
            if pressed == mapping.get('trigger_on_press', True):
                return {
                    'action_type': mapping['action'],
                    'parameters': mapping.get('parameters', {})
                }
        return None
```

### 運動控制整合
```python
# joystick/motion_controller.py
class JoystickMotionController:
    def __init__(self):
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.emergency_stop_publisher = self.create_publisher(Bool, '/emergency_stop', 1)
        self.current_velocity = Twist()
        
    def process_joystick_input(self, joystick_data):
        """處理搖桿輸入並轉換為運動指令"""
        # 線性速度控制 (左搖桿Y軸)
        if 'linear_x' in joystick_data:
            self.current_velocity.linear.x = joystick_data['linear_x'] * self.max_linear_speed
            
        # 角速度控制 (右搖桿X軸)  
        if 'angular_z' in joystick_data:
            self.current_velocity.angular.z = joystick_data['angular_z'] * self.max_angular_speed
            
        # 安全檢查
        if self.safety_check():
            self.velocity_publisher.publish(self.current_velocity)
        else:
            self.emergency_stop()
```

## 安全機制

### 死人開關
```python
# safety/deadman_switch.py
class DeadmanSwitch:
    def __init__(self, timeout=0.5):
        self.timeout = timeout
        self.last_heartbeat = time.time()
        self.is_active = False
        
    def update_heartbeat(self, button_pressed: bool):
        """更新死人開關狀態"""
        if button_pressed:
            self.last_heartbeat = time.time()
            self.is_active = True
        else:
            self.is_active = False
            
    def is_safety_ok(self):
        """檢查安全狀態"""
        current_time = time.time()
        if not self.is_active:
            return False
            
        if current_time - self.last_heartbeat > self.timeout:
            self.is_active = False
            return False
            
        return True
```

### 緊急停止
```python
# safety/emergency_stop.py
class EmergencyStopHandler:
    def __init__(self):
        self.emergency_active = False
        self.stop_publisher = self.create_publisher(EmergencyStop, '/emergency_stop', 1)
        
    def trigger_emergency_stop(self, reason: str):
        """觸發緊急停止"""
        self.emergency_active = True
        
        emergency_msg = EmergencyStop()
        emergency_msg.active = True
        emergency_msg.reason = reason
        emergency_msg.timestamp = self.get_clock().now().to_msg()
        
        self.stop_publisher.publish(emergency_msg)
        self.get_logger().error(f"緊急停止觸發: {reason}")
        
    def reset_emergency_stop(self):
        """重置緊急停止狀態"""
        if self.emergency_active:
            emergency_msg = EmergencyStop()
            emergency_msg.active = False
            emergency_msg.reason = "重置"
            emergency_msg.timestamp = self.get_clock().now().to_msg()
            
            self.stop_publisher.publish(emergency_msg)
            self.emergency_active = False
```

## 搖桿配置

### 控制器配置
```yaml
# /app/config/agv/joystick_config.yaml
joystick:
  device_path: "/dev/input/js0"
  update_rate: 50  # Hz
  
  # 軸映射配置
  axis_mapping:
    0:  # 左搖桿X軸
      command: "linear_y"
      scale: 1.0
      deadzone: 0.1
      invert: false
      
    1:  # 左搖桿Y軸  
      command: "linear_x"
      scale: 1.0
      deadzone: 0.1
      invert: true
      
    3:  # 右搖桿X軸
      command: "angular_z" 
      scale: 1.0
      deadzone: 0.15
      invert: false
      
  # 按鍵映射配置
  button_mapping:
    0:  # A按鍵 - 死人開關
      action: "deadman_switch"
      trigger_on_press: true
      
    1:  # B按鍵 - 緊急停止
      action: "emergency_stop"
      trigger_on_press: true
      
    2:  # X按鍵 - 慢速模式
      action: "slow_mode"
      trigger_on_press: true
      parameters:
        speed_factor: 0.3
        
    3:  # Y按鍵 - 重置
      action: "reset_emergency"
      trigger_on_press: true
      
  # 速度限制
  speed_limits:
    max_linear_speed: 2.0    # m/s
    max_angular_speed: 1.57  # rad/s
    slow_mode_factor: 0.3
    
  # 安全設定
  safety:
    deadman_timeout: 0.5     # 秒
    heartbeat_rate: 20       # Hz
    enable_emergency_stop: true
```

### 搖桿校準
```python
# calibration/joystick_calibration.py
class JoystickCalibration:
    def __init__(self):
        self.calibration_data = {}
        self.is_calibrating = False
        
    def start_calibration(self):
        """開始搖桿校準程序"""
        self.is_calibrating = True
        self.calibration_data = {
            'axis_min': {},
            'axis_max': {},
            'axis_center': {}
        }
        
    def calibrate_axis(self, axis_id: int, value: float):
        """校準指定軸"""
        if axis_id not in self.calibration_data['axis_min']:
            self.calibration_data['axis_min'][axis_id] = value
            self.calibration_data['axis_max'][axis_id] = value
        else:
            self.calibration_data['axis_min'][axis_id] = min(
                self.calibration_data['axis_min'][axis_id], value
            )
            self.calibration_data['axis_max'][axis_id] = max(
                self.calibration_data['axis_max'][axis_id], value
            )
```

## 測試與調試

### 搖桿測試
```bash
# 測試搖桿輸入
ros2 topic echo /joystick/raw_input

# 測試運動指令輸出
ros2 topic echo /cmd_vel

# 測試安全系統
ros2 topic echo /emergency_stop

# 搖桿診斷
ros2 run joystick joystick_diagnostics
```

### 校準程序
```bash
# 開始搖桿校準
ros2 service call /joystick/start_calibration

# 保存校準數據
ros2 service call /joystick/save_calibration

# 載入校準數據
ros2 service call /joystick/load_calibration
```

## 故障排除

### 常見問題
1. **搖桿無法檢測**: 檢查USB連線與設備權限
2. **輸入延遲**: 調整更新頻率與系統負載
3. **按鍵映射錯誤**: 驗證配置文件設定
4. **死人開關失效**: 檢查按鍵狀態與超時設定

### 診斷工具
```bash
# 檢查搖桿設備
lsusb | grep -i joystick

# 測試設備權限
ls -la /dev/input/js*

# 檢查輸入事件
evtest /dev/input/event*

# 搖桿設備信息
ros2 run joystick get_device_info
```

### 設備權限
```bash
# 添加使用者到input群組
sudo usermod -a -G input $USER

# 設定設備權限規則
echo 'SUBSYSTEM=="input", GROUP="input", MODE="0664"' | sudo tee /etc/udev/rules.d/99-input.rules

# 重新載入udev規則
sudo udevadm control --reload-rules
```

## 安全注意事項

### 操作安全
- **死人開關**: 必須持續按住死人開關才能控制
- **速度限制**: 嚴格的最大速度限制
- **緊急停止**: 一鍵緊急停止功能
- **超時保護**: 通訊中斷自動停止

### 系統安全
- **權限控制**: 適當的設備存取權限
- **輸入驗證**: 搖桿輸入數值範圍檢查
- **故障檢測**: 搖桿設備故障自動檢測
- **日誌記錄**: 詳細的操作日誌記錄

## 重要提醒
- 搖桿控制僅限AGV車載系統使用
- 安全機制不可繞過或禁用
- 校準數據需定期檢查更新
- 手動控制時需特別注意周圍環境安全