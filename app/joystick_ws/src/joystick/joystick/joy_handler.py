import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import time
import time


class JoyHandler:

    # 定義按鈕常量
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

    # 定義軸常量
    L_X_AXIS = 0
    L_Y_AXIS = 1
    R_X_AXIS = 2
    R_Y_AXIS = 3
    D_PAD_X_AXIS = 4
    D_PAD_Y_AXIS = 5

    # 定義搖桿DPAD常量,(axisIdx, direction)
    DPAD_UP = (5, 1)
    DPAD_DOWN = (5, -1)
    DPAD_LEFT = (4, 1)
    DPAD_RIGHT = (4, -1)
    DPAD_CENTER = (0, 0)

    def __init__(self, node: Node):
        self.node = node
        self.namespace = node.get_namespace()
        self.subscription = None

        self._button_callbacks = {}
        self._axis_callbacks = {}
        self._prev_buttons = []
        self._prev_axes = []
        self._prev_axes_int = []  # 內部整數狀態追蹤 (-1, 0, 1)
        self._prev_axes_int = []  # 內部整數狀態追蹤 (-1, 0, 1)
        self.d_pad_x_axis = 4
        self.d_pad_y_axis = 5
        
        # 搖桿檢測狀態
        self._axes_count_detected = False
        
        # 搖桿變化回調
        self._joystick_change_callback = None
        
        # 斷線檢測相關
        self._connected = False              # 連接狀態
        self._last_message_time = None       # 最後收到訊息的時間
        self._disconnect_timeout = 2.0       # 斷線超時（秒）
        self._disconnect_timer = None        # 斷線檢查定時器
        
        self.node.get_logger().info("✅ JoyHandlerNode 啟動完成")

    def _internal_quantize(self, raw_value):
        """內部三狀態量化：將原始軸值轉換為 -1, 0, 1"""
        if raw_value > 0.5:
            return 1
        elif raw_value < -0.5:
            return -1
        else:
            return 0

    def _int_to_float(self, int_state):
        """轉換為外部期望的浮點值：-1→-1.0, 0→0.0, 1→1.0"""
        return float(int_state)

    def _detect_joystick_type(self, axes_count):
        """首次檢測搖桿類型並設定軸索引"""
        if not self._axes_count_detected:
            # 1. 設定新的軸索引
            if axes_count == 6:
                JoyHandler.D_PAD_X_AXIS = 4
                JoyHandler.D_PAD_Y_AXIS = 5
                joystick_type = "6軸搖桿"
            else:  # 8軸或其他
                JoyHandler.D_PAD_X_AXIS = 6
                JoyHandler.D_PAD_Y_AXIS = 7
                joystick_type = f"{axes_count}軸搖桿"
            
            # 2. 完全重置狀態陣列（關鍵！）  
            self._prev_axes_int = [0] * axes_count
            self._prev_axes = [0.0] * axes_count
            
            # 3. 標記檢測完成
            self._axes_count_detected = True
            
            # 4. 記錄變化
            self.node.get_logger().info(
                f"🎮 檢測到{joystick_type}，D-PAD軸: X={JoyHandler.D_PAD_X_AXIS}, Y={JoyHandler.D_PAD_Y_AXIS}"
            )
            self.node.get_logger().info(f"🔄 搖桿狀態已重置，陣列大小: {axes_count}")
            
            # 5. 觸發變化回調（通知 manual_state.py）
            if self._joystick_change_callback:
                self._joystick_change_callback(axes_count, JoyHandler.D_PAD_X_AXIS, JoyHandler.D_PAD_Y_AXIS)

    def _detect_sign_change(self, axis_index, prev_int, current_int):
        """檢測正負切換（1→-1 或 -1→1）並插入停止信號"""
        if (prev_int == 1 and current_int == -1) or (prev_int == -1 and current_int == 1):
            self.node.get_logger().warn(f"⚠️ 軸{axis_index}正負切換，插入停止信號")
            # 先發送0.0停止信號給外部回調
            if axis_index in self._axis_callbacks:
                self._axis_callbacks[axis_index](axis_index, 0.0)
            return True
        return False

    def _start_disconnect_timer(self):
        """啟動斷線檢測定時器"""
        if self._disconnect_timer:
            self.node.destroy_timer(self._disconnect_timer)
        
        # 創建定時器，每0.5秒檢查一次
        self._disconnect_timer = self.node.create_timer(0.5, self._check_disconnect)

    def _check_disconnect(self):
        """檢查搖桿是否斷線"""
        if not self._connected:
            return  # 已經斷線，無需重複檢查
            
        current_time = time.time()
        if (self._last_message_time and 
            current_time - self._last_message_time > self._disconnect_timeout):
            self._handle_disconnect()

    def _handle_disconnect(self):
        """處理搖桿斷線"""
        if not self._connected:
            return  # 已經處理過斷線
            
        self.node.get_logger().warn("⚠️ 搖桿斷線檢測！發送安全停止信號")
        self._connected = False
        
        # 發送所有軸的 0.0 停止信號（先複製字典避免迭代時修改）
        axis_callbacks_copy = dict(self._axis_callbacks)
        for axis_index, callback in axis_callbacks_copy.items():
            if callback:  # 檢查回調是否存在
                callback(axis_index, 0.0)
        
        # 重置所有內部狀態為0
        for i in range(len(self._prev_axes_int)):
            self._prev_axes_int[i] = 0
            self._prev_axes[i] = 0.0
        
        self.node.get_logger().info("🔄 搖桿狀態已重置為安全狀態")

    def _handle_reconnect(self):
        """處理搖桿重連"""
        if self._connected:
            return  # 已經連接
            
        self.node.get_logger().info("🔌 搖桿重新連接檢測")
        self._connected = True
        
        # 重新檢測搖桿類型（重置檢測狀態）
        self._axes_count_detected = False

    def joy_callback(self, msg: Joy):
        # 更新連接狀態和時間戳
        current_time = time.time()
        self._last_message_time = current_time
        
        # 處理重連
        if not self._connected:
            self._handle_reconnect()
        
        # 首次檢測搖桿類型
        if not self._axes_count_detected:
            self._detect_joystick_type(len(msg.axes))
        
        # 安全補長所有內部陣列
        # 更新連接狀態和時間戳
        current_time = time.time()
        self._last_message_time = current_time
        
        # 處理重連
        if not self._connected:
            self._handle_reconnect()
        
        # 首次檢測搖桿類型
        if not self._axes_count_detected:
            self._detect_joystick_type(len(msg.axes))
        
        # 安全補長所有內部陣列
        if len(self._prev_buttons) < len(msg.buttons):
            self._prev_buttons.extend(
                [0] * (len(msg.buttons) - len(self._prev_buttons)))
        
        
        if len(self._prev_axes) < len(msg.axes):
            self._prev_axes.extend(
                [0.0] * (len(msg.axes) - len(self._prev_axes)))
        
        # 補長內部整數陣列
        while len(self._prev_axes_int) < len(msg.axes):
            self._prev_axes_int.append(0)

        try:
            # 處理按鈕事件（保持原有邏輯）  
        
        # 補長內部整數陣列
        while len(self._prev_axes_int) < len(msg.axes):
            self._prev_axes_int.append(0)

        try:
            # 處理按鈕事件（保持原有邏輯）  
            for i, state in enumerate(msg.buttons):
                if state != self._prev_buttons[i]:
                    self._prev_buttons[i] = state
                    action = 'pressed' if state else 'released'
                    if i in self._button_callbacks:
                        self._button_callbacks[i](i, action)

            # 處理軸事件（使用新的三狀態邏輯）
            for i, raw_value in enumerate(msg.axes):
                # 1. 內部三狀態量化
                current_int = self._internal_quantize(raw_value)
                
                # 2. 正負切換安全檢測
                if self._detect_sign_change(i, self._prev_axes_int[i], current_int):
                    # 已發送停止信號，更新內部狀態為0並繼續處理下一個軸
                    self._prev_axes_int[i] = 0
                    continue
                
                # 3. 簡單整數比較檢測變化（核心優化）
                if current_int != self._prev_axes_int[i]:
                    # 更新內部整數狀態
                    self._prev_axes_int[i] = current_int
                    
                    # 轉換為外部期望的浮點值並發送回調
                    external_float = self._int_to_float(current_int)
                    
            # 處理軸事件（使用新的三狀態邏輯）
            for i, raw_value in enumerate(msg.axes):
                # 1. 內部三狀態量化
                current_int = self._internal_quantize(raw_value)
                
                # 2. 正負切換安全檢測
                if self._detect_sign_change(i, self._prev_axes_int[i], current_int):
                    # 已發送停止信號，更新內部狀態為0並繼續處理下一個軸
                    self._prev_axes_int[i] = 0
                    continue
                
                # 3. 簡單整數比較檢測變化（核心優化）
                if current_int != self._prev_axes_int[i]:
                    # 更新內部整數狀態
                    self._prev_axes_int[i] = current_int
                    
                    # 轉換為外部期望的浮點值並發送回調
                    external_float = self._int_to_float(current_int)
                    
                    if i in self._axis_callbacks:
                        self._axis_callbacks[i](i, external_float)
                    
                    # 同步更新舊的浮點陣列（為了兼容性）
                    self._prev_axes[i] = external_float
                    
                        self._axis_callbacks[i](i, external_float)
                    
                    # 同步更新舊的浮點陣列（為了兼容性）
                    self._prev_axes[i] = external_float
                    
        except Exception as e:
            self.node.get_logger().error(f"Joy Callback Error: {e}")

    def register_button_callback(self, index: int, callback):
        self._button_callbacks[index] = callback

    def register_axis_callback(self, index: int, callback):
        self._axis_callbacks[index] = callback

    def register_joystick_change_callback(self, callback):
        """註冊搖桿類型變化回調"""
        self._joystick_change_callback = callback

    def is_connected(self):
        """檢查搖桿是否連接"""
        return self._connected

    def clear_all_callbacks(self):
        """清空所有回調註冊（不影響訂閱）"""
        self._button_callbacks.clear()
        self._axis_callbacks.clear()
        self.node.get_logger().info("🧹 已清空所有搖桿回調")

    def register_joystick_change_callback(self, callback):
        """註冊搖桿類型變化回調"""
        self._joystick_change_callback = callback

    def is_connected(self):
        """檢查搖桿是否連接"""
        return self._connected

    def clear_all_callbacks(self):
        """清空所有回調註冊（不影響訂閱）"""
        self._button_callbacks.clear()
        self._axis_callbacks.clear()
        self.node.get_logger().info("🧹 已清空所有搖桿回調")

    def start(self):
        topic = f"{self.namespace}/joy"
        self.subscription = self.node.create_subscription(
            Joy, topic, self.joy_callback, 10)
        self.node.get_logger().info(f"🔌 訂閱搖桿 Topic: {topic}")
        
        # 啟動斷線檢測定時器
        self._start_disconnect_timer()
        self.node.get_logger().info("⏰ 搖桿斷線檢測定時器已啟動")
        
        # 啟動斷線檢測定時器
        self._start_disconnect_timer()
        self.node.get_logger().info("⏰ 搖桿斷線檢測定時器已啟動")

    def unregister_all_callbacks(self):
        if self.subscription:
            self.node.destroy_subscription(self.subscription)
            self.subscription = None
        self._button_callbacks.clear()
        self._axis_callbacks.clear()

    def stop(self):
        self.unregister_all_callbacks()
        
        # 停止斷線檢測定時器
        if self._disconnect_timer:
            self.node.destroy_timer(self._disconnect_timer)
            self._disconnect_timer = None
        
        # 重置連接狀態
        self._connected = False
        self._last_message_time = None
        
        # 清理所有狀態
        
        # 停止斷線檢測定時器
        if self._disconnect_timer:
            self.node.destroy_timer(self._disconnect_timer)
            self._disconnect_timer = None
        
        # 重置連接狀態
        self._connected = False
        self._last_message_time = None
        
        # 清理所有狀態
        self._prev_buttons.clear()
        self._prev_axes.clear()
        self._prev_axes_int.clear()  # 清理內部整數狀態
        self._axes_count_detected = False  # 重置搖桿檢測狀態
        self._joystick_change_callback = None  # 清理搖桿變化回調
        
        self.node.get_logger().info("⏹️ 搖桿斷線檢測定時器已停止")
        self._prev_axes_int.clear()  # 清理內部整數狀態
        self._axes_count_detected = False  # 重置搖桿檢測狀態
        self._joystick_change_callback = None  # 清理搖桿變化回調
        
        self.node.get_logger().info("⏹️ 搖桿斷線檢測定時器已停止")


def main():
    rclpy.init()
    ros_node = Node('joy_handler', namespace='agvc')
    joy_handler = JoyHandler(ros_node)

    def on_button(index, action):
        ros_node.get_logger().info(f"🎮 Button {index} {action}")

    def on_axis(index, value):
        ros_node.get_logger().info(f"🕹️ Axis {index} = {value}")

    joy_handler.register_button_callback(0, on_button)  # A
    joy_handler.register_button_callback(1, on_button)  # B
    joy_handler.register_button_callback(2, on_button)  # X
    joy_handler.register_button_callback(3, on_button)  # Y
    joy_handler.register_button_callback(4, on_button)  # L1
    joy_handler.register_button_callback(5, on_button)  # R1
    joy_handler.register_button_callback(6, on_button)  # L2
    joy_handler.register_button_callback(7, on_button)  # R2
    joy_handler.register_button_callback(8, on_button)  # SELECT
    joy_handler.register_button_callback(9, on_button)  # START

    joy_handler.register_axis_callback(0, on_axis)  # L_X_AXIS
    joy_handler.register_axis_callback(1, on_axis)  # L_Y_AXIS
    joy_handler.register_axis_callback(2, on_axis)  # R_X_AXIS
    joy_handler.register_axis_callback(3, on_axis)  # R_Y_AXIS
    joy_handler.register_axis_callback(4, on_axis)  # R_X_AXIS D-PAD X 左正右負
    joy_handler.register_axis_callback(5, on_axis)  # R_Y_AXIS D-PAD Y 上正下負

    joy_handler.start()

    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass

    joy_handler.stop()
    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
