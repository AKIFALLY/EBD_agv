from agv_base.states.state import State
import rclpy
from rclpy.node import Node
from joystick.joy_handler import JoyHandler
from agv_cmd_service.agv_cmd_porxy import AGVCommandProxy
from agv_base.agv_status import AgvStatus
import time

DIRECTION_NONE = 0
DIRECTION_FORWARD = 1
DIRECTION_BACKWARD = 2
DIRECTION_RIGHT = 3
DIRECTION_LEFT = 4


class ManualState(State):
    def __init__(self, node: Node):
        super().__init__(node)
        # AGV 命令客戶端
        self.node = node
        self.joy_handler = None
        self.cmd_client = None

        # 初始化搖桿 現在方向
        self.direction = DIRECTION_NONE

        self.to_idle_state = False
        
        # 搖桿連接狀態管理
        self._last_joystick_connected = True  # 上次連接狀態
        self._connection_check_count = 0      # 連接檢查計數器

        # 按鈕功能對應表
        self.button_command_map = {
            # Example: No command for A button pressed
            (JoyHandler.A_BUTTON, "pressed"): (None, None),
            (JoyHandler.B_BUTTON, "pressed"): ("rotate_right", True),
            (JoyHandler.X_BUTTON, "pressed"): ("break", True),
            (JoyHandler.Y_BUTTON, "pressed"): ("rotate_left", True),
            (JoyHandler.L1_BUTTON, "pressed"): ("enable", True),


            # Example: No command for A button released
            (JoyHandler.A_BUTTON, "released"): (None, None),
            (JoyHandler.B_BUTTON, "released"): ("rotate_right", False),
            (JoyHandler.X_BUTTON, "released"): ("break", False),
            (JoyHandler.Y_BUTTON, "released"): ("rotate_left", False),
            (JoyHandler.L1_BUTTON, "released"): ("enable", False),

        }

    def _build_axis_state_machine(self):
        """動態建構軸狀態機，使用當前的D-PAD軸索引"""
        # 方向狀態機 + DPAD狀態機(joy linux 中的d-pad 也是 axis)
        self.axis_state_machine = {
            # 左搖桿軸控制 (固定軸索引)
            (DIRECTION_NONE, JoyHandler.L_Y_AXIS, 1.0): (DIRECTION_FORWARD, "forward", True),
            (DIRECTION_NONE, JoyHandler.L_Y_AXIS, -1.0):  (DIRECTION_BACKWARD, "backward", True),
            (DIRECTION_NONE, JoyHandler.L_X_AXIS, -1.0):  (DIRECTION_RIGHT, "shift_right", True),
            (DIRECTION_NONE, JoyHandler.L_X_AXIS, 1.0): (DIRECTION_LEFT, "shift_left", True),

            (DIRECTION_FORWARD, JoyHandler.L_Y_AXIS, 0.0): (DIRECTION_NONE, "forward", False),
            (DIRECTION_BACKWARD, JoyHandler.L_Y_AXIS, 0.0): (DIRECTION_NONE, "backward", False),
            (DIRECTION_RIGHT, JoyHandler.L_X_AXIS, 0.0): (DIRECTION_NONE, "shift_right", False),
            (DIRECTION_LEFT, JoyHandler.L_X_AXIS, 0.0): (DIRECTION_NONE, "shift_left", False),

            # D-PAD軸控制 (動態軸索引 - 6軸時為4,5；8軸時為6,7)
            (DIRECTION_NONE, JoyHandler.D_PAD_Y_AXIS, 1.0): (DIRECTION_FORWARD, "forward", True),
            (DIRECTION_NONE, JoyHandler.D_PAD_Y_AXIS, -1.0):  (DIRECTION_BACKWARD, "backward", True),
            (DIRECTION_NONE, JoyHandler.D_PAD_X_AXIS, -1.0):  (DIRECTION_RIGHT, "shift_right", True),
            (DIRECTION_NONE, JoyHandler.D_PAD_X_AXIS, 1.0): (DIRECTION_LEFT, "shift_left", True),

            (DIRECTION_FORWARD, JoyHandler.D_PAD_Y_AXIS, 0.0): (DIRECTION_NONE, "forward", False),
            (DIRECTION_BACKWARD, JoyHandler.D_PAD_Y_AXIS, 0.0): (DIRECTION_NONE, "backward", False),
            (DIRECTION_RIGHT, JoyHandler.D_PAD_X_AXIS, 0.0): (DIRECTION_NONE, "shift_right", False),
            (DIRECTION_LEFT, JoyHandler.D_PAD_X_AXIS, 0.0): (DIRECTION_NONE, "shift_left", False),
        }
        
        self.node.get_logger().info(
            f"🔧 狀態機已建構: D-PAD軸索引 X={JoyHandler.D_PAD_X_AXIS}, Y={JoyHandler.D_PAD_Y_AXIS}"
        )

    def enter(self):
        """進入 Manual 狀態，註冊搖桿按鈕與軸事件"""
        # 重置方向狀態
        self.direction = DIRECTION_NONE
        
        self.node.get_logger().info("🎮 進入 Manual 狀態，等待搖桿手動移動指令")

        self.cmd_client = AGVCommandProxy(self.node)

        self.joy_handler = JoyHandler(self.node)
        
        # 註冊搖桿變化監聽
        self.joy_handler.register_joystick_change_callback(self.on_joystick_change)
        
        # 啟動搖桿監聽
        self.joy_handler.start()
        
        # 注意：按鈕、軸回調和狀態機都會在收到第一個Joy訊息時自動註冊/建構（透過on_joystick_change回調）

    def _register_button_callbacks(self):
        """統一的按鈕回調註冊方法"""
        self.joy_handler.register_button_callback(JoyHandler.A_BUTTON, self.on_button_event)
        self.joy_handler.register_button_callback(JoyHandler.B_BUTTON, self.on_button_event)
        self.joy_handler.register_button_callback(JoyHandler.X_BUTTON, self.on_button_event)
        self.joy_handler.register_button_callback(JoyHandler.Y_BUTTON, self.on_button_event)
        self.joy_handler.register_button_callback(JoyHandler.L1_BUTTON, self.on_button_event)
        self.joy_handler.register_button_callback(JoyHandler.R1_BUTTON, self.on_button_event)
        self.joy_handler.register_button_callback(JoyHandler.L2_BUTTON, self.on_button_event)
        self.joy_handler.register_button_callback(JoyHandler.R2_BUTTON, self.on_button_event)
        self.joy_handler.register_button_callback(JoyHandler.SELECT_BUTTON, self.on_button_event)
        self.joy_handler.register_button_callback(JoyHandler.START_BUTTON, self.on_button_event)

    def _register_axis_callbacks(self):
        """統一的軸回調註冊方法（可重複呼叫）"""
        self.joy_handler.register_axis_callback(JoyHandler.L_X_AXIS, self.on_axis_event)
        self.joy_handler.register_axis_callback(JoyHandler.L_Y_AXIS, self.on_axis_event)
        self.joy_handler.register_axis_callback(JoyHandler.R_X_AXIS, self.on_axis_event)
        self.joy_handler.register_axis_callback(JoyHandler.R_Y_AXIS, self.on_axis_event)
        self.joy_handler.register_axis_callback(JoyHandler.D_PAD_X_AXIS, self.on_axis_event)
        self.joy_handler.register_axis_callback(JoyHandler.D_PAD_Y_AXIS, self.on_axis_event)

    def on_joystick_change(self, axes_count, d_pad_x, d_pad_y):
        """搖桿類型變化時重新註冊所有事件回調"""
        self.node.get_logger().info(f"🎮 搖桿變化通知: {axes_count}軸，D-PAD軸: X={d_pad_x}, Y={d_pad_y}")
        
        # 1. 先清空所有舊的回調註冊
        if self.joy_handler:
            self.joy_handler.clear_all_callbacks()
        
        # 2. 重新註冊所有按鈕回調
        self._register_button_callbacks()
        
        # 3. 重新建構軸狀態機（使用新的D-PAD軸索引）
        self._build_axis_state_machine()
        
        # 4. 重新註冊所有軸回調（使用新的D-PAD軸索引）
        self._register_axis_callbacks()
        
        # 5. 重置方向狀態（確保狀態一致性）
        self.direction = DIRECTION_NONE
        
        self.node.get_logger().info("🔄 所有搖桿事件回調和狀態機已重新建構")

    def _check_joystick_connection(self):
        """檢查搖桿連接狀態並處理狀態變化"""
        if not self.joy_handler:
            return
            
        current_connected = self.joy_handler.is_connected()
        
        # 連接狀態變化處理
        if current_connected != self._last_joystick_connected:
            if current_connected:
                self._handle_joystick_reconnect()
            else:
                self._handle_joystick_disconnect()
            
            self._last_joystick_connected = current_connected
        
        # 定期日誌記錄（每20次檢查記錄一次，約10秒）
        self._connection_check_count += 1
        if self._connection_check_count % 20 == 0:
            status = "連接" if current_connected else "斷線"
            self.node.get_logger().info(f"🎮 搖桿狀態: {status}")

    def _handle_joystick_disconnect(self):
        """處理搖桿斷線事件（內部處理，不切換狀態）"""
        self.node.get_logger().warn("⚠️ 搖桿斷線！停止所有移動指令")
        
        # 發送停止所有移動的指令
        if self.cmd_client:
            # 停止所有可能的移動指令
            movement_commands = ["forward", "backward", "shift_left", "shift_right", "rotate_left", "rotate_right"]
            for command in movement_commands:
                self.cmd_client.send_movement_command(command, False)
        
        # 清空所有回調（避免斷線期間的誤觸發）
        if self.joy_handler:
            self.joy_handler.clear_all_callbacks()
        
        # 重置方向狀態
        self.direction = DIRECTION_NONE
        self.node.get_logger().info("🛑 已發送安全停止指令並清空回調，重置方向狀態")

    def _handle_joystick_reconnect(self):
        """處理搖桿重連事件"""
        self.node.get_logger().info("🔌 搖桿重新連接！已準備好接收指令")
        
        # 重置方向狀態
        self.direction = DIRECTION_NONE
        
        # 重新建構狀態機（可能是不同類型的搖桿）
        self._build_axis_state_machine()
        
        # 重新註冊所有回調（按鈕和軸）
        self._register_button_callbacks()
        self._register_axis_callbacks()
        
        self.node.get_logger().info("🔄 搖桿重連後已重新建構狀態機和註冊所有回調")

    def leave(self):
        """離開 Manual 狀態，解除註冊搖桿按鈕與軸事件"""
        self.node.get_logger().info("🛑 離開 Manual 狀態，停止接收搖桿輸入")

        self.cmd_client.destroy()  # 銷毀命令客戶端
        self.cmd_client = None  # 重置命令客戶端

        self.joy_handler.stop()  # 停止搖桿監聽 會自動解除按鈕回調
        self.joy_handler = None  # 重置搖桿監聽

    def on_button_event(self, button, action):
        """處理按鈕事件"""
        # 檢查搖桿連接狀態
        self._check_joystick_connection()
        
        # 如果搖桿斷線，則忽略按鈕事件
        if not self._last_joystick_connected:
            self.node.get_logger().debug(f"🚫 搖桿斷線，忽略按鈕事件: {button} {action}")
            return
        
        self.node.get_logger().info(f"🔘 {button} {action}")
        command_state = self.button_command_map.get((button, action))
        if command_state:
            command, state = command_state
            if command and self.cmd_client:
                self.cmd_client.send_movement_command(command, state)
                self.node.get_logger().info(
                    f"{command} {'ON' if state else 'OFF'}")
        else:
            self.node.get_logger().info(f"未定義的按鈕事件: {button} {action}")

        if button == JoyHandler.SELECT_BUTTON and action == "pressed":
            self.node.get_logger().info("選擇按鈕被按下，返回 Idle 狀態")
            self.to_idle_state = True

    def on_axis_event(self, axis, value):
        """處理軸事件"""
        # 檢查搖桿連接狀態
        self._check_joystick_connection()
        
        # 如果搖桿斷線，則忽略軸事件
        if not self._last_joystick_connected:
            self.node.get_logger().debug(f"🚫 搖桿斷線，忽略軸事件: 軸{axis}={value}")
            return
            
        transition = self.axis_state_machine.get((self.direction, axis, value))
        if transition:
            new_direction, command, state = transition
            if self.cmd_client:
                self.cmd_client.send_movement_command(command, state)
                self.node.get_logger().info(
                    f"{command} {'ON' if state else 'OFF'}")
            self.direction = new_direction
        else:
            pass
            # self.node.get_logger().info("無效的軸移動指令")
        self.current_direction()

    def current_direction(self):
        """獲取當前狀態"""
        if self.direction == DIRECTION_NONE:
            pass
            # self.node.get_logger().info("現在方向: None")
        elif self.direction == DIRECTION_FORWARD:
            self.node.get_logger().info("現在方向: Forward")
        elif self.direction == DIRECTION_BACKWARD:
            self.node.get_logger().info("現在方向: Backward")
        elif self.direction == DIRECTION_RIGHT:
            self.node.get_logger().info("現在方向: Right")
        elif self.direction == DIRECTION_LEFT:
            self.node.get_logger().info("現在方向: Left")

    def handle(self, context):
        """處理狀態更新"""
        # 定期檢查搖桿連接狀態（在狀態更新中）
        self._check_joystick_connection()
        
        if self.node.agv_status.AGV_ALARM:
            self.node.get_logger().error("AGV_有警報，返回 Idle 狀態")
            from agv_base.states.error_state import ErrorState
            context.set_state(ErrorState(self.node))

        if not self.node.agv_status.AGV_MANUAL:
            self.node.get_logger().info("AGV_手動模式關閉，返回 Idle 狀態")
            from agv_base.states.idle_state import IdleState
            context.set_state(IdleState(self.node))
        #  如果有警報，則跳到警報 狀態

        # 測試用切換狀態
        # if self.to_idle_state:
        #    self.node.get_logger().info("AGV_測試切換到 Idle 狀態")
        #    from agv_base.states.idle_state import IdleState
        #    context.set_state(IdleState(self.node))
