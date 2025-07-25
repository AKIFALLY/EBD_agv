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

    def enter(self):
        # 方向狀態機 + DPAD狀態機(joy linux 中的d-pax 也是 axis)
        self.axis_state_machine = {
            (DIRECTION_NONE, JoyHandler.L_Y_AXIS, 1.0): (DIRECTION_FORWARD, "forward", True),
            (DIRECTION_NONE, JoyHandler.L_Y_AXIS, -1.0):  (DIRECTION_BACKWARD, "backward", True),
            (DIRECTION_NONE, JoyHandler.L_X_AXIS, -1.0):  (DIRECTION_RIGHT, "shift_right", True),
            (DIRECTION_NONE, JoyHandler.L_X_AXIS, 1.0): (DIRECTION_LEFT, "shift_left", True),

            (DIRECTION_FORWARD, JoyHandler.L_Y_AXIS, 0.0): (DIRECTION_NONE, "forward", False),
            (DIRECTION_BACKWARD, JoyHandler.L_Y_AXIS, 0.0): (DIRECTION_NONE, "backward", False),
            (DIRECTION_RIGHT, JoyHandler.L_X_AXIS, 0.0): (DIRECTION_NONE, "shift_right", False),
            (DIRECTION_LEFT, JoyHandler.L_X_AXIS, 0.0): (DIRECTION_NONE, "shift_left", False),

            (DIRECTION_NONE, JoyHandler.D_PAD_Y_AXIS, 1.0): (DIRECTION_FORWARD, "forward", True),
            (DIRECTION_NONE, JoyHandler.D_PAD_Y_AXIS, -1.0):  (DIRECTION_BACKWARD, "backward", True),
            (DIRECTION_NONE, JoyHandler.D_PAD_X_AXIS, -1.0):  (DIRECTION_RIGHT, "shift_right", True),
            (DIRECTION_NONE, JoyHandler.D_PAD_X_AXIS, 1.0): (DIRECTION_LEFT, "shift_left", True),

            (DIRECTION_FORWARD, JoyHandler.D_PAD_Y_AXIS, 0.0): (DIRECTION_NONE, "forward", False),
            (DIRECTION_BACKWARD, JoyHandler.D_PAD_Y_AXIS, 0.0): (DIRECTION_NONE, "backward", False),
            (DIRECTION_RIGHT, JoyHandler.D_PAD_X_AXIS, 0.0): (DIRECTION_NONE, "shift_right", False),
            (DIRECTION_LEFT, JoyHandler.D_PAD_X_AXIS, 0.0): (DIRECTION_NONE, "shift_left", False),

        }
        """進入 Manual 狀態，註冊搖桿按鈕與軸事件"""
        self.node.get_logger().info("🎮 進入 Manual 狀態，等待搖桿手動移動指令")

        self.cmd_client = AGVCommandProxy(self.node)

        self.joy_handler = JoyHandler(self.node)
        self.joy_handler.register_button_callback(
            JoyHandler.A_BUTTON, self.on_button_event)
        self.joy_handler.register_button_callback(
            JoyHandler.B_BUTTON, self.on_button_event)
        self.joy_handler.register_button_callback(
            JoyHandler.X_BUTTON, self.on_button_event)
        self.joy_handler.register_button_callback(
            JoyHandler.Y_BUTTON, self.on_button_event)
        self.joy_handler.register_button_callback(
            JoyHandler.L1_BUTTON, self.on_button_event)
        self.joy_handler.register_button_callback(
            JoyHandler.R1_BUTTON, self.on_button_event)
        self.joy_handler.register_button_callback(
            JoyHandler.L2_BUTTON, self.on_button_event)
        self.joy_handler.register_button_callback(
            JoyHandler.R2_BUTTON, self.on_button_event)
        self.joy_handler.register_button_callback(
            JoyHandler.SELECT_BUTTON, self.on_button_event)
        self.joy_handler.register_button_callback(
            JoyHandler.START_BUTTON, self.on_button_event)
        self.joy_handler.start()

        self.joy_handler.register_axis_callback(JoyHandler.L_X_AXIS,
                                                self.on_axis_event)
        self.joy_handler.register_axis_callback(JoyHandler.L_Y_AXIS,
                                                self.on_axis_event)
        self.joy_handler.register_axis_callback(JoyHandler.R_X_AXIS,
                                                self.on_axis_event)
        self.joy_handler.register_axis_callback(JoyHandler.R_Y_AXIS,
                                                self.on_axis_event)
        self.joy_handler.register_axis_callback(JoyHandler.D_PAD_X_AXIS,
                                                self.on_axis_event)
        self.joy_handler.register_axis_callback(JoyHandler.D_PAD_Y_AXIS,
                                                self.on_axis_event)

    def leave(self):
        """離開 Manual 狀態，解除註冊搖桿按鈕與軸事件"""
        self.node.get_logger().info("🛑 離開 Manual 狀態，停止接收搖桿輸入")

        self.cmd_client.destroy()  # 銷毀命令客戶端
        self.cmd_client = None  # 重置命令客戶端

        self.joy_handler.stop()  # 停止搖桿監聽 會自動解除按鈕回調
        self.joy_handler = None  # 重置搖桿監聽

    def on_button_event(self, button, action):
        """處理按鈕事件"""
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
