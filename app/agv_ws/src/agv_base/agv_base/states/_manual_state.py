from agv_base.states.state import State
from rclpy.node import Node
from joystick.joystick_handler import JoystickHandler
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
        self.cmd_client = None

        # 初始化搖桿 現在方向
        self.direction = DIRECTION_NONE

        # 方向狀態機
        self.axis_state_machine = {
            (DIRECTION_NONE, JoystickHandler.L_Y_AXIS, -1.0): (DIRECTION_FORWARD, "forward", True),
            (DIRECTION_NONE, JoystickHandler.L_Y_AXIS, 1.0):  (DIRECTION_BACKWARD, "backward", True),
            (DIRECTION_NONE, JoystickHandler.L_X_AXIS, 1.0):  (DIRECTION_RIGHT, "shift_right", True),
            (DIRECTION_NONE, JoystickHandler.L_X_AXIS, -1.0): (DIRECTION_LEFT, "shift_left", True),

            (DIRECTION_FORWARD, JoystickHandler.L_Y_AXIS, 0.0): (DIRECTION_NONE, "forward", False),
            (DIRECTION_BACKWARD, JoystickHandler.L_Y_AXIS, 0.0): (DIRECTION_NONE, "backward", False),
            (DIRECTION_RIGHT, JoystickHandler.L_X_AXIS, 0.0): (DIRECTION_NONE, "shift_right", False),
            (DIRECTION_LEFT, JoystickHandler.L_X_AXIS, 0.0): (DIRECTION_NONE, "shift_left", False),
        }
        # D-Pad 狀態機
        self.dpad_state_machine = {
            (DIRECTION_NONE, JoystickHandler.DPAD_UP): (DIRECTION_FORWARD, "forward", True),
            (DIRECTION_NONE, JoystickHandler.DPAD_DOWN):  (DIRECTION_BACKWARD, "backward", True),
            (DIRECTION_NONE, JoystickHandler.DPAD_RIGHT):  (DIRECTION_RIGHT, "shift_right", True),
            (DIRECTION_NONE, JoystickHandler.DPAD_LEFT): (DIRECTION_LEFT, "shift_left", True),

            (DIRECTION_FORWARD, JoystickHandler.DPAD_CENTER): (DIRECTION_NONE, "forward", False),
            (DIRECTION_BACKWARD, JoystickHandler.DPAD_CENTER): (DIRECTION_NONE, "backward", False),
            (DIRECTION_RIGHT, JoystickHandler.DPAD_CENTER): (DIRECTION_NONE, "shift_right", False),
            (DIRECTION_LEFT, JoystickHandler.DPAD_CENTER): (DIRECTION_NONE, "shift_left", False),
        }

        # 按鈕功能對應表
        self.button_command_map = {
            # Example: No command for A button pressed
            ("a_button", "pressed"): (None, None),
            ("b_button", "pressed"): ("rotate_right", True),
            ("x_button", "pressed"): ("break", True),
            ("y_button", "pressed"): ("rotate_left", True),
            ("l1_button", "pressed"): ("enable", True),


            # Example: No command for A button released
            ("a_button", "released"): (None, None),
            ("b_button", "released"): ("rotate_right", False),
            ("x_button", "released"): ("break", False),
            ("y_button", "released"): ("rotate_left", False),
            ("l1_button", "released"): ("enable", False),

        }

        JoystickHandler.init()

    def enter(self):
        # self.node.get_logger().info(
        #    f"📡 進入manual_state初始化{self.node.get_namespace()}")
        self.cmd_client = AGVCommandProxy(self.node)

        """進入 Manual 狀態，註冊搖桿按鈕與軸事件"""
        self.node.get_logger().info("🎮 進入 Manual 狀態，等待搖桿手動移動指令")

        JoystickHandler.register_button_callback(
            "a_button", self.on_button_event)
        JoystickHandler.register_button_callback(
            "b_button", self.on_button_event)
        JoystickHandler.register_button_callback(
            "x_button", self.on_button_event)
        JoystickHandler.register_button_callback(
            "y_button", self.on_button_event)

        JoystickHandler.register_button_callback(
            "l1_button", self.on_button_event)
        JoystickHandler.register_button_callback(
            "l2_button", self.on_button_event)
        JoystickHandler.register_button_callback(
            "r1_button", self.on_button_event)
        JoystickHandler.register_button_callback(
            "r2_button", self.on_button_event)

        JoystickHandler.register_button_callback(
            "select_button", self.on_button_event)
        JoystickHandler.register_button_callback(
            "start_button", self.on_button_event)
        JoystickHandler.register_dpad_callback(self.on_dpad_event)

        # 註冊軸回調
        JoystickHandler.register_axis_callback(
            JoystickHandler.L_X_AXIS, self.on_axis_event)
        JoystickHandler.register_axis_callback(
            JoystickHandler.L_Y_AXIS, self.on_axis_event)
        JoystickHandler.register_axis_callback(
            JoystickHandler.R_X_AXIS, self.on_axis_event)
        JoystickHandler.register_axis_callback(
            JoystickHandler.R_Y_AXIS, self.on_axis_event)

        # 啟動搖桿監聽（如果尚未啟動）
        # JoystickHandler.start()

    def leave(self):
        """離開 Manual 狀態，解除註冊搖桿按鈕與軸事件"""
        self.node.get_logger().info("🛑 離開 Manual 狀態，停止接收搖桿輸入")

        self.cmd_client.destroy()  # 銷毀命令客戶端
        self.cmd_client = None  # 重置命令客戶端

        # 解除按鈕回調
        JoystickHandler.unregister_button_callback("a_button")
        JoystickHandler.unregister_button_callback("b_button")
        JoystickHandler.unregister_button_callback("x_button")
        JoystickHandler.unregister_button_callback("y_button")

        JoystickHandler.unregister_button_callback("l1_button")
        JoystickHandler.unregister_button_callback("l2_button")
        JoystickHandler.unregister_button_callback("r1_button")
        JoystickHandler.unregister_button_callback("r2_button")

        JoystickHandler.unregister_button_callback("select_button")
        JoystickHandler.unregister_button_callback("start_button")
        JoystickHandler.unregister_dpad_callback()

        # 解除軸回調
        JoystickHandler.unregister_axis_callback(JoystickHandler.L_X_AXIS)
        JoystickHandler.unregister_axis_callback(JoystickHandler.L_Y_AXIS)
        JoystickHandler.unregister_axis_callback(JoystickHandler.R_X_AXIS)
        JoystickHandler.unregister_axis_callback(JoystickHandler.R_Y_AXIS)

        JoystickHandler.stop()  # 停止搖桿監聽

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

        if button == "select_button" and action == "pressed":
            self.node.get_logger().info("選擇按鈕被按下，返回 Idle 狀態")
            from agv_base.states.idle_state import IdleState
            self.node.base_context.set_state(IdleState(self.node))

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
            self.node.get_logger().info("無效的軸移動指令")
        self.current_direction()

    def on_dpad_event(self, hat_state):
        """處理 D-Pad 事件"""
        self.node.get_logger().info(f"hat_state {hat_state}")
        transition = self.dpad_state_machine.get(
            (self.direction, hat_state))
        if transition:
            new_direction, command, state = transition
            if self.cmd_client:
                self.cmd_client.send_movement_command(command, state)
                self.node.get_logger().info(
                    f"{command} {'ON' if state else 'OFF'}")
            self.direction = new_direction
        else:
            self.node.get_logger().info("無效的 D-Pad 移動指令")
            # 停止所有運動
            if self.cmd_client:
                self.cmd_client.send_movement_command("forward", False)
                self.cmd_client.send_movement_command("backward", False)
                self.cmd_client.send_movement_command("shift_right", False)
                self.cmd_client.send_movement_command("shift_left", False)
            self.direction = DIRECTION_NONE
        self.current_direction()

    def current_direction(self):
        """獲取當前狀態"""
        if self.direction == DIRECTION_NONE:
            self.node.get_logger().info("現在方向: None")
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

        self.node.plc_client.async_force_on("MR", "100", None)
        self.node.plc_client.async_force_on("MR", "100", None)
        self.node.plc_client.async_force_on("MR", "100", None)
        self.node.plc_client.async_force_on("MR", "100", None)
        # JoystickHandler._joystick_loop(0)
        if not self.node.agv_status.AGV_MANUAL:
            self.node.get_logger().info("AGV_手動模式關閉，返回 Idle 狀態")
            from agv_base.states.idle_state import IdleState
            context.set_state(IdleState(self.node))
        #  如果有警報，則返回 Idle 狀態
        if self.node.agv_status.AGV_ALARM:
            self.node.get_logger().info("AGV_有警報，返回 Idle 狀態")
            from agv_base.states.idle_state import IdleState
            context.set_state(IdleState(self.node))
