import rclpy
from rclpy.node import Node
from joystick.joystick_handler import JoystickHandler


class JoystickTestNode(Node):
    def __init__(self):
        super().__init__('joystick_test_node')

        # 初始化搖桿
        JoystickHandler.init()

        # 註冊按鈕和軸事件
        JoystickHandler.register_button_callback("a_button", self.button_event)
        JoystickHandler.register_button_callback("b_button", self.button_event)
        JoystickHandler.register_button_callback("x_button", self.button_event)
        JoystickHandler.register_button_callback("y_button", self.button_event)

        JoystickHandler.register_button_callback(
            "l1_button", self.button_event)
        JoystickHandler.register_button_callback(
            "l2_button", self.button_event)
        JoystickHandler.register_button_callback(
            "r1_button", self.button_event)
        JoystickHandler.register_button_callback(
            "r2_button", self.button_event)

        JoystickHandler.register_button_callback(
            "select_button", self.button_event)
        JoystickHandler.register_button_callback(
            "start_button", self.button_event)

        JoystickHandler.register_axis_callback(
            JoystickHandler.L_X_AXIS, self.axis_event)  # 註冊 L_X_AXIS
        JoystickHandler.register_axis_callback(
            JoystickHandler.L_Y_AXIS, self.axis_event)  # 註冊 L_Y_AXIS
        JoystickHandler.register_axis_callback(
            JoystickHandler.R_X_AXIS, self.axis_event)  # 註冊 R_X_AXIS
        JoystickHandler.register_axis_callback(
            JoystickHandler.R_Y_AXIS, self.axis_event)  # 註冊 R_Y_AXIS
        JoystickHandler.register_dpad_callback(self.dpad_event)

        # 開始監聽搖桿
        # JoystickHandler.start()
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("🎮 搖桿測試節點已啟動")

    def timer_callback(self):
        JoystickHandler._joystick_loop(0)

    def button_event(self, button, action):
        print(f"🕹️ {button} {action}")

    def axis_event(self, axis, value):
        print(f"🎮 軸 {axis} 變化: {value}")

    def dpad_event(self, hat_state):
        print(f"🎮 D-Pad 移動: x={hat_state[0]}, y={hat_state[1]}")

    def destroy_node(self):
        JoystickHandler.stop()  # 靜態類別，直接停止
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JoystickTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 搖桿測試節點關閉")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
