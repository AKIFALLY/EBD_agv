import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


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
        self.d_pad_x_axis = 4
        self.d_pad_y_axis = 5
        self.node.get_logger().info("✅ JoyHandlerNode 啟動完成")

    def joy_callback(self, msg: Joy):
        # 安全補長
        if len(self._prev_buttons) < len(msg.buttons):
            self._prev_buttons.extend(
                [0] * (len(msg.buttons) - len(self._prev_buttons)))
        if len(self._prev_axes) < len(msg.axes):
            self._prev_axes.extend(
                [0.0] * (len(msg.axes) - len(self._prev_axes)))
        if self._prev_axes == 6:
            JoyHandler.D_PAD_X_AXIS = 4
            JoyHandler.D_PAD_Y_AXIS = 5
        else :
            JoyHandler.D_PAD_X_AXIS = 6
            JoyHandler.D_PAD_Y_AXIS = 7
        #self.node.get_logger().info(
        #    f"buttons: {msg.buttons}, len: {len(self._prev_buttons)}")
        #self.node.get_logger().info(
        #    f"axes: {msg.axes}, len: {len(self._prev_axes)}")

        try:
            for i, state in enumerate(msg.buttons):
                if state != self._prev_buttons[i]:
                    self._prev_buttons[i] = state
                    action = 'pressed' if state else 'released'
                    if i in self._button_callbacks:
                        self._button_callbacks[i](i, action)

            for i, value in enumerate(msg.axes):
                value = round(value, 2)
                if abs(value - self._prev_axes[i]) > 0.95:
                    self._prev_axes[i] = value
                    if i in self._axis_callbacks:
                        self._axis_callbacks[i](i, value)
        except Exception as e:
            self.node.get_logger().error(f"Joy Callback Error: {e}")

    def register_button_callback(self, index: int, callback):
        self._button_callbacks[index] = callback

    def register_axis_callback(self, index: int, callback):
        self._axis_callbacks[index] = callback

    def start(self):
        topic = f"{self.namespace}/joy"
        self.subscription = self.node.create_subscription(
            Joy, topic, self.joy_callback, 10)
        self.node.get_logger().info(f"🔌 訂閱搖桿 Topic: {topic}")

    def unregister_all_callbacks(self):
        if self.subscription:
            self.node.destroy_subscription(self.subscription)
            self.subscription = None
        self._button_callbacks.clear()
        self._axis_callbacks.clear()

    def stop(self):
        self.unregister_all_callbacks()
        self._prev_buttons.clear()
        self._prev_axes.clear()


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
