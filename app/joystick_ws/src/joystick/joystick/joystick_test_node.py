"""
⚠️ DEPRECATED: This module is deprecated and no longer maintained.
   This test node uses the deprecated Pygame-based JoystickHandler.
   
   Please use standard ROS 2 Joy instead:
   
   ros2 run joy joy_node --ros-args --remap joy:/agv/joy
   ros2 topic echo /agv/joy
   
   Or use JoyHandler for processing sensor_msgs/Joy messages.
"""

import warnings
import rclpy
from rclpy.node import Node
from joystick.joystick_handler import JoystickHandler

# Issue deprecation warning when module is imported
warnings.warn(
    "JoystickTestNode is deprecated as it uses deprecated JoystickHandler. Use standard ROS 2 joy_node instead.",
    DeprecationWarning,
    stacklevel=2
)


class JoystickTestNode(Node):
    """
    ⚠️ DEPRECATED: JoystickTestNode is deprecated.
    
    This test node integrates the deprecated Pygame-based JoystickHandler.
    Please use standard ROS 2 joy_node instead:
    
    ros2 run joy joy_node --ros-args --remap joy:=/agv/joy
    
    Then use JoyHandler class for processing sensor_msgs/Joy messages.
    """
    
    def __init__(self):
        super().__init__('joystick_test_node')
        
        # Issue deprecation warning when node is created
        self.get_logger().warn(
            "⚠️ DEPRECATED: JoystickTestNode uses deprecated Pygame handler. "
            "Use standard ROS 2 joy_node instead."
        )

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
    print("⚠️ DEPRECATED: JoystickTestNode is deprecated.")
    print("Please use standard ROS 2 joy_node instead:")
    print("  ros2 run joy joy_node --ros-args --remap joy:=/agv/joy")
    print("  ros2 topic echo /agv/joy")
    print()
    
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
