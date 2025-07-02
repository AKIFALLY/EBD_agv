from agv_base.states.state import State
from rclpy.node import Node

class Practise_test(State):
    def __init__(self, node: Node):
        super().__init__(node)

    def enter(self):
        """進入 Practise_test 狀態"""
        self.node.get_logger().info("🚀 進入 Practise_test 狀態")
    def leave(self):
        """離開 Practise_test 狀態"""
        self.node.get_logger().info("🛑 離開 Practise_test 狀態")
    def handle(self):
        """處理 Practise_test 狀態的邏輯"""
        self.node.get_logger().info("🔄 處理 Practise_test 狀態的邏輯")
        # 在這裡添加狀態處理的邏輯
        # 例如：檢查條件、發送命令等
        # 如果需要切換狀態，可以調用 context.set_state(new_state)
        # 例如：context.set_state(NextState())
        # 這裡可以添加一些條件來決定是否切換狀態
        # if some_condition:
        #     context.set_state(NextState())
        # else:
        #     self.node.get_logger().info("保持在 Practise_test 狀態")
   
