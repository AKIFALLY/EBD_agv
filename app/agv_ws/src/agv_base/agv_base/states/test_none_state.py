import time
from agv_base.states.state import State
from agv_base.base_context import BaseContext
from rclpy.node import Node


class NoneState(State):
    def __init__(self, node: Node):
        super().__init__(node)
        self.node = node
        self.enter_time = None  # 記錄進入狀態的時間

    def enter(self):
        self.enter_time = time.time()
        self.node.get_logger().info("進入 None 狀態")

    def leave(self):
        self.node.get_logger().info("離開 None 狀態")

    def handle(self, context: BaseContext):
        self.node.get_logger().info(f"t={time.time() - self.enter_time:.2f}")
        if time.time() - self.enter_time >= 3:  # 超過 3 秒
            self.node.get_logger().info("3 秒到了，自動切 A 狀態")
            from agv_base.states.test_a_state import AState  # 切回用
            context.set_state(AState(self.node))
