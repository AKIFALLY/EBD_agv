import time
from agv_base.states.state import State
from agv_base.base_context import BaseContext
from rclpy.node import Node


class BState(State):
    def __init__(self, node: Node):
        super().__init__(node)
        self.node = node
        self.enter_time = None  # 記錄進入狀態的時間

    def enter(self):
        self.enter_time = time.time()
        self.node.get_logger().info("進入 B 狀態")

    def leave(self):
        self.node.get_logger().info("離開 B 狀態")

    def handle(self, context: BaseContext):
        self.node.get_logger().info(f"t={time.time() - self.enter_time:.2f}")
        if time.time() - self.enter_time >= 5:  # 超過 5 秒
            self.node.get_logger().info("5 秒到了，自動切回 None 狀態")
            from agv_base.states.test_none_state import NoneState  # 切回用
            context.set_state(NoneState(self.node))
