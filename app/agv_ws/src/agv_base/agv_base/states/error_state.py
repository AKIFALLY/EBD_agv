import time
from agv_base.states.state import State
from rclpy.node import Node


class ErrorState(State):
    def __init__(self, node: Node):
        super().__init__(node)
        self.node = node

    def enter(self):
        self.node.get_logger().info("目前狀態: Error")

    def leave(self):
        print("離開 Error 狀態")

    def handle(self, context):
        
        #self.node.get_logger().info(f"AGV_警報狀態: {self.alarm}")
        if not self.node.agv_status.AGV_ALARM:
            self.node.get_logger().info("AGV_無警報，進入 Idle 狀態")
            from agv_base.states.idle_state import IdleState
            context.set_state(IdleState(self.node))
