from agv_base.states.state import State
from rclpy.node import Node


class IdleState(State):
    def __init__(self, node: Node):
        super().__init__(node)

    def enter(self):
        self.node.get_logger().info("cargo 目前狀態: Idle")

    def leave(self):
        self.node.get_logger().info("cargo 離開 Idle 狀態")

    def handle(self, context):
        self.node.get_logger().info("cargo Idle 狀態")
