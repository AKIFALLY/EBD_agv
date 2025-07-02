from agv_base.states.state import State
from rclpy.node import Node
from loader_agv.robot_context import RobotContext  # 新增的匯入


class IdleState(State):
    def __init__(self, node: Node):
        super().__init__(node)

    def enter(self):
        self.node.get_logger().info("robot 目前狀態: Idle")

    def leave(self):
        self.node.get_logger().info("robot 離開 Idle 狀態")

    def handle(self, context):
        self.node.get_logger().info("robot Idle 狀態")
        #if self.mission == "entrance":
        #    from loader_agv.robot_states.entrance.transfer_vision_position_state import TranferVisionPositionState
        #    context.set_state(TranferVisionPositionState(self.node))
        #elif self.mission == "exit":
        #    pass
