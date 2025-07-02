from agv_base.states.state import State
from rclpy.node import Node
from loader_agv.robot_context import RobotContext


class CompleteState(State):
    def __init__(self, node: Node):
        super().__init__(node)

    def enter(self):
        self.node.get_logger().info("Robot Take Transfer 目前狀態: Complete")

    def leave(self):
        self.node.get_logger().info("Robot Take Transfer 離開 Complete 狀態")

    def handle(self, context: RobotContext):
        self.node.get_logger().info("Robot Take Transfer Complete 狀態")
        # 清理相關的狀態變數
        context.take_transfer_continue = False
        context.boxin_buffer = 0
        
        # 可以在這裡添加其他完成後的清理工作
        self.node.get_logger().info("✅ Take Transfer 流程完成，所有狀態已重置")
        
        # 轉換到 IdleState
        from loader_agv.robot_states.idle_state import IdleState
        context.set_state(IdleState(self.node))
