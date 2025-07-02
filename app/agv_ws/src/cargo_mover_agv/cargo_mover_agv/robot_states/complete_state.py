from agv_base.states.state import State
from rclpy.node import Node
from cargo_mover_agv.robot_context import RobotContext


class CompleteState(State):
    def __init__(self, node: Node):
        super().__init__(node)

    def enter(self):
        self.node.get_logger().info("Robot 目前狀態: Complete")

    def leave(self):
        self.node.get_logger().info("Robot 離開 Complete 狀態")

    def handle(self, context: RobotContext):
        self.node.get_logger().info("Robot Complete 狀態")
        
        # 清理共同的狀態變數
        context.rack_photo_up_or_down_buffer = None
        
        # 清理 buffer 變數 - 同時處理 entrance 和 exit 的情況
        if hasattr(context, 'boxin_buffer'):
            context.boxin_buffer = None
        if hasattr(context, 'boxout_buffer'):
            context.boxout_buffer = None
            
        self.node.get_logger().info("✅ Complete 狀態處理完成，所有相關狀態已重置")
