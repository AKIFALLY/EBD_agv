import time
from agv_base.states.state import State
from rclpy.node import Node


class AutoState(State):
    def __init__(self, node: Node):
        super().__init__(node)
        self.node = node

    def enter(self):
        self.node.get_logger().info("目前狀態: Auto")

    def leave(self):
        print("離開 Auto 狀態")

    def handle(self, context):
        # 如果有警報，則進入Error 狀態
        if self.node.agv_status.AGV_ALARM:  # 假設有一個 AGV_Error 屬性
            self.node.get_logger().info("AGV_有警報，進入 Error 狀態")
            from agv_base.states.error_state import ErrorState
            context.set_state(ErrorState(self.node))


        # 檢查是否進入自動模式
        if not self.node.agv_status.AGV_Auto:  # 測試延遲3秒後進入自動模式
            self.node.get_logger().info("AGV_Auto模式啟動")
            from agv_base.states.idle_state import IdleState
            context.set_state(IdleState(self.node))
