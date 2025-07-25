from agv_base.states.state import State
from cargo_mover_agv.robot_context import RobotContext
from rclpy.node import Node
from cargo_mover_agv.robot_states.entrance.select_rack_port_state import SelectRackPortState


class RunningState(State):
    def __init__(self, node: Node):
        super().__init__(node)
        self.count = 0  # 計數器
        self.next_node = 0 #下一個點位
        self.ask_traffic_area = [] #詢問AGVC交管區域
        self.traffic_area_registed = [] #註冊的交管區域
        # 假設有一個 RobotContext 類別來管理機器人狀態

    def enter(self):
        self.node.get_logger().info("🏃 AGV 進入: Running 狀態")

    def leave(self):
        self.node.get_logger().info("🚪 AGV 離開 Running 狀態")

    def handle(self, context):
        # self.node.get_logger().info("AGV Running 狀態")
        if not self.node.agv_status.AGV_PATH:
            self.node.get_logger().info("⚠️ AGV 沒有路徑資料，回到任務選擇狀態")
            from agv_base.agv_states.mission_select_state import MissionSelectState
            context.set_state(MissionSelectState(self.node))
        # 如果有路徑資料，則持續運行狀態
        if self.node.agv_status.AGV_PATH:

            if self.count > 100:
                self.count = 0
                self.node.get_logger().info(f"🏃 AGV RunningState... ")

            self.count += 1

        if self.node.agv_status.AGV_2POSITION:
            self.node.get_logger().info("✅ AGV 到達目標位置")
            self.node.robot_finished = False  # 重置機器人完成狀態
            from agv_base.agv_states.wait_robot_state import WaitRobotState
            context.set_state(WaitRobotState(self.node))
