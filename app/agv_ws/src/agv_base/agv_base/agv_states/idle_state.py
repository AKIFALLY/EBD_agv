from agv_base.states.state import State
from rclpy.node import Node
from agv_base.agv_states.mission_select_state import MissionSelectState
from agv_base.agv_states.Running_state import RunningState
from astar_algorithm.astar_algorithm import AStarAlgorithm
from plc_proxy.plc_client import PlcClient
import time

#AGV狀態機的空閒狀態
class IdleState(State):
    def __init__(self, node: Node):
        super().__init__(node)
        self.plc_client = PlcClient(node)
    def enter(self):
        self.node.get_logger().info("AGV 進入: Idle")

    def leave(self):
        self.node.get_logger().info("AGV 離開 Idle 狀態")

    def handle(self, context):
        self.node.get_logger().info("AGV Idle 狀態")
        

        # 判斷路徑是否有資料
        if self.pathdata == 0:
            #轉換到mission_select狀態
            self.node.get_logger().info("AGV 轉換到: Mission Select")
            context.set_state(MissionSelectState(self.node))

        if self.pathdata == 1:
            #轉換到Running狀態
            self.node.get_logger().info("AGV 轉換到: Running")
            context.set_state(RunningState(self.node))
           
        #處理TAG座標請求
        self.node.get_logger().info(f"self.node.agv_status.TAG_REQ{self.node.agv_status.TAG_REQ}")
        if self.node.agv_status.TAG_REQ :
            self.node.get_logger().info(f"詢問TAG X Y座標")
            self.As = AStarAlgorithm()
            x , y = self.As.getXY#取得XY座標
            data=[str(x*1000),str(y*1000)]
            #self.plc_client.async_write_continuous_data('DM', '5000', data, self.write_data_callback)


    def write_data_callback(self, response):
        if response.success:
            self.node.get_logger().info("✅ PLC 資料寫入成功")
        else:
            self.node.get_logger().warn("⚠️ PLC 資料寫入失敗")
        