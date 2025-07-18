import time
from agv_base.states.state import State
import astar_algorithm.data_tool
from rclpy.node import Node
from agv_base.agv_status import AgvStatus
from plc_proxy.plc_client import PlcClient
from astar_algorithm.astar_algorithm import AStarAlgorithm
import astar_algorithm.astar_algorithm


class IdleState(State):
    def __init__(self, node: Node):
        super().__init__(node)
        self.node = node
        self.enter_time = time.time()  # 記錄進入 Idle 狀態的時間
        self.plc_client = PlcClient(node)
        self.bt_trigger = False  # 按鈕觸發上微分

    def enter(self):
        self.enter_time = time.time()  # 記錄進入 Idle 狀態的時間
        self.node.get_logger().info("Base 目前狀態: Idle")

    def leave(self):
        print("離開 Idle 狀態")

    def handle(self, context):

        # 如果有警報，則進入Error 狀態
        if self.node.agv_status.AGV_ALARM:  # 假設有一個 AGV_Error 屬性
            self.node.get_logger().error("AGV_有警報，進入 Error 狀態")
            from agv_base.states.error_state import ErrorState
            context.set_state(ErrorState(self.node))

        # 檢查是否進入自動模式
        if self.node.agv_status.AGV_Auto:  # 測試延遲3秒後進入自動模式
            self.node.get_logger().info("AGV_Auto模式啟動")
            # 進入手動模式的邏輯
            # 這裡可以添加進入手動模式的具體操作
            # 例如，發送指令給 AGV 或者更新狀態
            from agv_base.states.auto_state import AutoState
            context.set_state(AutoState(self.node))

        # 檢查是否進入手動模式
        if self.node.agv_status.AGV_MANUAL:  # or time.time() - self.enter_time > 3:  # 測試延遲3秒後進入手動模式
            self.node.get_logger().info("AGV_手動模式啟動")
            from agv_base.states.manual_state import ManualState
            context.set_state(ManualState(self.node))  # 假設有一個 ManualState 狀態類
        # 處理TAG座標請求
        if self.node.agv_status.TAG_REQ and not self.bt_trigger:
            self.node.get_logger().info(f"詢問TAG X Y座標")
            self.As = AStarAlgorithm()
            x, y = self.As.getXY(self.node.agv_status.Req_TAGNo)  # 取得XY座標
            X = [str(i) for i in astar_algorithm.data_tool.int32_to_2_words(x)]
            Y = [str(j) for j in astar_algorithm.data_tool.int32_to_2_words(y)]
            data = X+Y
            self.node.get_logger().info(f"X Y座標:{data}")
            self.plc_client.async_write_continuous_data(
                'DM', '5000', data, self.write_data_callback)
            self.bt_trigger = True

    def write_data_callback(self, response):
        if response.success:
            self.node.get_logger().info("✅ PLC 資料寫入成功")
            self.bt_trigger = False
        else:
            self.node.get_logger().warn("⚠️ PLC 資料寫入失敗")
            self.bt_trigger = False
