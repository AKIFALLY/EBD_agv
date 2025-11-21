"""
Cargo Mover AGV 專用的 WritePathState

使用 Context 類變數機制自動替換狀態類型
"""

from agv_base.agv_states.write_path_state import WritePathState
from rclpy.node import Node


class CargoWritePathState(WritePathState):
    """
    Cargo Mover AGV 專用的 WritePathState

    檢查路徑狀態，使用 context 類變數自動轉換到 CargoRunningState
    """

    def __init__(self, node: Node):
        super().__init__(node)

    def handle(self, context):
        """
        覆寫 handle 方法

        檢查是否已有路徑，若有則直接轉換到 RunningState
        所有狀態轉換會透過 context 類變數自動替換為 Cargo 版本
        """
        # ✅ 直接檢查是否已有路徑
        if self.node.agv_status.AGV_PATH:
            self.node.get_logger().info(
                "[Cargo] ✅ AGV 已有路徑資料，離開 WritePathState → CargoRunningState"
            )
            # ✅ 使用 context 類變數，自動指向 CargoRunningState
            context.set_state(context.RunningState(self.node))
            return

        # ✅ 直接呼叫父類的 handle 邏輯
        # 所有狀態轉換會透過 context.RunningState 自動替換為 CargoRunningState
        super().handle(context)
