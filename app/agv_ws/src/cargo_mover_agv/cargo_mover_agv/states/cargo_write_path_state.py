"""
Cargo Mover AGV 專用的 WritePathState

覆寫 RunningState 的創建，使用 CargoRunningState 代替
"""

from agv_base.agv_states.write_path_state import WritePathState
from cargo_mover_agv.states.cargo_running_state import CargoRunningState
from rclpy.node import Node


class CargoWritePathState(WritePathState):
    """
    Cargo Mover AGV 專用的 WritePathState

    唯一的差異：在轉換到 RunningState 時，使用 CargoRunningState
    """

    def __init__(self, node: Node):
        super().__init__(node)

    def handle(self, context):
        """
        覆寫 handle 方法，替換 RunningState 為 CargoRunningState

        直接檢查路徑並創建 CargoRunningState（更可靠的方法）
        """
        # ✅ 直接檢查是否已有路徑，若有則使用 CargoRunningState
        if self.node.agv_status.AGV_PATH:
            self.node.get_logger().info(
                "[Cargo] ✅ AGV 已有路徑資料，離開 WritePathState → CargoRunningState"
            )
            context.set_state(CargoRunningState(self.node))
            return

        # 否則使用猴子補丁攔截父類中的狀態轉換
        original_set_state = context.set_state

        def patched_set_state(new_state):
            """攔截 set_state，將 RunningState 替換為 CargoRunningState"""
            from agv_base.agv_states.Running_state import RunningState

            # 如果是 RunningState，替換為 CargoRunningState
            if isinstance(new_state, RunningState) and not isinstance(new_state, CargoRunningState):
                self.node.get_logger().info(
                    "[Cargo] 🔄 攔截狀態轉換：RunningState → CargoRunningState"
                )
                original_set_state(CargoRunningState(self.node))
            else:
                original_set_state(new_state)

        # 暫時替換 set_state 方法
        context.set_state = patched_set_state

        try:
            # 呼叫父類的 handle 邏輯
            super().handle(context)
        finally:
            # 恢復原始的 set_state 方法
            context.set_state = original_set_state
