"""
Cargo Mover AGV 專用的 MissionSelectState

使用 Context 類變數機制自動替換狀態類型
"""

from agv_base.agv_states.mission_select_state import MissionSelectState
from rclpy.node import Node


class CargoMissionSelectState(MissionSelectState):
    """
    Cargo Mover AGV 專用的 MissionSelectState

    特殊行為：支援 Local 模式（MAGIC=21）在無任務時進入 RunningState
    """

    def __init__(self, node: Node):
        super().__init__(node)

    def handle(self, context):
        """
        覆寫 handle 方法

        允許 Local 模式（LOCAL=ON 且 MAGIC=21）時即使無任務也進入 RunningState
        所有狀態轉換會透過 context 類變數自動替換為 Cargo 版本
        """
        # 檢查是否為 Local 模式且有路徑（允許無任務進入 RunningState）
        if self._should_enter_running_without_task():
            self.node.get_logger().info(
                "[Cargo] ✅ Local 模式 (MAGIC=21)，有路徑但無任務，進入 RunningState"
            )
            # ✅ 使用 context 類變數，自動指向 CargoRunningState
            context.set_state(context.RunningState(self.node))
            return

        # ✅ 直接呼叫父類的 handle 邏輯
        # 所有狀態轉換會透過 context.XxxState 自動替換為 Cargo 版本
        super().handle(context)

    def _should_enter_running_without_task(self) -> bool:
        """
        檢查是否應該在無任務情況下進入 RunningState

        條件：
        1. 有路徑資料 (AGV_PATH)
        2. Local 模式開啟 (AGV_LOCAL)
        3. MAGIC = 21
        4. 沒有有效任務 (task_id = 0 或無任務)

        Returns:
            bool: 是否應該進入 RunningState
        """
        # 檢查是否有路徑
        has_path = bool(self.node.agv_status.AGV_PATH)

        # 檢查是否為 Local 模式
        is_local_mode = bool(self.node.agv_status.AGV_LOCAL)

        # 檢查 MAGIC 是否為 21
        is_magic_21 = (self.node.agv_status.MAGIC == 21)

        # 檢查是否沒有有效任務
        no_valid_task = not (
            hasattr(self.node, 'task') and
            self.node.task and
            hasattr(self.node.task, 'id') and
            self.node.task.id != 0
        )

        return has_path and is_local_mode and is_magic_21 and no_valid_task
