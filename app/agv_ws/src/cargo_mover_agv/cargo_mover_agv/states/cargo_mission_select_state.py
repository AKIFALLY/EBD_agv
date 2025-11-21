"""
Cargo Mover AGV 專用的 MissionSelectState

覆寫 WritePathState 的創建，使用 CargoWritePathState 代替
"""

from agv_base.agv_states.mission_select_state import MissionSelectState
from cargo_mover_agv.states.cargo_write_path_state import CargoWritePathState
from rclpy.node import Node


class CargoMissionSelectState(MissionSelectState):
    """
    Cargo Mover AGV 專用的 MissionSelectState

    唯一的差異：在轉換到 WritePathState 時，使用 CargoWritePathState
    """

    def __init__(self, node: Node):
        super().__init__(node)

    def handle(self, context):
        """
        覆寫 handle 方法

        1. 允許 Local 模式（LOCAL=ON 且 MAGIC=21）時即使無任務也進入 RunningState
        2. 替換 WritePathState 為 CargoWritePathState
        3. 替換 RunningState 為 CargoRunningState
        """
        # 檢查是否為 Local 模式且有路徑（允許無任務進入 RunningState）
        if self._should_enter_running_without_task():
            self.node.get_logger().info(
                "[Cargo] ✅ Local 模式 (MAGIC=21)，有路徑但無任務，進入 RunningState"
            )
            from cargo_mover_agv.states.cargo_running_state import CargoRunningState
            context.set_state(CargoRunningState(self.node))
            return

        # 使用猴子補丁（Monkey Patch）攔截 set_state
        original_set_state = context.set_state

        def patched_set_state(new_state):
            """
            攔截 set_state，替換狀態為 Cargo 專屬版本
            1. WritePathState → CargoWritePathState
            2. RunningState → CargoRunningState
            """
            from agv_base.agv_states.write_path_state import WritePathState
            from agv_base.agv_states.Running_state import RunningState
            from cargo_mover_agv.states.cargo_running_state import CargoRunningState

            # 替換 WritePathState
            if isinstance(new_state, WritePathState) and not isinstance(new_state, CargoWritePathState):
                self.node.get_logger().info(
                    "[Cargo] 🔄 攔截狀態轉換：WritePathState → CargoWritePathState"
                )
                original_set_state(CargoWritePathState(self.node))
            # 替換 RunningState（用於有路徑但無任務的情況）
            elif isinstance(new_state, RunningState) and not isinstance(new_state, CargoRunningState):
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
