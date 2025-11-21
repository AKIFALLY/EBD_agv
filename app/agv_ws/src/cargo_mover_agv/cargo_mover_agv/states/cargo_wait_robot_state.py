"""
Cargo Mover AGV 專用的 WaitRobotState

確保從 WaitRobotState 轉換回其他狀態時，使用 Cargo 專用版本
"""

from agv_base.agv_states.wait_robot_state import WaitRobotState
from rclpy.node import Node


class CargoWaitRobotState(WaitRobotState):
    """
    Cargo Mover AGV 專用的 WaitRobotState

    主要功能：攔截轉換到 MissionSelectState，確保使用 CargoMissionSelectState
    """

    def __init__(self, node: Node):
        super().__init__(node)

    def handle(self, context):
        """
        覆寫 handle 方法

        攔截 set_state，將基礎 MissionSelectState 替換為 CargoMissionSelectState
        """
        # 使用猴子補丁（Monkey Patch）攔截 set_state
        original_set_state = context.set_state

        def patched_set_state(new_state):
            """
            攔截 set_state，替換狀態為 Cargo 專屬版本
            - MissionSelectState → CargoMissionSelectState
            """
            from agv_base.agv_states.mission_select_state import MissionSelectState
            from cargo_mover_agv.states.cargo_mission_select_state import CargoMissionSelectState

            # 攔截 MissionSelectState
            if isinstance(new_state, MissionSelectState) and not isinstance(new_state, CargoMissionSelectState):
                self.node.get_logger().info(
                    "[Cargo] 🔄 攔截狀態轉換：MissionSelectState → CargoMissionSelectState"
                )
                original_set_state(CargoMissionSelectState(self.node))
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

    def _handle_verification_response(self, future, context):
        """
        覆寫 _handle_verification_response 方法

        攔截異步回調中的狀態轉換，確保使用 CargoMissionSelectState
        """
        from agv_base.agv_states.mission_select_state import MissionSelectState
        from cargo_mover_agv.states.cargo_mission_select_state import CargoMissionSelectState
        from shared_constants.task_status import TaskStatus

        # 保存原始的 set_state
        original_set_state = context.set_state

        def patched_set_state(new_state):
            """攔截 MissionSelectState → CargoMissionSelectState"""
            if isinstance(new_state, MissionSelectState) and not isinstance(new_state, CargoMissionSelectState):
                self.node.get_logger().info(
                    "[Cargo] 🔄 攔截異步回調狀態轉換：MissionSelectState → CargoMissionSelectState"
                )
                original_set_state(CargoMissionSelectState(self.node))
            else:
                original_set_state(new_state)

        # 暫時替換 set_state
        context.set_state = patched_set_state

        try:
            # 呼叫父類的驗證邏輯
            super()._handle_verification_response(future, context)
        finally:
            # 恢復原始的 set_state
            context.set_state = original_set_state
