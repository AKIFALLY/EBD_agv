from agv_base.states.state import State
from rclpy.node import Node
from loader_agv.robot_context import RobotContext
from agv_base.robot import Robot
from loader_agv.robot_states.base_robot_state import BaseVisionPositionState


class SoakerVisionPositionState(BaseVisionPositionState):

    def enter(self):
        self.node.get_logger().info(
            "[Station-based 1格] Loader Robot Put Soaker 目前狀態: SoakerVisionPosition")
        self._reset_state()

    def leave(self):
        self.node.get_logger().info(
            "[Station-based 1格] Loader Robot Put Soaker 離開 SoakerVisionPosition 狀態")
        self._reset_state()

    def handle(self, context: RobotContext):
        self.node.get_logger().info(
            f"[Station-based 1格] Loader Robot Put Soaker 泡藥機視覺定位中 "
            f"(Work ID {context.work_id})")

        # 視覺定位完成後進入 SoakerCheckHaveState（檢查泡藥機是否有空位）
        from loader_agv.robot_states.put_soaker.soaker_check_have_state import SoakerCheckHaveState
        self._handle_vision_steps(
            context, context.robot.PHOTO_SOAKER, SoakerCheckHaveState)
