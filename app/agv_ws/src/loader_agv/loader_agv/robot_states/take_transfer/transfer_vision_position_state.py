from agv_base.states.state import State
from rclpy.node import Node
from loader_agv.robot_context import RobotContext
from agv_base.robot import Robot
from loader_agv.robot_states.base_robot_state import BaseVisionPositionState


class TransferVisionPositionState(BaseVisionPositionState):

    def enter(self):
        self.node.get_logger().info(
            "[Station-based] Robot Take Transfer 目前狀態: TranferVisionPosition")
        self._reset_state()

    def leave(self):
        self.node.get_logger().info(
            "[Station-based] Robot Take Transfer 離開 TranferVisionPosition 狀態")
        self._reset_state()

    def handle(self, context: RobotContext):
        self.node.get_logger().info(
            f"[Station-based] Robot Take Transfer 傳送箱視覺定位中 "
            f"(Work ID {context.work_id})")

        from loader_agv.robot_states.take_transfer.transfer_check_have_state import TransferCheckHaveState
        # 執行視覺定位標準步驟
        # 視覺定位完成後進入 TransferCheckHaveState（檢查傳送箱載具）
        self._handle_vision_steps(
            context, context.robot.PHOTO_BOX_IN, TransferCheckHaveState)
