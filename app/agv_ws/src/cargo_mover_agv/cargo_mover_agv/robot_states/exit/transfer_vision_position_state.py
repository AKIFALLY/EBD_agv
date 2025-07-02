from agv_base.states.state import State
from rclpy.node import Node
from cargo_mover_agv.robot_context import RobotContext  # 新增的匯入
from agv_base.robot import Robot
from cargo_mover_agv.robot_states.base_robot_state import BaseVisionPositionState


class TransferVisionPositionState(BaseVisionPositionState):

    def enter(self):
        self.node.get_logger().info("Robot Exit 目前狀態: TranferVisionPosition")
        self._reset_common_state()

    def leave(self):
        self.node.get_logger().info("Robot Exit 離開 TranferVisionPosition 狀態")
        self._reset_common_state()

    def handle(self, context: RobotContext):
        context.rack_photo_up_or_down_buffer = None
        self.node.get_logger().info("Robot Exit 傳送箱視覺定位中 狀態")

        from cargo_mover_agv.robot_states.exit.transfer_check_have_state import TransferCheckHaveState
        self._handle_vision_steps(
            context, context.robot.PHOTO_BOX_OUT, TransferCheckHaveState)
