from agv_base.states.state import State
from rclpy.node import Node
from loader_agv.robot_context import RobotContext
from agv_base.robot import Robot
from loader_agv.robot_states.base_robot_state import BaseVisionPositionState


class TransferVisionPositionState(BaseVisionPositionState):

    def enter(self):
        self.node.get_logger().info("Robot Take Transfer 目前狀態: TranferVisionPosition")
        self._reset_common_state()

    def leave(self):
        self.node.get_logger().info("Robot Take Transfer 離開 TranferVisionPosition 狀態")
        self._reset_common_state()

    def handle(self, context: RobotContext):
        context.rack_photo_up_or_down_buffer = None
        self.node.get_logger().info("Robot Take Transfer 傳送箱視覺定位中 狀態")

        from loader_agv.robot_states.take_transfer.transfer_check_have_state import TransferCheckHaveState
        self._handle_vision_steps(
            context, context.robot.PHOTO_BOX_IN, TransferCheckHaveState)
