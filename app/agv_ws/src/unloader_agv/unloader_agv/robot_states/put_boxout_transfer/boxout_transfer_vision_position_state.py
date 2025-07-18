from agv_base.states.state import State
from rclpy.node import Node
from unloader_agv.robot_context import RobotContext
from agv_base.robot import Robot
from unloader_agv.robot_states.base_robot_state import BaseVisionPositionState


class BoxoutTransferVisionPositionState(BaseVisionPositionState):

    def enter(self):
        self.node.get_logger().info("Unloader Robot Put BoxoutTransfer 目前狀態: BoxoutTransferVisionPosition")
        self._reset_state()

    def leave(self):
        self.node.get_logger().info("Unloader Robot Put BoxoutTransfer 離開 BoxoutTransferVisionPosition 狀態")
        self._reset_state()

    def handle(self, context: RobotContext):
        self.node.get_logger().info("Unloader Robot Put BoxoutTransfer 出料傳送台視覺定位中 狀態")

        from unloader_agv.robot_states.put_boxout_transfer.boxout_transfer_check_empty_state import BoxoutTransferCheckEmptyState
        self._handle_vision_steps(
            context, context.robot.PHOTO_BOX_OUT, BoxoutTransferCheckEmptyState)
