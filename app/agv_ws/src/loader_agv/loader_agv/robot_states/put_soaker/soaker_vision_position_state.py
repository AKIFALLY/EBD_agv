from agv_base.states.state import State
from rclpy.node import Node
from loader_agv.robot_context import RobotContext
from agv_base.robot import Robot
from loader_agv.robot_states.base_robot_state import BaseVisionPositionState


class SoakerVisionPositionState(BaseVisionPositionState):

    def enter(self):
        self.node.get_logger().info("Robot Take Soaker 目前狀態: SoakerVisionPosition")
        self._reset_common_state()

    def leave(self):
        self.node.get_logger().info("Robot Take Soaker 離開 SoakerVisionPosition 狀態")
        self._reset_common_state()

    def handle(self, context: RobotContext):
        context.rack_photo_up_or_down_buffer = None
        self.node.get_logger().info("Robot Take Soaker 清洗台視覺定位中 狀態")

        from loader_agv.robot_states.take_soaker.soaker_check_have_state import SoakerCheckHaveState
        self._handle_vision_steps(
            context, context.robot.PHOTO_SOAKER, SoakerCheckHaveState)
