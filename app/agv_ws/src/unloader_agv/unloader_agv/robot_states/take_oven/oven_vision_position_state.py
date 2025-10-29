from agv_base.states.state import State
from rclpy.node import Node
from unloader_agv.robot_context import RobotContext
from agv_base.robot import Robot
from unloader_agv.robot_states.base_robot_state import BaseVisionPositionState


class OvenVisionPositionState(BaseVisionPositionState):

    def enter(self):
        self.node.get_logger().info("Unloader Robot Take Oven 目前狀態: OvenVisionPosition")
        self._reset_state()

    def leave(self):
        self.node.get_logger().info("Unloader Robot Take Oven 離開 OvenVisionPosition 狀態")
        self._reset_state()

    def handle(self, context: RobotContext):
        self.node.get_logger().info("Unloader Robot Take Oven 烤箱視覺定位中 狀態")

        from unloader_agv.robot_states.take_oven.oven_check_have_state import OvenCheckHaveState
        self._handle_vision_steps(
            context, context.robot.PHOTO_OVEN, OvenCheckHaveState)
