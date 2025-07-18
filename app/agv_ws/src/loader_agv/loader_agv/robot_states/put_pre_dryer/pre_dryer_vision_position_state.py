from agv_base.states.state import State
from rclpy.node import Node
from loader_agv.robot_context import RobotContext
from agv_base.robot import Robot
from loader_agv.robot_states.base_robot_state import BaseVisionPositionState


class PreDryerVisionPositionState(BaseVisionPositionState):

    def enter(self):
        self.node.get_logger().info("Robot Put PreDryer 目前狀態: PreDryerVisionPosition")
        self._reset_state()

    def leave(self):
        self.node.get_logger().info("Robot Put PreDryer 離開 PreDryerVisionPosition 狀態")
        self._reset_state()

    def handle(self, context: RobotContext):
        self.node.get_logger().info("Robot Put PreDryer 預烘台視覺定位中 狀態")

        from loader_agv.robot_states.put_pre_dryer.pre_dryer_check_have_state import PreDryerCheckHaveState
        self._handle_vision_steps(
            context, context.robot.PHOTO_PRE_DRYER, PreDryerCheckHaveState)
