from agv_base.states.state import State
from rclpy.node import Node
from loader_agv.robot_context import RobotContext
from agv_base.robot import Robot
from loader_agv.robot_states.base_robot_state import BaseVisionPositionState


class CleanerVisionPositionState(BaseVisionPositionState):

    def enter(self):
        self.node.get_logger().info("Robot Put Cleaner 目前狀態: CleanerVisionPosition")
        self._reset_state()

    def leave(self):
        self.node.get_logger().info("Robot Put Cleaner 離開 CleanerVisionPosition 狀態")
        self._reset_state()

    def handle(self, context: RobotContext):
        self.node.get_logger().info("Robot Put Cleaner 清洗台視覺定位中 狀態")

        # 避免循環引用，在需要時才導入
        from loader_agv.robot_states.put_cleaner.cleaner_check_have_state import CleanerCheckHaveState
        self._handle_vision_steps(
            context, context.robot.PHOTO_CLEANER, CleanerCheckHaveState)
