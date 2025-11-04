from agv_base.states.state import State
from rclpy.node import Node
from loader_agv.robot_context import RobotContext
from agv_base.robot import Robot
from loader_agv.robot_states.base_robot_state import BaseVisionPositionState


class CleanerVisionPositionState(BaseVisionPositionState):

    def enter(self):
        self.node.get_logger().info(
            "[Station-based 固定方向] Robot Put Cleaner 目前狀態: CleanerVisionPosition")
        self._reset_state()

    def leave(self):
        self.node.get_logger().info(
            "[Station-based 固定方向] Robot Put Cleaner 離開 CleanerVisionPosition 狀態")
        self._reset_state()

    def handle(self, context: RobotContext):
        self.node.get_logger().info(
            f"[Station-based 固定方向] Robot Put Cleaner 清洗台視覺定位中 "
            f"(Work ID {context.work_id})")

        # 避免循環引用，在需要時才導入
        # 視覺定位完成後進入 CleanerCheckHaveState（檢查清洗機空位）
        from loader_agv.robot_states.put_cleaner.cleaner_check_have_state import CleanerCheckHaveState
        self._handle_vision_steps(
            context, context.robot.PHOTO_CLEANER, CleanerCheckHaveState)
