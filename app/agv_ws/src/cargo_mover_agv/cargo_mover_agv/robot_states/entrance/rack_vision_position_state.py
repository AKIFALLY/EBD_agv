#from agv_base.states.state import State
from rclpy.node import Node
from cargo_mover_agv.robot_context import RobotContext  # 新增的匯入
from agv_base.robot import Robot
from cargo_mover_agv.robot_states.base_robot_state import BaseVisionPositionState


class RackVisionPositionState(BaseVisionPositionState):

    def enter(self):
        self.node.get_logger().info("Robot Entrance 目前狀態: RackVisionPosition")
        self._reset_state()

    def leave(self):
        self.node.get_logger().info("Robot Entrance 離開 RackVisionPosition 狀態")
        self._reset_state()

    def handle(self, context: RobotContext):
        # 確認拍照位置是上層還是下層
        photo_up_or_down = (Robot.PHOTO_RACK_UP
                            if 1 <= context.get_rack_port <= 8 or 17 <= context.get_rack_port <= 24
                            else Robot.PHOTO_RACK_DOWN)

        self.node.get_logger().info("Robot Entrance RackVisionPosition 狀態")

        # 如果拍照位置相同，直接完成
        if photo_up_or_down == context.rack_photo_up_or_down_buffer:
            self.step = RobotContext.FINISH

        # 處理視覺步驟
        if self.step == RobotContext.FINISH:
            context.rack_photo_up_or_down_buffer = photo_up_or_down
            from .take_rack_port_state import TakeRackPortState
            context.set_state(TakeRackPortState(self.node))
            self.step = RobotContext.IDLE
        else:
            from .take_rack_port_state import TakeRackPortState
            self._handle_vision_steps(
                context, photo_up_or_down, TakeRackPortState)

        # 在完成時更新緩衝
        if self.step == RobotContext.FINISH:
            context.rack_photo_up_or_down_buffer = photo_up_or_down
