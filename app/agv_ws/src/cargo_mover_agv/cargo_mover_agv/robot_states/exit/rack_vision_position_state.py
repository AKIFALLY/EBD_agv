from agv_base.states.state import State
from rclpy.node import Node
from cargo_mover_agv.robot_context import RobotContext  # 新增的匯入
from agv_base.robot import Robot


class RackVisionPositionState(State):
    def __init__(self, node: Node):
        super().__init__(node)
        self.step = RobotContext.IDLE
        self.sent = False

    def enter(self):
        self.node.get_logger().info("Robot Exit 目前狀態: RackVisionPosition")
        self.sent = False

    def leave(self):
        self.node.get_logger().info("Robot Exit 離開 RackVisionPosition 狀態")
        self.sent = False

    def handle(self, context: RobotContext):

        read_pgno = context.robot.read_pgno_response
        # 確認拍照位置是上層還是下層
        if 1 <= context.get_rack_port <= 8 or 17 <= context.get_rack_port <= 24:
            photo_up_or_down = Robot.PHOTO_RACK_UP

        else:
            photo_up_or_down = Robot.PHOTO_RACK_DOWN

        self.node.get_logger().info("Robot Exit RackVisionPosition 狀態")
        # 讀取PGNO狀態
        context.robot.read_pgno()
        # 根據當前步驟進行狀態處理
        match self.step:
            case RobotContext.IDLE:
                self.node.get_logger().info("Robot Exit RACK視覺定位中 IDLE")
                if photo_up_or_down != context.rack_photo_up_or_down_buffer:
                    self.step = RobotContext.CHECK_IDLE
                else:
                    self.step = RobotContext.FINISH

            case RobotContext.CHECK_IDLE:
                self.node.get_logger().info("Robot Exit RACK視覺定位中 CHECK_IDLE")
                if read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅Robot狀態為IDLE")
                    self.step = RobotContext.WRITE_CHG_PARA
                else:
                    self.node.get_logger().info("❌Robot狀態不為IDLE")

            case RobotContext.WRITE_CHG_PARA:
                self.node.get_logger().info("Robot Exit RACK視覺定位中 WRITE CHG PARA")
                if not self.sent:
                    context.robot.update_pgno(Robot.CHG_PARA)
                    self.sent = True
                if context.robot.update_pgno_success:
                    self.node.get_logger().info("✅傳送預執行成功")
                    self.step = RobotContext.CHECK_CHG_PARA
                    self.sent = False
                    context.robot.update_pgno_success = False
                elif context.robot.update_pgno_failed:
                    self.node.get_logger().info("❌傳送預執行失敗")
                    self.sent = False
                    context.robot.update_pgno_failed = False
                else:
                    self.node.get_logger().info("🕒傳送預執行中...")

            case RobotContext.CHECK_CHG_PARA:
                self.node.get_logger().info("Robot Exit RACK視覺定位中 CHECK CHG PARA")
                if read_pgno.value == Robot.CHG_PARA:
                    self.node.get_logger().info("✅讀取預執行成功")
                    self.step = RobotContext.WRITE_PGNO
                elif read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("🕒讀取預執行中...")
                else:
                    self.node.get_logger().info("❌讀取預執行失敗")

            case RobotContext.WRITE_PGNO:
                self.node.get_logger().info("Robot Exit RACK視覺定位中 WRITE_PGNO")
                if not self.sent:
                    context.robot.update_pgno(photo_up_or_down)
                    self.sent = True
                if context.robot.update_pgno_success:
                    self.node.get_logger().info("✅傳送PGNO成功")
                    self.step = RobotContext.CHECK_PGNO
                    self.sent = False
                    context.robot.update_pgno_success = False
                elif context.robot.update_pgno_failed:
                    self.node.get_logger().info("❌傳送PGNO失敗")
                    self.sent = False
                else:
                    self.node.get_logger().info("🕒傳送PGNO中...")

            case RobotContext.CHECK_PGNO:
                self.node.get_logger().info("Robot Exit RACK視覺定位中 CHECK_PGNO")
                if read_pgno.value == photo_up_or_down:
                    self.node.get_logger().info("✅讀取PGNO成功")
                    self.step = RobotContext.ACTING
                elif read_pgno.value == Robot.CHG_PARA:
                    self.node.get_logger().info("🕒讀取PGNO中...")
                else:
                    self.node.get_logger().info("❌讀取PGNO失敗")
                    self.step = RobotContext.CHECK_PGNO

            case RobotContext.ACTING:
                self.node.get_logger().info("Robot Exit RACK視覺定位中 ACTING")
                if read_pgno.value == photo_up_or_down:
                    self.node.get_logger().info("🤖手臂動作中")
                elif read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅手臂動作完成")
                    # 這裡可以添加完成後的邏輯
                    # 例如：context.set_state(AgvIdleState(self.node))
                    self.step = RobotContext.FINISH
                else:
                    self.node.get_logger().info("❌手臂動作失敗")

            case RobotContext.FINISH:
                self.node.get_logger().info("Robot Exit RACK視覺定位中 Finish")
                if read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅拍照完成")
                    if photo_up_or_down == Robot.PHOTO_RACK_UP:
                        self.node.get_logger().info("✅拍照位置為上層")
                        context.rack_photo_up_or_down_buffer = Robot.PHOTO_RACK_UP
                    elif photo_up_or_down == Robot.PHOTO_RACK_DOWN:
                        self.node.get_logger().info("✅拍照位置為下層")
                        context.rack_photo_up_or_down_buffer = Robot.PHOTO_RACK_DOWN
                    else:
                        self.node.get_logger().info("❌拍照位置錯誤，無法確定上層或下層")
                        context.rack_photo_up_or_down_buffer = None

                    from cargo_mover_agv.robot_states.exit.take_transfer_state import TakeTransferState
                    context.set_state(TakeTransferState(self.node))
                    self.step = RobotContext.IDLE
