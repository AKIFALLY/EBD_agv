from rclpy.node import Node
from cargo_mover_agv.robot_context import RobotContext
from agv_base.robot import Robot
from cargo_mover_agv.robot_states.base_robot_state import BaseRobotState


class TakeRackPortState(BaseRobotState):
    def __init__(self, node: Node):
        super().__init__(node)
        self.hokuyo_dms_8bit_1 = self.node.hokuyo_dms_8bit_1
        self.step = RobotContext.IDLE
        self.sent = False
        # hokuyo_input_updated 已移除，因為需要持續更新

    def enter(self):
        self.node.get_logger().info("Robot Entrance 目前狀態: TakeRackPort")
        self.sent = False
        # hokuyo_input_updated 已移除，因為需要持續更新

    def leave(self):
        self.node.get_logger().info("Robot Entrance 離開 TakeRackPort 狀態")
        self.sent = False
        # hokuyo_input_updated 已移除，因為需要持續更新

    def handle(self, context: RobotContext):
        self.node.get_logger().info("Robot Entrance TakeRackPort 狀態")

        # 並行執行：Hokuyo write_busy 設定
        self._set_hokuyo_busy_entrance()

        # 並行執行：其他操作（不需等待 Hokuyo 完成）
        TAKE_RACK_PGNO = context.robot.ACTION_FROM + \
            context.robot.RACK_IN_POSITION + context.robot.NONE_POSITION
        read_pgno = context.robot.read_pgno_response
        context.robot.read_robot_status()

        # 更新 Hokuyo Input - 使用統一方法
        self._handle_hokuyo_input_entrance()
        # 條件執行：只有機器人邏輯需要等待 Hokuyo 完成
        if self.hokuyo_busy_write_completed:
            self._execute_robot_logic(context, TAKE_RACK_PGNO, read_pgno)

    def _execute_robot_logic(self, context: RobotContext, TAKE_RACK_PGNO, read_pgno):
        """執行機器人邏輯"""
        match self.step:
            case RobotContext.IDLE:
                self.node.get_logger().info("Robot Entrance TAKE RACK IDLE")
                self.step = RobotContext.CHECK_IDLE
            case RobotContext.CHECK_IDLE:
                self.node.get_logger().info("Robot Entrance TAKE RACK CHECK_IDLE")
                if read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅Robot狀態為IDLE")
                    self.step = RobotContext.WRITE_CHG_PARAMTER
                else:
                    self.node.get_logger().info("❌Robot狀態不為IDLE")

            case RobotContext.WRITE_CHG_PARAMTER:
                if not self.sent:
                    context.update_rack_box_port()
                    self.sent = True
                if context.robot.update_parameter_success:
                    self.node.get_logger().info("✅更新參數成功")
                    self.sent = False
                    context.robot.update_parameter_success = False
                    self.step = RobotContext.WRITE_CHG_PARA
                elif context.robot.update_parameter_failed:
                    self.node.get_logger().info("❌更新參數失敗")
                    self.sent = False
                    context.robot.update_parameter_failed = False
                else:
                    self.node.get_logger().info("🕒更新參數中")

            case RobotContext.WRITE_CHG_PARA:
                self.node.get_logger().info("Robot Entrance TAKE RACK WRITE CHG PARA")
                if not self.sent:
                    context.robot.update_pgno(Robot.CHG_PARA)
                    self.sent = True
                if context.robot.update_pgno_success:
                    self.node.get_logger().info("✅傳送預執行成功")
                    self.sent = False
                    context.robot.update_pgno_success = False
                    self.step = RobotContext.CHECK_CHG_PARA

                elif context.robot.update_pgno_failed:
                    self.node.get_logger().info("❌傳送預執行失敗")
                    self.sent = False
                    context.robot.update_pgno_failed = False
                else:
                    self.node.get_logger().info("🕒傳送預執行中")

            case RobotContext.CHECK_CHG_PARA:
                self.node.get_logger().info("Robot Entrance TAKE RACK CHECK CHG PARA")
                if read_pgno.value == Robot.CHG_PARA:
                    self.node.get_logger().info("✅讀取預執行成功")
                    self.step = RobotContext.WRITE_PGNO
                elif read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("🕒讀取預執行中...")
                else:
                    self.node.get_logger().info("❌讀取預執行失敗")

            case RobotContext.WRITE_PGNO:
                self.node.get_logger().info("Robot Entrance TAKE RACK WRITE_PGNO")
                if not self.sent:
                    context.robot.update_pgno(TAKE_RACK_PGNO)
                    self.sent = True
                if context.robot.update_pgno_success:
                    self.node.get_logger().info("✅傳送PGNO成功")
                    self.sent = False
                    context.robot.update_pgno_success = False
                    self.step = RobotContext.CHECK_PGNO
                elif context.robot.update_pgno_failed:
                    self.node.get_logger().info("❌傳送PGNO失敗")
                    self.sent = False
                    context.robot.update_pgno_failed = False
                else:
                    self.node.get_logger().info("🕒傳送PGNO中...")

            case RobotContext.CHECK_PGNO:
                self.node.get_logger().info("Robot Entrance TAKE RACK CHECK_PGNO")
                if read_pgno.value == (TAKE_RACK_PGNO):
                    self.node.get_logger().info("✅讀取PGNO成功")
                    self.step = RobotContext.ACTING
                elif read_pgno.value == Robot.CHG_PARA:
                    self.node.get_logger().info("🕒讀取PGNO中...")
                else:
                    self.node.get_logger().info("❌讀取PGNO失敗")

            case RobotContext.ACTING:
                self.node.get_logger().info("Robot Entrance TAKE RACK ACTING")
                if read_pgno.value == (TAKE_RACK_PGNO):
                    self.node.get_logger().info("🤖手臂動作中")
                elif read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅手臂動作完成")
                    # 這裡可以添加完成後的邏輯
                    # 例如：context.set_state(AgvIdleState(self.node))
                    self.step = RobotContext.FINISH
                else:
                    self.node.get_logger().info("❌手臂動作失敗")

            case RobotContext.FINISH:
                self.node.get_logger().info("Robot Entrance TAKE RACK Finish")
                if read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅拿RACK完成")
                    from cargo_mover_agv.robot_states.entrance.put_tranfer_state import PutTransferState
                    context.set_state(PutTransferState(self.node))
                    self.step = RobotContext.IDLE
                else:
                    self.node.get_logger().info("❌拿RACK失敗")
