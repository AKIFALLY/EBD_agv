from rclpy.node import Node
from unloader_agv.robot_context import RobotContext
from agv_base.robot import Robot
from unloader_agv.robot_states.base_robot_state import BaseRobotState


class TakeOvenState(BaseRobotState):
    def __init__(self, node: Node):
        super().__init__(node)
        self.hokuyo_dms_8bit_1 = self.node.hokuyo_dms_8bit_1
        self.step = RobotContext.IDLE
        self.sent = False

    def enter(self):
        self.node.get_logger().info("Unloader Robot Take Oven 目前狀態: TakeOven")
        self.sent = False

    def leave(self):
        self.node.get_logger().info("Unloader Robot Take Oven 離開 TakeOven 狀態")
        self.sent = False

    def _set_hokuyo_busy(self):
        """設定 Hokuyo write_busy"""
        if not self.hokuyo_busy_write_completed:
            try:
                self.hokuyo_dms_8bit_1.write_busy("1")
                self.hokuyo_busy_write_completed = True
                self.node.get_logger().info("✅ Hokuyo write_busy=1 設定完成")
            except Exception as e:
                self.node.get_logger().error(f"❌ Hokuyo write_busy 設定失敗: {e}")

    def _execute_robot_logic(self, context: RobotContext, TAKE_OVEN_PGNO, read_pgno):
        """執行機器人邏輯"""
        match self.step:
            case RobotContext.IDLE:
                self.node.get_logger().info("Unloader Robot Take Oven TAKE OVEN IDLE")
                self.step = RobotContext.CHECK_IDLE
            case RobotContext.CHECK_IDLE:
                self.node.get_logger().info("Unloader Robot Take Oven TAKE OVEN CHECK_IDLE")
                if read_pgno is None:
                    self.node.get_logger().debug("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅Robot狀態為IDLE")
                    self.step = RobotContext.WRITE_CHG_PARAMTER
                else:
                    self.node.get_logger().info("❌Robot狀態不為IDLE")

            case RobotContext.WRITE_CHG_PARAMTER:
                self.node.get_logger().info("Unloader Robot Take Oven TAKE OVEN WRITE CHG PARAMTER")
                if not self.sent:
                    context.robot.update_pgno(Robot.CHG_PARA)
                    self.sent = True
                if context.robot.update_pgno_success:
                    self.node.get_logger().info("✅傳送參數成功")
                    self.sent = False
                    context.robot.update_pgno_success = False
                    self.step = RobotContext.WRITE_CHG_PARA
                elif context.robot.update_pgno_failed:
                    self.node.get_logger().info("❌傳送參數失敗")
                    self.sent = False
                    context.robot.update_pgno_failed = False
                else:
                    self.node.get_logger().info("🕒傳送參數中")

            case RobotContext.WRITE_CHG_PARA:
                self.node.get_logger().info("Unloader Robot Take Oven TAKE OVEN WRITE CHG PARA")
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
                self.node.get_logger().info("Unloader Robot Take Oven TAKE OVEN CHECK_CHG_PARA")
                if read_pgno is None:
                    self.node.get_logger().debug("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == Robot.CHG_PARA:
                    self.node.get_logger().info("✅讀取預執行成功")
                    self.step = RobotContext.WRITE_PGNO
                else:
                    self.node.get_logger().info("❌讀取預執行失敗")

            case RobotContext.WRITE_PGNO:
                self.node.get_logger().info("Unloader Robot Take Oven TAKE OVEN WRITE_PGNO")
                if not self.sent:
                    context.robot.update_pgno(TAKE_OVEN_PGNO)
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
                    self.node.get_logger().info("🕒傳送PGNO中")

            case RobotContext.CHECK_PGNO:
                self.node.get_logger().info("Unloader Robot Take Oven TAKE OVEN CHECK_PGNO")
                if read_pgno is None:
                    self.node.get_logger().debug("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == (TAKE_OVEN_PGNO):
                    self.node.get_logger().info("✅讀取PGNO成功")
                    self.step = RobotContext.ACTING
                elif read_pgno.value == Robot.CHG_PARA:
                    self.node.get_logger().info("🕒讀取PGNO中...")
                else:
                    self.node.get_logger().info("❌讀取PGNO失敗")

            case RobotContext.ACTING:
                self.node.get_logger().info("Unloader Robot Take Oven TAKE OVEN ACTING")
                if read_pgno is None:
                    self.node.get_logger().debug("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == (TAKE_OVEN_PGNO):
                    self.node.get_logger().debug("🤖手臂動作中")
                elif read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅手臂動作完成")
                    self.step = RobotContext.FINISH
                else:
                    self.node.get_logger().info("❌手臂動作失敗")
            case RobotContext.FINISH:
                self.node.get_logger().info("Unloader Robot Take Oven TAKE OVEN Finish")
                if read_pgno is None:
                    self.node.get_logger().debug("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅從烘乾機取貨完成")

                    # 直接進入下一個狀態，TAKE 操作不需要更新資料庫
                    self.node.get_logger().info("✅ Take Oven 完成: 進入放置到AGV狀態")
                    from unloader_agv.robot_states.take_oven.put_agv_state import PutAgvState
                    context.set_state(PutAgvState(self.node))

                    self.step = RobotContext.IDLE
                else:
                    self.node.get_logger().info("❌從烘乾機取貨失敗")

    def handle(self, context: RobotContext):
        self.node.get_logger().info("Unloader Robot Take Oven TakeOven 狀態")

        # 並行執行：Hokuyo write_busy 設定
        self._set_hokuyo_busy()

        # 並行執行：其他操作（不需等待 Hokuyo 完成）
        TAKE_OVEN_PGNO = context.robot.ACTION_FROM + \
            context.robot.OVEN_POSITION + context.robot.NONE_POSITION
        read_pgno = context.robot.read_pgno_response
        context.robot.read_robot_status()

        # 更新 Hokuyo Input
        self._handle_hokuyo_input()

        # 條件執行：只有機器人邏輯需要等待 Hokuyo 完成
        if self.hokuyo_busy_write_completed:
            self._execute_robot_logic(context, TAKE_OVEN_PGNO, read_pgno)
