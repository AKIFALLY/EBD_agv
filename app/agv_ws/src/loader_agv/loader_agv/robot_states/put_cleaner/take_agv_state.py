from rclpy.node import Node
from loader_agv.robot_context import RobotContext
from agv_base.robot import Robot
from agv_base.hokuyo_dms_8bit import HokuyoDMS8Bit


from loader_agv.robot_states.base_robot_state import BaseRobotState


class TakeAgvState(BaseRobotState):

    def __init__(self, node: Node):
        super().__init__(node)
        self.node = node
        # 動態參數計算，與 agv_port_check_have_state.py 中的參數一致
        self.port_id_address = self.node.room_id * 1000 + 100
        self.step = RobotContext.IDLE
        self.sent = False

    def enter(self):
        self.node.get_logger().info("Loader Robot Put Cleaner 目前狀態: TakeAgv")
        self.sent = False

    def leave(self):
        self.node.get_logger().info("Loader Robot Put Cleaner 離開 TakeAgv 狀態")
        self.sent = False

    def handle(self, context: RobotContext):
        self.node.get_logger().info("Loader Robot Put Cleaner TakeAgv 狀態")

        # 並行執行：Hokuyo write_busy 設定
        self._set_hokuyo_busy()

        # 並行執行：其他操作（不需等待 Hokuyo 完成）
        # 修正 PGNO 常數定義，適用於 loader_agv
        TAKE_LOADER_AGV_PGNO = context.robot.ACTION_FROM + \
            context.robot.AGV_POSITION + context.robot.NONE_POSITION
        read_pgno = context.robot.read_pgno_response
        context.robot.read_robot_status()

        # 更新 Hokuyo Input - 使用統一方法
        self._handle_hokuyo_input()

        # 條件執行：只有機器人邏輯需要等待 Hokuyo 完成
        if self.hokuyo_busy_write_completed:
            self._execute_robot_logic(context, TAKE_LOADER_AGV_PGNO, read_pgno)

    def _execute_robot_logic(self, context: RobotContext, TAKE_LOADER_AGV_PGNO, read_pgno):
        """執行機器人邏輯"""
        match self.step:
            case RobotContext.IDLE:
                self.node.get_logger().info("Loader Robot Put Cleaner TAKE LOADER AGV IDLE")
                self.step = RobotContext.CHECK_IDLE
            case RobotContext.CHECK_IDLE:
                self.node.get_logger().info("Loader Robot Put Cleaner TAKE LOADER AGV CHECK_IDLE")
                if read_pgno is None:
                    self.node.get_logger().info("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅Robot狀態為IDLE")
                    self.step = RobotContext.WRITE_CHG_PARAMTER
                else:
                    self.node.get_logger().info("❌Robot狀態不為IDLE")

            case RobotContext.WRITE_CHG_PARAMTER:
                if not self.sent:
                    context.update_port_parameters()
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
                self.node.get_logger().info("Loader Robot Put Cleaner TAKE LOADER AGV WRITE CHG PARA")
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
                self.node.get_logger().info("Loader Robot Put Cleaner TAKE LOADER AGV CHECK CHG PARA")
                if read_pgno.value == Robot.CHG_PARA:
                    self.node.get_logger().info("✅讀取預執行成功")
                    self.step = RobotContext.WRITE_PGNO
                elif read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("🕒讀取預執行中...")
                else:
                    self.node.get_logger().info("❌讀取預執行失敗")

            case RobotContext.WRITE_PGNO:
                self.node.get_logger().info("Loader Robot Put Cleaner TAKE LOADER AGV WRITE_PGNO")
                if not self.sent:
                    context.robot.update_pgno(TAKE_LOADER_AGV_PGNO)
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
                self.node.get_logger().info("Loader Robot Put Cleaner TAKE LOADER AGV CHECK_PGNO")
                if read_pgno is None:
                    self.node.get_logger().info("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == (TAKE_LOADER_AGV_PGNO):
                    self.node.get_logger().info("✅讀取PGNO成功")
                    self.step = RobotContext.ACTING
                elif read_pgno.value == Robot.CHG_PARA:
                    self.node.get_logger().info("🕒讀取PGNO中...")
                else:
                    self.node.get_logger().info("❌讀取PGNO失敗")

            case RobotContext.ACTING:
                self.node.get_logger().info("Loader Robot Put Cleaner TAKE LOADER AGV ACTING")
                if read_pgno is None:
                    self.node.get_logger().info("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == (TAKE_LOADER_AGV_PGNO):
                    self.node.get_logger().info("🤖手臂動作中")
                elif read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅手臂動作完成")
                    self.step = RobotContext.FINISH
                else:
                    self.node.get_logger().info("❌手臂動作失敗")
            case RobotContext.FINISH:
                self.node.get_logger().info("Loader Robot Put Cleaner TAKE LOADER AGV Finish")
                if read_pgno is None:
                    self.node.get_logger().info("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅取AGV箱完成")

                    # 直接進入下一個狀態 - 轉換到 PutCleanerState
                    self.node.get_logger().info("✅ Take AGV 完成: 進入 PutCleanerState")
                    from loader_agv.robot_states.put_cleaner.put_cleaner_state import PutCleanerState
                    context.set_state(PutCleanerState(self.node))

                    self.step = RobotContext.IDLE
                else:
                    self.node.get_logger().info("❌取AGV箱失敗")
