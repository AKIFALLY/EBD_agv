from agv_base.states.state import State
from rclpy.node import Node
from loader_agv.robot_context import RobotContext
from agv_base.robot import Robot
import time


class BaseRobotState(State):
    """機器人狀態的基礎類別，提供共用功能"""

    def __init__(self, node: Node):
        super().__init__(node)
        self.step = RobotContext.IDLE
        self.sent = False
        # Hokuyo write_busy 相關變數
        self.hokuyo_busy_write_completed = False
        # Hokuyo Input 頻率控制變數（每秒一次更新）
        self.last_hokuyo_input_update_time = 0.0
        self.hokuyo_input_update_interval = 1.0  # 1秒間隔

    def _reset_state(self):
        """重置共用狀態"""
        self.step = RobotContext.IDLE
        self.sent = False
        self.hokuyo_busy_write_completed = False
        self.last_hokuyo_input_update_time = 0.0

    def _set_hokuyo_busy(self):
        """設定 Hokuyo write_busy = 1 (loader_agv 使用 hokuyo_dms_8bit_1)"""
        if not self.hokuyo_busy_write_completed:
            try:
                self.node.hokuyo_dms_8bit_1.write_busy("1")
                self.node.get_logger().info("✅ Hokuyo_1 write_busy=1 設定完成")
                self.hokuyo_busy_write_completed = True
            except Exception as e:
                self.node.get_logger().error(f"❌ Hokuyo write_busy 設定失敗: {e}")
                # 即使失敗也標記為完成，避免無限重試
                self.hokuyo_busy_write_completed = True

    def _handle_hokuyo_input(self):
        """統一處理 Hokuyo Input 更新 - 頻率控制版本（每秒一次）"""
        hokuyo = self.node.hokuyo_dms_8bit_1
        current_time = time.time()

        # 頻率控制：只有當距離上次更新超過指定間隔時才執行更新
        if current_time - self.last_hokuyo_input_update_time >= self.hokuyo_input_update_interval:
            hokuyo.update_hokuyo_input()
            self.last_hokuyo_input_update_time = current_time
            self.node.get_logger().debug(
                f"Hokuyo Input 更新執行 (loader_agv) - 間隔: {current_time - self.last_hokuyo_input_update_time:.2f}s")

        # 處理成功/失敗狀態（無論是否執行更新都要檢查）
        if hokuyo.hokuyo_input_success:
            self.node.get_logger().info("Hokuyo Input 更新成功")
            hokuyo.hokuyo_input_success = False

        if hokuyo.hokuyo_input_failed:
            self.node.get_logger().error("Hokuyo Input 更新失敗")
            hokuyo.hokuyo_input_failed = False

    def _handle_robot_step(self, context: RobotContext, current_step: int, next_step: int,
                           operation_func, success_flag: str, failed_flag: str, step_name: str):
        """統一處理機器人步驟操作"""
        if self.step != current_step:
            return False

        if not self.sent:
            operation_func()
            self.sent = True

        robot = context.robot
        if getattr(robot, success_flag):
            self.node.get_logger().info(f"✅{step_name}成功")
            setattr(robot, success_flag, False)
            self.sent = False
            self.step = next_step
            return True
        elif getattr(robot, failed_flag):
            self.node.get_logger().error(f"❌{step_name}失敗")
            setattr(robot, failed_flag, False)
            self.sent = False
        else:
            self.node.get_logger().debug(f"⏳等待{step_name}")
        return False

    def _print_separator(self):
        """列印分隔線 - 已移除以減少日誌洪水"""
        pass


class BaseVisionPositionState(BaseRobotState):
    """視覺定位狀態的基礎類別"""

    def _handle_vision_steps(self, context: RobotContext, target_pgno: int, next_state_class):
        """處理視覺定位的標準步驟"""
        read_pgno = context.robot.read_pgno_response
        context.robot.read_robot_status()

        step_handlers = {
            RobotContext.IDLE: lambda: self._set_next_step(RobotContext.CHECK_IDLE),
            RobotContext.CHECK_IDLE: lambda: self._check_idle(read_pgno),
            RobotContext.WRITE_CHG_PARA: lambda: self._write_chg_para(context),
            RobotContext.CHECK_CHG_PARA: lambda: self._check_chg_para(read_pgno),
            RobotContext.WRITE_PGNO: lambda: self._write_pgno(context, target_pgno),
            RobotContext.CHECK_PGNO: lambda: self._check_pgno(read_pgno, target_pgno),
            RobotContext.ACTING: lambda: self._handle_acting(read_pgno),
            RobotContext.FINISH: lambda: self._handle_finish(
                context, read_pgno, next_state_class)
        }

        handler = step_handlers.get(self.step)
        if handler:
            handler()

    def _set_next_step(self, next_step):
        self.step = next_step

    def _check_idle(self, read_pgno):
        if read_pgno is None:
            self.node.get_logger().info("⏳等待讀取PGNO回應...")
            return
        if read_pgno.value == Robot.IDLE:
            self.node.get_logger().info("✅Robot狀態為IDLE")
            self.step = RobotContext.WRITE_CHG_PARA
        else:
            self.node.get_logger().error("❌Robot狀態不為IDLE")

    def _write_chg_para(self, context):
        return self._handle_robot_step(
            context, RobotContext.WRITE_CHG_PARA, RobotContext.CHECK_CHG_PARA,
            lambda: context.robot.update_pgno(Robot.CHG_PARA),
            "update_pgno_success", "update_pgno_failed", "傳送預執行"
        )

    def _check_chg_para(self, read_pgno):
        if read_pgno is None:
            self.node.get_logger().info("⏳等待讀取PGNO回應...")
            return
        if read_pgno.value == Robot.CHG_PARA:
            self.node.get_logger().info("✅讀取預執行成功")
            self.step = RobotContext.WRITE_PGNO
        elif read_pgno.value == Robot.IDLE:
            self.node.get_logger().info("🕒讀取預執行中...")
        else:
            self.node.get_logger().error("❌讀取預執行失敗")

    def _write_pgno(self, context, target_pgno):
        return self._handle_robot_step(
            context, RobotContext.WRITE_PGNO, RobotContext.CHECK_PGNO,
            lambda: context.robot.update_pgno(target_pgno),
            "update_pgno_success", "update_pgno_failed", "傳送PGNO"
        )

    def _check_pgno(self, read_pgno, target_pgno):
        if read_pgno is None:
            self.node.get_logger().info("⏳等待讀取PGNO回應...")
            return
        if read_pgno.value == target_pgno:
            self.node.get_logger().info("✅讀取PGNO成功")
            self.step = RobotContext.ACTING
        elif read_pgno.value == Robot.CHG_PARA:
            self.node.get_logger().info("🕒讀取PGNO中...")
        else:
            self.node.get_logger().error("❌讀取PGNO失敗")

    def _handle_acting(self, read_pgno):
        if read_pgno is None:
            self.node.get_logger().info("⏳等待讀取PGNO回應...")
            return
        if read_pgno.value == Robot.IDLE:
            self.node.get_logger().info("✅手臂動作完成")
            self.step = RobotContext.FINISH
        else:
            self.node.get_logger().info("🤖手臂動作中")

    def _handle_finish(self, context, read_pgno, next_state_class):
        if read_pgno is None:
            self.node.get_logger().info("⏳等待讀取PGNO回應...")
            return
        if read_pgno.value == Robot.IDLE:
            self.node.get_logger().info("✅操作完成")
            context.set_state(next_state_class(self.node))
            self.step = RobotContext.IDLE
