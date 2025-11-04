from rclpy.node import Node
from loader_agv.robot_context import RobotContext
from agv_base.robot import Robot
from agv_base.hokuyo_dms_8bit import HokuyoDMS8Bit


from loader_agv.robot_states.base_robot_state import BaseRobotState


class TakeSoakerState(BaseRobotState):
    def __init__(self, node: Node):
        super().__init__(node)
        self.node = node
        # 動態參數設定：浸泡設備參數
        self.port_id_address = self.node.room_id * 1000 + 40
        self.step = RobotContext.IDLE
        self.sent = False

    def enter(self):
        self.node.get_logger().info(
            "[Station-based 1格] Loader Robot Take Soaker 目前狀態: TakeSoaker")
        self.sent = False

    def leave(self):
        self.node.get_logger().info(
            "[Station-based 1格] Loader Robot Take Soaker 離開 TakeSoaker 狀態")
        self.sent = False

    def handle(self, context: RobotContext):
        # 單格處理：從泡藥機取1格 → 機械臂
        source_soaker_port = context.get_soaker_port
        target_agv_port = context.get_loader_agv_port_side

        self.node.get_logger().info(
            f"[Station-based 1格] 取料 (Work ID {context.work_id})")
        self.node.get_logger().info(
            f"來源: 泡藥機 Port {source_soaker_port} (Station {source_soaker_port:02d}) → 目標: 機械臂")
        self.node.get_logger().info(
            f"最終目標: AGV Port {target_agv_port} (偶數層策略)")

        # 並行執行：Hokuyo write_busy 設定
        self._set_hokuyo_busy()

        # 並行執行：其他操作（不需等待 Hokuyo 完成）
        # PGNO 常數調整：TAKE_SOAKER_PGNO
        TAKE_SOAKER_PGNO = context.robot.ACTION_FROM + \
            context.robot.SOAKER_POSISION + context.robot.NONE_POSITION
        read_pgno = context.robot.read_pgno_response
        context.robot.read_robot_status()

        # 更新 Hokuyo Input - 使用統一方法
        self._handle_hokuyo_input()

        # 條件執行：只有機器人邏輯需要等待 Hokuyo 完成
        if self.hokuyo_busy_write_completed:
            self._execute_robot_logic(context, TAKE_SOAKER_PGNO, read_pgno)

    def _execute_robot_logic(self, context: RobotContext, TAKE_SOAKER_PGNO, read_pgno):
        """執行機器人邏輯 - 單格處理"""
        match self.step:
            case RobotContext.IDLE:
                self.node.get_logger().info(
                    "[Station-based 1格] Loader Robot Take Soaker TAKE SOAKER IDLE")
                self.step = RobotContext.CHECK_IDLE
            case RobotContext.CHECK_IDLE:
                self.node.get_logger().info(
                    "[Station-based 1格] Loader Robot Take Soaker TAKE SOAKER CHECK_IDLE")
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
                    self.step = RobotContext.CHECK_CHG_PARAMETER
                elif context.robot.update_parameter_failed:
                    self.node.get_logger().info("❌更新參數失敗")
                    self.sent = False
                    context.robot.update_parameter_failed = False
                else:
                    self.node.get_logger().info("🕒更新參數中")

            case RobotContext.CHECK_CHG_PARAMETER:
                self.node.get_logger().info("Robot Take Soaker TAKE SOAKER CHECK CHG PARAMETER")

                # 構建預期參數字典
                expected_params = {}

                # 檢查 soaker_port → W118(layer_z_soaker), W119(layer_y_soaker)
                # 根據新邏輯，W118 固定為 1（不隨 port 變化）
                expected_params['w118'] = 1
                expected_params['w119'] = 0

                self.node.get_logger().info(
                    f"預期檢查: soaker_port={context.get_soaker_port} → "
                    f"W118=1 (固定值), W119=0"
                )

                # 執行檢查
                if self._handle_check_chg_parameter(context, expected_params):
                    # 檢查通過，進入下一步驟
                    self.step = RobotContext.WRITE_CHG_PARA
                # 否則繼續停留在此步驟

            case RobotContext.WRITE_CHG_PARA:
                self.node.get_logger().info("Loader Robot Take Soaker TAKE SOAKER WRITE CHG PARA")
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
                self.node.get_logger().info("Loader Robot Take Soaker TAKE SOAKER CHECK CHG PARA")
                if read_pgno.value == Robot.CHG_PARA:
                    self.node.get_logger().info("✅讀取預執行成功")
                    self.step = RobotContext.WRITE_PGNO
                elif read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("🕒讀取預執行中...")
                else:
                    self.node.get_logger().info("❌讀取預執行失敗")

            case RobotContext.WRITE_PGNO:
                self.node.get_logger().info("Loader Robot Take Soaker TAKE SOAKER WRITE_PGNO")
                if not self.sent:
                    context.robot.update_pgno(TAKE_SOAKER_PGNO)
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
                self.node.get_logger().info("Loader Robot Take Soaker TAKE SOAKER CHECK_PGNO")
                if read_pgno is None:
                    self.node.get_logger().info("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == (TAKE_SOAKER_PGNO):
                    self.node.get_logger().info("✅讀取PGNO成功")
                    self.step = RobotContext.ACTING
                elif read_pgno.value == Robot.CHG_PARA:
                    self.node.get_logger().info("🕒讀取PGNO中...")
                else:
                    self.node.get_logger().info("❌讀取PGNO失敗")

            case RobotContext.ACTING:
                self.node.get_logger().info("Loader Robot Take Soaker TAKE SOAKER ACTING")
                if read_pgno is None:
                    self.node.get_logger().info("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == (TAKE_SOAKER_PGNO):
                    self.node.get_logger().info("🤖手臂動作中")
                elif read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅手臂動作完成")
                    self.step = RobotContext.FINISH
                else:
                    self.node.get_logger().info("❌手臂動作失敗")
            case RobotContext.FINISH:
                self.node.get_logger().info(
                    "[Station-based 1格] Loader Robot Take Soaker TAKE SOAKER Finish")
                if read_pgno is None:
                    self.node.get_logger().info("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅取泡藥機完成")

                    # 單格處理：直接進入 PutAgvState（無批量循環）
                    self.node.get_logger().info(
                        f"✅ [Station-based 1格] Take Soaker 完成: 進入 PutAgvState")
                    self.node.get_logger().info(
                        f"完整路徑: 泡藥機 Port {context.get_soaker_port} → 機械臂 → "
                        f"AGV Port {context.get_loader_agv_port_side}")
                    from loader_agv.robot_states.take_soaker.put_agv_state import PutAgvState
                    context.set_state(PutAgvState(self.node))

                    self.step = RobotContext.IDLE
                else:
                    self.node.get_logger().info("❌取泡藥機失敗")
