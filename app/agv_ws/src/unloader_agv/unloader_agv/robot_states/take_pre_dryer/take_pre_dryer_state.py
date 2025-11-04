from rclpy.node import Node
from unloader_agv.robot_context import RobotContext
from agv_base.robot import Robot
from unloader_agv.robot_states.base_robot_state import BaseRobotState


class TakePreDryerState(BaseRobotState):
    def __init__(self, node: Node):
        super().__init__(node)
        self.hokuyo_dms_8bit_1 = self.node.hokuyo_dms_8bit_1
        self.step = RobotContext.IDLE
        self.sent = False

    def enter(self):
        self.node.get_logger().info("Unloader Robot Take Pre Dryer 目前狀態: TakePreDryer")
        self.sent = False

    def leave(self):
        self.node.get_logger().info("Unloader Robot Take Pre Dryer 離開 TakePreDryer 狀態")
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

    def _execute_robot_logic(self, context: RobotContext, TAKE_PRE_DRYER_PGNO, read_pgno):
        """執行機器人邏輯"""
        match self.step:
            case RobotContext.IDLE:
                self.node.get_logger().info("Unloader Robot Take Pre Dryer TAKE PRE DRYER IDLE")
                self.step = RobotContext.CHECK_IDLE
            case RobotContext.CHECK_IDLE:
                self.node.get_logger().info("Unloader Robot Take Pre Dryer TAKE PRE DRYER CHECK_IDLE")
                if read_pgno is None:
                    self.node.get_logger().debug("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅Robot狀態為IDLE")
                    self.step = RobotContext.WRITE_CHG_PARAMTER
                else:
                    self.node.get_logger().info("❌Robot狀態不為IDLE")

            case RobotContext.WRITE_CHG_PARAMTER:
                self.node.get_logger().info("Unloader Robot Take Pre Dryer TAKE PRE DRYER WRITE CHG PARAMTER")
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
                self.node.get_logger().info("Unloader Robot Take Pre Dryer CHECK CHG PARAMETER")

                # 構建預期參數字典
                expected_params = {}

                # 檢查 pre_dryer_port → W114
                # layer_z = 1 if port <= 4 else 2, layer_y = 0
                layer_z_pre_dryer = 1 if context.get_pre_dryer_port <= 4 else 2
                layer_y_pre_dryer = 0
                expected_params['w114'] = (layer_z_pre_dryer | (layer_y_pre_dryer << 16))

                self.node.get_logger().info(
                    f"預期檢查: pre_dryer_port={context.get_pre_dryer_port} → "
                    f"W114 (z={layer_z_pre_dryer}, y={layer_y_pre_dryer})")

                # 執行檢查
                if self._handle_check_chg_parameter(context, expected_params):
                    self.step = RobotContext.WRITE_CHG_PARA

            case RobotContext.WRITE_CHG_PARA:
                self.node.get_logger().info("Unloader Robot Take Pre Dryer TAKE PRE DRYER WRITE CHG PARA")
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
                self.node.get_logger().info("Unloader Robot Take Pre Dryer TAKE PRE DRYER CHECK_CHG_PARA")
                if read_pgno is None:
                    self.node.get_logger().debug("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == Robot.CHG_PARA:
                    self.node.get_logger().info("✅讀取預執行成功")
                    self.step = RobotContext.WRITE_PGNO
                else:
                    self.node.get_logger().info("❌讀取預執行失敗")

            case RobotContext.WRITE_PGNO:
                self.node.get_logger().info("Unloader Robot Take Pre Dryer TAKE PRE DRYER WRITE_PGNO")
                if not self.sent:
                    context.robot.update_pgno(TAKE_PRE_DRYER_PGNO)
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
                self.node.get_logger().info("Unloader Robot Take Pre Dryer TAKE PRE DRYER CHECK_PGNO")
                if read_pgno is None:
                    self.node.get_logger().debug("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == (TAKE_PRE_DRYER_PGNO):
                    self.node.get_logger().info("✅讀取PGNO成功")
                    self.step = RobotContext.ACTING
                elif read_pgno.value == Robot.CHG_PARA:
                    self.node.get_logger().info("🕒讀取PGNO中...")
                else:
                    self.node.get_logger().info("❌讀取PGNO失敗")

            case RobotContext.ACTING:
                self.node.get_logger().info("Unloader Robot Take Pre Dryer TAKE PRE DRYER ACTING")
                if read_pgno is None:
                    self.node.get_logger().debug("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == (TAKE_PRE_DRYER_PGNO):
                    self.node.get_logger().debug("🤖手臂動作中")
                elif read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅手臂動作完成")
                    self.step = RobotContext.FINISH
                else:
                    self.node.get_logger().info("❌手臂動作失敗")
            case RobotContext.FINISH:
                self.node.get_logger().info("Unloader Robot Take Pre Dryer TAKE PRE DRYER Finish")
                if read_pgno is None:
                    self.node.get_logger().debug("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅從預乾燥機取貨完成")

                    # 直接進入下一個狀態，TAKE 操作不需要更新資料庫
                    self.node.get_logger().info("✅ Take Pre Dryer 完成: 進入放置到AGV狀態")
                    from unloader_agv.robot_states.take_pre_dryer.put_agv_state import PutAgvState
                    context.set_state(PutAgvState(self.node))

                    self.step = RobotContext.IDLE
                else:
                    self.node.get_logger().info("❌從預乾燥機取貨失敗")

    def handle(self, context: RobotContext):
        # 根據當前循環次數獲取對應的ports
        if hasattr(context, 'take_put_current_batch') and context.take_put_current_batch:
            current_ports = context.take_put_current_batch
            context.get_pre_dryer_port = current_ports[0]  # 使用當前批次的第一個port

            cycle_num = context.take_put_cycle_count + 1
            self.node.get_logger().info(
                f"Unloader Robot Take Pre Dryer 狀態 (第{cycle_num}次) - "
                f"取 pre_dryer ports {current_ports}")
        else:
            self.node.get_logger().info("Unloader Robot Take Pre Dryer TakePreDryer 狀態")

        # 並行執行：Hokuyo write_busy 設定
        self._set_hokuyo_busy()

        # 並行執行：其他操作（不需等待 Hokuyo 完成）
        TAKE_PRE_DRYER_PGNO = context.robot.ACTION_FROM + \
            context.robot.PRE_DRYER_POSITION + context.robot.NONE_POSITION
        read_pgno = context.robot.read_pgno_response
        context.robot.read_robot_status()

        # 更新 Hokuyo Input
        self._handle_hokuyo_input()

        # 條件執行：只有機器人邏輯需要等待 Hokuyo 完成
        if self.hokuyo_busy_write_completed:
            self._execute_robot_logic(context, TAKE_PRE_DRYER_PGNO, read_pgno)
