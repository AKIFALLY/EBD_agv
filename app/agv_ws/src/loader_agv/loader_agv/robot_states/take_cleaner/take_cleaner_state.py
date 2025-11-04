from rclpy.node import Node
from loader_agv.robot_context import RobotContext
from agv_base.robot import Robot
from agv_base.hokuyo_dms_8bit import HokuyoDMS8Bit


from loader_agv.robot_states.base_robot_state import BaseRobotState


class TakeCleanerState(BaseRobotState):
    def __init__(self, node: Node):
        super().__init__(node)
        self.node = node
        # 動態參數設定：清洗機參數
        self.port_id_address = self.node.room_id * 1000 + 30
        self.step = RobotContext.IDLE
        self.sent = False

    def enter(self):
        self.node.get_logger().info(
            "[Station-based 批量] Loader Robot Take Cleaner 目前狀態: TakeCleaner")
        self.sent = False

    def leave(self):
        self.node.get_logger().info(
            "[Station-based 批量] Loader Robot Take Cleaner 離開 TakeCleaner 狀態")
        self.sent = False

    def handle(self, context: RobotContext):
        # 批量取料 port 映射：根據計數器決定來源 port
        # Station-based 設計：
        # - 第1次 (cleaner_take_count=0): Cleaner Port 1 → 機械臂
        # - 第2次 (cleaner_take_count=1): Cleaner Port 2 → 機械臂
        source_port = context.cleaner_ports[context.cleaner_take_count]
        context.get_cleaner_port = source_port

        # 更新當前使用的 carrier_id
        context.carrier_id = context.cleaner_carrier_ids[context.cleaner_take_count]

        target_agv_port = [1, 3][context.cleaner_take_count]

        self.node.get_logger().info(
            f"[Station-based 批量] 取料第 {context.cleaner_take_count + 1}/2 次 "
            f"(Work ID {context.work_id})")
        self.node.get_logger().info(
            f"來源: Cleaner Port {source_port} (Station 01 上層) → 目標: 機械臂, "
            f"carrier_id={context.carrier_id}")
        self.node.get_logger().info(
            f"最終目標: AGV Port {target_agv_port}")

        # 並行執行：Hokuyo write_busy 設定
        self._set_hokuyo_busy()

        # 並行執行：其他操作（不需等待 Hokuyo 完成）
        # PGNO 常數調整：TAKE_CLEANER_PGNO
        TAKE_CLEANER_PGNO = context.robot.ACTION_FROM + \
            context.robot.CLEANER_POSISION + context.robot.NONE_POSITION
        read_pgno = context.robot.read_pgno_response
        context.robot.read_robot_status()

        # 更新 Hokuyo Input - 使用統一方法
        self._handle_hokuyo_input()

        # 條件執行：只有機器人邏輯需要等待 Hokuyo 完成
        if self.hokuyo_busy_write_completed:
            self._execute_robot_logic(context, TAKE_CLEANER_PGNO, read_pgno)

    def _execute_robot_logic(self, context: RobotContext, TAKE_CLEANER_PGNO, read_pgno):
        """執行機器人邏輯"""
        match self.step:
            case RobotContext.IDLE:
                self.node.get_logger().info("Loader Robot Take Cleaner TAKE CLEANER IDLE")
                self.step = RobotContext.CHECK_IDLE
            case RobotContext.CHECK_IDLE:
                self.node.get_logger().info("Loader Robot Take Cleaner TAKE CLEANER CHECK_IDLE")
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
                self.node.get_logger().info("Robot Take Cleaner CHECK CHG PARAMETER")

                # 導入計算方法
                from loader_agv.robot_states.loader_robot_parameter import LoaderRobotParameter

                # 構建預期參數字典
                expected_params = {}

                # 檢查 cleaner_port → W116(layer_z_cleaner), W117(layer_y_cleaner)
                layer_z_cleaner, layer_y_cleaner = LoaderRobotParameter.calculate_layer_from_port(
                    context.get_cleaner_port
                )
                expected_params['w116'] = layer_z_cleaner
                expected_params['w117'] = layer_y_cleaner

                self.node.get_logger().info(
                    f"預期檢查: cleaner_port={context.get_cleaner_port} → "
                    f"W116={layer_z_cleaner}, W117={layer_y_cleaner}"
                )

                # 執行檢查
                if self._handle_check_chg_parameter(context, expected_params):
                    # 檢查通過，進入下一步驟
                    self.step = RobotContext.WRITE_CHG_PARA
                # 否則繼續停留在此步驟

            case RobotContext.WRITE_CHG_PARA:
                self.node.get_logger().info("Loader Robot Take Cleaner TAKE CLEANER WRITE CHG PARA")
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
                self.node.get_logger().info("Loader Robot Take Cleaner TAKE CLEANER CHECK CHG PARA")
                if read_pgno.value == Robot.CHG_PARA:
                    self.node.get_logger().info("✅讀取預執行成功")
                    self.step = RobotContext.WRITE_PGNO
                elif read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("🕒讀取預執行中...")
                else:
                    self.node.get_logger().info("❌讀取預執行失敗")

            case RobotContext.WRITE_PGNO:
                self.node.get_logger().info("Loader Robot Take Cleaner TAKE CLEANER WRITE_PGNO")
                if not self.sent:
                    context.robot.update_pgno(TAKE_CLEANER_PGNO)
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
                self.node.get_logger().info("Loader Robot Take Cleaner TAKE CLEANER CHECK_PGNO")
                if read_pgno is None:
                    self.node.get_logger().info("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == (TAKE_CLEANER_PGNO):
                    self.node.get_logger().info("✅讀取PGNO成功")
                    self.step = RobotContext.ACTING
                elif read_pgno.value == Robot.CHG_PARA:
                    self.node.get_logger().info("🕒讀取PGNO中...")
                else:
                    self.node.get_logger().info("❌讀取PGNO失敗")

            case RobotContext.ACTING:
                self.node.get_logger().info("Loader Robot Take Cleaner TAKE CLEANER ACTING")
                if read_pgno is None:
                    self.node.get_logger().info("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == (TAKE_CLEANER_PGNO):
                    self.node.get_logger().info("🤖手臂動作中")
                elif read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅手臂動作完成")
                    self.step = RobotContext.FINISH
                else:
                    self.node.get_logger().info("❌手臂動作失敗")
            case RobotContext.FINISH:
                self.node.get_logger().info("Loader Robot Take Cleaner TAKE CLEANER Finish")
                if read_pgno is None:
                    self.node.get_logger().info("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅取清洗機完成")

                    # 調整狀態轉換邏輯：在 FINISH 狀態確認手臂動作完成後，直接轉換到 PutAgvState
                    self.node.get_logger().info("✅ Take Cleaner 完成: 進入 PutAgvState")
                    from loader_agv.robot_states.take_cleaner.put_agv_state import PutAgvState
                    context.set_state(PutAgvState(self.node))

                    self.step = RobotContext.IDLE
                else:
                    self.node.get_logger().info("❌取清洗機失敗")
