from db_proxy_interfaces.msg import Carrier as CarrierMsg
from rclpy.node import Node
from loader_agv.robot_context import RobotContext
from agv_base.robot import Robot
from agv_base.hokuyo_dms_8bit import HokuyoDMS8Bit
from db_proxy.agvc_database_client import AGVCDatabaseClient


from loader_agv.robot_states.base_robot_state import BaseRobotState


class PutSoakerState(BaseRobotState):
    def __init__(self, node: Node):
        super().__init__(node)
        # 動態參數計算，與 soaker_check_have_state.py 中的 port_address 參數一致
        self.port_id_address = self.node.room_id * 1000 + 40
        self.step = RobotContext.IDLE
        self.agvc_client = AGVCDatabaseClient(self.node)
        # 簡化 Carrier 更新邏輯：只需更新一個 carrier
        self.update_carrier_success = False
        self.sent = False

    def enter(self):
        self.node.get_logger().info(
            "[Station-based 1格] Loader Robot Put Soaker 目前狀態: PutSoaker")
        self.update_carrier_success = False
        self.sent = False

    def leave(self):
        self.node.get_logger().info(
            "[Station-based 1格] Loader Robot Put Soaker 離開 PutSoaker 狀態")
        self.update_carrier_success = False
        self.sent = False

    def update_carrier_database(self, context: RobotContext):
        """更新單一 carrier 資料庫記錄 - 單格處理

        說明：Put Soaker 單格處理特點
        - 一次只更新1個 carrier（與 Put Cleaner 批量2個不同）
        - 更新目標：泡藥機指定站點（Port 1-6中的1個）
        - carrier 狀態：ENTER_SOAKER (402) - 進入泡藥機處理中
        """
        carrier = CarrierMsg()
        carrier.id = context.carrier_id
        carrier.room_id = self.node.room_id
        carrier.rack_id = 0
        carrier.port_id = self.port_id_address + context.get_soaker_port
        carrier.rack_index = 0
        carrier.status_id = Robot.CARRIER_STATUS_ENTER_SOAKER  # 進入強化機處理中 (402)

        self.agvc_client.async_update_carrier(
            carrier, self.update_carrier_database_callback)
        self.node.get_logger().info(
            f"🔄 [Station-based 1格] 開始更新 Carrier: {context.carrier_id} → "
            f"泡藥機 Port {context.get_soaker_port} (port_id={carrier.port_id})")

    def update_carrier_database_callback(self, result):
        """處理 carrier 資料庫更新回應"""
        if result is not None and result.success:
            self.node.get_logger().info(
                f"✅ [Station-based 1格] Carrier 更新成功: {result.success}, {result.message}")
            self.update_carrier_success = True
        else:
            self.node.get_logger().error("❌ [Station-based 1格] Carrier 更新失敗")
            self.update_carrier_success = False

    def handle(self, context: RobotContext):
        # 單格處理：從機械臂放1格 → 泡藥機指定站點
        target_soaker_port = context.get_soaker_port

        self.node.get_logger().info(
            f"[Station-based 1格] 放料 (Work ID {context.work_id})")
        self.node.get_logger().info(
            f"來源: 機械臂 → 目標: 泡藥機 Port {target_soaker_port} (Station {target_soaker_port:02d})")
        self.node.get_logger().info(
            f"完整路徑: AGV Port {context.get_loader_agv_port_side} → 機械臂 → "
            f"泡藥機 Port {target_soaker_port}")

        # 並行執行：Hokuyo write_busy 設定
        self._set_hokuyo_busy()

        # 並行執行：其他操作（不需等待 Hokuyo 完成）
        # 修正 PGNO 常數定義，適用於 loader_agv PUT_SOAKER
        PUT_SOAKER_PGNO = context.robot.ACTION_TO + \
            context.robot.NONE_POSITION + context.robot.SOAKER_POSISION
        read_pgno = context.robot.read_pgno_response
        context.robot.read_robot_status()

        # 更新 Hokuyo Input - 使用統一方法
        self._handle_hokuyo_input()

        # 條件執行：只有機器人邏輯需要等待 Hokuyo 完成
        if self.hokuyo_busy_write_completed:
            self._execute_robot_logic(context, PUT_SOAKER_PGNO, read_pgno)

    def _execute_robot_logic(self, context: RobotContext, PUT_SOAKER_PGNO, read_pgno):
        """執行機器人邏輯 - 單格處理（無批量循環）"""
        match self.step:
            case RobotContext.IDLE:
                self.node.get_logger().info(
                    "[Station-based 1格] Loader Robot Put Soaker PUT SOAKER IDLE")
                self.step = RobotContext.CHECK_IDLE
            case RobotContext.CHECK_IDLE:
                self.node.get_logger().info(
                    "[Station-based 1格] Loader Robot Put Soaker PUT SOAKER CHECK_IDLE")
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
                self.node.get_logger().info("Robot Put Soaker PUT SOAKER CHECK CHG PARAMETER")

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
                self.node.get_logger().info("Loader Robot Put Soaker PUT SOAKER WRITE CHG PARA")
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
                self.node.get_logger().info("Loader Robot Put Soaker PUT SOAKER CHECK CHG PARA")
                if read_pgno.value == Robot.CHG_PARA:
                    self.node.get_logger().info("✅讀取預執行成功")
                    self.step = RobotContext.WRITE_PGNO
                elif read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("🕒讀取預執行中...")
                else:
                    self.node.get_logger().info("❌讀取預執行失敗")

            case RobotContext.WRITE_PGNO:
                self.node.get_logger().info("Loader Robot Put Soaker PUT SOAKER WRITE_PGNO")
                if not self.sent:
                    context.robot.update_pgno(PUT_SOAKER_PGNO)
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
                self.node.get_logger().info("Loader Robot Put Soaker PUT SOAKER CHECK_PGNO")
                if read_pgno is None:
                    self.node.get_logger().info("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == (PUT_SOAKER_PGNO):
                    self.node.get_logger().info("✅讀取PGNO成功")
                    self.step = RobotContext.ACTING
                elif read_pgno.value == Robot.CHG_PARA:
                    self.node.get_logger().info("🕒讀取PGNO中...")
                else:
                    self.node.get_logger().info("❌讀取PGNO失敗")

            case RobotContext.ACTING:
                self.node.get_logger().info("Loader Robot Put Soaker PUT SOAKER ACTING")
                if read_pgno is None:
                    self.node.get_logger().info("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == (PUT_SOAKER_PGNO):
                    self.node.get_logger().info("🤖手臂動作中")
                elif read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅手臂動作完成")
                    self.step = RobotContext.FINISH
                else:
                    self.node.get_logger().info("❌手臂動作失敗")
            case RobotContext.FINISH:
                self.node.get_logger().info(
                    "[Station-based 1格] Loader Robot Put Soaker PUT SOAKER Finish")
                if read_pgno is None:
                    self.node.get_logger().info("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅放浸泡完成")
                    self.step = RobotContext.UPDATE_DATABASE
                else:
                    self.node.get_logger().info("❌放浸泡失敗")
            case RobotContext.UPDATE_DATABASE:
                self.node.get_logger().info(
                    "[Station-based 1格] Loader Robot Put Soaker PUT SOAKER UPDATE_DATABASE")
                if not self.sent:
                    self.update_carrier_database(context)
                    self.sent = True
                elif self.sent and self.update_carrier_success:
                    self.node.get_logger().info("✅更新 Carrier 資料庫成功")
                    self.sent = False

                    # 單格處理：直接進入 CompleteState（無批量循環邏輯）
                    self.node.get_logger().info(
                        f"✅ [Station-based 1格] Put Soaker 完成: 進入 CompleteState")
                    self.node.get_logger().info(
                        f"完整流程: AGV Port {context.get_loader_agv_port_side} → 機械臂 → "
                        f"泡藥機 Port {context.get_soaker_port} (Work ID {context.work_id})")
                    from loader_agv.robot_states.complete_state import CompleteState
                    context.set_state(CompleteState(self.node))

                    self.step = RobotContext.IDLE
