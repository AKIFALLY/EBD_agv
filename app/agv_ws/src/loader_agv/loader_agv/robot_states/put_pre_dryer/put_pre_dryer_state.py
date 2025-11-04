from db_proxy_interfaces.msg import Carrier as CarrierMsg
from rclpy.node import Node
from loader_agv.robot_context import RobotContext
from agv_base.robot import Robot
from agv_base.hokuyo_dms_8bit import HokuyoDMS8Bit
from db_proxy.agvc_database_client import AGVCDatabaseClient


from loader_agv.robot_states.base_robot_state import BaseRobotState


class PutPreDryerState(BaseRobotState):
    def __init__(self, node: Node):
        super().__init__(node)
        # 動態參數計算，與 pre_dryer_check_have_state.py 中的 port_address 參數一致
        self.port_id_address = self.node.room_id * 1000 + 50
        self.step = RobotContext.IDLE
        self.agvc_client = AGVCDatabaseClient(self.node)
        # 簡化 Carrier 更新邏輯：只需更新一個 carrier
        self.update_carrier_success = False
        self.sent = False

    def enter(self):
        self.node.get_logger().info(
            "[Station-based 2格] Loader Robot Put PreDryer 目前狀態: PutPreDryer")
        self.update_carrier_success = False
        self.sent = False

    def leave(self):
        self.node.get_logger().info(
            "[Station-based 2格] Loader Robot Put PreDryer 離開 PutPreDryer 狀態")
        self.update_carrier_success = False
        self.sent = False

    def update_carrier_database(self, context: RobotContext):
        """更新單一 carrier 資料庫記錄 - 批量放料（第N/2次）

        說明：批量處理每次更新一個 carrier
        - 第1次：更新從 AGV Port 2 取出的 carrier
        - 第2次：更新從 AGV Port 4 取出的 carrier
        - 最終位置：Pre-dryer Port（根據 Station 映射）
        """
        source_agv_port = context.pre_dryer_agv_ports[context.pre_dryer_take_count]
        target_pre_dryer_port = context.pre_dryer_device_ports[context.pre_dryer_take_count]

        carrier = CarrierMsg()
        carrier.id = context.carrier_id
        carrier.room_id = self.node.room_id
        carrier.rack_id = 0
        carrier.port_id = self.port_id_address + context.get_pre_dryer_port
        carrier.rack_index = 0
        carrier.status_id = Robot.CARRIER_STATUS_ENTER_PRE_DRYER  # 進入預烘機處理中 (502)

        self.agvc_client.async_update_carrier(
            carrier, self.update_carrier_database_callback)
        self.node.get_logger().info(
            f"🔄 [Station-based 2格] 開始更新 Carrier: {context.carrier_id} "
            f"(第 {context.pre_dryer_take_count + 1}/2 次, Work ID {context.work_id})")
        self.node.get_logger().info(
            f"完整路徑: AGV Port {source_agv_port} → 機械臂 → "
            f"Pre-dryer Port {target_pre_dryer_port}")
        self.node.get_logger().info(
            f"最終位置: port_id={carrier.port_id}, 狀態: ENTER_PRE_DRYER")

    def update_carrier_database_callback(self, result):
        """處理 carrier 資料庫更新回應"""
        if result is not None and result.success:
            self.node.get_logger().info(
                f"✅ [Station-based 2格] Carrier 更新成功: {result.success}, {result.message}")
            self.update_carrier_success = True
        else:
            self.node.get_logger().error(
                "❌ [Station-based 2格] Carrier 更新失敗")
            self.update_carrier_success = False

    def handle(self, context: RobotContext):
        # 批量放料 Pre-dryer port 映射：根據計數器決定目標 Pre-dryer port
        source_agv_port = context.pre_dryer_agv_ports[context.pre_dryer_take_count]
        target_pre_dryer_port = context.pre_dryer_device_ports[context.pre_dryer_take_count]
        context.get_pre_dryer_port = target_pre_dryer_port

        self.node.get_logger().info(
            f"[Station-based 2格] 第 {context.pre_dryer_take_count + 1}/2 次放料 "
            f"(Work ID {context.work_id})")
        self.node.get_logger().info(
            f"來源: 機械臂 → 目標: Pre-dryer Port {target_pre_dryer_port}")
        self.node.get_logger().info(
            f"完整路徑: AGV Port {source_agv_port} (Carrier {context.carrier_id}) → "
            f"機械臂 → Pre-dryer Port {target_pre_dryer_port}")

        # 並行執行：Hokuyo write_busy 設定
        self._set_hokuyo_busy()

        # 並行執行：其他操作（不需等待 Hokuyo 完成）
        # 修正 PGNO 常數定義，適用於 loader_agv PUT_PRE_DRYER
        PUT_PRE_DRYER_PGNO = context.robot.ACTION_TO + \
            context.robot.NONE_POSITION + context.robot.PRE_DRYER_POSISION
        read_pgno = context.robot.read_pgno_response
        context.robot.read_robot_status()

        # 更新 Hokuyo Input - 使用統一方法
        self._handle_hokuyo_input()

        # 條件執行：只有機器人邏輯需要等待 Hokuyo 完成
        if self.hokuyo_busy_write_completed:
            self._execute_robot_logic(context, PUT_PRE_DRYER_PGNO, read_pgno)

    def _execute_robot_logic(self, context: RobotContext, PUT_PRE_DRYER_PGNO, read_pgno):
        """執行機器人邏輯 - 批量放料（第N/2次）"""
        match self.step:
            case RobotContext.IDLE:
                self.node.get_logger().info(
                    "[Station-based 2格] Loader Robot Put PreDryer PUT PRE DRYER IDLE")
                self.step = RobotContext.CHECK_IDLE
            case RobotContext.CHECK_IDLE:
                self.node.get_logger().info(
                    "[Station-based 2格] Loader Robot Put PreDryer PUT PRE DRYER CHECK_IDLE")
                if read_pgno is None:
                    self.node.get_logger().debug("⏳等待讀取PGNO回應...")
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
                self.node.get_logger().info("Robot Put Pre-Dryer PUT PRE-DRYER CHECK CHG PARAMETER")

                # 導入計算方法
                from loader_agv.robot_states.loader_robot_parameter import LoaderRobotParameter

                # 構建預期參數字典
                expected_params = {}

                # 檢查 pre_dryer_port → W11A(layer_z_pre_dryer), W11B(layer_y_pre_dryer)
                layer_z_pre_dryer, layer_y_pre_dryer = LoaderRobotParameter.calculate_layer_from_port(
                    context.get_pre_dryer_port
                )
                expected_params['w11a'] = layer_z_pre_dryer
                expected_params['w11b'] = layer_y_pre_dryer

                self.node.get_logger().info(
                    f"預期檢查: pre_dryer_port={context.get_pre_dryer_port} → "
                    f"W11A={layer_z_pre_dryer}, W11B={layer_y_pre_dryer}"
                )

                # 執行檢查
                if self._handle_check_chg_parameter(context, expected_params):
                    # 檢查通過，進入下一步驟
                    self.step = RobotContext.WRITE_CHG_PARA
                # 否則繼續停留在此步驟

            case RobotContext.WRITE_CHG_PARA:
                self.node.get_logger().info("Loader Robot Put PreDryer PUT PRE DRYER WRITE CHG PARA")
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
                self.node.get_logger().info("Loader Robot Put PreDryer PUT PRE DRYER CHECK CHG PARA")
                if read_pgno.value == Robot.CHG_PARA:
                    self.node.get_logger().info("✅讀取預執行成功")
                    self.step = RobotContext.WRITE_PGNO
                elif read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("🕒讀取預執行中...")
                else:
                    self.node.get_logger().info("❌讀取預執行失敗")

            case RobotContext.WRITE_PGNO:
                self.node.get_logger().info("Loader Robot Put PreDryer PUT PRE DRYER WRITE_PGNO")
                if not self.sent:
                    context.robot.update_pgno(PUT_PRE_DRYER_PGNO)
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
                self.node.get_logger().info("Loader Robot Put PreDryer PUT PRE DRYER CHECK_PGNO")
                if read_pgno is None:
                    self.node.get_logger().debug("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == (PUT_PRE_DRYER_PGNO):
                    self.node.get_logger().info("✅讀取PGNO成功")
                    self.step = RobotContext.ACTING
                elif read_pgno.value == Robot.CHG_PARA:
                    self.node.get_logger().info("🕒讀取PGNO中...")
                else:
                    self.node.get_logger().info("❌讀取PGNO失敗")

            case RobotContext.ACTING:
                self.node.get_logger().info("Loader Robot Put PreDryer PUT PRE DRYER ACTING")
                if read_pgno is None:
                    self.node.get_logger().debug("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == (PUT_PRE_DRYER_PGNO):
                    self.node.get_logger().info("🤖手臂動作中")
                elif read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅手臂動作完成")
                    self.step = RobotContext.FINISH
                else:
                    self.node.get_logger().info("❌手臂動作失敗")
            case RobotContext.FINISH:
                self.node.get_logger().info("Loader Robot Put PreDryer PUT PRE DRYER Finish")
                if read_pgno is None:
                    self.node.get_logger().debug("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅放預乾燥完成")
                    self.step = RobotContext.UPDATE_DATABASE
                else:
                    self.node.get_logger().info("❌放預乾燥失敗")
            case RobotContext.UPDATE_DATABASE:
                self.node.get_logger().info(
                    "[Station-based 2格] Loader Robot Put PreDryer PUT PRE DRYER UPDATE_DATABASE")
                if not self.sent:
                    self.update_carrier_database(context)
                    self.sent = True
                elif self.sent and self.update_carrier_success:
                    self.node.get_logger().info(
                        "✅ [Station-based 2格] 更新 Carrier 資料庫成功")
                    self.sent = False

                    # 批量放料邏輯：檢查是否完成 2 次放料
                    if context.pre_dryer_take_count == 0:
                        # 第 1 次完成 → 繼續第 2 次
                        context.pre_dryer_take_count = 1
                        next_agv_port = context.pre_dryer_agv_ports[1]
                        next_pre_dryer_port = context.pre_dryer_device_ports[1]

                        self.node.get_logger().info(
                            f"🔄 [Station-based 2格] 第 1/2 次放料完成 (Work ID {context.work_id})")
                        self.node.get_logger().info(
                            f"[Station-based 2格] 繼續第 2 次放料: AGV Port {next_agv_port} → "
                            f"Pre-dryer Port {next_pre_dryer_port}")
                        from loader_agv.robot_states.put_pre_dryer.take_agv_state import TakeAgvState
                        context.set_state(TakeAgvState(self.node))
                    else:
                        # 第 2 次完成 → 任務完成
                        self.node.get_logger().info(
                            f"✅ [Station-based 2格] 批量放料完成 (2/2 次, Work ID {context.work_id})")
                        self.node.get_logger().info(
                            f"完整流程: AGV Ports {context.pre_dryer_agv_ports} → "
                            f"Pre-dryer Ports {context.pre_dryer_device_ports}")
                        self.node.get_logger().info(
                            f"[Station-based 2格] 進入 CompleteState")
                        from loader_agv.robot_states.complete_state import CompleteState
                        context.set_state(CompleteState(self.node))

                    self.step = RobotContext.IDLE
