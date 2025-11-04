from db_proxy_interfaces.msg import Carrier as CarrierMsg
from rclpy.node import Node
from unloader_agv.robot_context import RobotContext
from agv_base.robot import Robot
from agv_base.hokuyo_dms_8bit import HokuyoDMS8Bit
from db_proxy.agvc_database_client import AGVCDatabaseClient
from unloader_agv.robot_states.base_robot_state import BaseRobotState


class PutBoxoutTransferState(BaseRobotState):
    def __init__(self, node: Node):
        super().__init__(node)
        self.port_id_address = self.node.room_id * 1000 + 20
        self.step = RobotContext.IDLE
        self.agvc_client = AGVCDatabaseClient(self.node)
        self.update_carrier_success = False
        # 追蹤4個carrier的更新狀態
        self.update_carrier_1_success = False
        self.update_carrier_2_success = False
        self.update_carrier_3_success = False
        self.update_carrier_4_success = False
        self.carrier_updates_pending = 0  # 待更新的 carrier 數量
        self.sent = False

    def enter(self):
        self.node.get_logger().info("Unloader Robot Put BoxoutTransfer 目前狀態: PutBoxoutTransfer")
        self.update_carrier_success = False
        self.update_carrier_1_success = False
        self.update_carrier_2_success = False
        self.update_carrier_3_success = False
        self.update_carrier_4_success = False
        self.carrier_updates_pending = 0
        self.sent = False

    def leave(self):
        self.node.get_logger().info("Unloader Robot Put BoxoutTransfer 離開 PutBoxoutTransfer 狀態")
        self.update_carrier_success = False
        self.update_carrier_1_success = False
        self.update_carrier_2_success = False
        self.update_carrier_3_success = False
        self.update_carrier_4_success = False
        self.carrier_updates_pending = 0
        self.sent = False

    def update_carrier_database(self, context: RobotContext):
        """第2次循環後：更新所有4個carrier到對應的boxout ports"""
        self.carrier_updates_pending = 0
        self.update_carrier_1_success = False
        self.update_carrier_2_success = False
        self.update_carrier_3_success = False
        self.update_carrier_4_success = False

        # 從 port_groups 獲取所有 port IDs
        all_boxout_ports = []
        for group in context.take_put_port_groups:
            all_boxout_ports.extend(group)
        # all_boxout_ports = [2021, 2022, 2023, 2024]

        self.node.get_logger().info(
            f"準備更新所有4個Carrier到 boxout ports: {all_boxout_ports}")

        # 更新 carrier_id[0] → port all_boxout_ports[0]
        if context.carrier_id[0] is not None:
            carrier_0 = CarrierMsg()
            carrier_0.id = context.carrier_id[0]
            carrier_0.room_id = self.node.room_id
            carrier_0.rack_id = 0
            carrier_0.port_id = all_boxout_ports[0]
            carrier_0.rack_index = 0
            carrier_0.status_id = Robot.CARRIER_STATUS_ENTER_BOXOUT_TRANSFER

            self.agvc_client.async_update_carrier(
                carrier_0, lambda result: self.update_carrier_1_callback(result))
            self.carrier_updates_pending += 1
            self.node.get_logger().info(
                f"🔄 開始更新 Carrier 1: ID={context.carrier_id[0]} → Port {all_boxout_ports[0]}")
        else:
            self.update_carrier_1_success = True

        # 更新 carrier_id[1] → port all_boxout_ports[1]
        if context.carrier_id[1] is not None:
            carrier_1 = CarrierMsg()
            carrier_1.id = context.carrier_id[1]
            carrier_1.room_id = self.node.room_id
            carrier_1.rack_id = 0
            carrier_1.port_id = all_boxout_ports[1]
            carrier_1.rack_index = 0
            carrier_1.status_id = Robot.CARRIER_STATUS_ENTER_BOXOUT_TRANSFER

            self.agvc_client.async_update_carrier(
                carrier_1, lambda result: self.update_carrier_2_callback(result))
            self.carrier_updates_pending += 1
            self.node.get_logger().info(
                f"🔄 開始更新 Carrier 2: ID={context.carrier_id[1]} → Port {all_boxout_ports[1]}")
        else:
            self.update_carrier_2_success = True

        # 更新 carrier_id[2] → port all_boxout_ports[2]
        if context.carrier_id[2] is not None:
            carrier_2 = CarrierMsg()
            carrier_2.id = context.carrier_id[2]
            carrier_2.room_id = self.node.room_id
            carrier_2.rack_id = 0
            carrier_2.port_id = all_boxout_ports[2]
            carrier_2.rack_index = 0
            carrier_2.status_id = Robot.CARRIER_STATUS_ENTER_BOXOUT_TRANSFER

            self.agvc_client.async_update_carrier(
                carrier_2, lambda result: self.update_carrier_3_callback(result))
            self.carrier_updates_pending += 1
            self.node.get_logger().info(
                f"🔄 開始更新 Carrier 3: ID={context.carrier_id[2]} → Port {all_boxout_ports[2]}")
        else:
            self.update_carrier_3_success = True

        # 更新 carrier_id[3] → port all_boxout_ports[3]
        if context.carrier_id[3] is not None:
            carrier_3 = CarrierMsg()
            carrier_3.id = context.carrier_id[3]
            carrier_3.room_id = self.node.room_id
            carrier_3.rack_id = 0
            carrier_3.port_id = all_boxout_ports[3]
            carrier_3.rack_index = 0
            carrier_3.status_id = Robot.CARRIER_STATUS_ENTER_BOXOUT_TRANSFER

            self.agvc_client.async_update_carrier(
                carrier_3, lambda result: self.update_carrier_4_callback(result))
            self.carrier_updates_pending += 1
            self.node.get_logger().info(
                f"🔄 開始更新 Carrier 4: ID={context.carrier_id[3]} → Port {all_boxout_ports[3]}")
        else:
            self.update_carrier_4_success = True

        # 如果沒有需要更新的 carrier，直接設為成功
        if self.carrier_updates_pending == 0:
            self.update_carrier_success = True
            self.node.get_logger().info("ℹ️ 沒有需要更新的 Carrier")

    def update_carrier_1_callback(self, result):
        """Carrier 1 更新回調"""
        if result is not None and result.success:
            self.node.get_logger().info(f"✅ Carrier 1 更新成功: {result.message}")
            self.update_carrier_1_success = True
        else:
            error_msg = result.message if result is not None else "無回應"
            self.node.get_logger().error(f"❌ Carrier 1 更新失敗: {error_msg}")
            self.update_carrier_1_success = False
        self._check_all_carriers_updated()

    def update_carrier_2_callback(self, result):
        """Carrier 2 更新回調"""
        if result is not None and result.success:
            self.node.get_logger().info(f"✅ Carrier 2 更新成功: {result.message}")
            self.update_carrier_2_success = True
        else:
            error_msg = result.message if result is not None else "無回應"
            self.node.get_logger().error(f"❌ Carrier 2 更新失敗: {error_msg}")
            self.update_carrier_2_success = False
        self._check_all_carriers_updated()

    def update_carrier_3_callback(self, result):
        """Carrier 3 更新回調"""
        if result is not None and result.success:
            self.node.get_logger().info(f"✅ Carrier 3 更新成功: {result.message}")
            self.update_carrier_3_success = True
        else:
            error_msg = result.message if result is not None else "無回應"
            self.node.get_logger().error(f"❌ Carrier 3 更新失敗: {error_msg}")
            self.update_carrier_3_success = False
        self._check_all_carriers_updated()

    def update_carrier_4_callback(self, result):
        """Carrier 4 更新回調"""
        if result is not None and result.success:
            self.node.get_logger().info(f"✅ Carrier 4 更新成功: {result.message}")
            self.update_carrier_4_success = True
        else:
            error_msg = result.message if result is not None else "無回應"
            self.node.get_logger().error(f"❌ Carrier 4 更新失敗: {error_msg}")
            self.update_carrier_4_success = False
        self._check_all_carriers_updated()

    def _check_all_carriers_updated(self):
        """檢查所有 carrier 是否都已更新完成"""
        if (self.update_carrier_1_success and self.update_carrier_2_success and
            self.update_carrier_3_success and self.update_carrier_4_success):
            self.update_carrier_success = True
            self.node.get_logger().info("✅ 所有4個 Carrier 更新完成")

    def handle(self, context: RobotContext):
        # 添加循環日誌
        if hasattr(context, 'take_put_cycle_count') and hasattr(context, 'take_put_port_groups'):
            cycle_num = context.take_put_cycle_count + 1
            current_batch_index = context.take_put_cycle_count
            current_boxout_ports = context.take_put_port_groups[current_batch_index]
            self.node.get_logger().info(
                f"Unloader Robot Put Boxout Transfer (第{cycle_num}次) - "
                f"放到 boxout ports {current_boxout_ports}")
        else:
            self.node.get_logger().info("Unloader Robot Put BoxoutTransfer PutBoxoutTransfer 狀態")

        # 並行執行：Hokuyo write_busy 設定
        self._set_hokuyo_busy()

        # 並行執行：其他操作（不需等待 Hokuyo 完成）
        PUT_BOXOUT_TRANSFER_PGNO = context.robot.ACTION_TO + \
            context.robot.NONE_POSITION + context.robot.BOX_OUT_POSITION
        read_pgno = context.robot.read_pgno_response
        context.robot.read_robot_status()

        # 更新 Hokuyo Input - 使用統一方法
        self._handle_hokuyo_input()

        # 條件執行：只有機器人邏輯需要等待 Hokuyo 完成
        if self.hokuyo_busy_write_completed:
            self._execute_robot_logic(context, PUT_BOXOUT_TRANSFER_PGNO, read_pgno)

    def _execute_robot_logic(self, context: RobotContext, PUT_BOXOUT_TRANSFER_PGNO, read_pgno):
        """執行機器人邏輯"""
        match self.step:
            case RobotContext.IDLE:
                self.node.get_logger().info("Unloader Robot Put BoxoutTransfer PUT BOXOUT TRANSFER IDLE")
                self.step = RobotContext.CHECK_IDLE
            case RobotContext.CHECK_IDLE:
                self.node.get_logger().info("Unloader Robot Put BoxoutTransfer PUT BOXOUT TRANSFER CHECK_IDLE")
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
                self.node.get_logger().info("Unloader Robot Put Boxout Transfer CHECK CHG PARAMETER")

                # 構建預期參數字典
                expected_params = {}

                # 檢查 boxout_port → W112
                # layer_z = ((port-1) // 2) + 1, layer_y = 0
                layer_z_boxout = ((context.get_boxout_port - 1) // 2) + 1
                layer_y_boxout = 0
                expected_params['w112'] = (layer_z_boxout | (layer_y_boxout << 16))

                self.node.get_logger().info(
                    f"預期檢查: boxout_port={context.get_boxout_port} → "
                    f"W112 (z={layer_z_boxout}, y={layer_y_boxout})")

                # 執行檢查
                if self._handle_check_chg_parameter(context, expected_params):
                    self.step = RobotContext.WRITE_CHG_PARA

            case RobotContext.WRITE_CHG_PARA:
                self.node.get_logger().info("Unloader Robot Put BoxoutTransfer PUT BOXOUT TRANSFER WRITE CHG PARA")
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
                self.node.get_logger().info("Unloader Robot Put BoxoutTransfer PUT BOXOUT TRANSFER CHECK CHG PARA")
                if read_pgno.value == Robot.CHG_PARA:
                    self.node.get_logger().info("✅讀取預執行成功")
                    self.step = RobotContext.WRITE_PGNO
                elif read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("🕒讀取預執行中...")
                else:
                    self.node.get_logger().info("❌讀取預執行失敗")

            case RobotContext.WRITE_PGNO:
                self.node.get_logger().info("Unloader Robot Put BoxoutTransfer PUT BOXOUT TRANSFER WRITE_PGNO")
                if not self.sent:
                    context.robot.update_pgno(PUT_BOXOUT_TRANSFER_PGNO)
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
                self.node.get_logger().info("Unloader Robot Put BoxoutTransfer PUT BOXOUT TRANSFER CHECK_PGNO")
                if read_pgno is None:
                    self.node.get_logger().debug("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == (PUT_BOXOUT_TRANSFER_PGNO):
                    self.node.get_logger().info("✅讀取PGNO成功")
                    self.step = RobotContext.ACTING
                elif read_pgno.value == Robot.CHG_PARA:
                    self.node.get_logger().info("🕒讀取PGNO中...")
                else:
                    self.node.get_logger().info("❌讀取PGNO失敗")

            case RobotContext.ACTING:
                self.node.get_logger().info("Unloader Robot Put BoxoutTransfer PUT BOXOUT TRANSFER ACTING")
                if read_pgno is None:
                    self.node.get_logger().debug("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == (PUT_BOXOUT_TRANSFER_PGNO):
                    self.node.get_logger().debug("🤖手臂動作中")
                elif read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅手臂動作完成")
                    self.step = RobotContext.FINISH
                else:
                    self.node.get_logger().info("❌手臂動作失敗")
            case RobotContext.FINISH:
                self.node.get_logger().info("Unloader Robot Put BoxoutTransfer PUT BOXOUT TRANSFER Finish")
                if read_pgno is None:
                    self.node.get_logger().debug("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅放出料傳送箱完成")
                    self.step = RobotContext.UPDATE_DATABASE
                else:
                    self.node.get_logger().info("❌放出料傳送箱失敗")
            case RobotContext.UPDATE_DATABASE:
                self.node.get_logger().info("Unloader Robot Put BoxoutTransfer PUT BOXOUT TRANSFER UPDATE_DATABASE")
                if not self.sent:
                    cycle_num = context.take_put_cycle_count + 1

                    if context.take_put_cycle_count == 0:
                        # ====== 第1次：不更新數據庫 ======
                        self.node.get_logger().info(f"✅ 第{cycle_num}次放料完成，暫不更新數據庫")
                        self.sent = True
                    else:
                        # ====== 第2次：更新所有4個carrier ======
                        self.node.get_logger().info(f"✅ 第{cycle_num}次放料完成，開始更新所有4個Carrier")
                        self.update_carrier_database(context)
                        self.sent = True

                elif self.sent:
                    if context.take_put_cycle_count < context.take_put_max_cycles - 1:
                        # ====== 第1次完成，準備第2次 ======
                        context.take_put_cycle_count += 1
                        context.take_put_current_batch = [3, 4]  # AGV port numbers

                        self.node.get_logger().info(
                            f"準備第{context.take_put_cycle_count + 1}次循環："
                            f"\n  AGV批次: {context.take_put_current_batch}"
                            f"\n  Boxout批次: {context.take_put_port_groups[context.take_put_cycle_count]}")

                        # 重置狀態
                        self.sent = False
                        self.step = RobotContext.IDLE
                        self._reset_state()

                        # 返回 TakeAgvState 進行第2次循環
                        from unloader_agv.robot_states.put_boxout_transfer.take_agv_state import TakeAgvState
                        context.set_state(TakeAgvState(self.node))
                    else:
                        # ====== 第2次完成 ======
                        if self.update_carrier_success:
                            self.node.get_logger().info("✅ 更新所有4個 Carrier 資料庫成功")
                            self.sent = False

                            # 完成 PUT_BOXOUT_TRANSFER 流程，進入完成狀態
                            self.node.get_logger().info("✅ Put BoxoutTransfer 完成: 進入 CompleteState")
                            from unloader_agv.robot_states.complete_state import CompleteState
                            context.set_state(CompleteState(self.node))

                            self.step = RobotContext.IDLE
                        else:
                            self.node.get_logger().info("⏳ 等待 Carrier 資料庫更新完成...")
