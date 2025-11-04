from rclpy.node import Node
from unloader_agv.robot_context import RobotContext
from agv_base.robot import Robot
from unloader_agv.robot_states.base_robot_state import BaseRobotState
from db_proxy_interfaces.msg import Carrier as CarrierMsg
from db_proxy.agvc_database_client import AGVCDatabaseClient


class PutOvenState(BaseRobotState):
    def __init__(self, node: Node):
        super().__init__(node)
        self.hokuyo_dms_8bit_1 = self.node.hokuyo_dms_8bit_1
        self.step = RobotContext.IDLE
        self.sent = False
        self.update_carrier_success = False
        self.update_carrier_1_success = False
        self.update_carrier_2_success = False
        self.update_carrier_3_success = False
        self.update_carrier_4_success = False
        self.carrier_updates_pending = 0

        # 動態計算 port_id_address
        self.port_id_address = self.node.room_id * 1000 + 60  # OVEN port address
        self.agvc_client = AGVCDatabaseClient(self.node)

    def enter(self):
        self.node.get_logger().info("Unloader Robot Put Oven 目前狀態: PutOven")
        self.update_carrier_success = False
        self.update_carrier_1_success = False
        self.update_carrier_2_success = False
        self.update_carrier_3_success = False
        self.update_carrier_4_success = False
        self.carrier_updates_pending = 0
        self.sent = False

    def leave(self):
        self.node.get_logger().info("Unloader Robot Put Oven 離開 PutOven 狀態")
        self.update_carrier_success = False
        self.update_carrier_1_success = False
        self.update_carrier_2_success = False
        self.update_carrier_3_success = False
        self.update_carrier_4_success = False
        self.carrier_updates_pending = 0
        self.sent = False

    def update_carrier_database(self, context: RobotContext):
        """更新所有4個 carrier 資料庫（第2次循環時調用）"""
        self.carrier_updates_pending = 0
        self.update_carrier_1_success = False
        self.update_carrier_2_success = False
        self.update_carrier_3_success = False
        self.update_carrier_4_success = False

        # 從 context 獲取 oven port 分組
        # 第1次循環：oven_port_groups[0]，第2次循環：oven_port_groups[1]
        if hasattr(context, 'oven_port_groups') and len(context.oven_port_groups) == 2:
            oven_ports_batch1 = context.oven_port_groups[0]  # 第1次的2個oven ports
            oven_ports_batch2 = context.oven_port_groups[1]  # 第2次的2個oven ports
        else:
            # 兼容舊邏輯（如果沒有分組，使用 oven_number）
            oven_ports_batch1 = [context.oven_number, context.oven_number + 1]
            oven_ports_batch2 = [context.oven_number + 2, context.oven_number + 3]

        # 更新第1個 carrier (AGV port 1 → oven port batch1[0])
        if context.carrier_id[0] is not None:
            carrier_1 = CarrierMsg()
            carrier_1.id = context.carrier_id[0]
            carrier_1.room_id = self.node.room_id
            carrier_1.rack_id = 0
            carrier_1.port_id = self.port_id_address + oven_ports_batch1[0]
            carrier_1.rack_index = 0

            self.agvc_client.async_update_carrier(
                carrier_1, lambda result: self.update_carrier_1_callback(result))
            self.carrier_updates_pending += 1
            self.node.get_logger().info(
                f"🔄 開始更新第1個 Carrier: {context.carrier_id[0]} "
                f"→ Oven Port {oven_ports_batch1[0]} (ID={carrier_1.port_id})")
        else:
            self.update_carrier_1_success = True

        # 更新第2個 carrier (AGV port 2 → oven port batch1[1])
        if context.carrier_id[1] is not None:
            carrier_2 = CarrierMsg()
            carrier_2.id = context.carrier_id[1]
            carrier_2.room_id = self.node.room_id
            carrier_2.rack_id = 0
            carrier_2.port_id = self.port_id_address + oven_ports_batch1[1]
            carrier_2.rack_index = 0

            self.agvc_client.async_update_carrier(
                carrier_2, lambda result: self.update_carrier_2_callback(result))
            self.carrier_updates_pending += 1
            self.node.get_logger().info(
                f"🔄 開始更新第2個 Carrier: {context.carrier_id[1]} "
                f"→ Oven Port {oven_ports_batch1[1]} (ID={carrier_2.port_id})")
        else:
            self.update_carrier_2_success = True

        # 更新第3個 carrier (AGV port 3 → oven port batch2[0])
        if context.carrier_id[2] is not None:
            carrier_3 = CarrierMsg()
            carrier_3.id = context.carrier_id[2]
            carrier_3.room_id = self.node.room_id
            carrier_3.rack_id = 0
            carrier_3.port_id = self.port_id_address + oven_ports_batch2[0]
            carrier_3.rack_index = 0

            self.agvc_client.async_update_carrier(
                carrier_3, lambda result: self.update_carrier_3_callback(result))
            self.carrier_updates_pending += 1
            self.node.get_logger().info(
                f"🔄 開始更新第3個 Carrier: {context.carrier_id[2]} "
                f"→ Oven Port {oven_ports_batch2[0]} (ID={carrier_3.port_id})")
        else:
            self.update_carrier_3_success = True

        # 更新第4個 carrier (AGV port 4 → oven port batch2[1])
        if context.carrier_id[3] is not None:
            carrier_4 = CarrierMsg()
            carrier_4.id = context.carrier_id[3]
            carrier_4.room_id = self.node.room_id
            carrier_4.rack_id = 0
            carrier_4.port_id = self.port_id_address + oven_ports_batch2[1]
            carrier_4.rack_index = 0

            self.agvc_client.async_update_carrier(
                carrier_4, lambda result: self.update_carrier_4_callback(result))
            self.carrier_updates_pending += 1
            self.node.get_logger().info(
                f"🔄 開始更新第4個 Carrier: {context.carrier_id[3]} "
                f"→ Oven Port {oven_ports_batch2[1]} (ID={carrier_4.port_id})")
        else:
            self.update_carrier_4_success = True

        # 如果沒有需要更新的 carrier，直接設為成功
        if self.carrier_updates_pending == 0:
            self.update_carrier_success = True
            self.node.get_logger().info("ℹ️ 沒有需要更新的 Carrier")

    def update_carrier_1_callback(self, result):
        """第1個 carrier 更新回調"""
        if result is not None and result.success:
            self.node.get_logger().info(f"✅ 第1個 Carrier 更新成功: {result.message}")
            self.update_carrier_1_success = True
        else:
            error_msg = result.message if result is not None else "無回應"
            self.node.get_logger().error(f"❌ 第1個 Carrier 更新失敗: {error_msg}")
            self.update_carrier_1_success = False
        self._check_all_carriers_updated()

    def update_carrier_2_callback(self, result):
        """第2個 carrier 更新回調"""
        if result is not None and result.success:
            self.node.get_logger().info(f"✅ 第2個 Carrier 更新成功: {result.message}")
            self.update_carrier_2_success = True
        else:
            error_msg = result.message if result is not None else "無回應"
            self.node.get_logger().error(f"❌ 第2個 Carrier 更新失敗: {error_msg}")
            self.update_carrier_2_success = False
        self._check_all_carriers_updated()

    def update_carrier_3_callback(self, result):
        """第3個 carrier 更新回調"""
        if result is not None and result.success:
            self.node.get_logger().info(f"✅ 第3個 Carrier 更新成功: {result.message}")
            self.update_carrier_3_success = True
        else:
            error_msg = result.message if result is not None else "無回應"
            self.node.get_logger().error(f"❌ 第3個 Carrier 更新失敗: {error_msg}")
            self.update_carrier_3_success = False
        self._check_all_carriers_updated()

    def update_carrier_4_callback(self, result):
        """第4個 carrier 更新回調"""
        if result is not None and result.success:
            self.node.get_logger().info(f"✅ 第4個 Carrier 更新成功: {result.message}")
            self.update_carrier_4_success = True
        else:
            error_msg = result.message if result is not None else "無回應"
            self.node.get_logger().error(f"❌ 第4個 Carrier 更新失敗: {error_msg}")
            self.update_carrier_4_success = False
        self._check_all_carriers_updated()

    def _check_all_carriers_updated(self):
        """檢查所有4個 carrier 是否都已更新完成"""
        if (self.update_carrier_1_success and self.update_carrier_2_success and
            self.update_carrier_3_success and self.update_carrier_4_success):
            self.update_carrier_success = True
            self.node.get_logger().info("✅ 所有4個 Carrier 更新完成")

    def _set_hokuyo_busy(self):
        """設定 Hokuyo write_busy"""
        if not self.hokuyo_busy_write_completed:
            try:
                self.hokuyo_dms_8bit_1.write_busy("1")
                self.hokuyo_busy_write_completed = True
                self.node.get_logger().info("✅ Hokuyo write_busy=1 設定完成")
            except Exception as e:
                self.node.get_logger().error(f"❌ Hokuyo write_busy 設定失敗: {e}")

    def _execute_robot_logic(self, context: RobotContext, PUT_OVEN_PGNO, read_pgno):
        """執行機器人邏輯"""
        match self.step:
            case RobotContext.IDLE:
                self.node.get_logger().info("Unloader Robot Put Oven PUT OVEN IDLE")
                self.step = RobotContext.CHECK_IDLE
            case RobotContext.CHECK_IDLE:
                self.node.get_logger().info("Unloader Robot Put Oven PUT OVEN CHECK_IDLE")
                if read_pgno is None:
                    self.node.get_logger().debug("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅Robot狀態為IDLE")
                    self.step = RobotContext.WRITE_CHG_PARAMTER
                else:
                    self.node.get_logger().info("❌Robot狀態不為IDLE")

            case RobotContext.WRITE_CHG_PARAMTER:
                self.node.get_logger().info("Unloader Robot Put Oven PUT OVEN WRITE CHG PARAMTER")
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
                self.node.get_logger().info("Unloader Robot Put Oven PUT OVEN CHECK CHG PARAMETER")

                # 構建預期參數字典
                expected_params = {}

                # 檢查 oven_port → W116
                # layer_z = ((port-1) // 4) + 1
                # layer_y = ((port-1) // 2) % 2 + 1
                layer_z_oven = ((context.get_oven_port - 1) // 4) + 1
                layer_y_oven = ((context.get_oven_port - 1) // 2) % 2 + 1
                expected_params['w116'] = (layer_z_oven | (layer_y_oven << 16))

                self.node.get_logger().info(
                    f"預期檢查: oven_port={context.get_oven_port} → "
                    f"W116 (z={layer_z_oven}, y={layer_y_oven})")

                # 執行檢查
                if self._handle_check_chg_parameter(context, expected_params):
                    self.step = RobotContext.WRITE_CHG_PARA

            case RobotContext.WRITE_CHG_PARA:
                self.node.get_logger().info("Unloader Robot Put Oven PUT OVEN WRITE CHG PARA")
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
                self.node.get_logger().info("Unloader Robot Put Oven PUT OVEN CHECK_CHG_PARA")
                if read_pgno is None:
                    self.node.get_logger().debug("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == Robot.CHG_PARA:
                    self.node.get_logger().info("✅讀取預執行成功")
                    self.step = RobotContext.WRITE_PGNO
                else:
                    self.node.get_logger().info("❌讀取預執行失敗")

            case RobotContext.WRITE_PGNO:
                self.node.get_logger().info("Unloader Robot Put Oven PUT OVEN WRITE_PGNO")
                if not self.sent:
                    context.robot.update_pgno(PUT_OVEN_PGNO)
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
                self.node.get_logger().info("Unloader Robot Put Oven PUT OVEN CHECK_PGNO")
                if read_pgno is None:
                    self.node.get_logger().debug("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == (PUT_OVEN_PGNO):
                    self.node.get_logger().info("✅讀取PGNO成功")
                    self.step = RobotContext.ACTING
                elif read_pgno.value == Robot.CHG_PARA:
                    self.node.get_logger().info("🕒讀取PGNO中...")
                else:
                    self.node.get_logger().info("❌讀取PGNO失敗")

            case RobotContext.ACTING:
                self.node.get_logger().info("Unloader Robot Put Oven PUT OVEN ACTING")
                if read_pgno is None:
                    self.node.get_logger().debug("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == (PUT_OVEN_PGNO):
                    self.node.get_logger().debug("🤖手臂動作中")
                elif read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅手臂動作完成")
                    self.step = RobotContext.FINISH
                else:
                    self.node.get_logger().info("❌手臂動作失敗")
            case RobotContext.FINISH:
                self.node.get_logger().info("Unloader Robot Put Oven PUT OVEN Finish")
                if read_pgno is None:
                    self.node.get_logger().debug("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅放置到烤箱完成")
                    self.step = RobotContext.UPDATE_DATABASE
                else:
                    self.node.get_logger().info("❌放置到烤箱失敗")
            case RobotContext.UPDATE_DATABASE:
                self.node.get_logger().info("Unloader Robot Put Oven PUT OVEN UPDATE_DATABASE")

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
                            f"\n  Oven批次: {context.take_put_port_groups[context.take_put_cycle_count]}")

                        # 重置狀態
                        self.sent = False
                        self.step = RobotContext.IDLE
                        self.update_carrier_success = False

                        # 返回 TakeAgvState 進行第2次循環
                        from unloader_agv.robot_states.put_oven.take_agv_state import TakeAgvState
                        context.set_state(TakeAgvState(self.node))
                    else:
                        # ====== 第2次完成 ======
                        if self.update_carrier_success:
                            self.node.get_logger().info("✅ 更新所有4個 Carrier 資料庫成功")
                            self.sent = False
                            self.update_carrier_success = False

                            # 完成 PUT_OVEN 流程，進入完成狀態
                            self.step = RobotContext.IDLE
                            from unloader_agv.robot_states.complete_state import CompleteState
                            context.set_state(CompleteState(self.node))

    def handle(self, context: RobotContext):
        self.node.get_logger().info("Unloader Robot Put Oven PutOven 狀態")

        # 並行執行：Hokuyo write_busy 設定
        self._set_hokuyo_busy()

        # 並行執行：其他操作（不需等待 Hokuyo 完成）
        PUT_OVEN_PGNO = context.robot.ACTION_TO + \
            context.robot.NONE_POSITION + context.robot.OVEN_POSITION
        read_pgno = context.robot.read_pgno_response
        context.robot.read_robot_status()

        # 更新 Hokuyo Input
        self._handle_hokuyo_input()

        # 條件執行：只有機器人邏輯需要等待 Hokuyo 完成
        if self.hokuyo_busy_write_completed:
            self._execute_robot_logic(context, PUT_OVEN_PGNO, read_pgno)
