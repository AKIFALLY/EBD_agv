from rclpy.node import Node
from unloader_agv.robot_context import RobotContext
from agv_base.robot import Robot
from unloader_agv.robot_states.base_robot_state import BaseRobotState
from db_proxy_interfaces.msg import Carrier as CarrierMsg
from db_proxy.agvc_database_client import AGVCDatabaseClient


class PutAgvState(BaseRobotState):
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
        self.carrier_updates_pending = 0  # 待更新的 carrier 數量
        self.agvc_client = AGVCDatabaseClient(self.node)

        # 動態計算 port_id_address
        self.port_id_address = self.node.room_id * 1000 + 30  # AGV port address

    def enter(self):
        self.node.get_logger().info("Unloader Robot Take Oven 目前狀態: PutAgv")
        self.update_carrier_success = False
        self.update_carrier_1_success = False
        self.update_carrier_2_success = False
        self.update_carrier_3_success = False
        self.update_carrier_4_success = False
        self.carrier_updates_pending = 0
        self.sent = False

    def leave(self):
        self.node.get_logger().info("Unloader Robot Take Oven 離開 PutAgv 狀態")
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

        # 更新第1個 carrier (oven port 1 → AGV port 1)
        if context.carrier_id[0] is not None:
            carrier_1 = CarrierMsg()
            carrier_1.id = context.carrier_id[0]
            carrier_1.room_id = self.node.room_id
            carrier_1.rack_id = 0
            carrier_1.port_id = self.port_id_address + 1  # AGV port 1 (2111)
            carrier_1.rack_index = 0
            carrier_1.status_id = Robot.CARRIER_STATUS_OVEN_COMPLETED

            self.agvc_client.async_update_carrier(
                carrier_1, lambda result: self.update_carrier_1_callback(result))
            self.carrier_updates_pending += 1
            self.node.get_logger().info(
                f"🔄 開始更新第1個 Carrier: {context.carrier_id[0]} "
                f"→ AGV Port 1 (ID={carrier_1.port_id})")
        else:
            self.update_carrier_1_success = True

        # 更新第2個 carrier (oven port 2 → AGV port 2)
        if context.carrier_id[1] is not None:
            carrier_2 = CarrierMsg()
            carrier_2.id = context.carrier_id[1]
            carrier_2.room_id = self.node.room_id
            carrier_2.rack_id = 0
            carrier_2.port_id = self.port_id_address + 2  # AGV port 2 (2112)
            carrier_2.rack_index = 0
            carrier_2.status_id = Robot.CARRIER_STATUS_OVEN_COMPLETED

            self.agvc_client.async_update_carrier(
                carrier_2, lambda result: self.update_carrier_2_callback(result))
            self.carrier_updates_pending += 1
            self.node.get_logger().info(
                f"🔄 開始更新第2個 Carrier: {context.carrier_id[1]} "
                f"→ AGV Port 2 (ID={carrier_2.port_id})")
        else:
            self.update_carrier_2_success = True

        # 更新第3個 carrier (oven port 3 → AGV port 3)
        if context.carrier_id[2] is not None:
            carrier_3 = CarrierMsg()
            carrier_3.id = context.carrier_id[2]
            carrier_3.room_id = self.node.room_id
            carrier_3.rack_id = 0
            carrier_3.port_id = self.port_id_address + 3  # AGV port 3 (2113)
            carrier_3.rack_index = 0
            carrier_3.status_id = Robot.CARRIER_STATUS_OVEN_COMPLETED

            self.agvc_client.async_update_carrier(
                carrier_3, lambda result: self.update_carrier_3_callback(result))
            self.carrier_updates_pending += 1
            self.node.get_logger().info(
                f"🔄 開始更新第3個 Carrier: {context.carrier_id[2]} "
                f"→ AGV Port 3 (ID={carrier_3.port_id})")
        else:
            self.update_carrier_3_success = True

        # 更新第4個 carrier (oven port 4 → AGV port 4)
        if context.carrier_id[3] is not None:
            carrier_4 = CarrierMsg()
            carrier_4.id = context.carrier_id[3]
            carrier_4.room_id = self.node.room_id
            carrier_4.rack_id = 0
            carrier_4.port_id = self.port_id_address + 4  # AGV port 4 (2114)
            carrier_4.rack_index = 0
            carrier_4.status_id = Robot.CARRIER_STATUS_OVEN_COMPLETED

            self.agvc_client.async_update_carrier(
                carrier_4, lambda result: self.update_carrier_4_callback(result))
            self.carrier_updates_pending += 1
            self.node.get_logger().info(
                f"🔄 開始更新第4個 Carrier: {context.carrier_id[3]} "
                f"→ AGV Port 4 (ID={carrier_4.port_id})")
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

    def _execute_robot_logic(self, context: RobotContext, PUT_AGV_PGNO, read_pgno):
        """執行機器人邏輯"""
        match self.step:
            case RobotContext.IDLE:
                self.node.get_logger().info("Unloader Robot Take Oven PUT AGV IDLE")
                self.step = RobotContext.CHECK_IDLE
            case RobotContext.CHECK_IDLE:
                self.node.get_logger().info("Unloader Robot Take Oven PUT AGV CHECK_IDLE")
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
                self.node.get_logger().info("Unloader Robot Take Oven PUT AGV CHECK CHG PARAMETER")

                # 構建預期參數字典
                expected_params = {}

                # 檢查 unloader_agv_port_back → W110
                # layer_z = ((port-1) // 2) + 1, layer_y = 0
                layer_z_back = ((context.get_unloader_agv_port_back - 1) // 2) + 1
                layer_y_back = 0
                expected_params['w110'] = (layer_z_back | (layer_y_back << 16))

                self.node.get_logger().info(
                    f"預期檢查: agv_port_back={context.get_unloader_agv_port_back} → "
                    f"W110 (z={layer_z_back}, y={layer_y_back})")

                # 執行檢查
                if self._handle_check_chg_parameter(context, expected_params):
                    self.step = RobotContext.WRITE_CHG_PARA

            case RobotContext.WRITE_CHG_PARA:
                self.node.get_logger().info("Unloader Robot Take Oven PUT AGV WRITE CHG PARA")
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
                self.node.get_logger().info("Unloader Robot Take Oven PUT AGV CHECK CHG PARA")
                if read_pgno.value == Robot.CHG_PARA:
                    self.node.get_logger().info("✅讀取預執行成功")
                    self.step = RobotContext.WRITE_PGNO
                elif read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("🕒讀取預執行中...")
                else:
                    self.node.get_logger().info("❌讀取預執行失敗")

            case RobotContext.WRITE_PGNO:
                self.node.get_logger().info("Unloader Robot Take Oven PUT AGV WRITE_PGNO")
                if not self.sent:
                    context.robot.update_pgno(PUT_AGV_PGNO)
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
                self.node.get_logger().info("Unloader Robot Take Oven PUT AGV CHECK_PGNO")
                if read_pgno is None:
                    self.node.get_logger().debug("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == (PUT_AGV_PGNO):
                    self.node.get_logger().info("✅讀取PGNO成功")
                    self.step = RobotContext.ACTING
                elif read_pgno.value == Robot.CHG_PARA:
                    self.node.get_logger().info("🕒讀取PGNO中...")
                else:
                    self.node.get_logger().info("❌讀取PGNO失敗")

            case RobotContext.ACTING:
                self.node.get_logger().info("Unloader Robot Take Oven PUT AGV ACTING")
                if read_pgno is None:
                    self.node.get_logger().debug("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == (PUT_AGV_PGNO):
                    self.node.get_logger().debug("🤖手臂動作中")
                elif read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅手臂動作完成")
                    self.step = RobotContext.FINISH
                else:
                    self.node.get_logger().info("❌手臂動作失敗")
            case RobotContext.FINISH:
                self.node.get_logger().info("Unloader Robot Take Oven PUT AGV Finish")
                if read_pgno is None:
                    self.node.get_logger().debug("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅放置到AGV完成")

                    # TAKE 操作循環判斷邏輯（不更新資料庫）
                    cycle_num = context.take_put_cycle_count + 1

                    if context.take_put_cycle_count < context.take_put_max_cycles - 1:
                        # ====== 第1次完成，準備第2次 ======
                        self.node.get_logger().info(f"✅ 第{cycle_num}次 TAKE 完成")
                        context.take_put_cycle_count += 1
                        context.take_put_current_batch = context.take_put_port_groups[1]  # [3, 4]

                        self.node.get_logger().info(
                            f"準備第{context.take_put_cycle_count + 1}次循環："
                            f"\n  Oven批次: {context.take_put_current_batch}"
                            f"\n  AGV批次: ports {context.take_put_current_batch}")

                        # 重置狀態
                        self.sent = False
                        self.step = RobotContext.IDLE

                        # 返回 OvenVisionPositionState 進行第2次循環
                        from unloader_agv.robot_states.take_oven.oven_vision_position_state import OvenVisionPositionState
                        context.set_state(OvenVisionPositionState(self.node))
                    else:
                        # ====== 第2次完成，所有操作完成 ======
                        self.node.get_logger().info(f"✅ 第{cycle_num}次 TAKE 完成")
                        self.node.get_logger().info("✅ Take Oven 完整流程完成")

                        # 完成整個 TAKE_OVEN 流程，進入完成狀態
                        self.step = RobotContext.IDLE
                        from unloader_agv.robot_states.complete_state import CompleteState
                        context.set_state(CompleteState(self.node))
                else:
                    self.node.get_logger().info("❌放置到AGV失敗")

    def handle(self, context: RobotContext):
        self.node.get_logger().info("Unloader Robot Take Oven PutAgv 狀態")

        # 並行執行：Hokuyo write_busy 設定
        self._set_hokuyo_busy()

        # 並行執行：其他操作（不需等待 Hokuyo 完成）
        PUT_AGV_PGNO = context.robot.ACTION_TO + \
            context.robot.NONE_POSITION + context.robot.AGV_POSITION
        read_pgno = context.robot.read_pgno_response
        context.robot.read_robot_status()

        # 更新 Hokuyo Input
        self._handle_hokuyo_input()

        # 條件執行：只有機器人邏輯需要等待 Hokuyo 完成
        if self.hokuyo_busy_write_completed:
            self._execute_robot_logic(context, PUT_AGV_PGNO, read_pgno)
