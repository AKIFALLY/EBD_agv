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
        self.update_carrier_min_success = False  # 第一個 carrier 更新狀態
        self.update_carrier_max_success = False  # 第二個 carrier 更新狀態
        self.carrier_updates_pending = 0  # 待更新的 carrier 數量
        self.agvc_client = AGVCDatabaseClient(self.node)

        # 動態計算 port_id_address
        self.port_id_address = self.node.room_id * 1000 + 30  # AGV port address

    def enter(self):
        self.node.get_logger().info("Unloader Robot Take Pre Dryer 目前狀態: PutAgv")
        self.update_carrier_success = False
        self.update_carrier_min_success = False
        self.update_carrier_max_success = False
        self.carrier_updates_pending = 0
        self.sent = False

    def leave(self):
        self.node.get_logger().info("Unloader Robot Take Pre Dryer 離開 PutAgv 狀態")
        self.update_carrier_success = False
        self.update_carrier_min_success = False
        self.update_carrier_max_success = False
        self.carrier_updates_pending = 0
        self.sent = False

    def update_carrier_database(self, context: RobotContext):
        """更新所有4個 carrier 資料庫（第2次循環時調用）"""
        self.node.get_logger().info("📝 開始更新所有4個carrier到數據庫")

        # AGV port基礎地址
        port_id_address = context.robot.robot_parameter.room_id * 1000 + 110

        # 重置計數器
        self.carrier_updates_pending = 0
        self.carrier_update_results = [False, False, False, False]  # 追蹤4個carrier的更新狀態

        # 第1次操作的2個carrier (AGV上層 port 1,2)
        if context.carrier_id[0] is not None:
            carrier1 = CarrierMsg()
            carrier1.id = context.carrier_id[0]
            carrier1.room_id = self.node.room_id
            carrier1.rack_id = 0
            carrier1.port_id = port_id_address + 1  # 2111 (AGV port 1)
            carrier1.rack_index = 0
            carrier1.status_id = Robot.CARRIER_STATUS_PRE_DRYER_COMPLETED

            self.agvc_client.async_update_carrier(
                carrier1, lambda result, idx=0: self.carrier_update_callback(result, idx))
            self.carrier_updates_pending += 1
            self.node.get_logger().info(
                f"  📦 Carrier[0]: {carrier1.id} → port_id={carrier1.port_id}, status={carrier1.status_id}")
        else:
            self.carrier_update_results[0] = True

        if context.carrier_id[1] is not None:
            carrier2 = CarrierMsg()
            carrier2.id = context.carrier_id[1]
            carrier2.room_id = self.node.room_id
            carrier2.rack_id = 0
            carrier2.port_id = port_id_address + 2  # 2112 (AGV port 2)
            carrier2.rack_index = 0
            carrier2.status_id = Robot.CARRIER_STATUS_PREPARE_ENTER_OVEN

            self.agvc_client.async_update_carrier(
                carrier2, lambda result, idx=1: self.carrier_update_callback(result, idx))
            self.carrier_updates_pending += 1
            self.node.get_logger().info(
                f"  📦 Carrier[1]: {carrier2.id} → port_id={carrier2.port_id}, status={carrier2.status_id}")
        else:
            self.carrier_update_results[1] = True

        # 第2次操作的2個carrier (AGV下層 port 3,4)
        if context.carrier_id[2] is not None:
            carrier3 = CarrierMsg()
            carrier3.id = context.carrier_id[2]
            carrier3.room_id = self.node.room_id
            carrier3.rack_id = 0
            carrier3.port_id = port_id_address + 3  # 2113 (AGV port 3)
            carrier3.rack_index = 0
            carrier3.status_id = Robot.CARRIER_STATUS_PRE_DRYER_COMPLETED

            self.agvc_client.async_update_carrier(
                carrier3, lambda result, idx=2: self.carrier_update_callback(result, idx))
            self.carrier_updates_pending += 1
            self.node.get_logger().info(
                f"  📦 Carrier[2]: {carrier3.id} → port_id={carrier3.port_id}, status={carrier3.status_id}")
        else:
            self.carrier_update_results[2] = True

        if context.carrier_id[3] is not None:
            carrier4 = CarrierMsg()
            carrier4.id = context.carrier_id[3]
            carrier4.room_id = self.node.room_id
            carrier4.rack_id = 0
            carrier4.port_id = port_id_address + 4  # 2114 (AGV port 4)
            carrier4.rack_index = 0
            carrier4.status_id = Robot.CARRIER_STATUS_PREPARE_ENTER_OVEN

            self.agvc_client.async_update_carrier(
                carrier4, lambda result, idx=3: self.carrier_update_callback(result, idx))
            self.carrier_updates_pending += 1
            self.node.get_logger().info(
                f"  📦 Carrier[3]: {carrier4.id} → port_id={carrier4.port_id}, status={carrier4.status_id}")
        else:
            self.carrier_update_results[3] = True

        # 如果沒有需要更新的 carrier，直接設為成功
        if self.carrier_updates_pending == 0:
            self.update_carrier_success = True
            self.node.get_logger().info("ℹ️ 沒有需要更新的 Carrier")
        else:
            self.node.get_logger().info(f"✅ 已發送 {self.carrier_updates_pending} 個carrier更新請求")

    def carrier_update_callback(self, result, carrier_index):
        """統一的 carrier 更新回調（追蹤所有4個carrier的更新狀態）"""
        if result is not None and result.success:
            self.carrier_update_results[carrier_index] = True
            self.node.get_logger().info(
                f"✅ Carrier[{carrier_index}] 更新成功: {result.message}")
        else:
            self.carrier_update_results[carrier_index] = False
            self.node.get_logger().error(f"❌ Carrier[{carrier_index}] 更新失敗")

        # 檢查是否所有carrier都已更新完成
        self._check_all_carriers_updated()

    def _check_all_carriers_updated(self):
        """檢查所有 carrier 是否都已更新完成"""
        # 計算已完成的更新數量
        completed_count = sum(1 for result in self.carrier_update_results if result)

        # 如果所有待更新的carrier都已完成
        if completed_count >= self.carrier_updates_pending:
            # 檢查是否全部成功
            if all(self.carrier_update_results):
                self.update_carrier_success = True
                self.node.get_logger().info(f"✅✅ 所有{self.carrier_updates_pending}個Carrier更新完成！")
            else:
                self.update_carrier_success = False
                failed_indices = [i for i, result in enumerate(self.carrier_update_results) if not result]
                self.node.get_logger().error(f"❌ 部分Carrier更新失敗: {failed_indices}")

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
                self.node.get_logger().info("Unloader Robot Take Pre Dryer PUT AGV IDLE")
                self.step = RobotContext.CHECK_IDLE
            case RobotContext.CHECK_IDLE:
                self.node.get_logger().info("Unloader Robot Take Pre Dryer PUT AGV CHECK_IDLE")
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
                self.node.get_logger().info("Unloader Robot Take Pre Dryer PUT AGV CHECK CHG PARAMETER")

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
                self.node.get_logger().info("Unloader Robot Take Pre Dryer PUT AGV WRITE CHG PARA")
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
                self.node.get_logger().info("Unloader Robot Take Pre Dryer PUT AGV CHECK CHG PARA")
                if read_pgno.value == Robot.CHG_PARA:
                    self.node.get_logger().info("✅讀取預執行成功")
                    self.step = RobotContext.WRITE_PGNO
                elif read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("🕒讀取預執行中...")
                else:
                    self.node.get_logger().info("❌讀取預執行失敗")

            case RobotContext.WRITE_PGNO:
                self.node.get_logger().info("Unloader Robot Take Pre Dryer PUT AGV WRITE_PGNO")
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
                self.node.get_logger().info("Unloader Robot Take Pre Dryer PUT AGV CHECK_PGNO")
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
                self.node.get_logger().info("Unloader Robot Take Pre Dryer PUT AGV ACTING")
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
                self.node.get_logger().info("Unloader Robot Take Pre Dryer PUT AGV Finish")
                if read_pgno is None:
                    self.node.get_logger().debug("⏳等待讀取PGNO回應...")
                    return
                if read_pgno.value == Robot.IDLE:
                    self.node.get_logger().info("✅放置到AGV完成")
                    self.step = RobotContext.UPDATE_DATABASE
                else:
                    self.node.get_logger().info("❌放置到AGV失敗")
            case RobotContext.UPDATE_DATABASE:
                self.node.get_logger().info("Unloader Robot Take Pre Dryer PUT AGV UPDATE_DATABASE")
                if not self.sent:
                    # 根據循環次數決定是否更新數據庫
                    cycle_num = context.take_put_cycle_count + 1

                    if context.take_put_cycle_count == 0:
                        # 第1次: 不更新數據庫，只暫存
                        self.node.get_logger().info(f"✅ 第{cycle_num}次放料完成，暫不更新數據庫")
                    else:
                        # 第2次: 更新所有4個carrier
                        self.node.get_logger().info(f"✅ 第{cycle_num}次放料完成，開始更新所有4個carrier")
                        self.update_carrier_database(context)

                    self.sent = True

                elif self.sent:
                    # 判斷是否需要進行第2次循環
                    if context.take_put_cycle_count < context.take_put_max_cycles - 1:
                        # ====== 第1次完成，準備第2次 ======
                        context.take_put_cycle_count += 1

                        # 切換到第2組ports
                        context.take_put_current_batch = context.take_put_port_groups[1]
                        context.get_pre_dryer_port = context.take_put_current_batch[0]

                        # 更新AGV port參數（第2次放到下層port 3,4）
                        context.get_unloader_agv_port_back = 3

                        # 重置步驟和標誌
                        self.sent = False
                        self.step = RobotContext.IDLE
                        self.hokuyo_busy_write_completed = False

                        self.node.get_logger().info("=" * 80)
                        self.node.get_logger().info("✅✅ 第1次取放完成")
                        self.node.get_logger().info(
                            f"🔄 開始第2次操作："
                            f"\n  取 pre_dryer ports {context.take_put_current_batch}"
                            f"\n  放到 AGV ports [3, 4]")
                        self.node.get_logger().info("=" * 80)

                        # 返回 TakePreDryerState 進行第2次取放
                        from unloader_agv.robot_states.take_pre_dryer.take_pre_dryer_state import TakePreDryerState
                        context.set_state(TakePreDryerState(self.node))

                    else:
                        # ====== 第2次完成，所有操作完成 ======
                        # 確保數據庫更新成功
                        if self.update_carrier_success:
                            self.node.get_logger().info("=" * 80)
                            self.node.get_logger().info("✅✅✅ 兩次取放操作全部完成 ✅✅✅")
                            self.node.get_logger().info("✅ 已更新所有4個carrier的數據庫")
                            self.node.get_logger().info("=" * 80)

                            from unloader_agv.robot_states.complete_state import CompleteState
                            context.set_state(CompleteState(self.node))
                        else:
                            self.node.get_logger().error("❌ 數據庫更新失敗，等待重試")
                            return

                    self.sent = False
                    self.step = RobotContext.IDLE

    def handle(self, context: RobotContext):
        self.node.get_logger().info("Unloader Robot Take Pre Dryer PutAgv 狀態")

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
