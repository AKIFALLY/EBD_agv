from db_proxy.carrier_query_client import CarrierQueryClient
from db_proxy.eqp_signal_query_client import EqpSignalQueryClient
from rclpy.node import Node
from unloader_agv.robot_context import RobotContext  # 新增的匯入
from std_msgs.msg import Bool  # 匯入 ROS 2 的 Bool 訊息型態
from agv_base.hokuyo_dms_8bit import HokuyoDMS8Bit


from unloader_agv.robot_states.base_robot_state import BaseRobotState


class BoxoutTransferCheckEmptyState(BaseRobotState):
    RobotContext.boxout_up_both_empty = True

    # 8 BIT STEP
    IDLE = 0
    WRITE_VALID = 1
    WRITE_PORT_NUMBER = 2
    WAIT_LOAD_REQ = 3
    WRITE_TR_REQ = 4
    WAIT_READY = 5

    SELECT_PORT01_PORT02, SELECT_PORT03_PORT04, SELECT_NONE = 1, 3, 0

    def __init__(self, node: Node):
        super().__init__(node)
        self.hokuyo_dms_8bit_1: HokuyoDMS8Bit = self.node.hokuyo_dms_8bit_1
        self.eqp_signal_query_client = EqpSignalQueryClient(node)
        self.carrier_query_client = CarrierQueryClient(node)

        # 動態計算 port_address 和 eqp_id
        self.port_address = self.node.room_id * 1000 + 20
        self.eqp_id = self.node.room_id * 100 + 2

        self.select_boxout_port_table = {
            (0, 0, 0, 0): self.SELECT_PORT01_PORT02, (0, 0, 0, 1): self.SELECT_PORT01_PORT02, (0, 0, 1, 1): self.SELECT_PORT01_PORT02, (0, 0, 1, 0): self.SELECT_PORT01_PORT02,
            (1, 0, 0, 0): self.SELECT_PORT03_PORT04, (1, 1, 0, 0): self.SELECT_PORT03_PORT04, (0, 1, 0, 0): self.SELECT_PORT03_PORT04,
        }

        self._reset_state()

    def _reset_state(self):
        """重置所有狀態變數"""
        self.check_ok = False
        self.step = self.IDLE
        self.sent = False
        # hokuyo_input_updated 已移除，因為需要持續更新
        self.search_eqp_signal_ok = False
        self.carrier_query_sended = False
        self.carrier_query_success = False
        self.port_carriers = [True] * 4  # 統一管理四個port的狀態
        self.select_boxout_port = self.SELECT_NONE
        self.carrier_id = None
        self.carrier_id_min = None  # 存儲 port_id_min 對應的 carrier_id
        self.carrier_id_max = None  # 存儲 port_id_max 對應的 carrier_id

    def enter(self):
        self.node.get_logger().info("Unloader Robot Put BoxoutTransfer 目前狀態: BoxoutTransferCheckEmpty")
        self._reset_state()

    def leave(self):
        self.node.get_logger().info("Unloader Robot Put BoxoutTransfer 離開 BoxoutTransferCheckEmpty 狀態")
        self._reset_state()

    def eqp_signal_query_callback(self, response):
        for i in range(4):
            self.port_carriers[i] = EqpSignalQueryClient.eqp_signal_port(
                response, self.port_address + i + 1)
            self.node.get_logger().info(
                f"Port {i+1:02d} 有無貨: {self.port_carriers[i]}")

        self.search_eqp_signal_ok = True
        port_states = tuple(int(carrier) for carrier in self.port_carriers)
        self.select_boxout_port = self.select_boxout_port_table.get(
            port_states, self.SELECT_NONE)

    def carrier_callback(self, response):
        self.carrier_query_success = response.success
        # 查詢兩個連續 port 的 carrier 資料
        port_id_min = self.port_address + self.box_number
        port_id_max = port_id_min + 1

        self.carrier_id_min = CarrierQueryClient.carrier_port_id_carrier_id(
            response, port_id_min)
        self.carrier_id_max = CarrierQueryClient.carrier_port_id_carrier_id(
            response, port_id_max)

        # 保持向後兼容性
        self.carrier_id = self.carrier_id_min

    def _update_context_states(self, context: RobotContext):
        """更新context中的狀態"""
        if not self.search_eqp_signal_ok:
            return

        # 上層狀態
        context.boxout_up_both_empty = not self.port_carriers[0] and not self.port_carriers[1]
        context.boxout_up_left_empty = not self.port_carriers[0] and self.port_carriers[1]
        context.boxout_up_right_empty = self.port_carriers[0] and not self.port_carriers[1]

        # 下層狀態
        context.boxout_down_both_empty = not self.port_carriers[2] and not self.port_carriers[3]
        context.boxout_down_left_empty = not self.port_carriers[2] and self.port_carriers[3]
        context.boxout_down_right_empty = self.port_carriers[2] and not self.port_carriers[3]

    def _validate_eqp_states(self, context: RobotContext):
        """驗證 Carrier 查詢結果與 EQP 狀態的一致性（PUT 操作空位檢查）"""
        validation_passed = True
        validation_errors = []

        # 驗證第一個 port (carrier_id_min 應為 None，對應 EQP 狀態應為 False)
        if self.carrier_id_min is None:
            port_number = self.box_number
            # boxout transfer 使用 port_carriers 陣列，port_number 對應 index (port_number-1)
            eqp_state = self.port_carriers[port_number - 1]
            if eqp_state:
                validation_passed = False
                validation_errors.append(f"BOXOUT_PORT{port_number}: Carrier查詢顯示空位但EQP狀態顯示有貨")
                self.node.get_logger().error(
                    f"❌ 資料不一致 - BOXOUT_PORT{port_number}: Carrier查詢=空位, EQP狀態={eqp_state}")
            else:
                self.node.get_logger().info(
                    f"✅ BOXOUT_PORT{port_number} 驗證通過: Carrier查詢=空位, EQP狀態={eqp_state}")

        # 驗證第二個 port (carrier_id_max 應為 None，對應 EQP 狀態應為 False)
        if self.carrier_id_max is None:
            port_number = self.box_number + 1
            # boxout transfer 使用 port_carriers 陣列，port_number 對應 index (port_number-1)
            eqp_state = self.port_carriers[port_number - 1]
            if eqp_state:
                validation_passed = False
                validation_errors.append(f"BOXOUT_PORT{port_number}: Carrier查詢顯示空位但EQP狀態顯示有貨")
                self.node.get_logger().error(
                    f"❌ 資料不一致 - BOXOUT_PORT{port_number}: Carrier查詢=空位, EQP狀態={eqp_state}")
            else:
                self.node.get_logger().info(
                    f"✅ BOXOUT_PORT{port_number} 驗證通過: Carrier查詢=空位, EQP狀態={eqp_state}")

        return validation_passed, validation_errors

    def _handle_port_selection(self, context: RobotContext):
        """處理port選擇邏輯"""
        if self.check_ok or not self.search_eqp_signal_ok:
            return

        port_messages = {
            self.SELECT_PORT01_PORT02: ("上層兩邊都空的", "PORT1_PORT2", 1),
            self.SELECT_PORT03_PORT04: ("下層兩邊都空的", "PORT3_PORT4", 3)
        }

        if self.select_boxout_port in port_messages:
            desc, port, number = port_messages[self.select_boxout_port]
            self.node.get_logger().info(
                f"Unloader Robot Put BoxoutTransfer BoxoutTransferCheckEmpty 狀態: {desc}")
            self.node.get_logger().info(f"執行出料傳送箱{port}")
            context.get_unloader_agv_port_back = number
            self.check_ok = True
        else:
            self.node.get_logger().info("Unloader Robot Put BoxoutTransfer BoxoutTransferCheckEmpty 狀態: 傳送箱已滿")
            self.node.get_logger().info("無法執行傳送箱操作，請檢查傳送箱狀態。")
            context.get_unloader_agv_port_back = None
            self._reset_state()

    def _handle_step_operation(self, context: RobotContext, step_name: str, operation_func, success_attr: str, failed_attr: str, next_step: int):
        """統一處理步驟操作的邏輯"""
        if not self.sent:
            operation_func()
            self.sent = True

        if getattr(self.hokuyo_dms_8bit_1, success_attr):
            self.node.get_logger().info(f"✅{step_name}成功")
            setattr(self.hokuyo_dms_8bit_1, success_attr, False)
            self.sent = False
            self.step = next_step
        elif getattr(self.hokuyo_dms_8bit_1, failed_attr):
            self.node.get_logger().error(f"❌{step_name}失敗")
            setattr(self.hokuyo_dms_8bit_1, failed_attr, False)
            self.sent = False
        else:
            self.node.get_logger().info(f"⏳等待{step_name}")

    def handle(self, context: RobotContext):
        self._update_context_states(context)

        # 查詢EQP信號
        if not self.search_eqp_signal_ok and not self.sent:
            self.eqp_signal_query_client.search_eqp_signal_eqp_id(
                self.eqp_id, self.eqp_signal_query_callback)
            self.sent = True

        # 更新 Hokuyo Input - 使用統一方法
        self._handle_hokuyo_input()

        self._handle_port_selection(context)

        # 查詢Carrier
        if self.check_ok and not self.carrier_query_sended:
            self.box_number = context.get_unloader_agv_port_back
            port_id_target = self.port_address + self.box_number
            self.carrier_query_client.search_carrier_port_id(
                port_id_min=port_id_target, port_id_max=port_id_target+1, callback=self.carrier_callback)
            self.carrier_query_sended = True

        # 處理Carrier查詢結果
        if self.check_ok and self.carrier_query_success:
            port_id_min = self.port_address + self.box_number
            port_id_max = port_id_min + 1

            # 檢查兩個 port 是否都沒有 carrier（適合 PUT 操作的空位檢查）
            if self.carrier_id_min is not None or self.carrier_id_max is not None:
                # 至少有一個 port 有貨，無法執行 PUT 操作
                occupied_ports = []
                if self.carrier_id_min is not None:
                    occupied_ports.append(f"Port {port_id_min} (carrier: {self.carrier_id_min})")
                if self.carrier_id_max is not None:
                    occupied_ports.append(f"Port {port_id_max} (carrier: {self.carrier_id_max})")

                self.node.get_logger().info(f"Carrier 查詢成功，發現佔用的 Port: {', '.join(occupied_ports)}")
                self.node.get_logger().info(
                    f"雙 Port 組合 {port_id_min}-{port_id_max} 有貨物佔用，無法執行傳送箱操作。")
                self._reset_state()
            else:
                # 兩個 port 都沒有 carrier，進行 EQP 信號狀態驗證
                self.node.get_logger().info(
                    f"雙 Port 組合 {port_id_min}-{port_id_max} 都是空的，開始 EQP 狀態驗證。")

                # 執行 EQP 狀態驗證
                validation_passed, validation_errors = self._validate_eqp_states(context)

                if validation_passed:
                    # EQP 狀態驗證通過，可以執行 PUT 操作
                    self.node.get_logger().info(
                        f"✅ 傳送箱 {port_id_min}-{port_id_max} EQP 狀態驗證通過，可以執行傳送箱操作")
                    context.get_boxout_port = context.get_unloader_agv_port_back
                    self._handle_8bit_steps(context)
                else:
                    # EQP 狀態驗證失敗，重置狀態
                    self.node.get_logger().error(f"❌ 傳送箱 {port_id_min}-{port_id_max} EQP 狀態驗證失敗:")
                    for error in validation_errors:
                        self.node.get_logger().error(f"   - {error}")
                    self.node.get_logger().error("Carrier 查詢結果與 EQP 硬體信號狀態不一致，重置狀態")
                    self._reset_state()

    def _handle_8bit_steps(self, context: RobotContext):
        """處理8bit步驟"""
        match self.step:
            case self.IDLE:
                self.step = self.WRITE_VALID
                self.sent = False

            case self.WRITE_VALID:
                self._handle_step_operation(context, "valid寫入",
                                            lambda: self.hokuyo_dms_8bit_1.write_valid(
                                                "1"),
                                            "valid_success", "valid_failed", self.WRITE_PORT_NUMBER)

            case self.WRITE_PORT_NUMBER:
                self._handle_step_operation(context, "port number寫入",
                                            lambda: self.hokuyo_dms_8bit_1.write_port_number(
                                                context.get_unloader_agv_port_back),
                                            "port_number_success", "port_number_failed", self.WAIT_LOAD_REQ)

            case self.WAIT_LOAD_REQ:
                if self.hokuyo_dms_8bit_1.load_req:
                    self.node.get_logger().info("✅收到load_req")
                    self.step = self.WRITE_TR_REQ
                else:
                    self.node.get_logger().info("⏳等待load_req")

            case self.WRITE_TR_REQ:
                self._handle_step_operation(context, "tr_req寫入",
                                            lambda: self.hokuyo_dms_8bit_1.write_tr_req(
                                                "1"),
                                            "tr_req_success", "tr_req_failed", self.WAIT_READY)

            case self.WAIT_READY:
                if self.hokuyo_dms_8bit_1.ready:
                    self.node.get_logger().info("✅收到ready")
                    self.step = self.IDLE
                    from unloader_agv.robot_states.put_boxout_transfer.agv_port_check_have_state import AgvPortCheckHaveState
                    context.set_state(AgvPortCheckHaveState(self.node))
                else:
                    self.node.get_logger().info("⏳等待ready")
