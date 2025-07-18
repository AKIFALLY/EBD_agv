from db_proxy.carrier_query_client import CarrierQueryClient
from db_proxy.eqp_signal_query_client import EqpSignalQueryClient
from rclpy.node import Node
from loader_agv.robot_context import RobotContext
from agv_base.hokuyo_dms_8bit import HokuyoDMS8Bit
from loader_agv.robot_states.base_robot_state import BaseRobotState


class CleanerCheckHaveState(BaseRobotState):

    # 8 BIT STEP
    IDLE = 0
    WRITE_VALID = 1
    WRITE_PORT_NUMBER = 2
    WAIT_LOAD_REQ = 3
    WRITE_TR_REQ = 4
    WAIT_READY = 5

    SELECT_PORT01, SELECT_PORT02, SELECT_NONE = 1, 2, 0

    def __init__(self, node: Node):
        super().__init__(node)
        self.node = node
        self.hokuyo_dms_8bit_1: HokuyoDMS8Bit = self.node.hokuyo_dms_8bit_1
        self.eqp_signal_query_client = EqpSignalQueryClient(node)
        self.carrier_query_client = CarrierQueryClient(node)

        # 動態計算 port_address 和 eqp_id
        self.port_address = self.node.room_id * 1000 + 30
        self.eqp_id = self.node.room_id * 100 + 3

        self._reset_state()

    def _reset_state(self):
        """重置所有狀態變數"""
        self.check_ok = False
        self.step = self.IDLE
        self.sent = False
        self.search_eqp_signal_ok = False
        self.carrier_query_sended = False
        self.carrier_query_success = False
        self.port_carriers = [False] * 4  # 統一管理四個port的狀態
        self.select_cleaner_port = self.SELECT_NONE
        self.carrier_id = None
        # 重置 context 標誌重置標記
        self._context_flags_reset = False

    def enter(self):
        self.node.get_logger().info("Loader Robot Put Cleaner 目前狀態: CleanerCheckHave")
        self._reset_state()

    def _reset_context_flags(self, context: RobotContext):
        """重置 context 中的相關標誌"""
        # 重置 put_cleaner_continue 標誌，避免無限循環
        context.put_cleaner_continue = False
        self.node.get_logger().info("🔄 重置 put_cleaner_continue = False")

    def leave(self):
        self.node.get_logger().info("Loader Robot Put Cleaner 離開 CleanerCheckHave 狀態")
        self._reset_state()

    def eqp_signal_query_callback(self, response):
        for i in range(4):
            self.port_carriers[i] = EqpSignalQueryClient.eqp_signal_port(
                response, self.port_address + i + 1)

        # 簡化輸出 - 一次顯示所有 Port 狀態
        self.node.get_logger().debug(f"Cleaner Port 狀態: {self.port_carriers}")
        self.search_eqp_signal_ok = True

    def carrier_callback(self, response):
        self.carrier_query_success = response.success
        port_id_target = self.port_address + self.cleaner_number
        self.carrier_id = CarrierQueryClient.carrier_port_id_carrier_id(
            response, port_id_target)

    def _update_context_states(self, context: RobotContext):
        """更新context中的狀態"""
        if not self.search_eqp_signal_ok:
            return

        # 設定 cleaner port 狀態
        context.cleaner_port1 = self.port_carriers[0]
        context.cleaner_port2 = self.port_carriers[1]
        context.cleaner_port3 = self.port_carriers[2]
        context.cleaner_port4 = self.port_carriers[3]

    def _handle_port_selection(self, context: RobotContext):
        """處理 cleaner port 選擇邏輯"""
        if self.check_ok or not self.search_eqp_signal_ok:
            return

        # cleaner 埠位選擇邏輯
        if not context.cleaner_port1 and not context.cleaner_port2:
            # 如果 port1 和 port2 都沒有貨料（空的）
            self.node.get_logger().info("Loader Robot Put Cleaner CleanerCheckHave 狀態: Port1 和 Port2 都空的")
            self.node.get_logger().info("執行清洗機 PORT1")
            context.get_cleaner_port = 1
            context.put_cleaner_continue = True
            self.check_ok = True
        elif context.cleaner_port1 and not context.cleaner_port2:
            # 如果 port1 有貨料但 port2 沒有貨料
            self.node.get_logger().info("Loader Robot Put Cleaner CleanerCheckHave 狀態: Port1 有貨，Port2 空的")
            self.node.get_logger().info("執行清洗機 PORT2")
            context.get_cleaner_port = 2
            self.check_ok = True
        elif not context.cleaner_port1 and context.cleaner_port2:
            # 如果 port1 沒有貨料但 port2 有貨料
            self.node.get_logger().info("Loader Robot Put Cleaner CleanerCheckHave 狀態: Port1 空的，Port2 有貨")
            self.node.get_logger().info("執行清洗機 PORT1")
            context.get_cleaner_port = 1
            self.check_ok = True
        else:
            # 兩個 port 都有貨料
            self.node.get_logger().info("Loader Robot Put Cleaner CleanerCheckHave 狀態: 清洗機已滿")
            self.node.get_logger().info("無法執行清洗機操作，請檢查清洗機狀態。")
            context.get_cleaner_port = None
            self._reset_state()

    def _handle_step_operation(self, step_name: str, operation_func, success_attr: str, failed_attr: str, next_step: int):
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
        # 在處理開始時重置相關標誌
        if not hasattr(self, '_context_flags_reset'):
            self._reset_context_flags(context)
            self._context_flags_reset = True

        self._update_context_states(context)

        # 查詢EQP信號
        if not self.search_eqp_signal_ok and not self.sent:
            self.eqp_signal_query_client.search_eqp_signal_eqp_id(
                self.eqp_id, self.eqp_signal_query_callback)
            self.sent = True

        # 更新 Hokuyo Input - 使用統一方法
        self._handle_hokuyo_input()

        print("🔶=========================================================================🔶")

        self._handle_port_selection(context)

        # 查詢Carrier
        if self.check_ok and not self.carrier_query_sended:
            self.cleaner_number = context.get_cleaner_port
            port_id_target = self.port_address + self.cleaner_number
            self.carrier_query_client.search_carrier_port_id(
                port_id_min=port_id_target, port_id_max=port_id_target, callback=self.carrier_callback)
            self.carrier_query_sended = True

        # 處理Carrier查詢結果
        if self.check_ok and self.carrier_query_success:
            port_id_target = self.port_address + self.cleaner_number

            # 檢查選定的 port 是否有 carrier（適合 PUT 操作的空位檢查）
            if self.carrier_id is not None:
                # port 有貨，無法執行 PUT 操作
                self.node.get_logger().info(
                    f"Carrier 查詢成功，發現 Port {port_id_target} 有貨物 (carrier: {self.carrier_id})")
                self.node.get_logger().info(f"Port {port_id_target} 有貨物佔用，無法執行清洗機操作。")
                self._reset_state()
            else:
                # port 沒有 carrier，可以執行 PUT 操作
                self.node.get_logger().info(f"Port {port_id_target} 是空的，可以執行清洗機操作。")
                self._handle_8bit_steps(context)

    def _handle_8bit_steps(self, context: RobotContext):
        """處理8bit步驟"""
        match self.step:
            case self.IDLE:
                self.step = self.WRITE_VALID
                self.sent = False

            case self.WRITE_VALID:
                self._handle_step_operation("valid寫入",
                                            lambda: self.hokuyo_dms_8bit_1.write_valid("1"),
                                            "valid_success", "valid_failed", self.WRITE_PORT_NUMBER)

            case self.WRITE_PORT_NUMBER:
                self._handle_step_operation("port number寫入",
                                            lambda: self.hokuyo_dms_8bit_1.write_port_number(
                                                context.get_cleaner_port),
                                            "port_number_success", "port_number_failed", self.WAIT_LOAD_REQ)

            case self.WAIT_LOAD_REQ:
                if self.hokuyo_dms_8bit_1.load_req:
                    self.node.get_logger().info("✅收到load_req")
                    self.step = self.WRITE_TR_REQ
                else:
                    self.node.get_logger().info("⏳等待load_req")

            case self.WRITE_TR_REQ:
                self._handle_step_operation("tr_req寫入",
                                            lambda: self.hokuyo_dms_8bit_1.write_tr_req("1"),
                                            "tr_req_success", "tr_req_failed", self.WAIT_READY)

            case self.WAIT_READY:
                if self.hokuyo_dms_8bit_1.ready:
                    self.node.get_logger().info("✅收到ready")
                    self.step = self.IDLE
                    # 清洗機檢查完成，轉換到AGV端口檢查狀態
                    from loader_agv.robot_states.put_cleaner.agv_port_check_have_state import AgvPortCheckHaveState
                    context.set_state(AgvPortCheckHaveState(self.node))
                else:
                    self.node.get_logger().info("⏳等待ready")
