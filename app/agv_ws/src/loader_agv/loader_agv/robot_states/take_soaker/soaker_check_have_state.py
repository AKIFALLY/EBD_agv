from db_proxy.carrier_query_client import CarrierQueryClient
from db_proxy.eqp_signal_query_client import EqpSignalQueryClient
from rclpy.node import Node
from loader_agv.robot_context import RobotContext
from agv_base.hokuyo_dms_8bit import HokuyoDMS8Bit
from loader_agv.robot_states.base_robot_state import BaseRobotState


class SoakerCheckHaveState(BaseRobotState):

    # 8 BIT STEP
    IDLE = 0
    WRITE_VALID = 1
    WRITE_PORT_NUMBER = 2
    WAIT_LOAD_REQ = 3
    WRITE_TR_REQ = 4
    WAIT_READY = 5

    SELECT_PORT01, SELECT_PORT02, SELECT_PORT03, SELECT_PORT04, SELECT_PORT05, SELECT_PORT06, SELECT_NONE = 1, 2, 3, 4, 5, 6, 0

    def __init__(self, node: Node):
        super().__init__(node)
        self.node = node
        self.hokuyo_dms_8bit_1: HokuyoDMS8Bit = self.node.hokuyo_dms_8bit_1
        self.eqp_signal_query_client = EqpSignalQueryClient(node)
        self.carrier_query_client = CarrierQueryClient(node)

        # 動態計算 port_address 和 eqp_id
        self.port_address = self.node.room_id * 1000 + 40
        self.eqp_id = self.node.room_id * 100 + 4

        self._reset_state()

    def _reset_state(self):
        """重置所有狀態變數"""
        self.check_ok = False
        self.step = self.IDLE
        self.sent = False
        self.search_eqp_signal_ok = False
        self.carrier_query_sended = False
        self.carrier_query_success = False
        self.port_carriers = [False] * 6  # 浸泡有6個port
        self.select_soaker_port = self.SELECT_NONE
        self.carrier_id = None
        self.soaker_number = None

    def enter(self):
        self.node.get_logger().info("Loader Robot Take Soaker 目前狀態: SoakerCheckHave")
        self._reset_state()

    def leave(self):
        self.node.get_logger().info("Loader Robot Take Soaker 離開 SoakerCheckHave 狀態")
        self._reset_state()

    def eqp_signal_query_callback(self, response):
        for i in range(6):
            self.port_carriers[i] = EqpSignalQueryClient.eqp_signal_port(
                response, self.port_address + i + 1)
            self.node.get_logger().info(
                f"Soaker Port {i+1:02d} 有無貨: {self.port_carriers[i]}")

        self.search_eqp_signal_ok = True

    def carrier_callback(self, response):
        self.carrier_query_success = response.success
        port_id_target = self.port_address + self.soaker_number
        self.carrier_id = CarrierQueryClient.carrier_port_id_carrier_id(
            response, port_id_target)

    def _update_context_states(self, context: RobotContext):
        """更新context中的狀態"""
        if not self.search_eqp_signal_ok:
            return

        # 設定 soaker port 狀態
        context.soaker_port1 = self.port_carriers[0]
        context.soaker_port2 = self.port_carriers[1]
        context.soaker_port3 = self.port_carriers[2]
        context.soaker_port4 = self.port_carriers[3]
        context.soaker_port5 = self.port_carriers[4]
        context.soaker_port6 = self.port_carriers[5]

    def _extract_port_from_work_id(self, context: RobotContext):
        """從 work_id 中提取端口號碼"""
        try:
            work_id = context.work_id
            work_id_str = str(work_id)

            # work_id 格式: room_id + SOAKER + port_number + TAKE
            # 例如: room_id=1, SOAKER="04", port="01", TAKE="01" -> 1040101
            # 提取倒數第4和第3位數字作為端口號碼
            if len(work_id_str) >= 4:
                port_str = work_id_str[-4:-2]  # 提取端口號碼部分
                port_number = int(port_str)
                self.node.get_logger().info(f"從 work_id {work_id} 解析出端口號碼: {port_number}")
                return port_number
            else:
                self.node.get_logger().error(f"work_id {work_id} 格式不正確，無法解析端口號碼")
                return None
        except Exception as e:
            self.node.get_logger().error(f"解析 work_id 時發生錯誤: {e}")
            return None

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
        self._update_context_states(context)

        # 查詢EQP信號
        if not self.search_eqp_signal_ok and not self.sent:
            self.eqp_signal_query_client.search_eqp_signal_eqp_id(
                self.eqp_id, self.eqp_signal_query_callback)
            self.sent = True

        # 更新 Hokuyo Input - 使用統一方法
        self._handle_hokuyo_input()

        print("🔶=========================================================================🔶")

        # 直接從 work_id 解析端口號碼並進行驗證
        if self.search_eqp_signal_ok and not self.check_ok:
            # 從 work_id 中解析端口號碼
            self.soaker_number = self._extract_port_from_work_id(context)
            if self.soaker_number is None:
                self.node.get_logger().error("無法從 work_id 解析端口號碼，重置狀態")
                self._reset_state()
                return

            # 檢查解析出的端口號碼是否有效
            if not (1 <= self.soaker_number <= 6):
                self.node.get_logger().error(f"❌ 無效的端口號碼: {self.soaker_number}")
                self._reset_state()
                return

            # 使用 EQP 狀態進行驗證：檢查對應的 context.soaker_portX 狀態（TAKE 操作：檢查是否有貨物）
            port_eqp_has_cargo = self.port_carriers[self.soaker_number - 1]

            if port_eqp_has_cargo:
                self.node.get_logger().info(
                    f"✅ work_id 指定的浸泡端口 {self.soaker_number} EQP 狀態顯示有貨物，準備查詢 Carrier")
                self.check_ok = True
            else:
                self.node.get_logger().warn(
                    f"❌ work_id 指定的浸泡端口 {self.soaker_number} EQP 狀態顯示沒有貨物，無法執行 TAKE 操作")
                self._reset_state()
                return

        # 查詢Carrier
        if self.check_ok and not self.carrier_query_sended:
            port_id_target = self.port_address + self.soaker_number
            self.node.get_logger().info(
                f"查詢浸泡端口 {self.soaker_number} (port_id: {port_id_target}) 的 Carrier")
            self.carrier_query_client.search_carrier_port_id(
                port_id_min=port_id_target, port_id_max=port_id_target, callback=self.carrier_callback)
            self.carrier_query_sended = True

        # 處理Carrier查詢結果
        if self.check_ok and self.carrier_query_success:
            port_id_target = self.port_address + self.soaker_number
            port_eqp_has_cargo = self.port_carriers[self.soaker_number - 1]

            # 與 Carrier 查詢結果比較：檢查 EQP 狀態與 carrier 查詢結果是否一致（TAKE 操作：都應該有貨物）
            if self.carrier_id is not None and port_eqp_has_cargo:
                # 雙重驗證成功：EQP 狀態顯示有貨物，carrier 查詢也顯示有貨物
                self.node.get_logger().info(
                    f"✅ 雙重驗證成功：浸泡端口 {self.soaker_number} EQP 狀態和 Carrier 查詢都顯示有貨物")
                context.get_soaker_port = self.soaker_number
                self.node.get_logger().info(
                    f"設定 context.get_soaker_port = {self.soaker_number}")

                # 設定 carrier_id 給 context
                context.carrier_id = self.carrier_id
                self.node.get_logger().info(f"✅ 設定 context.carrier_id = {self.carrier_id}")

                self._handle_8bit_steps(context)
            elif self.carrier_id is None:
                # Carrier 查詢顯示沒有貨物
                self.node.get_logger().warn(
                    f"❌ 浸泡端口 {self.soaker_number} 沒有 carrier，無法執行 TAKE 操作")
                self._reset_state()
            elif not port_eqp_has_cargo:
                # EQP 狀態與 carrier 查詢結果不一致
                self.node.get_logger().warn(
                    f"❌ 數據不一致：浸泡端口 {self.soaker_number} EQP 狀態顯示沒有貨物，但 carrier 查詢顯示有貨物")
                self._reset_state()
            else:
                # 其他未預期的情況
                self.node.get_logger().error(f"❌ 未預期的驗證結果：端口 {self.soaker_number}")
                self._reset_state()

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
                                                context.get_soaker_port),
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
                    # 浸泡檢查完成，轉換到 AGV 端口檢查狀態
                    self.node.get_logger().info("✅ 浸泡檢查完成: 進入 AgvPortCheckEmptyState")
                    from loader_agv.robot_states.take_soaker.agv_port_check_empty_state import AgvPortCheckEmptyState
                    context.set_state(AgvPortCheckEmptyState(self.node))
                else:
                    self.node.get_logger().info("⏳等待ready")
