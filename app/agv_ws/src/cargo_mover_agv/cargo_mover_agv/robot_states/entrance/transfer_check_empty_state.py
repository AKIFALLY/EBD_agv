from agv_base.states.state import State
from db_proxy.carrier_query_client import CarrierQueryClient
from db_proxy.eqp_signal_query_client import EqpSignalQueryClient
from rclpy.node import Node
from cargo_mover_agv.robot_context import RobotContext  # 新增的匯入
from std_msgs.msg import Bool  # 匯入 ROS 2 的 Bool 訊息型態
from agv_base.hokuyo_dms_8bit import HokuyoDMS8Bit


class TransferCheckEmptyState(State):
    RobotContext.boxin_up_both_empty = True

    # 8 BIT STEP
    IDLE = 0
    WRITE_VAILD = 1
    WRITE_PORT_NUMBER = 2
    WAIT_LOAD_REQ = 3
    WRITE_TR_REQ = 4
    WAIT_READY = 5

    PORT_ADDRESS = 2010
    EQP_ID = 201

    SELECT_PORT01, SELECT_PORT02, SELECT_PORT03, SELECT_PORT04, SELECT_NONE = 1, 2, 3, 4, 0

    def __init__(self, node: Node):
        super().__init__(node)
        self.hokuyo_dms_8bit_1: HokuyoDMS8Bit = self.node.hokuyo_dms_8bit_1
        self.eqp_signal_query_client = EqpSignalQueryClient(node)
        self.carrier_query_client = CarrierQueryClient(node)

        self.select_boxin_port_table = {
            (0, 0, 0, 0): self.SELECT_PORT01, (0, 0, 0, 1): self.SELECT_PORT01, (0, 0, 1, 1): self.SELECT_PORT01,
            (1, 0, 0, 0): self.SELECT_PORT02, (1, 0, 0, 1): self.SELECT_PORT02, (1, 0, 1, 1): self.SELECT_PORT02,
            (1, 1, 0, 0): self.SELECT_PORT03, (0, 1, 0, 0): self.SELECT_PORT03,
            (1, 1, 1, 0): self.SELECT_PORT04, (0, 0, 1, 0): self.SELECT_PORT04, (0, 1, 1, 0): self.SELECT_PORT04,
        }

        self._reset_state()

    def _reset_state(self):
        """重置所有狀態變數"""
        self.check_ok = False
        self.step = self.IDLE
        self.sent = False
        self.hokuyo_input_updated = False
        self.search_eqp_signal_ok = False
        self.carrier_query_sended = False
        self.carrier_query_success = False
        self.port_carriers = [True] * 4  # 統一管理四個port的狀態
        self.select_boxin_port = self.SELECT_NONE
        self.carrier_id = None

    def enter(self):
        self.node.get_logger().info("Robot Entrance 目前狀態: TranferCheckEmpty")
        self._reset_state()

    def leave(self):
        self.node.get_logger().info("Robot Entrance 離開 TranferCheckEmpty 狀態")
        self._reset_state()

    def eqp_signal_query_callback(self, response):
        for i in range(4):
            self.port_carriers[i] = EqpSignalQueryClient.eqp_signal_port(
                response, self.PORT_ADDRESS + i + 1)
            self.node.get_logger().info(
                f"Port {i+1:02d} 有無貨: {self.port_carriers[i]}")

        self.search_eqp_signal_ok = True
        port_states = tuple(int(carrier) for carrier in self.port_carriers)
        self.select_boxin_port = self.select_boxin_port_table.get(
            port_states, self.SELECT_NONE)

    def carrier_callback(self, response):
        self.carrier_query_success = response.success
        self.carrier_id = CarrierQueryClient.carrier_port_id_carrier_id(
            response, self.PORT_ADDRESS + getattr(self, 'box_number', 0))

    def _update_context_states(self, context: RobotContext):
        """更新context中的狀態"""
        if not self.search_eqp_signal_ok:
            return

        # 上層狀態
        context.boxin_up_both_empty = not self.port_carriers[0] and not self.port_carriers[1]
        context.boxin_up_left_empty = not self.port_carriers[0] and self.port_carriers[1]
        context.boxin_up_right_empty = self.port_carriers[0] and not self.port_carriers[1]

        # 下層狀態
        context.boxin_down_both_empty = not self.port_carriers[2] and not self.port_carriers[3]
        context.boxin_down_left_empty = not self.port_carriers[2] and self.port_carriers[3]
        context.boxin_down_right_empty = self.port_carriers[2] and not self.port_carriers[3]

    def _handle_port_selection(self, context: RobotContext):
        """處理port選擇邏輯"""
        if self.check_ok or not self.search_eqp_signal_ok:
            return

        port_messages = {
            self.SELECT_PORT01: ("上層左邊空的", "PORT1", 1),
            self.SELECT_PORT02: ("上層右邊空的", "PORT2", 2),
            self.SELECT_PORT03: ("下層左邊空的", "PORT3", 3),
            self.SELECT_PORT04: ("下層右邊空的", "PORT4", 4)
        }

        if self.select_boxin_port in port_messages:
            desc, port, number = port_messages[self.select_boxin_port]
            self.node.get_logger().info(
                f"Robot Entrance TranferCheckEmpty 狀態: {desc}")
            self.node.get_logger().info(f"執行入口傳送箱{port}")
            context.boxin_number = number
            self.check_ok = True
        else:
            self.node.get_logger().info("Robot Entrance TranferCheckEmpty 狀態: 傳送箱已滿")
            self.node.get_logger().info("無法執行傳送箱操作，請檢查傳送箱狀態。")
            context.boxin_number = None
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
            self.node.get_logger().info(f"❌{step_name}失敗")
            setattr(self.hokuyo_dms_8bit_1, failed_attr, False)
            self.sent = False
        else:
            self.node.get_logger().info(f"⏳等待{step_name}")

    def handle(self, context: RobotContext):
        self._update_context_states(context)

        # 查詢EQP信號
        if not self.search_eqp_signal_ok and not self.sent:
            self.eqp_signal_query_client.search_eqp_signal_eqp_id(
                self.EQP_ID, self.eqp_signal_query_callback)
            self.sent = True

        # 更新Hokuyo Input
        if not self.hokuyo_input_updated:
            self.hokuyo_dms_8bit_1.update_hokuyo_input()
            self.hokuyo_input_updated = True

        if self.hokuyo_dms_8bit_1.hokuyo_input_success:
            self.node.get_logger().info("Hokuyo Input 更新成功")
            self.hokuyo_dms_8bit_1.hokuyo_input_success = False
            self.hokuyo_input_updated = False
        elif self.hokuyo_dms_8bit_1.hokuyo_input_failed:
            self.node.get_logger().info("Hokuyo Input 更新失敗")
            self.hokuyo_dms_8bit_1.hokuyo_input_failed = False
            self.hokuyo_input_updated = False

        print("🔶=========================================================================🔶")

        self._handle_port_selection(context)

        # 查詢Carrier
        if self.check_ok and not self.carrier_query_sended:
            self.box_number = context.boxin_number
            self.carrier_query_client.search_carrier_port_id(
                port_id=self.PORT_ADDRESS + self.box_number, callback=self.carrier_callback)
            self.carrier_query_sended = True

        # 處理Carrier查詢結果
        if self.check_ok and self.carrier_query_success:
            if self.carrier_id is not None:
                self.node.get_logger().info(
                    f"Carrier 查詢成功，資料: {self.carrier_id}")
                self.node.get_logger().info(
                    f"傳送箱{self.PORT_ADDRESS + self.box_number}已經有貨，無法執行傳送箱操作。")
                self._reset_state()
            else:
                context.get_boxin_port = context.boxin_number
                self._handle_8bit_steps(context)

    def _handle_8bit_steps(self, context: RobotContext):
        """處理8bit步驟"""
        match self.step:
            case self.IDLE:
                self.step = self.WRITE_VAILD
                self.sent = False

            case self.WRITE_VAILD:
                self._handle_step_operation(context, "vaild寫入",
                                            lambda: self.hokuyo_dms_8bit_1.write_vaild(
                                                "1"),
                                            "vaild_success", "vaild_failed", self.WRITE_PORT_NUMBER)

            case self.WRITE_PORT_NUMBER:
                self._handle_step_operation(context, "port number寫入",
                                            lambda: self.hokuyo_dms_8bit_1.write_port_number(
                                                context.boxin_number),
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
                    from cargo_mover_agv.robot_states.entrance.select_rack_port_state import SelectRackPortState
                    context.set_state(SelectRackPortState(self.node))
                else:
                    self.node.get_logger().info("⏳等待ready")
