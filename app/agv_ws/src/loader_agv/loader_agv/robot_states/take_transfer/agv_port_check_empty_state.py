from agv_base.states.state import State
from db_proxy.carrier_query_client import CarrierQueryClient
from db_proxy.eqp_signal_query_client import EqpSignalQueryClient
from rclpy.node import Node
from loader_agv.robot_context import RobotContext
from std_msgs.msg import Bool


class AgvPortCheckEmptyState(State):

    PORT_ADDRESS = 2100
    EQP_ID = 210

    SELECT_PORT01, SELECT_PORT02, SELECT_PORT03, SELECT_PORT04, SELECT_NONE = 1, 2, 3, 4, 0

    def __init__(self, node: Node):
        super().__init__(node)
        self.eqp_signal_query_client = EqpSignalQueryClient(node)
        self.carrier_query_client = CarrierQueryClient(node)

        self.select_agv_port_table = {
            (0, 0, 0, 0): self.SELECT_PORT01,
            (0, 0, 0, 1): self.SELECT_PORT01,
            (0, 0, 1, 0): self.SELECT_PORT01,
            (0, 0, 1, 1): self.SELECT_PORT01,
            (0, 1, 0, 0): self.SELECT_PORT01,
            (0, 1, 0, 1): self.SELECT_PORT01,
            (0, 1, 1, 0): self.SELECT_PORT01,
            (0, 1, 1, 1): self.SELECT_PORT01,
            (1, 0, 0, 0): self.SELECT_PORT02,
            (1, 0, 0, 1): self.SELECT_PORT02,
            (1, 0, 1, 0): self.SELECT_PORT02,
            (1, 0, 1, 1): self.SELECT_PORT02,
            (1, 1, 0, 0): self.SELECT_PORT03,
            (1, 1, 0, 1): self.SELECT_PORT03,
            (1, 1, 1, 0): self.SELECT_PORT04,
        }

        self._reset_state()

    def _reset_state(self):
        """重置所有狀態變數"""
        self.check_ok = False
        self.sent = False
        self.search_eqp_signal_ok = False
        self.carrier_query_sended = False
        self.carrier_query_success = False
        self.port_carriers = [True] * 4
        self.select_agv_port = self.SELECT_NONE
        self.carrier_id = None

    def enter(self):
        self.node.get_logger().info("Robot Take Transfer 目前狀態: AgvPortCheckEmpty")
        self._reset_state()

    def leave(self):
        self.node.get_logger().info("Robot Take Transfer 離開 AgvPortCheckEmpty 狀態")
        self._reset_state()

    def eqp_signal_query_callback(self, response):
        for i in range(4):
            self.port_carriers[i] = EqpSignalQueryClient.eqp_signal_port(
                response, self.PORT_ADDRESS + i + 1)
            self.node.get_logger().info(
                f"AGV Port {i+1:02d} 有無貨: {self.port_carriers[i]}")

        self.search_eqp_signal_ok = True
        port_states = tuple(int(carrier) for carrier in self.port_carriers)
        self.select_agv_port = self.select_agv_port_table.get(
            port_states, self.SELECT_NONE)

    def carrier_callback(self, response):
        self.carrier_query_success = response.success
        self.carrier_id = CarrierQueryClient.carrier_port_id_carrier_id(
            response, self.PORT_ADDRESS + getattr(self, 'select_agv_port_number', 0))

    def _update_context_states(self, context: RobotContext):
        """更新context中的狀態"""
        if not self.search_eqp_signal_ok:
            return
        # 更新AGV_PORT層狀態
        context.agv_port1 = self.port_carriers[0]
        context.agv_port2 = self.port_carriers[1]
        context.agv_port3 = self.port_carriers[2]
        context.agv_port4 = self.port_carriers[3]

    def _handle_port_selection(self, context: RobotContext):
        """處理port選擇邏輯"""
        if self.check_ok or not self.search_eqp_signal_ok:
            return

        port_messages = {
            self.SELECT_PORT01: ("第一格空的", "AGV_PORT1", 1),
            self.SELECT_PORT02: ("第二格空的", "AGV_PORT2", 2),
            self.SELECT_PORT03: ("第三格空的", "AGV_PORT3", 3),
            self.SELECT_PORT04: ("第四格空的", "AGV_PORT4", 4)
        }

        if self.select_agv_port in port_messages:
            desc, port, number = port_messages[self.select_agv_port]
            self.node.get_logger().info(
                f"Robot Take Transfer AgvPortCheckEmpty 狀態: {desc}")
            self.node.get_logger().info(f"執行AGV端口{port}")
            context.agv_port_number = number
            context.get_loader_agv_port_front = number
            self.check_ok = True
        else:
            self.node.get_logger().info("Robot Take Transfer AgvPortCheckEmpty 狀態: AGV端口已滿")
            self.node.get_logger().info("無法執行AGV端口操作，請檢查AGV端口狀態。")
            context.agv_port_number = None
            context.get_loader_agv_port_front = None
            self._reset_state()

    def handle(self, context: RobotContext):
        self._update_context_states(context)

        # 查詢EQP信號
        if not self.search_eqp_signal_ok and not self.sent:
            self.eqp_signal_query_client.search_eqp_signal_eqp_id(
                self.EQP_ID, self.eqp_signal_query_callback)
            self.sent = True

        print("🔶=========================================================================🔶")

        self._handle_port_selection(context)

        # 查詢Carrier
        if self.check_ok and not self.carrier_query_sended:
            self.select_agv_port_number = context.agv_port_number
            self.carrier_query_client.search_carrier_port_id(
                port_id=self.PORT_ADDRESS + self.select_agv_port_number, callback=self.carrier_callback)
            self.carrier_query_sended = True

        # 處理Carrier查詢結果
        if self.check_ok and self.carrier_query_success:
            if self.carrier_id is not None:
                self.node.get_logger().info(
                    f"Carrier 查詢成功，資料: {self.carrier_id}")
                self.node.get_logger().info(
                    f"AGV端口{self.PORT_ADDRESS + self.select_agv_port_number}已經有貨，無法執行AGV端口操作。")
                self._reset_state()
            else:
                context.get_boxin_port = context.agv_port_number
                # AGV端口檢查完成，可以進入下一個狀態
                self.node.get_logger().info("AGV端口檢查完成")
                from loader_agv.robot_states.take_transfer.take_transfer_state import TakeTransferState
                context.set_state(TakeTransferState(self.node))
