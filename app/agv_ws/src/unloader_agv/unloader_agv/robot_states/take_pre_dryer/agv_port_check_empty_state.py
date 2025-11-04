from agv_base.states.state import State
from db_proxy.carrier_query_client import CarrierQueryClient
from db_proxy.eqp_signal_query_client import EqpSignalQueryClient
from rclpy.node import Node
from unloader_agv.robot_context import RobotContext


class AgvPortCheckEmptyState(State):

    def __init__(self, node: Node):
        super().__init__(node)
        self.node = node
        self.eqp_signal_query_client = EqpSignalQueryClient(node)
        self.carrier_query_client = CarrierQueryClient(node)

        # 動態計算 port_address 和 eqp_id (unloader_agv 參數)
        self.port_address = self.node.room_id * 1000 + 110
        self.eqp_id = self.node.room_id * 100 + 11

        self._reset_state()

    def _reset_state(self):
        """重置所有狀態變數"""
        self.check_ok = False
        self.sent = False
        self.search_eqp_signal_ok = False
        self.carrier_query_sended = False
        self.carrier_query_success = False
        self.port_carriers = [False] * 4  # 初始假設所有port都空
        # 存儲 4 個 port 的 carrier_id
        self.carrier_id_port1 = None
        self.carrier_id_port2 = None
        self.carrier_id_port3 = None
        self.carrier_id_port4 = None

    def enter(self):
        self.node.get_logger().info("Unloader Robot Take Pre Dryer 目前狀態: AgvPortCheckEmpty")
        self._reset_state()

    def leave(self):
        self.node.get_logger().info("Unloader Robot Take Pre Dryer 離開 AgvPortCheckEmpty 狀態")
        self._reset_state()

    def eqp_signal_query_callback(self, response):
        """查詢所有 4 個 AGV ports 的 EQP 信號狀態"""
        for i in range(4):
            self.port_carriers[i] = EqpSignalQueryClient.eqp_signal_port(
                response, self.port_address + i + 1)
            self.node.get_logger().info(
                f"AGV Port {i+1:02d} 有無貨: {self.port_carriers[i]}")

        self.search_eqp_signal_ok = True

    def carrier_callback(self, response):
        """查詢所有 4 個 AGV ports 的 Carrier 資料"""
        self.carrier_query_success = response.success

        # 查詢所有 4 個 port 的 carrier_id
        port_id_1 = self.port_address + 1  # 2111
        port_id_2 = self.port_address + 2  # 2112
        port_id_3 = self.port_address + 3  # 2113
        port_id_4 = self.port_address + 4  # 2114

        self.carrier_id_port1 = CarrierQueryClient.carrier_port_id_carrier_id(
            response, port_id_1)
        self.carrier_id_port2 = CarrierQueryClient.carrier_port_id_carrier_id(
            response, port_id_2)
        self.carrier_id_port3 = CarrierQueryClient.carrier_port_id_carrier_id(
            response, port_id_3)
        self.carrier_id_port4 = CarrierQueryClient.carrier_port_id_carrier_id(
            response, port_id_4)

        self.node.get_logger().info(
            f"Carrier 查詢成功，資料: Port1={self.carrier_id_port1}, Port2={self.carrier_id_port2}, "
            f"Port3={self.carrier_id_port3}, Port4={self.carrier_id_port4}")

    def _update_context_states(self, context: RobotContext):
        """更新context中的狀態"""
        if not self.search_eqp_signal_ok:
            return
        # 更新AGV_PORT層狀態
        context.agv_port1 = self.port_carriers[0]
        context.agv_port2 = self.port_carriers[1]
        context.agv_port3 = self.port_carriers[2]
        context.agv_port4 = self.port_carriers[3]

    def _validate_eqp_states(self, context: RobotContext):
        """驗證所有 4 個 ports 是否都為空，並檢查 Carrier 查詢結果與 EQP 狀態的一致性"""
        validation_passed = True
        validation_errors = []

        # 準備 4 個 port 的驗證數據
        ports_data = [
            (1, self.carrier_id_port1, context.agv_port1),
            (2, self.carrier_id_port2, context.agv_port2),
            (3, self.carrier_id_port3, context.agv_port3),
            (4, self.carrier_id_port4, context.agv_port4),
        ]

        # 檢查所有 4 個 ports 是否都為空
        all_ports_empty = all(carrier_id is None for _, carrier_id, _ in ports_data)

        if not all_ports_empty:
            # 至少有一個 port 有貨，記錄哪些 port 有貨
            ports_with_cargo = [
                port_num for port_num, carrier_id, _ in ports_data if carrier_id is not None
            ]
            validation_passed = False
            validation_errors.append(
                f"AGV 有 {len(ports_with_cargo)} 個 port 有貨物: Port {ports_with_cargo}"
            )
            self.node.get_logger().error(
                f"❌ AGV 端口檢查失敗：Port {ports_with_cargo} 有貨物，需要全部 4 個 ports 都為空"
            )
            return validation_passed, validation_errors

        # 所有 carrier_id 都為 None，驗證 EQP 狀態是否一致
        for port_num, carrier_id, eqp_state in ports_data:
            if carrier_id is None:
                # Carrier 查詢顯示空位，EQP 狀態應該也為 False
                if eqp_state:
                    validation_passed = False
                    validation_errors.append(
                        f"AGV_PORT{port_num}: Carrier查詢顯示空位但EQP狀態顯示有貨"
                    )
                    self.node.get_logger().error(
                        f"❌ 資料不一致 - AGV_PORT{port_num}: Carrier查詢=空位, EQP狀態={eqp_state}"
                    )
                else:
                    self.node.get_logger().info(
                        f"✅ AGV_PORT{port_num} 驗證通過: Carrier查詢=空位, EQP狀態={eqp_state}"
                    )

        return validation_passed, validation_errors

    def handle(self, context: RobotContext):
        self._update_context_states(context)

        # 步驟 1: 查詢 EQP 信號
        if not self.search_eqp_signal_ok and not self.sent:
            self.node.get_logger().info("🔍 開始查詢 AGV 所有 4 個 ports 的 EQP 信號狀態")
            self.eqp_signal_query_client.search_eqp_signal_eqp_id(
                self.eqp_id, self.eqp_signal_query_callback)
            self.sent = True
            return

        print("🔶=========================================================================🔶")

        # 步驟 2: 查詢所有 4 個 ports 的 Carrier
        if self.search_eqp_signal_ok and not self.carrier_query_sended:
            port_id_min = self.port_address + 1  # 2111
            port_id_max = self.port_address + 4  # 2114
            self.node.get_logger().info(
                f"🔍 查詢 AGV 所有 4 個 ports ({port_id_min}-{port_id_max}) 的 Carrier")
            self.carrier_query_client.search_carrier_port_id(
                port_id_min=port_id_min,
                port_id_max=port_id_max,
                callback=self.carrier_callback)
            self.carrier_query_sended = True
            return

        # 步驟 3: 處理 Carrier 查詢結果
        if self.carrier_query_success:
            # 執行驗證：檢查所有 4 個 ports 是否都為空
            self.node.get_logger().info("🔍 開始驗證所有 4 個 ports 是否都為空")
            validation_passed, validation_errors = self._validate_eqp_states(context)

            if validation_passed:
                # 所有 4 個 ports 都為空，驗證通過
                self.node.get_logger().info(
                    f"✅ AGV 所有 4 個 ports 都為空，EQP 狀態驗證通過，可以執行取料操作")
                self.node.get_logger().info("✅ AGV 端口檢查完成，進入取料狀態")
                from unloader_agv.robot_states.take_pre_dryer.take_pre_dryer_state import TakePreDryerState
                context.set_state(TakePreDryerState(self.node))
            else:
                # 驗證失敗（部分 port 有貨或數據不一致），重置狀態
                self.node.get_logger().error(f"❌ AGV 端口驗證失敗:")
                for error in validation_errors:
                    self.node.get_logger().error(f"   - {error}")
                self.node.get_logger().error("🔄 重置狀態，等待 AGV 端口清空")
                self._reset_state()
