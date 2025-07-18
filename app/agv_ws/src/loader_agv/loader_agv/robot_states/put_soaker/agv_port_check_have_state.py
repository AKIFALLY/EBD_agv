from agv_base.states.state import State
from db_proxy.carrier_query_client import CarrierQueryClient
from db_proxy.eqp_signal_query_client import EqpSignalQueryClient
from rclpy.node import Node
from loader_agv.robot_context import RobotContext


class AgvPortCheckHaveState(State):

    def __init__(self, node: Node):
        super().__init__(node)
        self.node = node
        self.eqp_signal_query_client = EqpSignalQueryClient(node)
        self.carrier_query_client = CarrierQueryClient(node)

        # 動態計算 port_address 和 eqp_id (loader_agv AGV端口參數)
        self.port_address = self.node.room_id * 1000 + 100
        self.eqp_id = self.node.room_id * 100 + 10

        self._reset_state()

    def _reset_state(self):
        """重置所有狀態變數"""
        self.check_ok = False
        self.sent = False
        self.search_eqp_signal_ok = False
        self.carrier_query_sended = False
        self.carrier_query_success = False
        self.port_carriers = [False] * 4  # 初始假設所有port都空
        self.earliest_carrier = None
        self.selected_port_id = None

    def enter(self):
        self.node.get_logger().info("Loader Robot Put Soaker 目前狀態: AgvPortCheckHave")
        self._reset_state()

    def leave(self):
        self.node.get_logger().info("Loader Robot Put Soaker 離開 AgvPortCheckHave 狀態")
        self._reset_state()

    def eqp_signal_query_callback(self, response):
        for i in range(4):
            self.port_carriers[i] = EqpSignalQueryClient.eqp_signal_port(
                response, self.port_address + i + 1)
            self.node.get_logger().info(
                f"AGV Port {i+1:02d} 有無貨: {self.port_carriers[i]}")

        self.search_eqp_signal_ok = True

    def carrier_query_callback(self, response):
        """處理 carrier 查詢回應"""
        self.carrier_query_success = response.success

        if not (response and response.success and response.datas):
            self.node.get_logger().error("❌ Carrier 查詢失敗或沒有資料")
            return

        # 找出 updated_at 時間最早的 carrier
        self.earliest_carrier = min(response.datas, key=lambda c: c.updated_at)
        self.selected_port_id = self.earliest_carrier.port_id

        self.node.get_logger().info(
            f"✅ 找到最早的 Carrier: port_id={self.earliest_carrier.port_id}, "
            f"carrier_id={self.earliest_carrier.id}, updated_at={self.earliest_carrier.updated_at}")

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

        # 直接檢查是否有任何端口有貨物
        has_cargo = any(self.port_carriers)

        if has_cargo:
            # 計算有貨物的端口數量
            cargo_ports = [i+1 for i, has_cargo in enumerate(self.port_carriers) if has_cargo]
            self.node.get_logger().info(
                f"Loader Robot Put Soaker AgvPortCheckHave 狀態: 檢測到 AGV 端口 {cargo_ports} 有貨物")
            self.node.get_logger().info("準備查詢 Carrier 以選擇最適合的端口")
            # 不在此處設定 context.get_loader_agv_port_side，等 carrier 查詢完成後再設定
            self.check_ok = True
        else:
            self.node.get_logger().info("Loader Robot Put Soaker AgvPortCheckHave 狀態: AGV端口都沒有貨物")
            self.node.get_logger().info("無法執行AGV端口操作，請檢查AGV端口狀態。")
            context.get_loader_agv_port_side = None
            self._reset_state()

    def handle(self, context: RobotContext):
        self._update_context_states(context)

        # 查詢EQP信號
        if not self.search_eqp_signal_ok and not self.sent:
            self.eqp_signal_query_client.search_eqp_signal_eqp_id(
                self.eqp_id, self.eqp_signal_query_callback)
            self.sent = True

        print("🔶=========================================================================🔶")

        self._handle_port_selection(context)

        # 查詢所有有貨物的端口的 Carrier
        if self.check_ok and not self.carrier_query_sended:
            # 查詢所有有貨物的端口
            port_id_min = self.port_address + 1
            port_id_max = self.port_address + 4
            self.node.get_logger().info(
                f"🔍 查詢 AGV 端口 {port_id_min}-{port_id_max} 的 Carrier")
            self.carrier_query_client.search_carrier_port_id(
                port_id_min=port_id_min, port_id_max=port_id_max, callback=self.carrier_query_callback)
            self.carrier_query_sended = True

        # 處理Carrier查詢結果
        if self.check_ok and self.carrier_query_success and self.earliest_carrier:
            # 驗證查詢到的 carrier 所在端口是否確實有貨物
            selected_port_number = self.selected_port_id - self.port_address

            # 檢查對應的 context.agv_port[port_number-1] 是否為 True (有貨物)
            if 1 <= selected_port_number <= 4:
                port_has_cargo = self.port_carriers[selected_port_number - 1]

                if port_has_cargo:
                    # 驗證成功：設定 context.get_loader_agv_port_side 並轉換到 TakeAgvState
                    context.get_loader_agv_port_side = selected_port_number
                    context.carrier_id = self.earliest_carrier.id

                    self.node.get_logger().info(
                        f"✅ 驗證成功：AGV Port {selected_port_number} 有貨物且有 carrier (ID: {self.earliest_carrier.id})")
                    self.node.get_logger().info("AGV端口檢查完成，準備執行取AGV操作")

                    # 轉換到下一個狀態 - TakeAgvState
                    from loader_agv.robot_states.put_soaker.take_agv_state import TakeAgvState
                    context.set_state(TakeAgvState(self.node))
                else:
                    # 驗證失敗：重置狀態並重新執行查詢
                    self.node.get_logger().warn(
                        f"❌ 驗證失敗：Carrier 所在端口 {selected_port_number} 實際沒有貨物，重新查詢")
                    self._reset_and_restart_queries()
            else:
                # 端口號碼無效
                self.node.get_logger().error(f"❌ 無效的端口號碼: {selected_port_number}")
                self._reset_and_restart_queries()

        elif self.check_ok and self.carrier_query_success and not self.earliest_carrier:
            # 查詢成功但沒有找到 carrier
            self.node.get_logger().error("Carrier 查詢成功，但沒有找到符合條件的 carrier")
            self.node.get_logger().error("無法執行AGV端口操作。")
            self._reset_state()

    def _reset_and_restart_queries(self):
        """重置狀態並重新執行完整的查詢流程"""
        self.node.get_logger().info("🔄 重置狀態並重新執行查詢流程")
        # 重置查詢標誌，讓系統重新執行完整的查詢流程
        self.search_eqp_signal_ok = False
        self.carrier_query_sended = False
        self.carrier_query_success = False
        self.check_ok = False
        self.sent = False
        self.earliest_carrier = None
        self.selected_port_id = None
