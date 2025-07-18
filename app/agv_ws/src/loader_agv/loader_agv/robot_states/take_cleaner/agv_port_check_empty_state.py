from agv_base.states.state import State
from db_proxy.carrier_query_client import CarrierQueryClient
from db_proxy.eqp_signal_query_client import EqpSignalQueryClient
from rclpy.node import Node
from loader_agv.robot_context import RobotContext


class AgvPortCheckEmptyState(State):

    SELECT_PORT01, SELECT_PORT02, SELECT_PORT03, SELECT_PORT04, SELECT_NONE = 1, 2, 3, 4, 0

    def __init__(self, node: Node):
        super().__init__(node)
        self.node = node
        self.eqp_signal_query_client = EqpSignalQueryClient(node)
        self.carrier_query_client = CarrierQueryClient(node)

        # 動態計算 port_address 和 eqp_id (AGV端口參數)
        self.port_address = self.node.room_id * 1000 + 100
        self.eqp_id = self.node.room_id * 100 + 10

        # AGV 端口選擇表格：檢查哪些端口是空的（適合放入從清洗機取出的貨物）
        # 0 = 空的端口，1 = 有貨的端口
        # 優先選擇編號較小的空端口
        self.select_agv_port_table = {
            (0, 0, 0, 0): self.SELECT_PORT01,  # 全空，選擇 port1
            (0, 0, 0, 1): self.SELECT_PORT01,  # port1-3 空，選擇 port1
            (0, 0, 1, 0): self.SELECT_PORT01,  # port1-2,4 空，選擇 port1
            (0, 0, 1, 1): self.SELECT_PORT01,  # port1-2 空，選擇 port1
            (0, 1, 0, 0): self.SELECT_PORT01,  # port1,3-4 空，選擇 port1
            (0, 1, 0, 1): self.SELECT_PORT01,  # port1,3 空，選擇 port1
            (0, 1, 1, 0): self.SELECT_PORT01,  # port1,4 空，選擇 port1
            (0, 1, 1, 1): self.SELECT_PORT01,  # 只有 port1 空，選擇 port1
            (1, 0, 0, 0): self.SELECT_PORT02,  # port2-4 空，選擇 port2
            (1, 0, 0, 1): self.SELECT_PORT02,  # port2-3 空，選擇 port2
            (1, 0, 1, 0): self.SELECT_PORT02,  # port2,4 空，選擇 port2
            (1, 0, 1, 1): self.SELECT_PORT02,  # 只有 port2 空，選擇 port2
            (1, 1, 0, 0): self.SELECT_PORT03,  # port3-4 空，選擇 port3
            (1, 1, 0, 1): self.SELECT_PORT03,  # 只有 port3 空，選擇 port3
            (1, 1, 1, 0): self.SELECT_PORT04,  # 只有 port4 空，選擇 port4
            # (1, 1, 1, 1) 全滿時不包含，返回 SELECT_NONE
        }

        self._reset_state()

    def _reset_state(self):
        """重置所有狀態變數"""
        self.check_ok = False
        self.sent = False
        self.search_eqp_signal_ok = False
        self.carrier_query_sended = False
        self.carrier_query_success = False
        self.port_carriers = [True] * 4  # 初始假設所有port都有貨
        self.select_agv_port = self.SELECT_NONE
        self.earliest_carrier = None
        self.selected_port_id = None

    def enter(self):
        self.node.get_logger().info("Loader Robot Take Cleaner 目前狀態: AgvPortCheckEmpty")
        self._reset_state()

    def leave(self):
        self.node.get_logger().info("Loader Robot Take Cleaner 離開 AgvPortCheckEmpty 狀態")
        self._reset_state()

    def eqp_signal_query_callback(self, response):
        for i in range(4):
            self.port_carriers[i] = EqpSignalQueryClient.eqp_signal_port(
                response, self.port_address + i + 1)
            self.node.get_logger().info(
                f"AGV Port {i+1:02d} 有無貨: {self.port_carriers[i]}")

        self.search_eqp_signal_ok = True
        port_states = tuple(int(carrier) for carrier in self.port_carriers)
        self.select_agv_port = self.select_agv_port_table.get(
            port_states, self.SELECT_NONE)

    def carrier_query_callback(self, response):
        """處理 carrier 查詢回應"""
        self.carrier_query_success = response.success

        if not (response and response.success and response.datas):
            self.node.get_logger().error("❌ Carrier 查詢失敗或沒有資料")
            return

        # 找出 updated_at 時間最早的 carrier（如果有多個空端口）
        self.earliest_carrier = min(
            response.datas, key=lambda c: c.updated_at) if response.datas else None
        if self.earliest_carrier:
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

        port_messages = {
            self.SELECT_PORT01: ("第一格空的", "AGV_PORT1", 1),
            self.SELECT_PORT02: ("第二格空的", "AGV_PORT2", 2),
            self.SELECT_PORT03: ("第三格空的", "AGV_PORT3", 3),
            self.SELECT_PORT04: ("第四格空的", "AGV_PORT4", 4)
        }

        if self.select_agv_port in port_messages:
            desc, port, number = port_messages[self.select_agv_port]
            self.node.get_logger().info(
                f"Loader Robot Take Cleaner AgvPortCheckEmpty 狀態: {desc}")
            self.node.get_logger().info(f"檢測到AGV端口{port}為空，準備查詢 Carrier")
            # 暫時設定，等 carrier 查詢完成後會根據最早時間重新設定
            context.get_loader_agv_port_side = number
            self.check_ok = True
        else:
            self.node.get_logger().info("Loader Robot Take Cleaner AgvPortCheckEmpty 狀態: AGV端口已滿")
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

        # 查詢所有空端口的 Carrier
        if self.check_ok and not self.carrier_query_sended:
            # 查詢所有端口以找到最早的空端口
            port_id_min = self.port_address + 1
            port_id_max = self.port_address + 4
            self.node.get_logger().info(
                f"🔍 查詢 AGV 端口 {port_id_min}-{port_id_max} 的 Carrier")
            self.carrier_query_client.search_carrier_port_id(
                port_id_min=port_id_min, port_id_max=port_id_max, callback=self.carrier_query_callback)
            self.carrier_query_sended = True

        # 處理Carrier查詢結果
        if self.check_ok and self.carrier_query_success:
            # 驗證機制：檢查選定的端口是否確實為空
            selected_port_number = context.get_loader_agv_port_side

            if 1 <= selected_port_number <= 4:
                port_eqp_empty = not self.port_carriers[selected_port_number - 1]

                # 雙重驗證：EQP 信號查詢結果與 carrier 查詢結果的一致性檢查
                if self.earliest_carrier is None and port_eqp_empty:
                    # 驗證成功：EQP 顯示空，carrier 查詢也顯示空
                    self.node.get_logger().info(
                        f"✅ 雙重驗證成功：AGV Port {selected_port_number} EQP 狀態和 Carrier 查詢都顯示為空")
                    self.node.get_logger().info("AGV端口檢查完成，準備執行取清洗機操作")

                    # 轉換到下一個狀態 - TakeCleanerState
                    self.node.get_logger().info("✅ AGV 端口檢查完成: 進入 TakeCleanerState")
                    from loader_agv.robot_states.take_cleaner.take_cleaner_state import TakeCleanerState
                    context.set_state(TakeCleanerState(self.node))
                elif self.earliest_carrier is not None:
                    # EQP 顯示空但 carrier 查詢顯示有貨，數據不一致
                    self.node.get_logger().warn(
                        f"❌ 數據不一致：AGV Port {selected_port_number} EQP 顯示空但有 carrier (ID: {self.earliest_carrier.id})")
                    self._reset_and_restart_queries()
                elif not port_eqp_empty:
                    # EQP 顯示有貨，與選擇邏輯不一致
                    self.node.get_logger().warn(
                        f"❌ 驗證失敗：AGV Port {selected_port_number} EQP 狀態顯示有貨物，重新查詢")
                    self._reset_and_restart_queries()
            else:
                # 端口號碼無效
                self.node.get_logger().error(f"❌ 無效的端口號碼: {selected_port_number}")
                self._reset_and_restart_queries()

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
