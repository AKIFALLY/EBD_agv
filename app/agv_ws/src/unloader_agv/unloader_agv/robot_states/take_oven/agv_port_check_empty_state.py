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
        self.port_carriers = [False] * 4  # AGV 四個 port 的狀態
        # 存儲所有4個carrier_id
        self.carrier_id_port1 = None
        self.carrier_id_port2 = None
        self.carrier_id_port3 = None
        self.carrier_id_port4 = None

    def enter(self):
        self.node.get_logger().info("Unloader Robot Take Oven 目前狀態: AgvPortCheckEmpty")
        self._reset_state()

    def leave(self):
        self.node.get_logger().info("Unloader Robot Take Oven 離開 AgvPortCheckEmpty 狀態")
        self._reset_state()

    def eqp_signal_query_callback(self, response):
        """EQP信號查詢回調"""
        if response.success:
            self.node.get_logger().info("✅ AGV Port EQP信號查詢成功")
            # 解析AGV port狀態 (port 1-4)
            for i in range(4):
                self.port_carriers[i] = EqpSignalQueryClient.eqp_signal_port(
                    response, self.port_address + i + 1)
                self.node.get_logger().debug(
                    f"AGV Port {i+1} 有無貨: {self.port_carriers[i]}")

            self.search_eqp_signal_ok = True
            self.sent = False
        else:
            self.node.get_logger().error(f"❌ AGV Port EQP信號查詢失敗: {response.message}")

    def carrier_callback(self, response):
        """Carrier查詢回調 - 查詢所有4個AGV ports的carrier_id"""
        self.carrier_query_success = response.success
        if response.success:
            self.node.get_logger().info("✅ AGV 所有4個ports Carrier查詢成功")

            # 查詢所有4個port IDs的carrier
            port_id_1 = self.port_address + 1
            port_id_2 = self.port_address + 2
            port_id_3 = self.port_address + 3
            port_id_4 = self.port_address + 4

            self.carrier_id_port1 = CarrierQueryClient.carrier_port_id_carrier_id(response, port_id_1)
            self.carrier_id_port2 = CarrierQueryClient.carrier_port_id_carrier_id(response, port_id_2)
            self.carrier_id_port3 = CarrierQueryClient.carrier_port_id_carrier_id(response, port_id_3)
            self.carrier_id_port4 = CarrierQueryClient.carrier_port_id_carrier_id(response, port_id_4)

            # 記錄查詢結果
            self.node.get_logger().info(
                f"查詢 AGV Carrier 結果："
                f"\n  Port 1 (ID: {port_id_1}): carrier_id={self.carrier_id_port1}"
                f"\n  Port 2 (ID: {port_id_2}): carrier_id={self.carrier_id_port2}"
                f"\n  Port 3 (ID: {port_id_3}): carrier_id={self.carrier_id_port3}"
                f"\n  Port 4 (ID: {port_id_4}): carrier_id={self.carrier_id_port4}")
        else:
            self.node.get_logger().error(f"❌ AGV Carrier查詢失敗: {response.message}")

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
        """驗證所有4個 Port的 Carrier 查詢結果與 EQP 狀態的一致性（TAKE 操作空位檢查）"""
        validation_passed = True
        validation_errors = []

        # 存儲所有4個carrier_id以便統一驗證
        carrier_ids = [
            self.carrier_id_port1,
            self.carrier_id_port2,
            self.carrier_id_port3,
            self.carrier_id_port4
        ]

        # 存儲所有4個EQP狀態
        eqp_states = [
            context.agv_port1,
            context.agv_port2,
            context.agv_port3,
            context.agv_port4
        ]

        # 逐一驗證所有4個ports
        for port_number, carrier_id, eqp_state in zip(range(1, 5), carrier_ids, eqp_states):
            port_id = self.port_address + port_number

            # TAKE 操作：carrier_id 應為 None (無貨)，EQP 狀態應為 False (無貨)
            if carrier_id is None:
                if eqp_state:
                    # 不一致：Carrier 顯示無貨，但 EQP 顯示有貨
                    validation_passed = False
                    validation_errors.append(
                        f"PORT {port_number} (ID: {port_id}): Carrier查詢無貨但EQP狀態顯示有貨")
                    self.node.get_logger().error(
                        f"❌ 資料不一致 - AGV PORT {port_number} (ID: {port_id}): "
                        f"Carrier ID=None, EQP狀態={eqp_state}(有貨)")
                else:
                    # 一致：都顯示無貨
                    self.node.get_logger().info(
                        f"✅ AGV PORT {port_number} (ID: {port_id}) 驗證通過: "
                        f"Carrier ID=None, EQP狀態={eqp_state}(無貨)")
            else:
                # carrier_id 不為 None，表示有貨物
                if not eqp_state:
                    # 不一致：Carrier 顯示有貨，但 EQP 顯示無貨
                    validation_passed = False
                    validation_errors.append(
                        f"PORT {port_number} (ID: {port_id}): Carrier查詢有貨但EQP狀態顯示無貨")
                    self.node.get_logger().error(
                        f"❌ 資料不一致 - AGV PORT {port_number} (ID: {port_id}): "
                        f"Carrier ID={carrier_id}, EQP狀態={eqp_state}(無貨)")
                else:
                    # 一致：都顯示有貨
                    self.node.get_logger().info(
                        f"✅ AGV PORT {port_number} (ID: {port_id}) 驗證通過: "
                        f"Carrier ID={carrier_id}, EQP狀態={eqp_state}(有貨)")

        return validation_passed, validation_errors

    def _handle_port_selection(self, context: RobotContext):
        """檢查所有4個AGV ports是否都為空"""
        if self.search_eqp_signal_ok and not self.check_ok:
            # 檢查所有4個 ports 是否都為空
            all_ports_empty = all(not carrier for carrier in self.port_carriers)

            if all_ports_empty:
                # 所有4個 port 都為空
                self.check_ok = True
                self.node.get_logger().info("✅ AGV 所有4個 PORT 都為空，確認完成，準備查詢 Carrier 驗證")
            else:
                # 有port有貨物
                occupied_ports = [i + 1 for i, has_carrier in enumerate(self.port_carriers) if has_carrier]
                self.node.get_logger().warn(
                    f"❌ AGV 有port有貨物: {occupied_ports}，無法執行 TAKE 操作")
                self._reset_state()
                from unloader_agv.robot_states.complete_state import CompleteState
                context.set_state(CompleteState(self.node))

    def handle(self, context: RobotContext):
        self._update_context_states(context)

        # 查詢EQP信號
        if not self.search_eqp_signal_ok and not self.sent:
            self.eqp_signal_query_client.search_eqp_signal_eqp_id(
                self.eqp_id, self.eqp_signal_query_callback)
            self.sent = True

        # 更新 Hokuyo Input
        self._handle_hokuyo_input()

        self._handle_port_selection(context)

        # 查詢所有4個 AGV PORT 的 Carrier
        if self.check_ok and not self.carrier_query_sended:
            port_id_min = self.port_address + 1  # 2111
            port_id_max = self.port_address + 4  # 2114
            self.node.get_logger().info(
                f"查詢AGV 所有4個 PORT Carrier：PORT 1-4 (ID: {port_id_min}-{port_id_max})")
            self.carrier_query_client.search_carrier_port_id(
                port_id_min=port_id_min, port_id_max=port_id_max, callback=self.carrier_callback)
            self.carrier_query_sended = True

        # 處理 Carrier 查詢結果
        if self.carrier_query_sended and self.carrier_query_success:
            # 檢查所有4個 port 是否都沒有 carrier
            carrier_ids = [self.carrier_id_port1, self.carrier_id_port2, self.carrier_id_port3, self.carrier_id_port4]
            all_empty = all(cid is None for cid in carrier_ids)

            if all_empty:
                # 所有4個 port 都為空，進行 EQP 信號狀態驗證
                self.node.get_logger().info("所有4個 AGV PORT 都沒有 Carrier，開始 EQP 狀態驗證")

                # 執行 EQP 狀態驗證
                validation_passed, validation_errors = self._validate_eqp_states(context)

                if validation_passed:
                    # EQP 狀態驗證通過，可以執行 TAKE 操作
                    self.node.get_logger().info("✅ 所有 AGV PORT EQP 狀態驗證通過")
                    self.node.get_logger().info("AGV 所有4個ports檢查完成，進入下一個狀態")
                    from unloader_agv.robot_states.take_oven.take_oven_state import TakeOvenState
                    context.set_state(TakeOvenState(self.node))
                else:
                    # EQP 狀態驗證失敗，重置狀態
                    self.node.get_logger().error("❌ AGV PORT EQP 狀態驗證失敗:")
                    for error in validation_errors:
                        self.node.get_logger().error(f"   - {error}")
                    self.node.get_logger().error("Carrier 查詢結果與 EQP 硬體信號狀態不一致，重置狀態")
                    self._reset_state()
            else:
                # 有 port 有貨
                occupied_ports = [i + 1 for i, cid in enumerate(carrier_ids) if cid is not None]
                self.node.get_logger().error(
                    f"❌ Carrier 查詢成功，但 AGV PORT {occupied_ports} 有貨物")
                self.node.get_logger().error(
                    "所有4個 PORT 都需要為空才能執行 TAKE_OVEN 操作")
                self._reset_state()
