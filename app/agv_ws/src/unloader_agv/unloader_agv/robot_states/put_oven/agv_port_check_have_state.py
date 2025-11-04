from db_proxy.carrier_query_client import CarrierQueryClient
from db_proxy.eqp_signal_query_client import EqpSignalQueryClient
from rclpy.node import Node
from unloader_agv.robot_context import RobotContext
from unloader_agv.robot_states.base_robot_state import BaseRobotState


class AgvPortCheckHaveState(BaseRobotState):

    def __init__(self, node: Node):
        super().__init__(node)
        self.eqp_signal_query_client = EqpSignalQueryClient(node)
        self.carrier_query_client = CarrierQueryClient(node)

        # 動態計算 AGV port 相關參數
        self.port_address = self.node.room_id * 1000 + 30  # AGV port address
        self.eqp_id = self.node.room_id * 100 + 3  # AGV eqp_id

        self._reset_state()

    def _reset_state(self):
        """重置所有狀態變數"""
        self.check_ok = False
        self.search_eqp_signal_ok = False
        self.carrier_query_sended = False
        self.carrier_query_success = False
        self.sent = False
        self.port_carriers = [False] * 4  # AGV 四個 port 的狀態
        self.carrier_id_1 = None  # AGV Port 1 的 carrier_id
        self.carrier_id_2 = None  # AGV Port 2 的 carrier_id
        self.carrier_id_3 = None  # AGV Port 3 的 carrier_id
        self.carrier_id_4 = None  # AGV Port 4 的 carrier_id

    def enter(self):
        self.node.get_logger().info("Unloader Robot Put Oven 目前狀態: AgvPortCheckHave")
        self._reset_state()

    def leave(self):
        self.node.get_logger().info("Unloader Robot Put Oven 離開 AgvPortCheckHave 狀態")
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
        """Carrier查詢回調 - 處理所有4個 AGV PORT 的查詢結果"""
        self.carrier_query_success = response.success
        if response.success:
            self.node.get_logger().info("✅ AGV 所有4個 PORT Carrier查詢成功")

            # 查詢所有4個 AGV PORT 的 carrier_id (2111-2114)
            for i in range(4):
                port_id = self.port_address + i + 1  # 2111, 2112, 2113, 2114
                carrier_id = CarrierQueryClient.carrier_port_id_carrier_id(response, port_id)

                # 存儲到對應的變數
                if i == 0:
                    self.carrier_id_1 = carrier_id
                elif i == 1:
                    self.carrier_id_2 = carrier_id
                elif i == 2:
                    self.carrier_id_3 = carrier_id
                elif i == 3:
                    self.carrier_id_4 = carrier_id

                # 記錄查詢結果
                if carrier_id is not None:
                    self.node.get_logger().info(
                        f"AGV PORT {i+1} (ID={port_id}) 有 Carrier ID: {carrier_id}")
                else:
                    self.node.get_logger().warn(
                        f"AGV PORT {i+1} (ID={port_id}) 沒有 Carrier")
        else:
            self.node.get_logger().error(
                f"❌ AGV 所有 PORT Carrier查詢失敗: {response.message}")

    def _update_context_states(self, context: RobotContext):
        """更新context中的狀態"""
        if not self.search_eqp_signal_ok:
            return
        # 更新AGV_PORT狀態
        context.agv_port1 = self.port_carriers[0]
        context.agv_port2 = self.port_carriers[1]
        context.agv_port3 = self.port_carriers[2]
        context.agv_port4 = self.port_carriers[3]

    def _validate_eqp_states(self, context: RobotContext):
        """驗證 Carrier 查詢結果與 EQP 狀態的一致性（所有4個 PORT 檢查）"""
        validation_passed = True
        validation_errors = []

        # 驗證所有4個 port
        carrier_ids = [self.carrier_id_1, self.carrier_id_2, self.carrier_id_3, self.carrier_id_4]
        for i, carrier_id in enumerate(carrier_ids):
            port_number = i + 1
            eqp_state = getattr(context, f'agv_port{port_number}')

            if carrier_id is not None:
                # Carrier 有貨，檢查 EQP 狀態是否一致
                if not eqp_state:
                    validation_passed = False
                    validation_errors.append(
                        f"AGV_PORT{port_number}: Carrier查詢有貨但EQP狀態顯示無貨")
                    self.node.get_logger().error(
                        f"❌ 資料不一致 - AGV_PORT{port_number}: "
                        f"Carrier ID={carrier_id}, EQP狀態={eqp_state}")
                else:
                    self.node.get_logger().info(
                        f"✅ AGV_PORT{port_number} 驗證通過: "
                        f"Carrier ID={carrier_id}, EQP狀態={eqp_state}")
            else:
                # Carrier 無貨
                if eqp_state:
                    validation_passed = False
                    validation_errors.append(
                        f"AGV_PORT{port_number}: Carrier查詢無貨但EQP狀態顯示有貨")
                    self.node.get_logger().error(
                        f"❌ 資料不一致 - AGV_PORT{port_number}: "
                        f"Carrier查詢無貨, EQP狀態={eqp_state}")

        return validation_passed, validation_errors

    def _check_all_ports_have_cargo(self, context: RobotContext):
        """檢查所有4個 AGV PORT 是否都有貨物"""
        if self.search_eqp_signal_ok and not self.check_ok:
            # 檢查所有4個 port 是否都有貨物
            all_ports_have_cargo = all(self.port_carriers)

            if all_ports_have_cargo:
                self.node.get_logger().info(
                    "✅ AGV 所有4個 PORT 都有貨物，準備查詢 Carrier 驗證")
                self.check_ok = True
            else:
                # 記錄哪些 port 沒有貨物
                empty_ports = [i + 1 for i, has_cargo in enumerate(self.port_carriers) if not has_cargo]
                self.node.get_logger().error(
                    f"❌ AGV PORT {empty_ports} 沒有貨物，無法執行 PUT_OVEN 操作（需要所有4個 PORT 都有貨）")
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

        # 檢查所有4個 port 是否都有貨物
        self._check_all_ports_have_cargo(context)

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
            # 檢查所有4個 port 是否都有 carrier
            carrier_ids = [self.carrier_id_1, self.carrier_id_2, self.carrier_id_3, self.carrier_id_4]
            all_have_carrier = all(cid is not None for cid in carrier_ids)

            if all_have_carrier:
                # 所有4個 port 都有貨，進行 EQP 信號狀態驗證
                self.node.get_logger().info("所有4個 AGV PORT 都有 Carrier，開始 EQP 狀態驗證")

                # 執行 EQP 狀態驗證
                validation_passed, validation_errors = self._validate_eqp_states(context)

                if validation_passed:
                    # EQP 狀態驗證通過，存儲所有4個 carrier_id
                    self.node.get_logger().info("✅ 所有 AGV PORT EQP 狀態驗證通過")

                    # 存儲所有4個 carrier_id 到 context
                    context.carrier_id[0] = self.carrier_id_1
                    context.carrier_id[1] = self.carrier_id_2
                    context.carrier_id[2] = self.carrier_id_3
                    context.carrier_id[3] = self.carrier_id_4

                    self.node.get_logger().info(
                        f"✅ AGV 檢查完成，已保存所有4個 carrier_id：{carrier_ids}")

                    # 進入烤箱檢查狀態
                    from unloader_agv.robot_states.put_oven.oven_check_empty_state import OvenCheckEmptyState
                    context.set_state(OvenCheckEmptyState(self.node))
                else:
                    # EQP 狀態驗證失敗，重置狀態
                    self.node.get_logger().error("❌ AGV PORT EQP 狀態驗證失敗:")
                    for error in validation_errors:
                        self.node.get_logger().error(f"   - {error}")
                    self.node.get_logger().error("Carrier 查詢結果與 EQP 硬體信號狀態不一致，重置狀態")
                    self._reset_state()
            else:
                # 有 port 沒有貨
                missing_ports = [i + 1 for i, cid in enumerate(carrier_ids) if cid is None]
                self.node.get_logger().error(
                    f"❌ Carrier 查詢成功，但 AGV PORT {missing_ports} 沒有貨物")
                self.node.get_logger().error(
                    "所有4個 PORT 都需要有貨物才能執行 PUT_OVEN 操作")
                self._reset_state()
