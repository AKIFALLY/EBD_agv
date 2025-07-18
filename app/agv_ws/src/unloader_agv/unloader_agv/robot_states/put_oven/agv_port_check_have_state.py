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
        self.carrier_id_min = None  # 存儲選定組合第一個 port 的 carrier_id
        self.carrier_id_max = None  # 存儲選定組合第二個 port 的 carrier_id
        self.selected_port_pair = None  # 存儲選定的 PORT 組合 (port1, port2)
        self.selected_pair_name = None  # 存儲選定組合的名稱
        self.agv_port_number = None  # 存儲選定組合的起始 port 號碼

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
        """Carrier查詢回調 - 處理選定 PORT 組合查詢結果"""
        self.carrier_query_success = response.success
        if response.success:
            self.node.get_logger().info(f"✅ AGV {self.selected_pair_name} Carrier查詢成功")

            # 計算選定 port 組合的兩個 PORT ID
            if self.selected_port_pair:
                port_id_min = self.port_address + self.selected_port_pair[0]
                port_id_max = self.port_address + self.selected_port_pair[1]

                # 使用 CarrierQueryClient 的靜態方法獲取對應 port 的 carrier_id
                self.carrier_id_min = CarrierQueryClient.carrier_port_id_carrier_id(
                    response, port_id_min)
                self.carrier_id_max = CarrierQueryClient.carrier_port_id_carrier_id(
                    response, port_id_max)

                # 記錄查詢結果
                if self.carrier_id_min is not None:
                    self.node.get_logger().info(
                        f"AGV PORT {self.selected_port_pair[0]} 有 Carrier ID: {self.carrier_id_min}")
                else:
                    self.node.get_logger().debug(
                        f"AGV PORT {self.selected_port_pair[0]} 沒有 Carrier")

                if self.carrier_id_max is not None:
                    self.node.get_logger().info(
                        f"AGV PORT {self.selected_port_pair[1]} 有 Carrier ID: {self.carrier_id_max}")
                else:
                    self.node.get_logger().debug(
                        f"AGV PORT {self.selected_port_pair[1]} 沒有 Carrier")
        else:
            self.node.get_logger().error(
                f"❌ AGV {self.selected_pair_name} Carrier查詢失敗: {response.message}")

    def _update_context_states(self, context: RobotContext):
        """更新context中的狀態"""
        if not self.search_eqp_signal_ok:
            return
        # 更新AGV_PORT狀態
        context.agv_port1 = self.port_carriers[0]
        context.agv_port2 = self.port_carriers[1]
        context.agv_port3 = self.port_carriers[2]
        context.agv_port4 = self.port_carriers[3]

    def _calculate_port_ids(self, selected_port_pair):
        """計算選定 port 組合的兩個 PORT ID"""
        if not selected_port_pair:
            return None, None
        return (self.port_address + selected_port_pair[0],
                self.port_address + selected_port_pair[1])

    def _get_selected_carrier_info(self):
        """獲取選定的 carrier 資訊"""
        if self.carrier_id_min is not None:
            return self.selected_port_pair[0], self.carrier_id_min
        else:
            return self.selected_port_pair[1], self.carrier_id_max

    def _validate_eqp_states(self, context: RobotContext):
        """驗證 Carrier 查詢結果與 EQP 狀態的一致性（TAKE 操作有貨檢查）"""
        validation_passed = True
        validation_errors = []

        # 驗證第一個 port (carrier_id_min)
        if self.carrier_id_min is not None:
            port_number = self.selected_port_pair[0]
            eqp_state = getattr(context, f'agv_port{port_number}')
            if not eqp_state:
                validation_passed = False
                validation_errors.append(f"AGV_PORT{port_number}: Carrier查詢有貨但EQP狀態顯示無貨")
                self.node.get_logger().error(
                    f"❌ 資料不一致 - AGV_PORT{port_number}: Carrier ID={self.carrier_id_min}, EQP狀態={eqp_state}")
            else:
                self.node.get_logger().info(
                    f"✅ AGV_PORT{port_number} 驗證通過: Carrier ID={self.carrier_id_min}, EQP狀態={eqp_state}")

        # 驗證第二個 port (carrier_id_max)
        if self.carrier_id_max is not None:
            port_number = self.selected_port_pair[1]
            eqp_state = getattr(context, f'agv_port{port_number}')
            if not eqp_state:
                validation_passed = False
                validation_errors.append(f"AGV_PORT{port_number}: Carrier查詢有貨但EQP狀態顯示無貨")
                self.node.get_logger().error(
                    f"❌ 資料不一致 - AGV_PORT{port_number}: Carrier ID={self.carrier_id_max}, EQP狀態={eqp_state}")
            else:
                self.node.get_logger().info(
                    f"✅ AGV_PORT{port_number} 驗證通過: Carrier ID={self.carrier_id_max}, EQP狀態={eqp_state}")

        return validation_passed, validation_errors

    def _handle_port_selection(self, context: RobotContext):
        """處理port選擇邏輯 - 選擇有貨物的 PORT 組合"""
        if self.search_eqp_signal_ok and not self.check_ok:
            # AGV port 組合定義：port1+2, port3+4
            port_pairs = [(1, 2), (3, 4)]
            pair_names = ["port1+2", "port3+4"]

            # 第一優先：找出兩個 port 都有貨物的組合
            selected_index = None
            for i, (port1, port2) in enumerate(port_pairs):
                port1_has_cargo = self.port_carriers[port1 - 1]  # port 1-4 對應 index 0-3
                port2_has_cargo = self.port_carriers[port2 - 1]

                self.node.get_logger().debug(
                    f"檢查AGV {pair_names[i]}: PORT{port1}={port1_has_cargo}, PORT{port2}={port2_has_cargo}")

                # 第一優先：兩個 port 都有貨物
                if port1_has_cargo and port2_has_cargo:
                    selected_index = i
                    self.node.get_logger().info(
                        f"✅ 第一優先選擇AGV {pair_names[i]}，兩個 port 都有貨物")
                    break

            # 第二優先：找出至少有一個 port 有貨物的組合
            if selected_index is None:
                for i, (port1, port2) in enumerate(port_pairs):
                    port1_has_cargo = self.port_carriers[port1 - 1]
                    port2_has_cargo = self.port_carriers[port2 - 1]

                    # 第二優先：至少有一個 port 有貨物
                    if port1_has_cargo or port2_has_cargo:
                        selected_index = i
                        self.node.get_logger().info(
                            f"✅ 第二優先選擇AGV {pair_names[i]}，至少有一個 port 有貨物")
                        break

            if selected_index is not None:
                # 保存選定的 port 組合和相關資訊
                self.selected_port_pair = port_pairs[selected_index]
                self.selected_pair_name = pair_names[selected_index]
                self.agv_port_number = self.selected_port_pair[0]  # 起始 port 號碼
                self.check_ok = True
                self.node.get_logger().info(
                    f"✅ AGV 選擇 {self.selected_pair_name}，準備查詢 Carrier 驗證")
            else:
                self.node.get_logger().warn(
                    f"❌ AGV 的所有 port 組合都沒有貨物，無法執行 TAKE 操作")
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

        # 查詢選定 PORT 組合的 Carrier
        if self.check_ok and not self.carrier_query_sended and self.selected_port_pair:
            port_id_min, port_id_max = self._calculate_port_ids(self.selected_port_pair)
            self.node.get_logger().info(
                f"查詢AGV {self.selected_pair_name} Carrier：PORT {self.selected_port_pair[0]}-{self.selected_port_pair[1]} (ID: {port_id_min}-{port_id_max})")
            self.carrier_query_client.search_carrier_port_id(
                port_id_min=port_id_min, port_id_max=port_id_max, callback=self.carrier_callback)
            self.carrier_query_sended = True

        # 處理 Carrier 查詢結果
        if self.carrier_query_sended and self.carrier_query_success and self.selected_port_pair:
            port_id_min, port_id_max = self._calculate_port_ids(self.selected_port_pair)

            # 檢查兩個 port 是否至少有一個有 carrier（適合 TAKE 操作的有貨檢查）
            if self.carrier_id_min is not None or self.carrier_id_max is not None:
                # 至少有一個 port 有貨，進行 EQP 信號狀態驗證
                self.node.get_logger().info(
                    f"{self.selected_pair_name} 組合 {port_id_min}-{port_id_max} 至少有一個port有貨物，開始 EQP 狀態驗證。")

                # 執行 EQP 狀態驗證
                validation_passed, validation_errors = self._validate_eqp_states(context)

                if validation_passed:
                    # EQP 狀態驗證通過，可以執行 TAKE 操作
                    self.node.get_logger().info(
                        f"✅ {self.selected_pair_name} EQP 狀態驗證通過，可以執行AGV操作")

                    # 獲取選定的 carrier 資訊
                    selected_physical_port, selected_carrier_id = self._get_selected_carrier_info()
                    self.node.get_logger().info(
                        f"選擇AGV PORT {selected_physical_port}，Carrier ID: {selected_carrier_id}")

                    # 設定 context 變數
                    context.agv_port_number = self.agv_port_number
                    context.carrier_id[0] = self.carrier_id_min
                    context.carrier_id[1] = self.carrier_id_max

                    self.node.get_logger().info(f"AGV {self.selected_pair_name} 檢查完成，進入烤箱檢查狀態")
                    from unloader_agv.robot_states.put_oven.oven_check_empty_state import OvenCheckEmptyState
                    context.set_state(OvenCheckEmptyState(self.node))
                else:
                    # EQP 狀態驗證失敗，重置狀態
                    self.node.get_logger().error(f"❌ {self.selected_pair_name} EQP 狀態驗證失敗:")
                    for error in validation_errors:
                        self.node.get_logger().error(f"   - {error}")
                    self.node.get_logger().error("Carrier 查詢結果與 EQP 硬體信號狀態不一致，重置狀態")
                    self._reset_state()
            else:
                # 兩個 port 都沒有貨，無法執行 TAKE 操作
                self.node.get_logger().error(
                    f"Carrier 查詢成功，{self.selected_pair_name} 兩個 Port 都沒有貨物")
                self.node.get_logger().error(
                    f"{self.selected_pair_name} 組合 {port_id_min}-{port_id_max} 都沒有貨物，無法執行AGV操作。")
                self._reset_state()
