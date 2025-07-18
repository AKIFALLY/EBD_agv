from agv_base.states.state import State
from db_proxy.carrier_query_client import CarrierQueryClient
from db_proxy.eqp_signal_query_client import EqpSignalQueryClient
from rclpy.node import Node
from unloader_agv.robot_context import RobotContext


class AgvPortCheckEmptyState(State):

    SELECT_PORT01_PORT02, SELECT_PORT03_PORT04, SELECT_NONE = 1, 3, 0

    def __init__(self, node: Node):
        super().__init__(node)
        self.node = node
        self.eqp_signal_query_client = EqpSignalQueryClient(node)
        self.carrier_query_client = CarrierQueryClient(node)

        # 動態計算 port_address 和 eqp_id (unloader_agv 參數)
        self.port_address = self.node.room_id * 1000 + 110
        self.eqp_id = self.node.room_id * 100 + 11

        # 雙 port 組合選擇邏輯：只選擇兩個連續port都為空的組合
        # 元組格式：(port1, port2, port3, port4)，其中 0=空，1=有貨
        self.select_agv_port_table = {
            # 上層組合 (PORT1_PORT2)：只有當 port1=0 AND port2=0 時才選擇
            # 如果上層都空，優先選擇上層，不管下層狀態如何
            (0, 0, 0, 0): self.SELECT_PORT01_PORT02,  # 全空，選上層
            (0, 0, 0, 1): self.SELECT_PORT01_PORT02,  # 上層空，下層部分有貨，選上層
            (0, 0, 1, 0): self.SELECT_PORT01_PORT02,  # 上層空，下層部分有貨，選上層
            (0, 0, 1, 1): self.SELECT_PORT01_PORT02,  # 上層空，下層滿，選上層

            # 下層組合 (PORT3_PORT4)：只有當上層不可用且 port3=0 AND port4=0 時才選擇
            # 上層至少有一個有貨，但下層都空時選擇下層
            (1, 0, 0, 0): self.SELECT_PORT03_PORT04,  # 上層部分有貨，下層空，選下層
            (0, 1, 0, 0): self.SELECT_PORT03_PORT04,  # 上層部分有貨，下層空，選下層
            (1, 1, 0, 0): self.SELECT_PORT03_PORT04,  # 上層滿，下層空，選下層

            # 其他情況都返回 SELECT_NONE：
            # (1, 0, 0, 1), (1, 0, 1, 0), (1, 0, 1, 1) - 上層部分有貨，下層不完全空
            # (0, 1, 0, 1), (0, 1, 1, 0), (0, 1, 1, 1) - 上層部分有貨，下層不完全空
            # (1, 1, 0, 1), (1, 1, 1, 0), (1, 1, 1, 1) - 上層滿，下層不完全空
        }

        self._reset_state()

    def _reset_state(self):
        """重置所有狀態變數"""
        self.check_ok = False
        self.sent = False
        self.search_eqp_signal_ok = False
        self.carrier_query_sended = False
        self.carrier_query_success = False
        self.port_carriers = [False] * 4  # 初始假設所有port都空
        self.select_agv_port = self.SELECT_NONE
        self.carrier_id = None
        self.carrier_id_min = None  # 存儲 port_id_min 對應的 carrier_id
        self.carrier_id_max = None  # 存儲 port_id_max 對應的 carrier_id

    def enter(self):
        self.node.get_logger().info("Unloader Robot Take Oven 目前狀態: AgvPortCheckEmpty")
        self._reset_state()

    def leave(self):
        self.node.get_logger().info("Unloader Robot Take Oven 離開 AgvPortCheckEmpty 狀態")
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

    def carrier_callback(self, response):
        self.carrier_query_success = response.success
        # 查詢兩個連續 port 的 carrier 資料
        port_id_min = self.port_address + self.agv_port_number
        port_id_max = port_id_min + 1

        self.carrier_id_min = CarrierQueryClient.carrier_port_id_carrier_id(
            response, port_id_min)
        self.carrier_id_max = CarrierQueryClient.carrier_port_id_carrier_id(
            response, port_id_max)

        # 保持向後兼容性
        self.carrier_id = self.carrier_id_min

        self.node.get_logger().info(
            f"Carrier 查詢成功，資料: min={self.carrier_id_min}, max={self.carrier_id_max}")

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
        """驗證 Carrier 查詢結果與 EQP 狀態的一致性（PUT 操作空位檢查）"""
        validation_passed = True
        validation_errors = []

        # 驗證第一個 port (carrier_id_min 應為 None，對應 EQP 狀態應為 False)
        if self.carrier_id_min is None:
            port_number = self.agv_port_number
            eqp_state = getattr(context, f'agv_port{port_number}')
            if eqp_state:
                validation_passed = False
                validation_errors.append(f"AGV_PORT{port_number}: Carrier查詢顯示空位但EQP狀態顯示有貨")
                self.node.get_logger().error(
                    f"❌ 資料不一致 - AGV_PORT{port_number}: Carrier查詢=空位, EQP狀態={eqp_state}")
            else:
                self.node.get_logger().info(
                    f"✅ AGV_PORT{port_number} 驗證通過: Carrier查詢=空位, EQP狀態={eqp_state}")

        # 驗證第二個 port (carrier_id_max 應為 None，對應 EQP 狀態應為 False)
        if self.carrier_id_max is None:
            port_number = self.agv_port_number + 1
            eqp_state = getattr(context, f'agv_port{port_number}')
            if eqp_state:
                validation_passed = False
                validation_errors.append(f"AGV_PORT{port_number}: Carrier查詢顯示空位但EQP狀態顯示有貨")
                self.node.get_logger().error(
                    f"❌ 資料不一致 - AGV_PORT{port_number}: Carrier查詢=空位, EQP狀態={eqp_state}")
            else:
                self.node.get_logger().info(
                    f"✅ AGV_PORT{port_number} 驗證通過: Carrier查詢=空位, EQP狀態={eqp_state}")

        return validation_passed, validation_errors

    def _handle_port_selection(self, context: RobotContext):
        """處理port選擇邏輯"""
        if self.check_ok or not self.search_eqp_signal_ok:
            return

        port_messages = {
            self.SELECT_PORT01_PORT02: ("上層有空位", "AGV_PORT1_PORT2", 1),
            self.SELECT_PORT03_PORT04: ("下層有空位", "AGV_PORT3_PORT4", 3)
        }

        if self.select_agv_port in port_messages:
            desc, port, number = port_messages[self.select_agv_port]
            self.node.get_logger().info(
                f"Unloader Robot Take Oven AgvPortCheckEmpty 狀態: {desc}")
            self.node.get_logger().info(f"執行AGV端口{port}")
            context.get_unloader_agv_port_back = number
            self.check_ok = True
        else:
            self.node.get_logger().error("Unloader Robot Take Oven AgvPortCheckEmpty 狀態: AGV端口都沒有空位")
            self.node.get_logger().error("無法執行AGV端口操作，請檢查AGV端口狀態。")
            context.get_unloader_agv_port_back = None
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

        # 查詢Carrier
        if self.check_ok and not self.carrier_query_sended:
            self.agv_port_number = context.get_unloader_agv_port_back
            self.node.get_logger().info(
                f"🔍 查詢 AGV 端口 {self.port_address + self.agv_port_number} 的 Carrier")
            port_id_target = self.port_address + self.agv_port_number
            self.carrier_query_client.search_carrier_port_id(
                port_id_min=port_id_target, port_id_max=port_id_target+1, callback=self.carrier_callback)
            self.carrier_query_sended = True

        # 處理Carrier查詢結果
        if self.check_ok and self.carrier_query_success:
            port_id_min = self.port_address + self.agv_port_number
            port_id_max = port_id_min + 1

            # 檢查兩個 port 是否都為空（適合 PUT 操作的空位檢查）
            if self.carrier_id_min is None and self.carrier_id_max is None:
                # 兩個 port 都為空，進行 EQP 信號狀態驗證
                self.node.get_logger().info(
                    f"雙 Port 組合 {port_id_min}-{port_id_max} 都為空，開始 EQP 狀態驗證。")

                # 執行 EQP 狀態驗證
                validation_passed, validation_errors = self._validate_eqp_states(context)

                if validation_passed:
                    # EQP 狀態驗證通過，可以執行 PUT 操作
                    self.node.get_logger().info(
                        f"✅ AGV端口 {port_id_min}-{port_id_max} EQP 狀態驗證通過，可以執行AGV端口操作")
                    # AGV端口檢查完成，可以進入下一個狀態
                    self.node.get_logger().info("AGV端口檢查完成")
                    from unloader_agv.robot_states.take_oven.take_oven_state import TakeOvenState
                    context.set_state(TakeOvenState(self.node))
                else:
                    # EQP 狀態驗證失敗，重置狀態
                    self.node.get_logger().error(f"❌ AGV端口 {port_id_min}-{port_id_max} EQP 狀態驗證失敗:")
                    for error in validation_errors:
                        self.node.get_logger().error(f"   - {error}")
                    self.node.get_logger().error("Carrier 查詢結果與 EQP 硬體信號狀態不一致，重置狀態")
                    self._reset_state()
            else:
                # 至少有一個 port 有貨，無法執行 PUT 操作
                self.node.get_logger().error(f"Carrier 查詢成功，至少有一個 Port 有貨物")
                self.node.get_logger().error(
                    f"雙 Port 組合 {port_id_min}-{port_id_max} 至少有一個port有貨物，無法執行AGV端口操作。")
                self._reset_state()
