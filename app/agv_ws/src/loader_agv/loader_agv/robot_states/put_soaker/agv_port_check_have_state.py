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
        self.carrier_queried = False
        self.port1_has_cargo = False
        self.port3_has_cargo = False

    def enter(self):
        self.node.get_logger().info(
            "[Station-based 1格] Loader Robot Put Soaker 目前狀態: AgvPortCheckHave")
        self._reset_state()

    def leave(self):
        self.node.get_logger().info(
            "[Station-based 1格] Loader Robot Put Soaker 離開 AgvPortCheckHave 狀態")
        self._reset_state()

    def eqp_signal_query_callback(self, response):
        """處理 EqpSignal 查詢回應 - 只檢查 port1 和 port3（L尺寸配置）

        說明：Put Soaker 單格處理邏輯
        - L尺寸 AGV 配置：只使用 port1 和 port3（第1層和第3層）
        - 單格處理：一次只取1格，優先 port1 > port3
        """
        if not response or not response.datas:
            self.node.get_logger().error("❌ [Station-based 1格] EqpSignal 查詢失敗或沒有資料")
            self.search_eqp_signal_ok = True
            return

        # 只檢查 port1 和 port3（L尺寸配置）
        self.port1_has_cargo = EqpSignalQueryClient.eqp_signal_port(
            response, self.port_address + 1)
        self.port3_has_cargo = EqpSignalQueryClient.eqp_signal_port(
            response, self.port_address + 3)

        self.node.get_logger().info(
            f"[Station-based 1格] AGV Port 1 是否有貨: {self.port1_has_cargo} (L尺寸第1層)")
        self.node.get_logger().info(
            f"[Station-based 1格] AGV Port 3 是否有貨: {self.port3_has_cargo} (L尺寸第3層)")

        self.search_eqp_signal_ok = True

    def carrier_query_callback(self, response):
        """處理 carrier 查詢回應 - 單格處理只需取1個 carrier"""
        if not (response and response.success and response.datas):
            self.node.get_logger().error("❌ [Station-based 1格] Carrier 查詢失敗或沒有資料")
            self.carrier_queried = True
            return

        # 只取第一個 carrier（優先順序已由 port 選擇決定）
        if response.datas:
            carrier = response.datas[0]
            self.node.get_logger().info(
                f"✅ [Station-based 1格] 找到 Carrier: port_id={carrier.port_id}, carrier_id={carrier.id}")

        self.carrier_queried = True

    def _update_context_states(self, context: RobotContext):
        """更新context中的狀態 - 只更新 port1 和 port3"""
        if not self.search_eqp_signal_ok:
            return
        # 更新 AGV_PORT 層狀態（只更新 port1 和 port3）
        context.agv_port1 = self.port1_has_cargo
        context.agv_port3 = self.port3_has_cargo

    def _handle_port_selection(self, context: RobotContext):
        """處理 port 選擇邏輯 - 單格處理，優先 port1 > port3

        說明：Put Soaker 單格取料邏輯
        - 一次只取1格（與 Put Cleaner 批量2格不同）
        - 選擇順序：Port 1（第1層）> Port 3（第3層）
        - 取1格後直接放入指定的泡藥機站點
        """
        if self.check_ok or not (self.search_eqp_signal_ok and self.carrier_queried):
            return

        # 優先選擇 port1
        if self.port1_has_cargo:
            self.node.get_logger().info(
                f"✅ [Station-based 1格] AGV Port 1 有貨物 (Work ID {context.work_id})")
            self.node.get_logger().info(
                f"[Station-based 1格] 選擇 AGV Port 1 取貨 → 泡藥機 Port {context.get_soaker_port}")
            context.get_loader_agv_port_side = 1
            self.check_ok = True
        elif self.port3_has_cargo:
            self.node.get_logger().info(
                f"✅ [Station-based 1格] AGV Port 3 有貨物 (Work ID {context.work_id})")
            self.node.get_logger().info(
                f"[Station-based 1格] Port 1 沒貨，選擇 AGV Port 3 取貨 → 泡藥機 Port {context.get_soaker_port}")
            context.get_loader_agv_port_side = 3
            self.check_ok = True
        else:
            self.node.get_logger().warn(
                f"❌ [Station-based 1格] AGV Port 1 和 3 都沒有貨物 (Work ID {context.work_id})")
            self.node.get_logger().warn(
                "[Station-based 1格] 無法執行取貨操作，請等待 AGV 載貨。")
            context.get_loader_agv_port_side = None
            self._reset_state()

    def handle(self, context: RobotContext):
        # 1. 更新 context 狀態
        self._update_context_states(context)

        # 2. 查詢 EQP 信號（只執行一次）
        if not self.search_eqp_signal_ok and not self.sent:
            self.node.get_logger().info(
                f"[Station-based 1格] 查詢 AGV 端口狀態 (eqp_id={self.eqp_id})，"
                f"檢查 port1 和 port3 (Work ID {context.work_id})")
            self.eqp_signal_query_client.search_eqp_signal_eqp_id(
                self.eqp_id, self.eqp_signal_query_callback)
            self.sent = True

        # 3. 查詢 Carrier（只執行一次）
        if self.search_eqp_signal_ok and not self.carrier_queried and not self.sent:
            # 根據 port 選擇順序查詢對應的 carrier
            if self.port1_has_cargo:
                port_id = self.port_address + 1
            elif self.port3_has_cargo:
                port_id = self.port_address + 3
            else:
                # 沒有貨物，跳過 carrier 查詢
                self.carrier_queried = True
                self.sent = True
                return

            self.node.get_logger().info(
                f"[Station-based 1格] 查詢 Carrier: port_id={port_id}")
            self.carrier_query_client.search_carrier_port_id(
                port_id_min=port_id,
                port_id_max=port_id,
                callback=self.carrier_query_callback
            )
            self.sent = True

        # 4. 處理 port 選擇邏輯
        self._handle_port_selection(context)

        # 5. 檢查完成，進入下一個狀態
        if self.check_ok:
            self.node.get_logger().info(
                f"✅ [Station-based 1格] AGV 端口檢查完成 (選擇 port{context.get_loader_agv_port_side})")
            self.node.get_logger().info(
                f"[Station-based 1格] 進入 TakeAgvState: AGV Port {context.get_loader_agv_port_side} → "
                f"泡藥機 Port {context.get_soaker_port}")
            from loader_agv.robot_states.put_soaker.take_agv_state import TakeAgvState
            context.set_state(TakeAgvState(self.node))
