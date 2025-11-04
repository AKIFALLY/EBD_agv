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
        self.carrier_ids = [None, None]  # 記錄 port1 和 port3 的 carrier_id

    def enter(self):
        self.node.get_logger().info(
            "[Station-based 批量] Loader Robot Put Cleaner 目前狀態: AgvPortCheckHave")
        self._reset_state()

    def leave(self):
        self.node.get_logger().info(
            "[Station-based 批量] Loader Robot Put Cleaner 離開 AgvPortCheckHave 狀態")
        self._reset_state()

    def eqp_signal_query_callback(self, response):
        """處理 EqpSignal 查詢回應 - 只檢查 port1 和 port3 (L尺寸配置)

        說明：L尺寸產品配置只使用第1層和第3層（port1 和 port3）
        S尺寸產品配置使用全部4層，但 Put Cleaner 流程只處理L尺寸配置
        """
        if not response or not response.datas:
            self.node.get_logger().error("❌ EqpSignal 查詢失敗或沒有資料")
            self.search_eqp_signal_ok = True
            return

        # 只檢查 port1 和 port3 (L尺寸產品配置)
        self.port1_has_cargo = EqpSignalQueryClient.eqp_signal_port(
            response, self.port_address + 1)
        self.port3_has_cargo = EqpSignalQueryClient.eqp_signal_port(
            response, self.port_address + 3)

        self.node.get_logger().info(
            f"[AGV Port 檢查] Port 1 是否有貨: {self.port1_has_cargo} (L尺寸第1層)")
        self.node.get_logger().info(
            f"[AGV Port 檢查] Port 3 是否有貨: {self.port3_has_cargo} (L尺寸第3層)")

        self.search_eqp_signal_ok = True

    def carrier_query_callback(self, response):
        """處理 carrier 查詢回應 - 記錄 port1 和 port3 的 carrier_id

        說明：PUT Cleaner 批量操作需要檢查 AGV port1 和 port3 都有 carrier
        - 批量處理：AGV Port 1 → Cleaner Port 3, AGV Port 3 → Cleaner Port 4
        """
        if not (response and response.success and response.datas):
            self.node.get_logger().error("❌ Carrier 查詢失敗或沒有資料")
            self.carrier_queried = True
            return

        # 記錄 port1 和 port3 的 carrier_id
        for carrier in response.datas:
            if carrier.port_id == self.port_address + 1:
                self.carrier_ids[0] = carrier.id
                self.node.get_logger().info(
                    f"✅ [Station-based] AGV Port 1 carrier_id = {carrier.id}")
            elif carrier.port_id == self.port_address + 3:
                self.carrier_ids[1] = carrier.id
                self.node.get_logger().info(
                    f"✅ [Station-based] AGV Port 3 carrier_id = {carrier.id}")

        self.carrier_queried = True
        self.node.get_logger().info(
            f"[Station-based 批量] Carrier IDs: Port1={self.carrier_ids[0]}, "
            f"Port3={self.carrier_ids[1]}")

    def _update_context_states(self, context: RobotContext):
        """更新context中的狀態 - 只更新 port1 和 port3"""
        if not self.search_eqp_signal_ok:
            return
        # 更新 AGV_PORT 層狀態（只更新 port1 和 port3）
        context.agv_port1 = self.port1_has_cargo
        context.agv_port3 = self.port3_has_cargo

    def _handle_port_selection(self, context: RobotContext):
        """處理 port 選擇邏輯 - 檢查是否兩個 port 都有貨（批量處理必要條件）

        說明：新的 Station-based 設計批量放料流程
        - 從 AGV 取2格 → 放到 Cleaner 的 Station 03 (port 3-4)
        - 必須確認 AGV 的 port1 和 port3 都有貨物才能執行
        """
        if self.check_ok or not (self.search_eqp_signal_ok and self.carrier_queried):
            return

        # 批量處理邏輯：檢查兩個 carrier_id 是否都有值
        if self.carrier_ids[0] and self.carrier_ids[1]:
            self.node.get_logger().info(
                "✅ [Station-based 批量] AGV Port 1 和 Port 3 都有貨物")
            self.node.get_logger().info(
                f"可以執行批量放料操作（Work ID {context.work_id}）: "
                f"AGV Port 1 + Port 3 → Cleaner Station 03（2格批量）")

            # 批量放料配置（2格批量操作）
            # Station-based 設計：
            # - AGV 包含 2 個有效 port (Port 1, 3 for L-size)
            # - 批量處理：AGV Port 1 → Cleaner Port 3, AGV Port 3 → Cleaner Port 4
            context.cleaner_take_count = 0  # 計數器 (0=第1次, 1=第2次)
            context.cleaner_carrier_ids = self.carrier_ids  # [carrier_id1, carrier_id3]
            context.cleaner_agv_ports = [1, 3]  # AGV port1 和 port3
            context.cleaner_device_ports = [3, 4]  # Cleaner port3 和 port4

            # 設置第一次的初始值
            context.get_loader_agv_port_side = 1  # 第1次從 AGV port1 取料
            context.carrier_id = self.carrier_ids[0]

            self.node.get_logger().info(
                f"[Station-based 批量] 配置完成: carrier_ids={self.carrier_ids}")
            self.node.get_logger().info(
                f"第 1 次: AGV Port {context.cleaner_agv_ports[0]} → Cleaner Port {context.cleaner_device_ports[0]}")
            self.node.get_logger().info(
                f"第 2 次: AGV Port {context.cleaner_agv_ports[1]} → Cleaner Port {context.cleaner_device_ports[1]}")

            self.check_ok = True
        else:
            self.node.get_logger().warn(
                "❌ [Station-based 批量] AGV 端口未全部有貨物，無法執行批量放料")
            self.node.get_logger().warn(
                f"AGV Port 1 carrier_id: {self.carrier_ids[0]}, "
                f"Port 3 carrier_id: {self.carrier_ids[1]}")
            self.node.get_logger().warn(
                "批量放料需要 Port 1 和 Port 3 都有貨物，請等待 AGV 載入貨物。")
            # 重置並等待下次檢查
            self._reset_state()

    def handle(self, context: RobotContext):
        # 1. 更新 context 狀態
        self._update_context_states(context)

        # 2. 查詢 EQP 信號（只執行一次）
        if not self.search_eqp_signal_ok and not self.sent:
            self.node.get_logger().info(
                f"[Station-based 批量] 查詢 AGV 端口狀態 (eqp_id={self.eqp_id})，"
                f"檢查 port1 和 port3 (Work ID {context.work_id})")
            self.eqp_signal_query_client.search_eqp_signal_eqp_id(
                self.eqp_id, self.eqp_signal_query_callback)
            self.sent = True

        # 3. 查詢 Carrier（只執行一次）
        if self.search_eqp_signal_ok and not self.carrier_queried and not self.sent:
            # 查詢 port1 和 port3 的 carrier
            port_id_min = self.port_address + 1
            port_id_max = self.port_address + 3
            self.node.get_logger().info(
                f"[Station-based 批量] 查詢 Carrier: port_id 範圍 {port_id_min}-{port_id_max} "
                f"(AGV Port 1 和 3)")
            self.carrier_query_client.search_carrier_port_id(
                port_id_min=port_id_min,
                port_id_max=port_id_max,
                callback=self.carrier_query_callback
            )
            self.sent = True

        # 4. 處理 port 選擇邏輯
        self._handle_port_selection(context)

        # 5. 檢查完成，進入下一個狀態
        if self.check_ok:
            self.node.get_logger().info(
                f"✅ [Station-based 批量] AGV 端口檢查完成 (Work ID {context.work_id})")
            self.node.get_logger().info(
                f"批量放料配置: AGV Ports={context.cleaner_agv_ports}, "
                f"Cleaner Ports={context.cleaner_device_ports}, "
                f"carrier_ids={context.cleaner_carrier_ids}")
            self.node.get_logger().info("進入 TakeAgvState 狀態")
            from loader_agv.robot_states.put_cleaner.take_agv_state import TakeAgvState
            context.set_state(TakeAgvState(self.node))
