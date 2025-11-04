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
        self.port2_has_cargo = False
        self.port4_has_cargo = False
        self.carrier_ids = [None, None]  # 記錄 port2 和 port4 的 carrier_id

    def enter(self):
        self.node.get_logger().info(
            "[Station-based 2格] Loader Robot Put PreDryer 目前狀態: AgvPortCheckHave")
        self._reset_state()

    def leave(self):
        self.node.get_logger().info(
            "[Station-based 2格] Loader Robot Put PreDryer 離開 AgvPortCheckHave 狀態")
        self._reset_state()

    def eqp_signal_query_callback(self, response):
        """處理 EqpSignal 查詢回應 - 檢查 AGV Port 2 和 4（L尺寸配置）

        說明：Put Pre-dryer AGV port 分層策略 ⭐
        - L尺寸 AGV 配置：使用 port 1-4（第1-4層）
        - **關鍵設計差異**：
          - Put Cleaner 使用 Port 1 和 3（第1層和第3層）
          - Put Pre-dryer 使用 Port 2 和 4（第2層和第4層）⭐
        - 批量處理：2次取放動作，順序 port 2 → port 4
        - 目的：避免與其他流程的端口衝突，支援並行執行
        - 原因：Pre-dryer 在製程後段，可能與前段流程（Cleaner/Soaker）並行
        - 設計考量：
          * Put Cleaner → Take Cleaner 使用 Port 1, 3
          * Put Soaker → Take Soaker 使用 Port 1（放），Port 2, 4（取）
          * Put Pre-dryer 使用 Port 2, 4，與 Cleaner 錯開
        """
        if not response or not response.datas:
            self.node.get_logger().error(
                "❌ [Station-based 2格] EqpSignal 查詢失敗或沒有資料")
            self.search_eqp_signal_ok = True
            return

        # 只檢查 port2 和 port4（Put Pre-dryer 專用配置）
        self.port2_has_cargo = EqpSignalQueryClient.eqp_signal_port(
            response, self.port_address + 2)
        self.port4_has_cargo = EqpSignalQueryClient.eqp_signal_port(
            response, self.port_address + 4)

        self.node.get_logger().info(
            f"[Station-based 2格] AGV Port 2 是否有貨: {self.port2_has_cargo} "
            f"(L尺寸第2層，Put Pre-dryer 偶數層策略)")
        self.node.get_logger().info(
            f"[Station-based 2格] AGV Port 4 是否有貨: {self.port4_has_cargo} "
            f"(L尺寸第4層，Put Pre-dryer 偶數層策略)")

        self.search_eqp_signal_ok = True

    def carrier_query_callback(self, response):
        """處理 carrier 查詢回應 - 記錄 port2 和 port4 的 carrier_id

        說明：批量取料需要檢查兩個 port 的 carrier_id
        - Port 2（第2層）的 carrier_id → 第1次取料
        - Port 4（第4層）的 carrier_id → 第2次取料
        """
        if not (response and response.success and response.datas):
            self.node.get_logger().error(
                "❌ [Station-based 2格] Carrier 查詢失敗或沒有資料")
            self.carrier_queried = True
            return

        # 記錄 port2 和 port4 的 carrier_id（批量取料配置）
        for carrier in response.datas:
            if carrier.port_id == self.port_address + 2:
                self.carrier_ids[0] = carrier.id
                self.node.get_logger().info(
                    f"✅ [Station-based 2格] Port 2 carrier_id = {carrier.id} (第1次取料)")
            elif carrier.port_id == self.port_address + 4:
                self.carrier_ids[1] = carrier.id
                self.node.get_logger().info(
                    f"✅ [Station-based 2格] Port 4 carrier_id = {carrier.id} (第2次取料)")

        self.carrier_queried = True
        self.node.get_logger().info(
            f"[Station-based 2格] Carrier IDs: Port2={self.carrier_ids[0]}, Port4={self.carrier_ids[1]}")

    def _update_context_states(self, context: RobotContext):
        """更新context中的狀態 - 只更新 port2 和 port4"""
        if not self.search_eqp_signal_ok:
            return
        # 更新 AGV_PORT 層狀態（只更新 port2 和 port4）
        context.agv_port2 = self.port2_has_cargo
        context.agv_port4 = self.port4_has_cargo

    def _handle_port_selection(self, context: RobotContext):
        """處理 port 選擇邏輯 - 檢查是否兩個 port 都有貨（批量處理必要條件）

        說明：Put Pre-dryer 批量取料邏輯
        - 批量處理：一次任務處理2格，需要兩個 port 都有貨
        - AGV Port 順序：Port 2（第1次）→ Port 4（第2次）
        - 原因：使用偶數層（2, 4）與 Cleaner 的奇數層（1, 3）錯開
        """
        if self.check_ok or not (self.search_eqp_signal_ok and self.carrier_queried):
            return

        # 批量處理邏輯：檢查兩個 carrier_id 是否都有值
        if self.carrier_ids[0] and self.carrier_ids[1]:
            self.node.get_logger().info(
                f"✅ [Station-based 2格] AGV Port 2 和 4 都有貨物 (Work ID {context.work_id})")
            self.node.get_logger().info(
                f"[Station-based 2格] 批量取料配置: carrier_ids={self.carrier_ids}")
            self.node.get_logger().info(
                f"[Station-based 2格] AGV Port 使用偶數層策略（Port 2, 4）")

            # 設置批量取料配置
            context.pre_dryer_carrier_ids = self.carrier_ids  # [carrier_id2, carrier_id4]
            context.pre_dryer_agv_ports = [2, 4]              # AGV port2 和 port4（偶數層）

            # 設置第一次的初始值
            context.get_loader_agv_port_front = 2
            context.carrier_id = self.carrier_ids[0]
            self.node.get_logger().info(
                f"[Station-based 2格] 設定第1次取料: AGV Port 2, Carrier ID {self.carrier_ids[0]}")

            self.check_ok = True
        else:
            self.node.get_logger().warn(
                f"❌ [Station-based 2格] AGV Port 2 和 4 未全部有貨物 (Work ID {context.work_id})")
            self.node.get_logger().warn(
                f"[Station-based 2格] Carrier IDs: Port2={self.carrier_ids[0]}, Port4={self.carrier_ids[1]}")
            self.node.get_logger().warn(
                "[Station-based 2格] 批量取料需要兩個端口都有貨物，請等待。")
            # 重置並等待下次檢查
            self._reset_state()

    def handle(self, context: RobotContext):
        # 1. 更新 context 狀態
        self._update_context_states(context)

        # 2. 查詢 EQP 信號（只執行一次）
        if not self.search_eqp_signal_ok and not self.sent:
            self.node.get_logger().info(
                f"[Station-based 2格] 查詢 AGV 端口狀態 (eqp_id={self.eqp_id}), "
                f"檢查 port 2 和 4 (Work ID {context.work_id})")
            self.node.get_logger().info(
                "[Station-based 2格] Put Pre-dryer 使用偶數層（Port 2, 4）策略")
            self.eqp_signal_query_client.search_eqp_signal_eqp_id(
                self.eqp_id, self.eqp_signal_query_callback)
            self.sent = True

        # 3. 查詢 Carrier（只執行一次）
        if self.search_eqp_signal_ok and not self.carrier_queried and not self.sent:
            # 查詢 port2 和 port4 的 carrier（批量取料配置）
            port_id_min = self.port_address + 2
            port_id_max = self.port_address + 4
            self.node.get_logger().info(
                f"[Station-based 2格] 查詢 Carrier: port_id 範圍 {port_id_min}-{port_id_max} "
                f"(Work ID {context.work_id})")
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
                f"✅ [Station-based 2格] AGV 端口檢查完成 (Work ID {context.work_id})")
            self.node.get_logger().info(
                f"批量取料配置: ports={context.pre_dryer_agv_ports}, "
                f"carrier_ids={context.pre_dryer_carrier_ids}")
            self.node.get_logger().info(
                f"✅ [Station-based 2格] 進入 TakeAgvState: 第1次從 AGV Port 2 取料")
            from loader_agv.robot_states.put_pre_dryer.take_agv_state import TakeAgvState
            context.set_state(TakeAgvState(self.node))
