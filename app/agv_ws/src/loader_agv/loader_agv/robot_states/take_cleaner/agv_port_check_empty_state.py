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

        # 動態計算 port_address 和 eqp_id (AGV端口參數)
        self.port_address = self.node.room_id * 1000 + 100
        self.eqp_id = self.node.room_id * 100 + 10

        self._reset_state()

    def _reset_state(self):
        """重置所有狀態變數"""
        self.check_ok = False
        self.sent = False
        self.search_eqp_signal_ok = False
        self.agv_port1_empty = False
        self.agv_port3_empty = False

    def enter(self):
        self.node.get_logger().info(
            "[Station-based 批量] Loader Robot Take Cleaner 目前狀態: AgvPortCheckEmpty")
        self._reset_state()

    def leave(self):
        self.node.get_logger().info(
            "[Station-based 批量] Loader Robot Take Cleaner 離開 AgvPortCheckEmpty 狀態")
        self._reset_state()

    def eqp_signal_query_callback(self, response):
        """處理 EqpSignal 查詢回應 - 只檢查 port1 和 port3 (L尺寸配置)

        說明：L尺寸產品配置只使用第1層和第3層（port1 和 port3）
        S尺寸產品配置使用全部4層，但 Take Cleaner 流程只處理L尺寸配置
        """
        if not response or not response.datas:
            self.node.get_logger().error("❌ EqpSignal 查詢失敗或沒有資料")
            self.search_eqp_signal_ok = True
            return

        # 只檢查 port1 和 port3 (L尺寸產品配置)
        port1_has_cargo = EqpSignalQueryClient.eqp_signal_port(
            response, self.port_address + 1)
        port3_has_cargo = EqpSignalQueryClient.eqp_signal_port(
            response, self.port_address + 3)

        self.agv_port1_empty = not port1_has_cargo
        self.agv_port3_empty = not port3_has_cargo

        self.node.get_logger().info(
            f"[AGV Port 檢查] Port 1 是否為空: {self.agv_port1_empty} (L尺寸第1層)")
        self.node.get_logger().info(
            f"[AGV Port 檢查] Port 3 是否為空: {self.agv_port3_empty} (L尺寸第3層)")

        self.search_eqp_signal_ok = True

    def _update_context_states(self, context: RobotContext):
        """更新context中的狀態 - 只更新 port1 和 port3"""
        if not self.search_eqp_signal_ok:
            return
        # 更新 AGV_PORT 層狀態（只更新 port1 和 port3）
        context.agv_port1 = not self.agv_port1_empty
        context.agv_port3 = not self.agv_port3_empty

    def _handle_port_selection(self, context: RobotContext):
        """處理 port 選擇邏輯 - 檢查 port1 和 port3 都是空的（批量取料必要條件）

        說明：新的 Station-based 設計批量取料流程
        - 從 Cleaner 取2格 → 放到 AGV 的 port1 和 port3
        - 必須確認 AGV 的 port1 和 port3 都是空的才能執行
        """
        if self.check_ok or not self.search_eqp_signal_ok:
            return

        # 檢查 port1 和 port3 是否都是空的
        if self.agv_port1_empty and self.agv_port3_empty:
            # ✅ port1 和 port3 都是空的 → 可以批量取料 (2格)
            self.node.get_logger().info(
                "✅ [Station-based 批量] AGV Port 1 和 Port 3 都是空的")
            self.node.get_logger().info(
                f"可以執行批量取料操作（Work ID {context.work_id}）: "
                f"Cleaner Station → AGV Port 1 + Port 3（2格批量）")
            context.get_loader_agv_port_front = 1  # 使用 port1 作為第一個目標
            self.check_ok = True
        else:
            # ❌ port1 或 port3 有貨 → AGV 端口已滿，無法取料
            self.node.get_logger().warn(
                f"❌ [Station-based 批量] AGV 端口已滿，無法執行批量取料")
            self.node.get_logger().warn(
                f"AGV Port 1 是否為空: {self.agv_port1_empty}, "
                f"Port 3 是否為空: {self.agv_port3_empty}")
            self.node.get_logger().warn(
                "批量取料需要 Port 1 和 Port 3 都是空的，請等待 AGV 卸載貨物。")
            context.get_loader_agv_port_front = None
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

        # 3. 處理 port 選擇邏輯
        self._handle_port_selection(context)

        # 4. 檢查完成，進入下一個狀態
        if self.check_ok:
            self.node.get_logger().info(
                "✅ [Station-based 批量] AGV 端口檢查完成 (port1 和 port3 都是空的)")
            self.node.get_logger().info("進入 TakeCleanerState 狀態")
            from loader_agv.robot_states.take_cleaner.take_cleaner_state import TakeCleanerState
            context.set_state(TakeCleanerState(self.node))
