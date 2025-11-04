from agv_base.states.state import State
from db_proxy.eqp_signal_query_client import EqpSignalQueryClient
from rclpy.node import Node
from loader_agv.robot_context import RobotContext


class AgvPortCheckEmptyState(State):

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
        self.port2_empty = False
        self.port4_empty = False

    def enter(self):
        self.node.get_logger().info(
            "[Station-based 1格] Loader Robot Take Soaker 目前狀態: AgvPortCheckEmpty")
        self._reset_state()

    def leave(self):
        self.node.get_logger().info(
            "[Station-based 1格] Loader Robot Take Soaker 離開 AgvPortCheckEmpty 狀態")
        self._reset_state()

    def eqp_signal_query_callback(self, response):
        """處理 EqpSignal 查詢回應 - 只檢查 port 2 和 4（L尺寸偶數層配置）

        說明：Take Soaker AGV port 分層策略
        - L尺寸 AGV 配置：只使用 port 1-4（第1-4層）
        - **關鍵設計差異**：
          - Put Soaker（放料）使用 Port 1 和 3（奇數層）
          - Take Soaker（取料）使用 Port 2 和 4（偶數層）⭐
        - 單格處理：一次只放1格，優先 port 2 > port 4
        - 目的：避免 Put/Take 衝突，確保流程順暢
        """
        if not response or not response.datas:
            self.node.get_logger().error("❌ [Station-based 1格] EqpSignal 查詢失敗或沒有資料")
            self.search_eqp_signal_ok = True
            return

        # 只檢查 port 2 和 4（Take Soaker 專用偶數層）
        port2_has_cargo = EqpSignalQueryClient.eqp_signal_port(
            response, self.port_address + 2)
        port4_has_cargo = EqpSignalQueryClient.eqp_signal_port(
            response, self.port_address + 4)

        self.port2_empty = not port2_has_cargo
        self.port4_empty = not port4_has_cargo

        self.node.get_logger().info(
            f"[Station-based 1格] AGV Port 2 是否為空: {self.port2_empty} (L尺寸第2層，Take Soaker 偶數層)")
        self.node.get_logger().info(
            f"[Station-based 1格] AGV Port 4 是否為空: {self.port4_empty} (L尺寸第4層，Take Soaker 偶數層)")

        self.search_eqp_signal_ok = True

    def _update_context_states(self, context: RobotContext):
        """更新context中的狀態 - 只更新 port2 和 port4"""
        if not self.search_eqp_signal_ok:
            return
        # 更新 AGV_PORT 層狀態（只更新 port2 和 port4）
        context.agv_port2 = not self.port2_empty
        context.agv_port4 = not self.port4_empty

    def _handle_port_selection(self, context: RobotContext):
        """處理 port 選擇邏輯 - 單格處理，優先 port 2 > port 4

        說明：Take Soaker 單格放料邏輯
        - 一次只放1格（與 Take Cleaner 批量2格不同）
        - 選擇順序：Port 2（第2層）> Port 4（第4層）
        - 放1格後直接進入 CompleteState
        - 使用偶數層（2, 4）與 Put Soaker 的奇數層（1, 3）區隔
        """
        if self.check_ok or not self.search_eqp_signal_ok:
            return

        # 優先選擇 port 2
        if self.port2_empty:
            self.node.get_logger().info(
                f"✅ [Station-based 1格] AGV Port 2 是空的 (Work ID {context.work_id})")
            self.node.get_logger().info(
                f"[Station-based 1格] 選擇 AGV Port 2 放置貨物 → 來自泡藥機 Port {context.get_soaker_port}")
            context.get_loader_agv_port_side = 2
            self.check_ok = True
        elif self.port4_empty:
            self.node.get_logger().info(
                f"✅ [Station-based 1格] AGV Port 4 是空的 (Work ID {context.work_id})")
            self.node.get_logger().info(
                f"[Station-based 1格] Port 2 已滿，選擇 AGV Port 4 放置貨物 → 來自泡藥機 Port {context.get_soaker_port}")
            context.get_loader_agv_port_side = 4
            self.check_ok = True
        else:
            self.node.get_logger().warn(
                f"❌ [Station-based 1格] AGV Port 2 和 4 都已滿 (Work ID {context.work_id})")
            self.node.get_logger().warn(
                "[Station-based 1格] 無法執行放置操作，請等待 AGV 卸載貨物。")
            context.get_loader_agv_port_side = None
            self._reset_state()

    def handle(self, context: RobotContext):
        # 1. 更新 context 狀態
        self._update_context_states(context)

        # 2. 查詢 EQP 信號（只執行一次）
        if not self.search_eqp_signal_ok and not self.sent:
            self.node.get_logger().info(
                f"[Station-based 1格] 查詢 AGV 端口狀態 (eqp_id={self.eqp_id})，"
                f"檢查 port 2 和 4 (Work ID {context.work_id})")
            self.node.get_logger().info(
                "[Station-based 1格] Take Soaker 使用偶數層（Port 2, 4）策略")
            self.eqp_signal_query_client.search_eqp_signal_eqp_id(
                self.eqp_id, self.eqp_signal_query_callback)
            self.sent = True

        # 3. 處理 port 選擇邏輯
        self._handle_port_selection(context)

        # 4. 檢查完成，進入下一個狀態
        if self.check_ok:
            self.node.get_logger().info(
                f"✅ [Station-based 1格] AGV 端口檢查完成 (選擇 port{context.get_loader_agv_port_side})")
            self.node.get_logger().info(
                f"[Station-based 1格] 進入 TakeSoakerState: 泡藥機 Port {context.get_soaker_port} → "
                f"機械臂 → AGV Port {context.get_loader_agv_port_side}")
            from loader_agv.robot_states.take_soaker.take_soaker_state import TakeSoakerState
            context.set_state(TakeSoakerState(self.node))
