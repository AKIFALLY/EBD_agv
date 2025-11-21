"""
Cargo Mover AGV 專用的 RunningState

簡化邏輯：
1. 進入狀態時請求交管區控制權
2. 若拒絕，則觸發 PLC MR7005 停止移動
3. 離開狀態時釋放交管區並關閉 PLC MR7005
"""

from agv_base.agv_states.Running_state import RunningState
from cargo_mover_agv.utils.traffic_client import TrafficClient
from rclpy.node import Node


class CargoRunningState(RunningState):
    """
    Cargo Mover AGV 專用的 RunningState

    在基礎 RunningState 之上增加交通管制功能：
    - 進入時請求交管區控制權
    - 被拒絕時觸發 PLC MR7005 停止移動
    - 離開時釋放交管區並關閉 PLC MR7005
    """

    def __init__(self, node: Node):
        super().__init__(node)

        # 初始化 HTTP 客戶端
        self.traffic_client = TrafficClient(node)

        # 交管區配置（TODO: 從配置文件讀取）
        self.traffic_zone_id = 1  # 固定為 room2 交管區

        # 追蹤是否已觸發 PLC 停止
        self.plc_stop_triggered = False

        # PLC 客戶端引用（從 node 取得）
        self.plc_client = node.plc_client

        # 定時重試機制
        self.traffic_retry_timer = None  # 重試計時器
        self.traffic_retry_count = 0  # 重試次數
        self.traffic_allowed = False  # 是否已獲得通行許可
        self.traffic_retry_interval = 3.0  # 重試間隔（3秒）

    def enter(self):
        """
        進入 RunningState 時的處理

        新流程：
        1. 呼叫父類的 enter()
        2. 直接請求交管區
        3. 如果被拒絕 → 才觸發 PLC MR7005 停止，並啟動 3 秒重試
        4. 如果允許 → 直接允許移動（不需要 PLC 操作）

        註：即使沒有有效任務（local 模式），也執行交管邏輯
        """
        # 呼叫父類的 enter() 方法
        super().enter()

        # 直接請求交管區（不先停止 AGV）
        self._request_traffic_control()

    def leave(self):
        """
        離開 RunningState 時的處理

        1. 取消重試計時器（如果有）
        2. 釋放交管區控制權
        3. 無論釋放成功或失敗，都關閉 PLC MR7005
        4. 呼叫父類的 leave()
        """
        # 【新增】取消重試計時器
        self._cancel_retry_timer()

        # 釋放交管區控制權
        self._release_traffic_control()

        # 呼叫父類的 leave() 方法
        super().leave()

    def handle(self, context):
        """
        覆寫 handle 方法

        攔截轉換到 WaitRobotState，確保使用 CargoWaitRobotState
        """
        # 使用猴子補丁（Monkey Patch）攔截 set_state
        original_set_state = context.set_state

        def patched_set_state(new_state):
            """
            攔截 set_state，替換狀態為 Cargo 專屬版本
            - WaitRobotState → CargoWaitRobotState
            """
            from agv_base.agv_states.wait_robot_state import WaitRobotState
            from cargo_mover_agv.states.cargo_wait_robot_state import CargoWaitRobotState

            # 攔截 WaitRobotState
            if isinstance(new_state, WaitRobotState) and not isinstance(new_state, CargoWaitRobotState):
                self.node.get_logger().info(
                    "[Cargo] 🔄 攔截狀態轉換：WaitRobotState → CargoWaitRobotState"
                )
                original_set_state(CargoWaitRobotState(self.node))
            else:
                original_set_state(new_state)

        # 暫時替換 set_state 方法
        context.set_state = patched_set_state

        try:
            # 呼叫父類的 handle 邏輯
            super().handle(context)
        finally:
            # 恢復原始的 set_state 方法
            context.set_state = original_set_state

    def _trigger_plc_stop(self):
        """
        觸發 PLC MR7005 停止（當交管拒絕時才呼叫）
        """
        try:
            self.plc_client.async_force_on('MR', '7005', self._plc_stop_callback)
            self.plc_stop_triggered = True
        except Exception as e:
            self.node.get_logger().error(f"[交管] ❌ PLC 停止失敗: {e}")

    def _plc_stop_callback(self, response):
        """PLC force_on 回調（停止信號）"""
        if response is None:
            self.node.get_logger().error("[交管] ❌ PLC 停止未收到回應")
        elif response.success:
            self.node.get_logger().info("[交管] ✅ PLC MR7005 停止成功")
        else:
            self.node.get_logger().error(f"[交管] ❌ PLC 停止失敗: {response.message}")

    def _request_traffic_control(self):
        """
        請求交管區控制權

        流程:
        (a) 發送 API 請求 → 回應允許 → 直接允許移動（不需要 PLC 操作）
        (b) 發送 API 請求 → 回應拒絕 → 觸發 PLC MR7005 停止，啟動 3 秒重試
        """
        # 日誌顯示重試次數（如果有）
        retry_info = f" (第 {self.traffic_retry_count + 1} 次嘗試)" if self.traffic_retry_count > 0 else ""
        self.node.get_logger().info(
            f"[交管] 請求交管區 {self.traffic_zone_id}...{retry_info}"
        )

        # 發送 acquire 請求
        result = self.traffic_client.acquire_traffic_zone(
            self.traffic_zone_id,
            self.node.agv_id
        )

        # 檢查回應
        if result.get("isAllow", False):
            # (a) 回應允許：直接允許移動，不需要 PLC 操作
            if self.traffic_retry_count > 0:
                self.node.get_logger().info(
                    f"[交管] ✅ 允許通行（重試 {self.traffic_retry_count} 次後成功）"
                )
                # 如果之前有停止，現在要關閉 MR7005
                if self.plc_stop_triggered:
                    self._clear_plc_stop()
            else:
                self.node.get_logger().info("[交管] ✅ 允許通行")

            self.traffic_allowed = True
            self._cancel_retry_timer()  # 取消重試計時器
        else:
            # (b) 回應拒絕：觸發 PLC MR7005 停止（如果還沒觸發），啟動 3 秒重試
            owner_id = result.get("owner_agv_id", "未知")
            self.node.get_logger().warn(
                f"[交管] ⛔ 拒絕通行：交管區 {self.traffic_zone_id} 被 AGV {owner_id} 佔用"
            )

            # 如果還沒觸發 PLC 停止，現在才觸發
            if not self.plc_stop_triggered:
                self.node.get_logger().info("[交管] 🚨 觸發 PLC MR7005 停止 AGV")
                self._trigger_plc_stop()

            # 啟動重試計時器
            self._start_retry_timer()

    def _release_traffic_control(self):
        """
        離開 RunningState 時釋放交管區控制權

        流程:
        (a) 發送 API 請求 → 回應成功 → 關閉 PLC MR7005
        (b) 發送 API 請求 → 回應失敗 → 仍然關閉 PLC MR7005

        無論釋放成功或失敗，都關閉 PLC MR7005
        """
        self.node.get_logger().info(
            f"[交管] 離開 RunningState，釋放交管區 {self.traffic_zone_id}..."
        )

        # 發送 release 請求
        success = self.traffic_client.release_traffic_zone(
            self.traffic_zone_id,
            self.node.agv_id
        )

        # 無論成功或失敗，都關閉 PLC MR7005
        if self.plc_stop_triggered:
            self._clear_plc_stop()
        else:
            # 即使沒有觸發過停止，也嘗試關閉（安全機制）
            self._clear_plc_stop()


    def _clear_plc_stop(self):
        """
        關閉 PLC MR7005 停止信號（async_force_off）

        使用 async_force_off('MR', '7005', callback) 關閉 PLC 停止信號
        """
        try:
            # 呼叫 PLC 客戶端的 async_force_off 方法
            self.plc_client.async_force_off('MR', '7005', self._plc_force_off_callback)
            self.plc_stop_triggered = False
            self.node.get_logger().info("[交管] ✅ 已關閉 PLC MR7005 停止信號")
        except Exception as e:
            self.node.get_logger().error(f"[交管] ❌ 關閉 PLC 停止失敗: {e}")

    def _plc_force_off_callback(self, response):
        """PLC force_off 回調函數"""
        if response is None:
            self.node.get_logger().error("[交管] ❌ PLC force_off 未收到回應")
            return

        if response.success:
            self.node.get_logger().info(f"[交管] ✅ PLC MR7005 強制關閉成功")
        else:
            self.node.get_logger().error(
                f"[交管] ❌ PLC MR7005 強制關閉失敗: {response.message}"
            )

    def _start_retry_timer(self):
        """
        啟動重試計時器（3秒後重新請求交管）
        """
        # 先取消現有計時器（如果有）
        self._cancel_retry_timer()

        # 創建新的計時器
        self.traffic_retry_timer = self.node.create_timer(
            self.traffic_retry_interval,
            self._retry_traffic_request
        )
        self.node.get_logger().info(
            f"[交管] ⏱️ 啟動重試計時器，{self.traffic_retry_interval} 秒後重新請求"
        )

    def _cancel_retry_timer(self):
        """
        取消重試計時器
        """
        if self.traffic_retry_timer is not None:
            self.traffic_retry_timer.cancel()
            self.traffic_retry_timer = None
            self.node.get_logger().info("[交管] ⏹️ 取消重試計時器")

    def _retry_traffic_request(self):
        """
        定時器觸發：重新請求交管區
        """
        # 取消當前計時器（單次觸發）
        if self.traffic_retry_timer is not None:
            self.traffic_retry_timer.cancel()
            self.traffic_retry_timer = None

        # 增加重試次數
        self.traffic_retry_count += 1

        # 重新請求交管區
        self._request_traffic_control()
