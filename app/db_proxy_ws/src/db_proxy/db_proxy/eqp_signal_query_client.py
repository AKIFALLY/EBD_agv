import rclpy
from rclpy.node import Node
from db_proxy_interfaces.srv import EqpSignalQuery


class EqpSignalQueryClient:
    def __init__(self, node: Node):
        self.node = node
        self.node.get_logger().info("🚀 EqpSignal Query 用戶端已啟動")

        # 創建 EqpSignalQuery 服務客戶端
        self.client = self.node.create_client(
            EqpSignalQuery, "/agvc/eqp_signal_query")

        # 等待服務可用
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("等待 EqpSignalQuery 服務...")
            if not rclpy.ok():
                self.node.get_logger().error("ROS 2 已被關閉，退出等待")
                return

        self.node.get_logger().info("✅ 已連接到 EqpSignalQuery 服務")

        self.response = None

    def search_eqp_signal_eqp_id(self, eqp_id, callback):
        """根據 eqp_id 查詢 EqpSignal，結果通過 callback 回呼"""
        if not self.client.service_is_ready():
            self.node.get_logger().warn('❌ Service /agvc/eqp_signal_query 尚未就緒')
            return None

        request = EqpSignalQuery.Request()
        request.eqp_id = eqp_id
        self.node.get_logger().info(f"📤 發送查詢請求: eqp_id={eqp_id}")
        future = self.client.call_async(request)

        def _internal_callback(_future):
            try:
                if _future.done() and _future.result() is not None:
                    result = _future.result()
                    response = result
                    if response and response.success:
                        self.node.get_logger().info(
                            f"✅ 查詢成功: {response.message}")
                        self.node.get_logger().info(
                            f"📦 查詢結果: {len(response.datas)} 條 EqpSignal 資料")
                        callback(result)
                    elif response:
                        self.node.get_logger().error(
                            f"❌ 查詢失敗: {response.message}")
                        callback(None)
                    else:
                        self.node.get_logger().error("❌ 查詢失敗: 無回應")
                        callback(None)
                else:
                    self.node.get_logger().warn("⚠️ search_eqp_signal_eqp_id 未完成或無回應")
                    callback(None)

            except Exception as e:
                self.node.get_logger().error(
                    f"❌ search_eqp_signal_eqp_id 發生錯誤: {e}")
                callback(None)

        future.add_done_callback(_internal_callback)
        return future

    @staticmethod
    def eqp_signal_list(response):
        """靜態函式：取得以 eqp_id 為 key 的 EqpSignal 資料字典"""
        if response and response.datas:
            eqp_signal_dict = {}
            for eqp_signal in response.datas:
                eqp_signal_dict[eqp_signal.id] = eqp_signal
            return eqp_signal_dict
        return {}

    @staticmethod
    def eqp_signal_port(response, eqp_port_id):
        """靜態函式：根據 eqp_port_id 查詢 value 並使用 type_of_value 轉換"""
        if response and response.datas:
            for eqp_signal in response.datas:
                if eqp_signal.eqp_port_id == eqp_port_id:
                    # 根據 type_of_value 轉換 value
                    try:
                        type_of_value = eqp_signal.type_of_value.lower()
                        if type_of_value == 'int':
                            return int(eqp_signal.value)
                        elif type_of_value == 'float':
                            return float(eqp_signal.value)
                        elif type_of_value == 'bool':
                            return eqp_signal.value.lower() in ('true', '1', 'yes', 'on')
                        elif type_of_value == 'string':
                            return str(eqp_signal.value)
                        else:
                            # 如果 type_of_value 不是已知類型，返回原始字串
                            return eqp_signal.value
                    except (ValueError, AttributeError):
                        # 轉換失敗時返回原始字串
                        return eqp_signal.value
        return None

    def shutdown(self):
        """關閉用戶端"""
        self.node.get_logger().info("🔻 EqpSignal Query 用戶端已關閉")
        if self.client:
            self.node.destroy_client(self.client)
