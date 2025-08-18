import rclpy
from rclpy.node import Node
from db_proxy_interfaces.srv import CarrierQuery


class CarrierQueryClient:
    def __init__(self, node: Node):
        self.node = node
        self.node.get_logger().info("🚀 Carrier Query 用戶端已啟動")

        # 創建 CarrierQuery 服務客戶端
        self.client = self.node.create_client(
            CarrierQuery, "/agvc/carrier_query")

        # 等待服務可用
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("等待 CarrierQuery 服務...")
            if not rclpy.ok():
                self.node.get_logger().error("ROS 2 已被關閉，退出等待")
                return

        self.node.get_logger().info("✅ 已連接到 CarrierQuery 服務")

        self.response = None

    def search_carrier_rack_id(self, rack_id, callback):
        """根據 rack_id 查詢 Carrier，結果通過 callback 回呼"""
        if not self.client.service_is_ready():
            self.node.get_logger().warn('❌ Service /agvc/carrier_query 尚未就緒')
            return None

        request = CarrierQuery.Request()
        # 初始化所有 uint64 欄位為 0，避免類型檢查錯誤
        request.id = 0
        request.room_id = 0
        request.rack_id = int(rack_id)  # 確保是整數類型
        request.port_id = 0
        request.port_id_min = 0
        request.port_id_max = 0
        request.rack_index = 0
        request.status_id = 0
        request.sort_order = 1  # 按 rack_index 升序排序
        self.node.get_logger().info(f"📤 發送查詢請求: rack_id={rack_id}")
        future = self.client.call_async(request)

        def _internal_callback(_future):
            try:
                if _future.done() and _future.result() is not None:
                    result = _future.result()
                    response = result
                    if response and response.success:
                        self.node.get_logger().info(
                            f"✅ 查詢成功: {response.message}")
                        callback(result)
                    elif response:
                        self.node.get_logger().error(
                            f"❌ 查詢失敗: {response.message}")
                        callback(None)
                    else:
                        self.node.get_logger().error("❌ 查詢失敗: 無回應")
                        callback(None)
                else:
                    self.node.get_logger().warn("⚠️ search_carrier_rack_id 未完成或無回應")
                    callback(None, None, None, False, None)

            except Exception as e:
                self.node.get_logger().error(
                    f"❌ search_carrier_rack_id 發生錯誤: {e}")
                callback(None, None, None, False, None)

        future.add_done_callback(_internal_callback)
        return future

    def search_carrier_port_id(self, port_id_min, port_id_max, callback):
        """根據 port_id 範圍查詢 Carrier，結果通過 callback 回呼

        Args:
            port_id_min: port_id 最小值
            port_id_max: port_id 最大值
            callback: 回調函數
        """
        if not self.client.service_is_ready():
            self.node.get_logger().warn('❌ Service /agvc/carrier_query 尚未就緒')
            return None

        request = CarrierQuery.Request()
        # 初始化所有 uint64 欄位為 0，避免類型檢查錯誤
        request.id = 0
        request.room_id = 0
        request.rack_id = 0
        request.port_id = 0
        request.port_id_min = int(port_id_min)  # 確保是整數類型
        request.port_id_max = int(port_id_max)  # 確保是整數類型
        request.rack_index = 0
        request.status_id = 0
        request.sort_order = 1  # 按 rack_index 升序排序

        self.node.get_logger().info(
            f"📤 發送查詢請求: port_id範圍 {port_id_min} - {port_id_max}")
        future = self.client.call_async(request)

        def _internal_callback(_future):
            try:
                if _future.done() and _future.result() is not None:
                    result = _future.result()
                    response = result
                    if response and response.success:
                        self.node.get_logger().info(
                            f"✅ 查詢成功: {response.message}")
                        if callback:
                            callback(result)
                    elif response:
                        self.node.get_logger().error(
                            f"❌ 查詢失敗: {response.message}")
                        if callback:
                            callback(None)
                    else:
                        self.node.get_logger().error("❌ 查詢失敗: 無回應")
                        if callback:
                            callback(None)
                else:
                    self.node.get_logger().warn("⚠️ search_carrier_port_id 未完成或無回應")
                    if callback:
                        callback(None)

            except Exception as e:
                self.node.get_logger().error(
                    f"❌ search_carrier_port_id 發生錯誤: {e}")
                if callback:
                    callback(None)

        future.add_done_callback(_internal_callback)
        return future

    @staticmethod
    def carrier_min_rack_index(response):
        """靜態函式：計算 Carrier 資料中的最小 rack_index"""
        if response and response.datas:
            min_rack_index = min(
                carrier.rack_index for carrier in response.datas if carrier.rack_index is not None)
            return min_rack_index
        return None

    @staticmethod
    def carrier_max_rack_index(response):
        """靜態函式：計算 Carrier 資料中的最大 rack_index"""
        if response and response.datas:
            max_rack_index = max(
                carrier.rack_index for carrier in response.datas if carrier.rack_index is not None)
            return max_rack_index
        return None

    @staticmethod
    def no_carrier(response):
        """靜態函式：檢查 Carrier 資料是否為空"""
        if response and response.datas:
            return False
        return True

    @staticmethod
    def carrier_min_rack_index_carrier_id(response):
        """靜態函式：取得 Carrier 資料中最小 rack_index 的 Carrier ID"""
        if response and response.datas:
            min_rack_index = CarrierQueryClient.carrier_min_rack_index(
                response)
            for carrier in response.datas:
                if carrier.rack_index == min_rack_index:
                    return carrier.id
        return None

    @staticmethod
    def carrier_port_id_carrier_id(response, port_id):
        """靜態函式：取得 Carrier 資料中指定 port_id 的 Carrier ID

        Args:
            response: CarrierQuery 服務的回應
            port_id: 要查詢的 port_id

        Returns:
            str: 找到的 Carrier ID，如果沒有查詢到指定 port_id 則返回 None
        """
        if response and response.datas:
            for carrier in response.datas:
                if carrier.port_id == port_id:
                    return carrier.id
        return None

    @staticmethod
    def carrier_list(response):
        """靜態函式：取得以 carrier_id 為 key 的 Carrier 資料字典"""
        if response and response.datas:
            carrier_dict = {}
            for carrier in response.datas:
                carrier_dict[carrier.id] = carrier
            return carrier_dict
        return {}

    @staticmethod
    def delete_carrier_id(carrier_dict, carrier_id):
        """靜態函式：從 Carrier 字典中刪除指定的 carrier_id"""
        if carrier_dict and isinstance(carrier_dict, dict):
            # 建立新的字典，排除指定的 carrier_id
            filtered_dict = {k: v for k,
                             v in carrier_dict.items() if k != carrier_id}
            return filtered_dict
        return carrier_dict

    def shutdown(self):
        """關閉用戶端"""
        self.node.get_logger().info("🔻 Carrier Query 用戶端已關閉")
        if self.client:
            self.node.destroy_client(self.client)
