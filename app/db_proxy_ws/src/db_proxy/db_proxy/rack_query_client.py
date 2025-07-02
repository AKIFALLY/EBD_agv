import rclpy
from rclpy.node import Node
from db_proxy_interfaces.srv import RackQuery


class RackQueryClient:
    def __init__(self, node: Node):
        self.node = node
        self.node.get_logger().info("🚀 Rack Query 用戶端已啟動")

        # 創建 RackQuery 服務客戶端
        self.client = self.node.create_client(
            RackQuery, "/agvc/rack_query")

        # 等待服務可用
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("等待 RackQuery 服務...")
            if not rclpy.ok():
                self.node.get_logger().error("ROS 2 已被關閉，退出等待")
                return

        self.node.get_logger().info("✅ 已連接到 RackQuery 服務")

    def search_rack_id(self, rack_id, callback):
        """根據 rack_id 查詢 Rack，結果通過 callback 回呼"""
        if not self.client.service_is_ready():
            self.node.get_logger().warn('❌ Service /agvc/rack_query 尚未就緒')
            return None

        request = RackQuery.Request()
        request.id = rack_id
        self.node.get_logger().info(f"📤 發送查詢請求: rack_id={rack_id}")
        future = self.client.call_async(request)

        def _internal_callback(_future):
            try:
                if (_future.done() and _future.result() is not None):
                    result = _future.result()
                    response = result
                    if response and response.success:
                        self.node.get_logger().info(
                            f"✅ 查詢成功: {response.message}")

                        callback(result)
                    elif response:
                        self.node.get_logger().error(
                            f"❌ 查詢失敗: {response.message}")
                        callback(result)
                    else:
                        self.node.get_logger().error("❌ 查詢失敗: 無回應")
                        callback(result)
                else:
                    self.node.get_logger().warn("⚠️ search_rack 未完成或無回應")
                    callback(None)
            except Exception as e:
                self.node.get_logger().error(
                    f"❌ search_rack 發生錯誤: {e}")
                callback(None)

        future.add_done_callback(_internal_callback)
        return future

    def shutdown(self):
        """關閉用戶端"""
        self.node.get_logger().info("🔻 Rack Query 用戶端已關閉")
        if self.client:
            self.node.destroy_client(self.client)


def main(args=None):
    rclpy.init(args=args)
    node = Node("rack_query_client_node")
    client = RackQueryClient(node)

    try:
        # 示例查詢：查詢 rack_id 為 1 的 Rack
        client.search_rack(
            rack_id=1, callback=lambda response, success: node.get_logger().info(
                f"查詢回應: 成功={success}, 回應={response}")
        )
        rclpy.spin(node)  # 保持節點運行以處理非同步回呼
    except KeyboardInterrupt:
        node.get_logger().info("⚠️ 收到 Ctrl+C，關閉程式")
    except Exception as e:
        node.get_logger().error(f"❌ 程式發生錯誤: {e}")
    finally:
        client.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
