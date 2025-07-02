import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from db_proxy_interfaces.msg import Fetch, Tables  # 請根據你的實際套件名稱修改
import uuid


class FetchClientNode(Node):
    def __init__(self):
        super().__init__('fetch_client_node')

        # 建立一個唯一的回應 topic
        self.response_topic = f"/agvc/response_{uuid.uuid4().hex[:8]}"
        self.get_logger().info(f"📨 回應 topic: {self.response_topic}")

        # 訂閱回應
        self.subscription = self.create_subscription(
            Tables, self.response_topic, self.handle_response, 10
        )

        # 建立 fetch publisher
        self.fetch_pub = self.create_publisher(Fetch, "/agvc/fetch", 10)


        self.send_fetch_request()

    def send_fetch_request(self):
        self.get_logger().info("📤 發送 Fetch 請求中...")
        fetch_msg = Fetch()
        fetch_msg.works = False
        fetch_msg.tasks = True
        fetch_msg.racks = False
        fetch_msg.locations = False
        fetch_msg.eqps = False
        fetch_msg.response_to_topic = self.response_topic

        self.fetch_pub.publish(fetch_msg)
        self.get_logger().info("✅ Fetch 請求已送出")


    def handle_response(self, msg: Tables):
        self.get_logger().info("📥 收到 Tables 回應")
        self.get_logger().info(f"🧱 Works: {len(msg.works)} 筆")
        self.get_logger().info(f"📦 Racks: {len(msg.racks)} 筆")
        self.get_logger().info(f"📝 Tasks: {len(msg.tasks)} 筆")
        self.get_logger().info(f"✅ Success: {msg.success}, Message: {msg.message}")

        self.get_logger().info(f"🧱 Works: {msg.works}")
        self.get_logger().info(f"📦 Racks: {msg.racks}")
        self.get_logger().info(f"📝 Tasks: {msg.tasks}")


def main(args=None):
    rclpy.init(args=args)
    node = FetchClientNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
