import rclpy
from rclpy.node import Node
from db_proxy.agvc_database_client import AGVCDatabaseClient
from db_proxy_interfaces.msg import Rack as RackMsg
from builtin_interfaces.msg import Time
import time


class TestClient(Node):
    def __init__(self):
        super().__init__('test_update_task_client')
        self.agvc_client = AGVCDatabaseClient(self)

        # 等待 service 就緒
        while not self.agvc_client.rack_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 /agvc/update_rack 服務啟動...')
        self.get_logger().info('/agvc/update_rack 服務已啟動')

    def send_request(self):
        ros_time = Time()
        ros_time.sec = int(time.time())
        ros_time.nanosec = 0

        rack = RackMsg()
        rack.id = 4
        rack.location_id = 1
        rack.product_id = 1
        rack.status_id = 1

        self.agvc_client.async_update_rack(rack, self.callback)

    def callback(self, result):
        if result is not None:
            self.get_logger().info(
                f"✅ 非同步回應：success={result.success}, message={result.message}")
        else:
            self.get_logger().error("❌ 未收到非同步回應")
        self.get_logger().info("非同步回應Callback結束")
        rclpy.shutdown()


def main():
    rclpy.init()
    node = TestClient()
    node.send_request()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
