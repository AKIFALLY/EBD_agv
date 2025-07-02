from datetime import datetime, timezone
import rclpy
from rclpy.node import Node
from db_proxy.agvc_database_client import AGVCDatabaseClient
from db_proxy_interfaces.msg import Task as TaskMsg
from builtin_interfaces.msg import Time
import time


class TestClient(Node):
    def __init__(self):
        super().__init__('test_update_task_client')
        self.agvc_client = AGVCDatabaseClient(self)

        # 等待 service 就緒
        while not self.agvc_client.task_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 /agvc/update_task 服務啟動...')
        self.get_logger().info('/agvc/update_task 服務已啟動')

    def send_request(self):

        # 模擬 TaskMsg
        test_task = TaskMsg()
        test_task.id = 1
        test_task.name = "test_task"
        test_task.agv_id = 1
        test_task.priority = 1
        test_task.work_id = 1
        test_task.room_id = 1
        test_task.status_id = 1
        test_task.description = "這是一個測試任務"
        test_task.parameters = "{}"

        print(f"TaskMsg type: {type(test_task)}")
        print(
            f"TaskMsg class path: {test_task.__class__.__module__}.{test_task.__class__.__name__}")

        # ✅ 測試同步呼叫
        response = self.agvc_client.update_task(test_task)
        if response:
            self.get_logger().info(f"✅ 同步呼叫結果: {response.success}")
            self.get_logger().info(f"✅ 同步呼叫訊息: {response.message}")
            self.get_logger().info(f"✅ 同步呼叫任務: {response.task}")
        else:
            self.get_logger().warn("❌ 同步呼叫失敗或逾時")


def main():
    rclpy.init()
    node = TestClient()
    node.send_request()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
