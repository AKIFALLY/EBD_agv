from db_proxy.models import Task
import rclpy
from rclpy.node import Node
from db_proxy_interfaces.srv import SqlQuery
from sqlmodel import select
import json
from sqlalchemy.dialects import postgresql
from db_proxy.sql.sql_builder import SQLBuilder


class SqlQueryClientNode(Node):
    def __init__(self):
        super().__init__('query_client_node')

        self.cli = self.create_client(SqlQuery, 'sql_query')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def send_request(self):
        filters = {"agv_id": 3, "status_id": 1}
        req = SqlQuery.Request()
        req.query_string = SQLBuilder.select_by(
            Task, **filters)
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            res = future.result()
            print("Success:", res.success)
            print("Message:", res.message)
            print("JSON:", res.json_result)

            if res.success and res.json_result:
                try:
                    data = json.loads(res.json_result)
                    for item in data:
                        print("description:", item.get('description'))
                except json.JSONDecodeError:
                    print("回傳資料非合法 JSON 格式")
        else:
            print("Service call failed")


def main():
    rclpy.init()
    node = SqlQueryClientNode()
    node.send_request()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
