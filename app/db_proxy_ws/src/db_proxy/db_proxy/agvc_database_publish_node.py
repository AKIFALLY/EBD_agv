import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.models import Task
from sqlmodel import select
import json

class AGVCDatabasePublishNode(Node):
    def __init__(self):
        super().__init__('agvc_database_publish_node')
        self.publisher_ = self.create_publisher(String, '/task_table', 10)
        db_url_agvc = self.declare_parameter(
            'db_url_agvc',
            'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'
        ).value
        self.pool_agvc = ConnectionPoolManager(db_url_agvc)
        self.timer = self.create_timer(1.5, self.publish_task_table)
        self.get_logger().info("AGVC Database Publish Node 已啟動，開始發佈 task_table topic")

    def publish_task_table(self):
        try:
            with self.pool_agvc.get_session() as session:
                tasks = session.exec(select(Task)).all()
                # 將 ORM 物件轉換為 dict
                task_list = [task.dict() for task in tasks]
                msg = String()
                msg.data = json.dumps(task_list, default=str)
                self.publisher_.publish(msg)
                self.get_logger().debug(f"已發佈 {len(task_list)} 筆任務資料")
        except Exception as e:
            self.get_logger().error(f"查詢或發佈任務資料失敗: {e}")

    def shutdown(self):
        self.pool_agvc.shutdown()
        self.get_logger().info("🔻 agvc_database_publish_node 已關閉")

def main(args=None):
    rclpy.init(args=args)
    node = AGVCDatabasePublishNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
