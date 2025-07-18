import rclpy
from rclpy.node import Node
from db_proxy.connection_pool_manager import ConnectionPoolManager

from wcs.task_manager import TaskManager

class WcsCore(Node):
    def __init__(self):
        super().__init__('wcs_core')

        # 初始化資料庫連線池
        db_url_agvc = self.declare_parameter(
            'db_url_agvc',
            'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'
        ).value

        try:
            self.db_pool = ConnectionPoolManager(db_url_agvc)
            self.get_logger().info(f"資料庫連線池已建立，使用 URL: {db_url_agvc}")
        except Exception as e:
            self.get_logger().fatal(f"建立資料庫連線池失敗: {e}")
            self.db_pool = None
            return

        self.task_manager = TaskManager(self.db_pool, self.get_logger())

        self.timer = self.create_timer(5.0, self.timer_callback)

        self.get_logger().info("WCS Node started")

    def timer_callback(self):
        try:
            self.task_manager.process_tasks()
        except Exception as e:
            self.get_logger().error(f"錯誤: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = WcsCore()

    try:
        # 只有在資料庫連線成功時才 spin
        if node.db_pool:
            rclpy.spin(node)
        else:
            node.get_logger().error("資料庫連線失敗，節點無法啟動")
    finally:
        # 關閉 TaskManager
        if hasattr(node, 'task_manager') and node.task_manager:
            node.task_manager.shutdown()

        # 關閉連線池
        if hasattr(node, 'db_pool') and node.db_pool:
            node.db_pool.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
