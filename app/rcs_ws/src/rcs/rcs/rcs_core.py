import rclpy
from rclpy.node import Node
from db_proxy.connection_pool_manager import ConnectionPoolManager
from rcs.simple_kuka_manager import KukaManager
from rcs.simple_ct_manager import CtManager


class RcsCore(Node):

    def __init__(self):
        super().__init__("rcs_core")
        self.get_logger().info("簡化版 RCS Core 節點啟動中...")

        # 初始化資料庫連線池
        try:
            self.db_pool = ConnectionPoolManager(
                'postgresql+psycopg2://agvc:password@192.168.100.254/agvc')
            self.get_logger().info("資料庫連線池已建立。")
        except Exception as e:
            self.get_logger().fatal(f"建立資料庫連線池失敗: {e}", exc_info=True)
            # 如果資料庫連線失敗，可能需要決定是否要讓節點繼續執行
            self.db_pool = None

        # --- 初始化簡化的車隊管理器 ---
        # 簡化的 KUKA 車隊管理器
        self.kuka_manager = KukaManager(self)

        # 簡化的 CT 車隊管理器
        self.ct_manager = CtManager(self)

        # 1秒定時器
        self.timer_1s = self.create_timer(1.0, self.main_loop)
        
        self.get_logger().info("✅ 簡化版 RCS Core 節點啟動完成")

    def main_loop(self):
        """主迴圈：每秒執行一次的任務派發"""
        self.get_logger().debug("1秒定時器觸發 (timer_1s)")

        # KUKA 車隊任務派發 (簡化版)
        self.kuka_manager.dispatch()

        # CT 車隊任務派發 (簡化版)
        self.ct_manager.dispatch()


def main(args=None):
    rclpy.init(args=args)
    node = RcsCore()
    try:
        # 只有在資料庫連線成功時才 spin
        if node.db_pool:
            rclpy.spin(node)
        else:
            node.get_logger().fatal("因資料庫連線失敗，節點無法啟動。")
    except KeyboardInterrupt:
        node.get_logger().info("🛑 偵測到 Ctrl+C，正在關閉簡化版 RcsCore 節點...")
    finally:
        if hasattr(node, 'kuka_manager') and node.kuka_manager:
            node.kuka_manager.stop_monitoring()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
