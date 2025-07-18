import rclpy
from rclpy.node import Node
from db_proxy.connection_pool_manager import ConnectionPoolManager
from rcs.kuka_manager import KukaManager
from rcs.ct_manager import CtManager
from rcs.task_status_simulator import TaskStatusSimulator


class RcsCore(Node):

    def __init__(self):
        super().__init__("rcs_core")
        self.get_logger().info("RCS Core 節點啟動中...")

        # 初始化資料庫連線池
        try:
            self.db_pool = ConnectionPoolManager(
                'postgresql+psycopg2://agvc:password@192.168.100.254/agvc')
            self.get_logger().info("資料庫連線池已建立。")
        except Exception as e:
            self.get_logger().fatal(f"建立資料庫連線池失敗: {e}", exc_info=True)
            # 如果資料庫連線失敗，可能需要決定是否要讓節點繼續執行
            self.db_pool = None

        # --- 初始化車隊管理器 ---
        # KUKA 車隊管理器 - 處理 KUKA 相關功能
        self.kuka_manager = KukaManager(self)

        # CT 車隊管理器 - 處理您自己的 AGV 車隊
        self.ct_manager = CtManager(self)

        # --- 初始化任務狀態模擬器 ---
        self.task_status_simulator = TaskStatusSimulator(self.db_pool, self.get_logger())

        # 新增 1 秒觸發一次的 timer callback
        self.timer_1s = self.create_timer(1.0, self.main_loop)

    def main_loop(self):
        self.get_logger().debug("1秒定時器觸發 (timer_1s)")

        # 任務狀態模擬處理 (優先處理)
        self.task_status_simulator.process_task_status_transitions()

        # KUKA 車隊任務派發
        self.kuka_manager.dispatch()

        # CT 車隊任務派發
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
        node.get_logger().info("🛑 偵測到 Ctrl+C，正在關閉 RcsCore 節點...")
    finally:
        # 關閉任務狀態模擬器
        if hasattr(node, 'task_status_simulator') and node.task_status_simulator:
            node.task_status_simulator.shutdown()

        if hasattr(node, 'kuka_manager') and node.kuka_manager:
            node.kuka_manager.stop_monitoring()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
