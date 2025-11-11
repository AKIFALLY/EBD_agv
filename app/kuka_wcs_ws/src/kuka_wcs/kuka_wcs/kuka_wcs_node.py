"""
KUKA WCS Node - KUKA warehouse control system核心节点

专注于 KUKA rack 搬移和旋转的业务逻辑处理
采用纯 Python 实现，不依赖 TAFL

设计理念：
- 协作模式：kuka_wcs 负责业务逻辑（判断和创建任务），rcs 负责任务调度
- 1秒定时扫描：定时检查各种条件并创建相应任务
- 处理器模式：每种业务逻辑独立为一个处理器
"""
import rclpy
from rclpy.node import Node
from db_proxy.connection_pool_manager import ConnectionPoolManager
import signal
import sys


class KukaWcsNode(Node):
    """KUKA WCS 主节点"""

    def __init__(self):
        super().__init__("kuka_wcs_node")
        self.get_logger().info("KUKA WCS 节点启动中...")

        # 标记节点是否正在关闭
        self.is_shutting_down = False

        # 初始化资料库连线池
        try:
            self.db_pool = ConnectionPoolManager(
                'postgresql+psycopg2://agvc:password@192.168.100.254/agvc')
            self.get_logger().info("资料库连线池已建立。")
        except Exception as e:
            self.get_logger().fatal(f"建立资料库连线池失败: {e}")
            self.db_pool = None
            raise

        # 初始化任务处理器列表
        self.handlers = []
        self._init_handlers()

        # 1秒定时器：扫描所有处理器
        self.scan_timer = self.create_timer(1.0, self.scan_and_create_tasks)

        self.get_logger().info("✅ KUKA WCS 节点启动完成")

    def _init_handlers(self):
        """初始化所有任务处理器"""
        from kuka_wcs.task_handlers.rack_full_handler import RackFullHandler
        from kuka_wcs.task_handlers.rack_rotation_handler import RackRotationHandler
        from kuka_wcs.task_handlers.system_area_handler import SystemAreaHandler

        self.handlers = [
            RackFullHandler(self),
            # RackRotationHandler(self),  # TODO: 实现旋转逻辑后启用
            # SystemAreaHandler(self),    # TODO: 实现系统区域逻辑后启用
        ]

        self.get_logger().info(f"已初始化 {len(self.handlers)} 个任务处理器")

    def scan_and_create_tasks(self):
        """定时扫描：检查所有处理器并创建任务"""
        if self.is_shutting_down:
            return

        if not self.db_pool:
            self.get_logger().warn("资料库连线池未初始化，跳过扫描")
            return

        self.get_logger().debug("1秒定时器触发 (scan_timer)")

        # 遍历所有处理器，让它们检查条件并创建任务
        for handler in self.handlers:
            try:
                with self.db_pool.get_session() as session:
                    tasks_created = handler.check_and_create_tasks(session)
                    if tasks_created:
                        self.get_logger().info(
                            f"{handler.__class__.__name__} 创建了 {len(tasks_created)} 个任务")
            except Exception as e:
                self.get_logger().error(
                    f"{handler.__class__.__name__} 处理失败: {e}",
                    throttle_duration_sec=10.0)

    def shutdown(self):
        """优雅地关闭节点"""
        if self.is_shutting_down:
            return

        self.is_shutting_down = True
        self.get_logger().info("🛑 正在优雅地关闭 KUKA WCS 节点...")

        # 停止定时器
        if hasattr(self, 'scan_timer'):
            self.scan_timer.cancel()
            self.get_logger().info("定时器已停止")

        # 关闭资料库连线池
        if hasattr(self, 'db_pool') and self.db_pool:
            try:
                self.db_pool.close_all()
                self.get_logger().info("资料库连线池已关闭")
            except Exception as e:
                self.get_logger().error(f"关闭资料库连线池时出错: {e}")

        self.get_logger().info("✅ KUKA WCS 节点已关闭")


def signal_handler(signum, frame):
    """信号处理器：捕获 SIGINT (Ctrl+C) 和 SIGTERM"""
    print(f"\n接收到信号 {signum}，正在优雅地关闭...")
    rclpy.shutdown()
    sys.exit(0)


def main(args=None):
    """主函数：节点入口点"""
    # 注册信号处理器
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    rclpy.init(args=args)

    try:
        node = KukaWcsNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"节点运行失败: {e}")
    finally:
        if 'node' in locals():
            node.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
