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
import yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory


class KukaWcsNode(Node):
    """KUKA WCS 主节点"""

    def __init__(self):
        super().__init__("kuka_wcs_node")
        self.get_logger().info("KUKA WCS 节点启动中...")

        # 标记节点是否正在关闭
        self.is_shutting_down = False

        # 载入配置文件
        self.config = self._load_config()

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

    def _load_config(self):
        """
        载入 YAML 配置文件

        Returns:
            配置字典
        """
        try:
            # 获取包的 share 目录
            package_dir = get_package_share_directory('kuka_wcs')
            config_path = Path(package_dir) / 'config' / 'kuka_wcs_config.yaml'

            # 如果找不到，尝试开发目录
            if not config_path.exists():
                config_path = Path(__file__).parent.parent / 'config' / 'kuka_wcs_config.yaml'

            self.get_logger().info(f"载入配置文件: {config_path}")

            with open(config_path, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)

            self.get_logger().info("✅ 配置文件载入成功")
            return config

        except Exception as e:
            self.get_logger().error(f"载入配置文件失败: {e}")
            # 返回默认配置
            return {
                'kuka_wcs': {
                    'rack_rotation': {
                        'enabled': False,
                        'rooms': {}
                    }
                }
            }

    def _init_handlers(self):
        """初始化所有任务处理器"""
        from kuka_wcs.task_handlers.rack_rotation_handler import RackRotationHandler
        from kuka_wcs.task_handlers.system_area_handler import SystemAreaHandler

        self.handlers = []

        # 根据配置初始化 RackRotationHandler（每个 room 独立）
        rotation_config = self.config.get('kuka_wcs', {}).get('rack_rotation', {})
        if rotation_config.get('enabled', False):
            rooms = rotation_config.get('rooms', {})
            work_id = rotation_config.get('work_id', 220001)
            b_side_threshold = rotation_config.get('b_side_threshold', 12)
            a_side_threshold = rotation_config.get('a_side_threshold', 0)

            for room_id, room_config in rooms.items():
                # 将全局配置合并到 room 配置中
                full_config = {
                    **room_config,
                    'work_id': work_id,
                    'b_side_threshold': b_side_threshold,
                    'a_side_threshold': a_side_threshold,
                }
                handler = RackRotationHandler(self, int(room_id), full_config)
                self.handlers.append(handler)
                self.get_logger().info(f"✅ 已初始化 Room {room_id} 的 RackRotationHandler")
        else:
            self.get_logger().warn("RackRotationHandler 未启用（配置文件中 enabled=false）")

        # 根据配置初始化 SystemAreaHandler（系统区域监控）
        # ⚠️ 注意：满料回收功能已整合到 SystemAreaHandler 的流程5中
        monitor_config = self.config.get('kuka_wcs', {}).get('system_area_monitor', {})
        if monitor_config.get('enabled', False):
            handler = SystemAreaHandler(self, monitor_config)
            self.handlers.append(handler)
            self.get_logger().info("✅ 已初始化 SystemAreaHandler（系统区域监控 + 满料回收）")
        else:
            self.get_logger().warn("SystemAreaHandler 未启用（配置文件中 enabled=false）")

        # 根据配置初始化 SpecialHandler（特殊任务处理器）
        from kuka_wcs.task_handlers.special_handler import SpecialHandler
        special_config = self.config.get('kuka_wcs', {}).get('special_handler', {})
        if special_config.get('enabled', False):
            handler = SpecialHandler(self, special_config)
            self.handlers.append(handler)
            self.get_logger().info("✅ 已初始化 SpecialHandler（特殊任务处理器）")
        else:
            self.get_logger().warn("SpecialHandler 未启用（配置文件中 enabled=false）")

        self.get_logger().info(f"已初始化 {len(self.handlers)} 个任务处理器")

    def scan_and_create_tasks(self):
        """定时扫描：检查所有处理器并创建任务、标记完成任务"""
        if self.is_shutting_down:
            return

        if not self.db_pool:
            self.get_logger().warn("资料库连线池未初始化，跳过扫描")
            return

        self.get_logger().debug("1秒定时器触发 (scan_timer)")

        # 1. 遍历所有处理器，让它们检查条件并创建任务
        for handler in self.handlers:
            try:
                with self.db_pool.get_session() as session:
                    tasks_created = handler.check_and_create_tasks(session)
                    if tasks_created:
                        self.get_logger().info(
                            f"{handler.__class__.__name__} 创建了 {len(tasks_created)} 个任务")
            except Exception as e:
                self.get_logger().error(
                    f"{handler.__class__.__name__} 创建任务失败: {e}",
                    throttle_duration_sec=10.0)

        # 2. 遍历所有处理器，让它们检查并标记已完成的任务
        for handler in self.handlers:
            try:
                with self.db_pool.get_session() as session:
                    marked_count = handler.check_and_mark_completed_tasks(session)
                    if marked_count > 0:
                        self.get_logger().info(
                            f"{handler.__class__.__name__} 标记了 {marked_count} 个任务为完成")
            except Exception as e:
                self.get_logger().error(
                    f"{handler.__class__.__name__} 标记任务完成失败: {e}",
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
