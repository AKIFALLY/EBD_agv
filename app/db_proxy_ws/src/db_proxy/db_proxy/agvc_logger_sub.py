from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.crud.rosout_log_crud import rosout_log_crud
from db_proxy.crud.runtime_log_crud import runtime_log_crud
from db_proxy.models import RosoutLog, RuntimeLog
from db_proxy.batch_log_writer import BatchLogWriter
from db_proxy.log_cleanup_service import LogCleanupService
from rclpy.node import Node
from rcl_interfaces.msg import Log
from datetime import datetime, timezone
from shared_constants.log_cleanup_config import LogCleanupConfig


class AgvcLogger():
    def __init__(self, node: Node, pool_agvc: ConnectionPoolManager):
        self.pool_agvc = pool_agvc
        self.node = node

        # 初始化批次日志写入器（时间触发 + 数量辅助）
        self.batch_writer = BatchLogWriter(
            connection_pool=pool_agvc,
            logger=node.get_logger(),
            batch_size=100,      # 达到 100 条立即刷新
            flush_interval=1.0   # 每 1 秒自动刷新
        )

        # 訂閱 ROSOUT 的訊息（已启用批次写入）
        node.create_subscription(Log, "/rosout", self.rosout_callback, 30)
        node.get_logger().info("✅ ROSOUT Logger 已啟動，開始訂閱 /rosout（批次写入模式）")

        # 訂閱 Runtime Log 的訊息（已启用批次写入）
        node.create_subscription(
            Log, "/runtime_log", self.runtime_log_callback, 30)
        node.get_logger().info("✅ Runtime Logger 已啟動，開始訂閱 /runtime_log（批次写入模式）")

        # 初始化日誌清理服務
        if LogCleanupConfig.ENABLED:
            self.cleanup_service = LogCleanupService(
                connection_pool=pool_agvc,
                logger=node.get_logger(),
                retention_days=LogCleanupConfig.RETENTION_DAYS,
                batch_size=LogCleanupConfig.BATCH_SIZE
            )

            # 記錄當日是否已執行清理（避免重複執行）
            self.last_cleanup_date = None

            # 創建定時器：每小時檢查一次是否需要清理
            self.cleanup_timer = node.create_timer(
                LogCleanupConfig.CLEANUP_CHECK_INTERVAL,
                self.check_and_cleanup
            )
            node.get_logger().info(
                f"✅ 日誌自動清理已啟用: "
                f"保留 {LogCleanupConfig.RETENTION_DAYS} 天, "
                f"每天 {LogCleanupConfig.CLEANUP_HOUR:02d}:00 執行清理"
            )
        else:
            self.cleanup_service = None
            node.get_logger().info("ℹ️  日誌自動清理已停用")

    def rosout_callback(self, msg: Log):
        """處理 ROSOUT 的訊息（批次写入模式）"""
        # 轉換 ROS2 時間為 datetime
        timestamp = datetime.fromtimestamp(
            msg.stamp.sec + msg.stamp.nanosec / 1e9,
            tz=timezone.utc
        )

        # 创建 RosoutLog 对象
        log = RosoutLog(
            timestamp=timestamp,
            level=msg.level,
            name=msg.name,
            message=msg.msg,
            file=msg.file,
            function=msg.function,
            line=msg.line
        )

        # ✅ 加入批次缓冲区（不立即写入数据库）
        self.batch_writer.add_rosout_log(log)

    def runtime_log_callback(self, msg: Log):
        """處理 Runtime Logger 的訊息（批次写入模式）"""
        # 轉換 ROS2 時間為 datetime
        timestamp = datetime.fromtimestamp(
            msg.stamp.sec + msg.stamp.nanosec / 1e9,
            tz=timezone.utc
        )

        # 创建 RuntimeLog 对象
        log = RuntimeLog(
            timestamp=timestamp,
            level=msg.level,
            name=msg.name,
            message=msg.msg,
            file=msg.file,
            function=msg.function,
            line=msg.line
        )

        # ✅ 加入批次缓冲区（不立即写入数据库）
        self.batch_writer.add_runtime_log(log)

    def check_and_cleanup(self):
        """
        定時檢查並執行日誌清理

        每小時執行一次，檢查當前時間是否為清理時間（預設凌晨 2:00）
        且當日尚未執行清理，則觸發清理操作。
        """
        if not self.cleanup_service:
            return

        now = datetime.now()

        # 檢查是否為清理時間（凌晨 2:00-2:59）且當日未執行
        if now.hour == LogCleanupConfig.CLEANUP_HOUR and now.date() != self.last_cleanup_date:
            self.node.get_logger().info(
                f"⏰ 觸發每日日誌清理 ({now.strftime('%Y-%m-%d %H:%M:%S')})"
            )

            try:
                # 執行清理
                result = self.cleanup_service.cleanup_old_logs()

                # 記錄清理完成
                self.last_cleanup_date = now.date()

                # 輸出清理結果
                if result['success']:
                    self.node.get_logger().info(
                        f"✅ 每日清理成功: "
                        f"rosout={result['rosout_deleted']}, "
                        f"runtime={result['runtime_deleted']}, "
                        f"audit={result['audit_deleted']}, "
                        f"總計={result['total_deleted']} 筆"
                    )
                else:
                    self.node.get_logger().error(
                        f"❌ 每日清理失敗: {result.get('error', 'Unknown error')}"
                    )

            except Exception as e:
                self.node.get_logger().error(f"❌ 日誌清理異常: {e}")
        else:
            # Debug: 記錄檢查狀態（可選）
            if LogCleanupConfig.LOG_CLEANUP_DEBUG:
                self.node.get_logger().debug(
                    f"🔍 清理檢查: hour={now.hour}, "
                    f"target={LogCleanupConfig.CLEANUP_HOUR}, "
                    f"last_cleanup={self.last_cleanup_date}"
                )

    def shutdown(self):
        """优雅关闭批次写入器和清理服務（刷新剩余日志）"""
        # 關閉批次寫入器
        if hasattr(self, 'batch_writer'):
            self.batch_writer.shutdown()

        # 輸出清理服務統計
        if hasattr(self, 'cleanup_service') and self.cleanup_service:
            stats = self.cleanup_service.get_stats()
            self.node.get_logger().info(
                f"📊 日誌清理統計: "
                f"執行次數={stats['cleanup_count']}, "
                f"累計刪除={stats['total_deleted']} 筆 "
                f"(rosout={stats['rosout_deleted']}, "
                f"runtime={stats['runtime_deleted']}, "
                f"audit={stats['audit_deleted']})"
            )
