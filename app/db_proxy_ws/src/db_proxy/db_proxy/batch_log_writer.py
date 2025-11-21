"""
批次日志写入器
用于优化高频日志写入性能，减少数据库事务次数
"""
import threading
import time
from typing import List
from datetime import datetime
from sqlmodel import Session
from db_proxy.models.rosout_log import RosoutLog
from db_proxy.models.runtime_log import RuntimeLog
from db_proxy.connection_pool_manager import ConnectionPoolManager


class BatchLogWriter:
    """
    批次日志写入器

    特性：
    - 时间触发：每 flush_interval 秒自动刷新
    - 数量辅助：达到 batch_size 条立即刷新
    - 线程安全：使用锁保护缓冲区
    - 优雅关闭：节点销毁时自动刷新剩余日志
    - 错误处理：失败时记录错误但不阻塞系统
    """

    def __init__(self,
                 connection_pool: ConnectionPoolManager,
                 logger,
                 batch_size: int = 100,
                 flush_interval: float = 1.0):
        """
        初始化批次日志写入器

        Args:
            connection_pool: 数据库连接池管理器
            logger: ROS 2 logger 实例
            batch_size: 批次大小（达到此数量立即刷新）
            flush_interval: 刷新间隔（秒）
        """
        self.pool = connection_pool
        self.logger = logger
        self.batch_size = batch_size
        self.flush_interval = flush_interval

        # 双缓冲区
        self.rosout_buffer: List[RosoutLog] = []
        self.runtime_buffer: List[RuntimeLog] = []

        # 线程安全锁
        self.lock = threading.Lock()

        # 运行状态
        self.running = True

        # 统计信息
        self.stats = {
            'rosout_received': 0,
            'rosout_written': 0,
            'runtime_received': 0,
            'runtime_written': 0,
            'flush_count': 0,
            'error_count': 0
        }

        # 启动定时刷新线程
        self.timer_thread = threading.Thread(
            target=self._timer_loop,
            daemon=True,
            name="BatchLogWriter-Timer"
        )
        self.timer_thread.start()

        self.logger.info(
            f"✅ BatchLogWriter 已启动 "
            f"(batch_size={batch_size}, flush_interval={flush_interval}s)"
        )

    def add_rosout_log(self, log: RosoutLog):
        """
        添加 rosout 日志到缓冲区

        Args:
            log: RosoutLog 实例
        """
        with self.lock:
            self.rosout_buffer.append(log)
            self.stats['rosout_received'] += 1

            # 达到批次大小立即刷新
            if len(self.rosout_buffer) >= self.batch_size:
                self._flush_rosout_locked()

    def add_runtime_log(self, log: RuntimeLog):
        """
        添加 runtime 日志到缓冲区

        Args:
            log: RuntimeLog 实例
        """
        with self.lock:
            self.runtime_buffer.append(log)
            self.stats['runtime_received'] += 1

            # 达到批次大小立即刷新
            if len(self.runtime_buffer) >= self.batch_size:
                self._flush_runtime_locked()

    def _timer_loop(self):
        """定时刷新循环（在独立线程中运行）"""
        while self.running:
            time.sleep(self.flush_interval)
            if self.running:
                self._flush_all()

    def _flush_all(self):
        """刷新所有缓冲区"""
        with self.lock:
            self._flush_rosout_locked()
            self._flush_runtime_locked()

    def _flush_rosout_locked(self):
        """
        刷新 rosout 缓冲区（必须在锁内调用）

        注意：此方法假设调用者已持有 self.lock
        """
        if not self.rosout_buffer:
            return

        # 复制缓冲区并清空
        logs_to_write = self.rosout_buffer[:]
        self.rosout_buffer = []

        # 在新线程中异步写入（避免阻塞）
        threading.Thread(
            target=self._batch_insert_rosout,
            args=(logs_to_write,),
            daemon=True,
            name=f"BatchLogWriter-RosoutInsert-{int(time.time())}"
        ).start()

    def _flush_runtime_locked(self):
        """
        刷新 runtime 缓冲区（必须在锁内调用）

        注意：此方法假设调用者已持有 self.lock
        """
        if not self.runtime_buffer:
            return

        # 复制缓冲区并清空
        logs_to_write = self.runtime_buffer[:]
        self.runtime_buffer = []

        # 在新线程中异步写入（避免阻塞）
        threading.Thread(
            target=self._batch_insert_runtime,
            args=(logs_to_write,),
            daemon=True,
            name=f"BatchLogWriter-RuntimeInsert-{int(time.time())}"
        ).start()

    def _batch_insert_rosout(self, logs: List[RosoutLog]):
        """
        批次插入 rosout 日志到数据库

        使用 session.add_all() + commit() 实现真正的批次插入：
        - SQLAlchemy 会生成单一的 INSERT 语句
        - 只执行 1 次数据库 query

        Args:
            logs: RosoutLog 实例列表
        """
        if not logs:
            return

        try:
            with self.pool.get_session() as session:
                # ✅ 批次插入：session.add_all() 会生成单一 INSERT
                session.add_all(logs)
                session.commit()

                # 更新统计
                with self.lock:
                    self.stats['rosout_written'] += len(logs)
                    self.stats['flush_count'] += 1

                self.logger.debug(
                    f"✅ 批次写入 {len(logs)} 条 rosout 日志"
                )

        except Exception as e:
            with self.lock:
                self.stats['error_count'] += 1

            self.logger.error(
                f"❌ rosout 批次写入失败: {e}，丢弃 {len(logs)} 条日志"
            )

    def _batch_insert_runtime(self, logs: List[RuntimeLog]):
        """
        批次插入 runtime 日志到数据库

        使用 session.add_all() + commit() 实现真正的批次插入：
        - SQLAlchemy 会生成单一的 INSERT 语句
        - 只执行 1 次数据库 query

        Args:
            logs: RuntimeLog 实例列表
        """
        if not logs:
            return

        try:
            with self.pool.get_session() as session:
                # ✅ 批次插入：session.add_all() 会生成单一 INSERT
                session.add_all(logs)
                session.commit()

                # 更新统计
                with self.lock:
                    self.stats['runtime_written'] += len(logs)
                    self.stats['flush_count'] += 1

                self.logger.debug(
                    f"✅ 批次写入 {len(logs)} 条 runtime 日志"
                )

        except Exception as e:
            with self.lock:
                self.stats['error_count'] += 1

            self.logger.error(
                f"❌ runtime 批次写入失败: {e}，丢弃 {len(logs)} 条日志"
            )

    def get_stats(self) -> dict:
        """
        获取统计信息

        Returns:
            统计信息字典
        """
        with self.lock:
            return self.stats.copy()

    def shutdown(self):
        """
        优雅关闭批次写入器

        流程：
        1. 停止定时器线程
        2. 刷新所有剩余日志
        3. 等待写入线程完成
        """
        self.logger.info("⏸️  BatchLogWriter 正在关闭...")

        # 停止定时器
        self.running = False

        # 等待定时器线程结束
        if self.timer_thread.is_alive():
            self.timer_thread.join(timeout=2.0)

        # 最后刷新所有剩余日志
        self._flush_all()

        # 等待异步写入完成（最多 3 秒）
        time.sleep(0.5)

        # 输出最终统计
        stats = self.get_stats()
        self.logger.info(
            f"✅ BatchLogWriter 已关闭 - 统计: "
            f"rosout({stats['rosout_received']}/{stats['rosout_written']}), "
            f"runtime({stats['runtime_received']}/{stats['runtime_written']}), "
            f"刷新次数={stats['flush_count']}, 错误={stats['error_count']}"
        )
