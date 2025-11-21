"""
日誌自動清理服務

負責定期清理過期的日誌記錄，包括：
- RosoutLog (rosout_log)
- RuntimeLog (runtime_log)
- AuditLog (audit_log)

使用批次刪除策略避免長時間鎖表。
"""

from datetime import datetime, timedelta, timezone
from typing import Type, Dict, Any
from sqlmodel import select, delete, SQLModel
from db_proxy.models.rosout_log import RosoutLog
from db_proxy.models.runtime_log import RuntimeLog
from db_proxy.models.audit_log import AuditLog
from db_proxy.connection_pool_manager import ConnectionPoolManager
import time


class LogCleanupService:
    """日誌清理服務"""

    def __init__(
        self,
        connection_pool: ConnectionPoolManager,
        logger,
        retention_days: int = 90,
        batch_size: int = 1000
    ):
        """
        初始化清理服務

        Args:
            connection_pool: 資料庫連線池
            logger: ROS2 logger 實例
            retention_days: 日誌保留天數（預設 90 天）
            batch_size: 每次批次刪除的筆數（預設 1000 筆）
        """
        self.pool = connection_pool
        self.logger = logger
        self.retention_days = retention_days
        self.batch_size = batch_size

        # 清理統計
        self.stats = {
            'rosout_deleted': 0,        # rosout_log 累計刪除數
            'runtime_deleted': 0,       # runtime_log 累計刪除數
            'audit_deleted': 0,         # audit_log 累計刪除數
            'total_deleted': 0,         # 總累計刪除數
            'cleanup_count': 0,         # 清理執行次數
            'last_cleanup_time': None,  # 最後清理時間
            'last_cleanup_duration': 0.0,  # 最後清理耗時(秒)
            'last_cleanup_result': None    # 最後清理結果
        }

        self.logger.info(
            f"📋 LogCleanupService 初始化: "
            f"保留 {retention_days} 天, "
            f"批次大小 {batch_size}"
        )

    def cleanup_old_logs(self) -> Dict[str, Any]:
        """
        清理所有表的過期日誌

        Returns:
            清理統計資訊字典，包含：
            - rosout_deleted: rosout_log 刪除數
            - runtime_deleted: runtime_log 刪除數
            - audit_deleted: audit_log 刪除數
            - total_deleted: 總刪除數
            - duration: 清理耗時(秒)
            - cutoff_time: 清理時間點(ISO格式)
        """
        start_time = time.time()
        cutoff_time = datetime.now(timezone.utc) - timedelta(days=self.retention_days)

        self.logger.info(
            f"🧹 開始清理 {self.retention_days} 天前的日誌 "
            f"(cutoff: {cutoff_time.strftime('%Y-%m-%d %H:%M:%S %Z')})"
        )

        try:
            # 清理各表（依序執行）
            rosout_deleted = self._cleanup_table(RosoutLog, cutoff_time)
            runtime_deleted = self._cleanup_table(RuntimeLog, cutoff_time)
            audit_deleted = self._cleanup_table(AuditLog, cutoff_time)

            # 計算統計
            total_deleted = rosout_deleted + runtime_deleted + audit_deleted
            duration = time.time() - start_time

            # 更新累計統計
            self.stats['rosout_deleted'] += rosout_deleted
            self.stats['runtime_deleted'] += runtime_deleted
            self.stats['audit_deleted'] += audit_deleted
            self.stats['total_deleted'] += total_deleted
            self.stats['cleanup_count'] += 1
            self.stats['last_cleanup_time'] = datetime.now(timezone.utc)
            self.stats['last_cleanup_duration'] = duration

            # 構建結果
            result = {
                'rosout_deleted': rosout_deleted,
                'runtime_deleted': runtime_deleted,
                'audit_deleted': audit_deleted,
                'total_deleted': total_deleted,
                'duration': duration,
                'cutoff_time': cutoff_time.isoformat(),
                'success': True
            }
            self.stats['last_cleanup_result'] = result

            self.logger.info(
                f"✅ 清理完成! "
                f"rosout: {rosout_deleted}, "
                f"runtime: {runtime_deleted}, "
                f"audit: {audit_deleted}, "
                f"總計: {total_deleted} 筆, "
                f"耗時: {duration:.2f}s"
            )

            return result

        except Exception as e:
            duration = time.time() - start_time
            self.logger.error(f"❌ 清理失敗: {e}, 耗時: {duration:.2f}s")
            result = {
                'success': False,
                'error': str(e),
                'duration': duration
            }
            self.stats['last_cleanup_result'] = result
            raise

    def _cleanup_table(self, model: Type[SQLModel], cutoff_time: datetime) -> int:
        """
        批次刪除單一表的過期日誌

        Args:
            model: SQLModel 模型類 (RosoutLog, RuntimeLog, AuditLog)
            cutoff_time: 刪除此時間之前的日誌

        Returns:
            刪除的總筆數
        """
        table_name = model.__tablename__
        total_deleted = 0

        self.logger.info(f"  🗑️  清理表 {table_name}...")

        try:
            while True:
                with self.pool.get_session() as session:
                    # 查詢要刪除的 ID（批次）
                    # 使用 ID 查詢避免在 DELETE 時掃描大量資料
                    stmt = (
                        select(model.id)
                        .where(model.timestamp < cutoff_time)
                        .limit(self.batch_size)
                    )
                    ids_to_delete = session.exec(stmt).all()

                    # 沒有資料需要刪除，結束循環
                    if not ids_to_delete:
                        break

                    # 批次刪除
                    delete_stmt = delete(model).where(model.id.in_(ids_to_delete))
                    session.exec(delete_stmt)
                    session.commit()

                    batch_count = len(ids_to_delete)
                    total_deleted += batch_count

                    self.logger.debug(
                        f"    {table_name}: 已刪除 {batch_count} 筆 "
                        f"(累計: {total_deleted})"
                    )

                    # 如果本次刪除數量小於 batch_size，表示已清理完畢
                    if batch_count < self.batch_size:
                        break

            if total_deleted > 0:
                self.logger.info(f"  ✅ {table_name}: 共刪除 {total_deleted} 筆")
            else:
                self.logger.debug(f"  ℹ️  {table_name}: 無過期資料")

        except Exception as e:
            self.logger.error(f"  ❌ {table_name} 清理失敗: {e}")
            raise

        return total_deleted

    def get_stats(self) -> Dict[str, Any]:
        """
        獲取清理統計資訊

        Returns:
            統計資訊字典（副本）
        """
        stats_copy = self.stats.copy()
        # 將 datetime 轉換為 ISO 字串
        if stats_copy['last_cleanup_time']:
            stats_copy['last_cleanup_time'] = stats_copy['last_cleanup_time'].isoformat()
        return stats_copy

    def reset_stats(self):
        """重置統計資訊（保留配置）"""
        self.stats = {
            'rosout_deleted': 0,
            'runtime_deleted': 0,
            'audit_deleted': 0,
            'total_deleted': 0,
            'cleanup_count': 0,
            'last_cleanup_time': None,
            'last_cleanup_duration': 0.0,
            'last_cleanup_result': None
        }
        self.logger.info("📊 清理統計已重置")
