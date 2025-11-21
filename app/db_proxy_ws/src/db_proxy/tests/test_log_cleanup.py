"""
日誌清理服務單元測試

測試 LogCleanupService 的核心功能，包括：
- 過期日誌識別
- 批次刪除邏輯
- 統計資料正確性
"""

import pytest
from datetime import datetime, timedelta, timezone
from unittest.mock import Mock, MagicMock
from sqlmodel import Session, create_engine, SQLModel, select

from db_proxy.log_cleanup_service import LogCleanupService
from db_proxy.models.rosout_log import RosoutLog
from db_proxy.models.runtime_log import RuntimeLog
from db_proxy.models.audit_log import AuditLog
from db_proxy.connection_pool_manager import ConnectionPoolManager


class TestLogCleanupService:
    """LogCleanupService 測試套件"""

    @pytest.fixture
    def mock_logger(self):
        """建立模擬 logger"""
        logger = Mock()
        logger.info = Mock()
        logger.debug = Mock()
        logger.error = Mock()
        return logger

    @pytest.fixture
    def test_engine(self):
        """建立測試用記憶體資料庫引擎"""
        engine = create_engine("sqlite:///:memory:")
        SQLModel.metadata.create_all(engine)
        return engine

    @pytest.fixture
    def mock_pool(self, test_engine):
        """建立模擬連線池"""
        pool = Mock(spec=ConnectionPoolManager)

        # 模擬 get_session 返回真實的 session
        def get_session_context():
            session = Session(test_engine)
            return MagicMock(__enter__=lambda self: session, __exit__=lambda self, *args: session.close())

        pool.get_session = get_session_context
        return pool

    @pytest.fixture
    def cleanup_service(self, mock_pool, mock_logger):
        """建立清理服務實例"""
        return LogCleanupService(
            connection_pool=mock_pool,
            logger=mock_logger,
            retention_days=30,
            batch_size=100
        )

    def test_init(self, cleanup_service, mock_logger):
        """測試初始化"""
        assert cleanup_service.retention_days == 30
        assert cleanup_service.batch_size == 100
        assert cleanup_service.stats['cleanup_count'] == 0
        assert cleanup_service.stats['total_deleted'] == 0
        mock_logger.info.assert_called_once()

    def test_cleanup_empty_database(self, cleanup_service):
        """測試清理空資料庫"""
        result = cleanup_service.cleanup_old_logs()

        assert result['success'] is True
        assert result['rosout_deleted'] == 0
        assert result['runtime_deleted'] == 0
        assert result['audit_deleted'] == 0
        assert result['total_deleted'] == 0

    def test_cleanup_with_old_logs(self, cleanup_service, mock_pool, test_engine):
        """測試清理過期日誌"""
        # 插入測試資料
        with Session(test_engine) as session:
            # 35 天前的日誌（應被刪除）
            old_time = datetime.now(timezone.utc) - timedelta(days=35)
            for i in range(5):
                log = RosoutLog(
                    timestamp=old_time,
                    level=20,
                    name=f"test_node_{i}",
                    message=f"Old log {i}"
                )
                session.add(log)

            # 20 天前的日誌（應保留）
            recent_time = datetime.now(timezone.utc) - timedelta(days=20)
            for i in range(3):
                log = RosoutLog(
                    timestamp=recent_time,
                    level=20,
                    name=f"test_node_recent_{i}",
                    message=f"Recent log {i}"
                )
                session.add(log)

            session.commit()

        # 執行清理
        result = cleanup_service.cleanup_old_logs()

        # 驗證結果
        assert result['success'] is True
        assert result['rosout_deleted'] == 5  # 只刪除 35 天前的 5 筆
        assert result['total_deleted'] == 5

        # 驗證資料庫狀態
        with Session(test_engine) as session:
            remaining_logs = session.exec(select(RosoutLog)).all()
            assert len(remaining_logs) == 3  # 只剩下 20 天前的 3 筆
            for log in remaining_logs:
                assert log.timestamp >= datetime.now(timezone.utc) - timedelta(days=30)

    def test_cleanup_multiple_tables(self, cleanup_service, mock_pool, test_engine):
        """測試清理多個表"""
        with Session(test_engine) as session:
            old_time = datetime.now(timezone.utc) - timedelta(days=35)

            # RosoutLog
            for i in range(3):
                session.add(RosoutLog(
                    timestamp=old_time,
                    level=20,
                    name="test",
                    message=f"rosout {i}"
                ))

            # RuntimeLog
            for i in range(2):
                session.add(RuntimeLog(
                    timestamp=old_time,
                    level=20,
                    name="test",
                    message=f"runtime {i}"
                ))

            # AuditLog
            for i in range(4):
                session.add(AuditLog(
                    timestamp=old_time,
                    operation_id=f"op_{i}",
                    operation_type="test",
                    resource_type="test"
                ))

            session.commit()

        # 執行清理
        result = cleanup_service.cleanup_old_logs()

        # 驗證結果
        assert result['success'] is True
        assert result['rosout_deleted'] == 3
        assert result['runtime_deleted'] == 2
        assert result['audit_deleted'] == 4
        assert result['total_deleted'] == 9

    def test_batch_deletion(self, mock_pool, mock_logger, test_engine):
        """測試批次刪除（batch_size=10）"""
        # 建立小批次的清理服務
        cleanup_service = LogCleanupService(
            connection_pool=mock_pool,
            logger=mock_logger,
            retention_days=30,
            batch_size=10  # 小批次
        )

        # 插入 25 筆過期日誌
        with Session(test_engine) as session:
            old_time = datetime.now(timezone.utc) - timedelta(days=35)
            for i in range(25):
                session.add(RosoutLog(
                    timestamp=old_time,
                    level=20,
                    name="test",
                    message=f"log {i}"
                ))
            session.commit()

        # 執行清理
        result = cleanup_service.cleanup_old_logs()

        # 驗證結果（應分 3 批：10+10+5）
        assert result['rosout_deleted'] == 25
        assert result['total_deleted'] == 25

        # 驗證資料庫已清空
        with Session(test_engine) as session:
            count = len(session.exec(select(RosoutLog)).all())
            assert count == 0

    def test_get_stats(self, cleanup_service):
        """測試統計資訊"""
        # 初始統計
        stats = cleanup_service.get_stats()
        assert stats['cleanup_count'] == 0
        assert stats['total_deleted'] == 0

        # 模擬清理（更新內部統計）
        cleanup_service.stats['cleanup_count'] = 2
        cleanup_service.stats['rosout_deleted'] = 100
        cleanup_service.stats['total_deleted'] = 150

        # 取得統計
        stats = cleanup_service.get_stats()
        assert stats['cleanup_count'] == 2
        assert stats['rosout_deleted'] == 100
        assert stats['total_deleted'] == 150

    def test_reset_stats(self, cleanup_service, mock_logger):
        """測試重置統計"""
        # 設定初始值
        cleanup_service.stats['cleanup_count'] = 5
        cleanup_service.stats['total_deleted'] = 200

        # 重置
        cleanup_service.reset_stats()

        # 驗證重置
        stats = cleanup_service.get_stats()
        assert stats['cleanup_count'] == 0
        assert stats['total_deleted'] == 0
        mock_logger.info.assert_called()

    def test_error_handling(self, mock_pool, mock_logger):
        """測試錯誤處理"""
        # 建立會拋出異常的連線池
        def failing_session():
            raise Exception("Database connection failed")

        mock_pool.get_session = failing_session

        cleanup_service = LogCleanupService(
            connection_pool=mock_pool,
            logger=mock_logger,
            retention_days=30,
            batch_size=100
        )

        # 執行清理應該拋出異常
        with pytest.raises(Exception):
            cleanup_service.cleanup_old_logs()

        # 驗證錯誤日誌
        assert cleanup_service.stats['last_cleanup_result']['success'] is False


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
