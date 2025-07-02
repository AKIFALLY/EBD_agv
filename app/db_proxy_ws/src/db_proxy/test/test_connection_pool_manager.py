import pytest
from unittest.mock import patch, MagicMock
from db_proxy.connection_pool_manager import ConnectionPoolManager
from sqlmodel import SQLModel, Field, select

# 測試用模型


class SampleModel(SQLModel, table=True):
    id: int | None = Field(default=None, primary_key=True)
    name: str


@pytest.fixture
def db_url():
    # 使用 SQLite 記憶體資料庫進行測試（不會有持久化）
    return "sqlite:///:memory:"


@pytest.fixture
def conn_pool(db_url):
    # mock 掉 rclpy logger，避免需要 ROS 初始化
    with patch("rclpy.logging.get_logger") as mock_logger:
        mock_logger.return_value = MagicMock()
        pool = ConnectionPoolManager(db_url)
        yield pool
        pool.shutdown()


def test_create_tables(conn_pool):
    # 建立測試資料表
    SQLModel.metadata.create_all(conn_pool.engine)

    # 插入資料
    with conn_pool.get_session() as session:
        item = SampleModel(name="測試項目")
        session.add(item)
        session.commit()

    # 查詢確認
    with conn_pool.get_session() as session:
        result = session.exec(select(SampleModel).where(
            SampleModel.name == "測試項目")).first()
        assert result is not None
        assert result.name == "測試項目"


def test_pool_status_logging(conn_pool):
    # 確認 log_pool_status 不會拋錯
    conn_pool.log_pool_status()
