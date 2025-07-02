import time
import threading
import rclpy.logging
from sqlalchemy import create_engine, text
from sqlalchemy.pool import QueuePool
from sqlmodel import SQLModel, Session, select

# 常數定義
POOL_SIZE = 5
MAX_POOL_SIZE = 10
POOL_TIMEOUT = 30
POOL_RECYCLE = 180
LOG_INTERVAL = 5


class ConnectionPoolManager:
    def __init__(self, db_url, pool_size=POOL_SIZE, max_overflow=MAX_POOL_SIZE-POOL_SIZE, pool_timeout=POOL_TIMEOUT, pool_recycle=POOL_RECYCLE):
        self.logger = rclpy.logging.get_logger('db_connection_pool')
        self.engine = create_engine(
            db_url,
            poolclass=QueuePool,
            pool_size=pool_size,
            max_overflow=max_overflow,
            pool_timeout=pool_timeout,
            pool_recycle=pool_recycle
        )
        # self.SessionLocal = sessionmaker(...)  # ❌ 不要再用這個了
        self.create_tables()
        self.monitoring = True
        self.monitor_thread = threading.Thread(
            target=self.monitor_pool, daemon=True)
        self.monitor_thread.start()

    def create_tables(self):
        self.logger.info("🔄 嘗試建立資料表...")
        SQLModel.metadata.create_all(self.engine)
        self.logger.info("✅ 資料表已建立")

    def monitor_pool(self):
        while self.monitoring:
            self.log_pool_status()
            time.sleep(LOG_INTERVAL)

    def log_pool_status(self):
        pool = self.engine.pool
        self.logger.info(
            f"📊 總連線數: {pool.size()} ✅ 可用連線數: {pool.checkedin()} ⏳ 使用中連線數: {pool.checkedout()} 🔄 排隊中請求數: {max(0, pool.overflow())}")

    def get_session(self) -> Session:
        session = Session(self.engine)
        session.exec(text("SET TIME ZONE 'Asia/Taipei'")) # set timezone 為 +8 時區
        return session

    def shutdown(self):
        self.monitoring = False
        self.engine.dispose()
        self.logger.info("🔻 連線池已關閉")
