import time
import threading
import rclpy.logging
from sqlalchemy import create_engine
from sqlalchemy.pool import QueuePool
from sqlmodel import SQLModel, Session, select, text
import json
from contextlib import contextmanager

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
            pool_recycle=pool_recycle,
            json_serializer=lambda obj: json.dumps(obj, ensure_ascii=False)
        )
        # self.SessionLocal = sessionmaker(...)  # ❌ 不要再用這個了
        
        # 設定時區 - 只在初始化時執行一次
        if 'postgresql' in str(self.engine.url):
            try:
                with Session(self.engine) as session:
                    # 使用 SQLModel 的 exec 方法來執行 SQL
                    session.exec(text("SET TIME ZONE 'Asia/Taipei'"))
                    session.commit()
                    self.logger.info("✅ 時區設定為 Asia/Taipei")
            except Exception as e:
                self.logger.warning(f"⚠️ 初始化時區設定失敗: {e}")
        
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
        """
        取得標準 session，適用於可能有寫入的操作。
        注意：只讀查詢結束時會自動 rollback，這是正常的！
        PostgreSQL 的 rollback 對只讀事務幾乎沒有成本。
        """
        session = Session(self.engine)
        return session
    
    @contextmanager
    def get_autocommit_session(self):
        """
        取得 autocommit session，適用於純粹的只讀查詢。
        這會完全避免事務，因此不會有 rollback。
        警告：不適合需要事務一致性的多語句查詢！
        """
        # 創建一個 autocommit 連接
        conn = self.engine.connect()
        conn.execution_options(isolation_level="AUTOCOMMIT")
        session = Session(bind=conn)
        try:
            yield session
        finally:
            session.close()
            conn.close()

    def shutdown(self):
        self.monitoring = False
        self.engine.dispose()
        self.logger.info("🔻 連線池已關閉")
