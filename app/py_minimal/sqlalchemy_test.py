import time
from concurrent.futures import ThreadPoolExecutor
from sqlalchemy import create_engine, text  # 匯入 text 函數
from sqlalchemy.pool import QueuePool
import threading

# 📌 建立 SQLAlchemy 連線池
engine = create_engine(
    "postgresql+psycopg2://webagv:password@192.168.100.254/webagv",
    poolclass=QueuePool,  # 使用 QueuePool 作為連線池
    pool_size=5,          # 🚀 最大 5 條並發連線
    max_overflow=5,      # 🔄 允許最多額外 5 條溢出連線
    pool_timeout=30,      # ⏳ 連線池等待 30 秒
    pool_recycle=180     # 🔄 連線 3 分鐘未使用則回收
)

# 📌 監控連線池狀態的函式
def log_pool_status():
    pool = engine.pool
    total_conn = pool.size()                 # 總連線數
    available_conn = pool.checkedin()       # 可用連線數
    active_conn = pool.checkedout()        # 使用中連線數
    waiting_conn = max(0, pool.overflow())  # 排隊中的請求數（避免負數）

    print(f"📊 總連線數: {total_conn}")
    print(f"✅ 可用連線數: {available_conn}")
    print(f"⏳ 使用中連線數: {active_conn}")
    print(f"🔄 排隊中請求數: {waiting_conn}")
    print('--------------------------------')

# 📌 定時監控連線池狀態
def  monitor_pool():
    while True:
        log_pool_status()
        time.sleep(0.5)  # 每 0.5 秒監控一次

# 📌 查詢函式（模擬長時間查詢）
def connect_and_query(i):
    with engine.connect() as conn:
        print(f"🔹 啟動連線 {i}")
        conn.execute(text("SELECT pg_sleep(10)"))  # 修正為 text() 包裝 SQL 語句
        print(f"✅ 連線 {i} 完成")


# 📌 測試連線池的主要函式
def test_connection_pool():
    print("🚀 開始測試連線池")

    # 🔥 啟動連線池監控的背景執行緒
    monitor_thread = threading.Thread(target=monitor_pool, daemon=True)
    monitor_thread.start()

    # 🔥 使用 ThreadPoolExecutor 執行查詢，每秒增加一條連線
    with ThreadPoolExecutor(max_workers=10) as executor:
        futures = []
        futures.append(executor.submit(connect_and_query, 1))  # 提交查詢任務
        futures.append(executor.submit(connect_and_query, 2))  # 提交查詢任務
        futures.append(executor.submit(connect_and_query, 3))  # 提交查詢任務
        futures.append(executor.submit(connect_and_query, 4))  # 提交查詢任務
        futures.append(executor.submit(connect_and_query, 5))  # 提交查詢任務
        time.sleep(5)  # 每秒啟動一個新連線
        futures.append(executor.submit(connect_and_query, 6))  # 提交查詢任務
        futures.append(executor.submit(connect_and_query, 7))  # 提交查詢任務
        futures.append(executor.submit(connect_and_query, 8))  # 提交查詢任務
        futures.append(executor.submit(connect_and_query, 9))  # 提交查詢任務
        futures.append(executor.submit(connect_and_query, 10))  # 提交查詢任務
        time.sleep(5)  # 每秒啟動一個新連線



        # 等待所有查詢完成
        for future in futures:
            future.result()  # 確保所有任務完成後才繼續

    print("🎉 測試完成")
    log_pool_status()  # 測試結束後顯示連線池狀態

# 🔥 執行測試
test_connection_pool()
