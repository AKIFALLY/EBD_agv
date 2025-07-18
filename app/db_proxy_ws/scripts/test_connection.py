#!/usr/bin/env python3

"""
=============================================================================
PostgreSQL 連線測試腳本
=============================================================================
用途：測試 PostgreSQL 資料庫連線和 ConnectionPoolManager 功能
測試項目：
  - psycopg2 直接連線測試
  - ConnectionPoolManager 連線池測試
  - SQLModel 模型載入測試
  - 基本 SQL 查詢測試

使用方法：
  python3 test_connection.py

基於：docker-compose.agvc.yml 配置
=============================================================================
"""

import sys
import os
from datetime import datetime

# 設定顏色輸出
class Colors:
    RED = '\033[0;31m'
    GREEN = '\033[0;32m'
    YELLOW = '\033[1;33m'
    BLUE = '\033[0;34m'
    NC = '\033[0m'  # No Color

# 資料庫連線參數 (基於 docker-compose.agvc.yml)
DB_CONFIG = {
    'host': '192.168.100.254',
    'port': 5432,
    'database': 'agvc',
    'user': 'agvc',
    'password': 'password'
}

TEST_DB_CONFIG = {
    'host': '192.168.100.254',
    'port': 5432,
    'database': 'test_db',
    'user': 'agvc',
    'password': 'password'
}

def print_header(title):
    """列印標題"""
    print(f"{Colors.BLUE}=== {title} ==={Colors.NC}")

def print_success(message):
    """列印成功訊息"""
    print(f"{Colors.GREEN}✅ {message}{Colors.NC}")

def print_error(message):
    """列印錯誤訊息"""
    print(f"{Colors.RED}❌ {message}{Colors.NC}")

def print_warning(message):
    """列印警告訊息"""
    print(f"{Colors.YELLOW}⚠️ {message}{Colors.NC}")

def test_psycopg2_connection():
    """測試 psycopg2 直接連線"""
    print_header("psycopg2 直接連線測試")
    
    try:
        import psycopg2
        print_success("psycopg2 模組載入成功")
    except ImportError as e:
        print_error(f"psycopg2 模組載入失敗: {e}")
        return False
    
    # 測試生產資料庫連線
    try:
        conn = psycopg2.connect(**DB_CONFIG)
        cursor = conn.cursor()
        cursor.execute("SELECT current_user, current_database(), version();")
        result = cursor.fetchone()
        print_success(f"生產資料庫連線成功")
        print(f"  使用者: {result[0]}")
        print(f"  資料庫: {result[1]}")
        print(f"  版本: {result[2][:50]}...")
        cursor.close()
        conn.close()
    except Exception as e:
        print_error(f"生產資料庫連線失敗: {e}")
        return False
    
    # 測試測試資料庫連線
    try:
        conn = psycopg2.connect(**TEST_DB_CONFIG)
        cursor = conn.cursor()
        cursor.execute("SELECT current_user, current_database();")
        result = cursor.fetchone()
        print_success(f"測試資料庫連線成功")
        print(f"  使用者: {result[0]}")
        print(f"  資料庫: {result[1]}")
        cursor.close()
        conn.close()
    except Exception as e:
        print_error(f"測試資料庫連線失敗: {e}")
        return False
    
    return True

def test_sqlmodel_import():
    """測試 SQLModel 模型載入"""
    print_header("SQLModel 模型載入測試")
    
    try:
        # 新增 db_proxy 模組路徑
        sys.path.insert(0, '/app/db_proxy_ws/src/db_proxy')
        
        from db_proxy.models import Task, Work, Rack, Carrier, AGV
        print_success("SQLModel 模型載入成功")
        
        # 檢查模型資料表名稱
        models = [Task, Work, Rack, Carrier, AGV]
        for model in models:
            if hasattr(model, '__tablename__'):
                print(f"  {model.__name__} -> {model.__tablename__}")
            else:
                print(f"  {model.__name__} -> (無資料表名稱)")
        
        return True
    except ImportError as e:
        print_error(f"SQLModel 模型載入失敗: {e}")
        return False
    except Exception as e:
        print_error(f"模型檢查失敗: {e}")
        return False

def test_connection_pool():
    """測試 ConnectionPoolManager"""
    print_header("ConnectionPoolManager 測試")

    # 新增 db_proxy 模組路徑
    sys.path.insert(0, '/app/db_proxy_ws/src/db_proxy')

    # 嘗試 ROS 2 環境測試
    try:
        import os
        # 設定 RMW 實作為標準版本
        os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'

        # 測試 rclpy 載入
        import rclpy
        print_success("rclpy 模組載入成功")

        from db_proxy.connection_pool_manager import ConnectionPoolManager
        print_success("ConnectionPoolManager 模組載入成功")

        return _test_connection_pool_with_ros2(ConnectionPoolManager, rclpy)

    except ImportError as e:
        print_warning(f"ROS 2 環境不可用: {e}")
        print_warning("嘗試簡化版本的 SQLAlchemy 連線池測試...")
        return _test_sqlalchemy_pool_only()
    except Exception as e:
        print_error(f"ROS 2 環境初始化失敗: {e}")
        print_warning("嘗試簡化版本的 SQLAlchemy 連線池測試...")
        return _test_sqlalchemy_pool_only()

def _test_sqlalchemy_pool_only():
    """簡化版本的連線池測試，不依賴 ROS 2"""
    try:
        from sqlalchemy import create_engine, text
        from sqlalchemy.pool import QueuePool
        print_success("SQLAlchemy 模組載入成功")

        # 建立 SQLAlchemy 引擎和連線池
        db_url = f"postgresql+psycopg2://{DB_CONFIG['user']}:{DB_CONFIG['password']}@{DB_CONFIG['host']}:{DB_CONFIG['port']}/{DB_CONFIG['database']}"
        engine = create_engine(
            db_url,
            poolclass=QueuePool,
            pool_size=5,
            max_overflow=5,
            pool_timeout=30,
            pool_recycle=180
        )
        print_success("SQLAlchemy 連線池建立成功")

        # 測試連線
        with engine.connect() as conn:
            result = conn.execute(text("SELECT current_user, current_database(), COUNT(*) FROM information_schema.tables WHERE table_schema = 'public';")).fetchone()
            print_success("SQLAlchemy 連線池測試成功")
            print(f"  使用者: {result[0]}")
            print(f"  資料庫: {result[1]}")
            print(f"  公開資料表數量: {result[2]}")

        # 關閉引擎
        engine.dispose()
        print_success("SQLAlchemy 連線池關閉成功")
        return True

    except Exception as e:
        print_error(f"SQLAlchemy 連線池測試失敗: {e}")
        return False

def _test_connection_pool_with_ros2(ConnectionPoolManager, rclpy):
    """使用 ROS 2 的完整 ConnectionPoolManager 測試"""
    
    # 建立連線池
    db_url = f"postgresql+psycopg2://{DB_CONFIG['user']}:{DB_CONFIG['password']}@{DB_CONFIG['host']}:{DB_CONFIG['port']}/{DB_CONFIG['database']}"

    try:
        # 初始化 ROS 2 (如果需要)
        try:
            rclpy.init()
            print_success("ROS 2 初始化成功")
        except Exception as ros_e:
            print_warning(f"ROS 2 初始化警告: {ros_e}")

        pool = ConnectionPoolManager(db_url)
        print_success("連線池建立成功")

        # 測試 Session
        with pool.get_session() as session:
            from sqlalchemy import text
            result = session.execute(text("SELECT current_user, current_database(), COUNT(*) FROM information_schema.tables WHERE table_schema = 'public';")).fetchone()
            print_success("Session 操作成功")
            print(f"  使用者: {result[0]}")
            print(f"  資料庫: {result[1]}")
            print(f"  公開資料表數量: {result[2]}")

        # 關閉連線池
        pool.shutdown()
        print_success("連線池關閉成功")

        # 清理 ROS 2
        try:
            rclpy.shutdown()
            print_success("ROS 2 清理成功")
        except Exception:
            pass

        return True

    except Exception as e:
        print_error(f"ConnectionPoolManager 測試失敗: {e}")
        # 嘗試清理 ROS 2
        try:
            rclpy.shutdown()
        except Exception:
            pass
        return False

def test_database_tables():
    """測試資料庫資料表"""
    print_header("資料庫資料表檢查")
    
    try:
        import psycopg2
        conn = psycopg2.connect(**DB_CONFIG)
        cursor = conn.cursor()
        
        # 檢查資料表
        cursor.execute("""
            SELECT table_name, table_type 
            FROM information_schema.tables 
            WHERE table_schema = 'public' 
            ORDER BY table_name;
        """)
        tables = cursor.fetchall()
        
        if tables:
            print_success(f"找到 {len(tables)} 個資料表")
            for table_name, table_type in tables[:10]:  # 只顯示前10個
                print(f"  {table_name} ({table_type})")
            if len(tables) > 10:
                print(f"  ... 還有 {len(tables) - 10} 個資料表")
        else:
            print_warning("沒有找到資料表，可能需要執行 db_install")
        
        cursor.close()
        conn.close()
        return True
        
    except Exception as e:
        print_error(f"資料表檢查失敗: {e}")
        return False

def main():
    """主函數"""
    print_header("PostgreSQL 連線測試")
    print(f"測試時間: {datetime.now()}")
    print()
    
    # 執行所有測試
    tests = [
        ("psycopg2 連線", test_psycopg2_connection),
        ("SQLModel 模型", test_sqlmodel_import),
        ("連線池管理", test_connection_pool),
        ("資料庫資料表", test_database_tables),
    ]
    
    results = []
    for test_name, test_func in tests:
        try:
            result = test_func()
            results.append((test_name, result))
        except Exception as e:
            print_error(f"{test_name} 測試發生異常: {e}")
            results.append((test_name, False))
        print()
    
    # 總結
    print_header("測試總結")
    passed = 0
    for test_name, result in results:
        if result:
            print_success(f"{test_name}: 通過")
            passed += 1
        else:
            print_error(f"{test_name}: 失敗")
    
    print()
    print(f"測試結果: {passed}/{len(results)} 通過")
    
    if passed == len(results):
        print_success("所有測試通過！資料庫連線正常")
        return 0
    else:
        print_error("部分測試失敗，請檢查配置")
        return 1

if __name__ == "__main__":
    sys.exit(main())
