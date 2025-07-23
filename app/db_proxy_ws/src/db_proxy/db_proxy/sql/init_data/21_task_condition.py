"""
初始化任務條件相關資料表
包含 TaskCondition、TaskConditionHistory 和 TaskConditionCache
"""

from sqlmodel import Session
from db_proxy.models.task_condition_history import (
    TaskCondition, TaskConditionHistory, TaskConditionCache
)


def init_task_condition_tables(session: Session):
    """
    初始化任務條件相關資料表

    Args:
        session: 資料庫會話
    """
    print("🔧 初始化任務條件相關資料表...")

    # 這些表格會由 SQLModel 自動創建，這裡主要是確保表格存在

    try:
        # 檢查 task_condition 表格是否存在
        session.exec("SELECT 1 FROM task_condition LIMIT 1").first()
        print("✅ task_condition 表格已存在")
    except Exception:
        print("📋 task_condition 表格將由 SQLModel 自動創建")

    try:
        # 檢查 task_condition_history 表格是否存在
        session.exec("SELECT 1 FROM task_condition_history LIMIT 1").first()
        print("✅ task_condition_history 表格已存在")
    except Exception:
        print("📋 task_condition_history 表格將由 SQLModel 自動創建")

    try:
        # 檢查 task_condition_cache 表格是否存在
        session.exec("SELECT 1 FROM task_condition_cache LIMIT 1").first()
        print("✅ task_condition_cache 表格已存在")
    except Exception:
        print("📋 task_condition_cache 表格將由 SQLModel 自動創建")

    # 創建必要的索引（如果不存在）
    create_indexes(session)

    # 初始化一些範例資料（可選）
    init_sample_data(session)

    print("✅ 任務條件相關資料表初始化完成")


def create_indexes(session: Session):
    """
    創建必要的索引以提升查詢效能 - 簡化版本

    Args:
        session: 資料庫會話
    """
    indexes = [
        # task_condition_history 表的索引
        "CREATE INDEX IF NOT EXISTS idx_task_condition_history_condition_key ON task_condition_history(condition_key)",
        "CREATE INDEX IF NOT EXISTS idx_task_condition_history_condition_name ON task_condition_history(condition_name)",
        "CREATE INDEX IF NOT EXISTS idx_task_condition_history_expires_at ON task_condition_history(expires_at)",
        "CREATE INDEX IF NOT EXISTS idx_task_condition_history_updated_at ON task_condition_history(updated_at)",

        # task_condition_cache 表的索引
        "CREATE INDEX IF NOT EXISTS idx_task_condition_cache_cache_key ON task_condition_cache(cache_key)",
        "CREATE INDEX IF NOT EXISTS idx_task_condition_cache_cache_group ON task_condition_cache(cache_group)",
        "CREATE INDEX IF NOT EXISTS idx_task_condition_cache_expires_at ON task_condition_cache(expires_at)",
    ]
    
    from sqlmodel import text

    for index_sql in indexes:
        try:
            session.exec(text(index_sql))
            print(f"✅ 索引創建成功: {index_sql.split('idx_')[1].split(' ')[0] if 'idx_' in index_sql else 'unknown'}")
        except Exception as e:
            print(f"⚠️ 索引創建失敗或已存在: {e}")
    
    session.commit()


def init_sample_data(session: Session):
    """
    初始化範例資料
    先清除所有欄位，再插入 sample_conditions 的內容

    Args:
        session: 資料庫會話
    """
    print("🔄 開始初始化 task_condition 範例資料...")

    try:
        # 1. 先清除所有欄位
        print("🧹 清除 task_condition 表格中的所有資料...")
        try:
            from sqlmodel import text
            # 清除 task_condition 表格的所有資料
            session.exec(text("DELETE FROM task_condition"))
            print("✅ task_condition 表格資料清除完成")
        except Exception as clear_error:
            print(f"⚠️ 清除資料時發生錯誤: {clear_error}")
            # 如果清除失敗，繼續嘗試插入資料

        # 2. 從外部檔案載入範例條件記錄
        try:
            from .task_condition_samples import sample_conditions
            print(f"✅ 成功載入 {len(sample_conditions)} 筆範例條件資料")
        except ImportError as import_error:
            print(f"❌ 無法載入範例資料檔案: {import_error}")
            print("📋 使用預設的空範例資料列表")
            sample_conditions = []

        # 3. 插入 sample_conditions 的內容
        print(f"🔄 準備插入 {len(sample_conditions)} 筆範例資料...")

        for i, condition_data in enumerate(sample_conditions):
            try:
                print(f"🔄 處理第 {i+1}/{len(sample_conditions)} 筆資料...")

                # 檢查資料格式
                if "conditions" not in condition_data:
                    print(f"⚠️ 第 {i+1} 筆資料缺少 conditions 欄位，跳過")
                    continue

                if "results" not in condition_data:
                    print(f"⚠️ 第 {i+1} 筆資料缺少 results 欄位，跳過")
                    continue

                # 創建 TaskCondition 物件
                task_condition = TaskCondition(
                    id=condition_data["id"],
                    conditions=condition_data["conditions"],
                    results=condition_data["results"],
                    description=condition_data.get("description")
                )

                # 添加到 session
                session.add(task_condition)
                print(f"✅ 第 {i+1} 筆資料處理完成")

            except Exception as item_error:
                print(f"❌ 處理第 {i+1} 筆資料時發生錯誤: {item_error}")
                # 繼續處理下一筆資料
                continue

        try:
            # 提交所有變更
            print("🔄 提交所有變更到資料庫...")
            session.commit()
            print(f"✅ 已成功初始化 {len(sample_conditions)} 筆 task_condition 範例資料")
        except Exception as commit_error:
            print(f"❌ 提交變更時發生錯誤: {commit_error}")
            session.rollback()

    except Exception as e:
        print(f"⚠️ 範例資料初始化失敗: {e}")
        session.rollback()


def cleanup_old_records(session: Session, days_to_keep: int = 7):
    """
    清理舊的記錄以節省空間 - 簡化版本

    Args:
        session: 資料庫會話
        days_to_keep: 保留的天數
    """
    from datetime import datetime, timezone, timedelta
    from sqlmodel import text

    print(f"🧹 開始清理超過 {days_to_keep} 天的舊記錄...")
    cutoff_date = datetime.now(timezone.utc) - timedelta(days=days_to_keep)

    try:
        # 清理過期的條件歷史記錄
        try:
            delete_history_sql = text(f"DELETE FROM task_condition_history WHERE expires_at < '{cutoff_date.isoformat()}'")
            result = session.exec(delete_history_sql)
            affected_rows = 0
            if hasattr(result, 'rowcount'):
                affected_rows = result.rowcount
            print(f"🧹 清理了 {affected_rows} 個過期的條件歷史記錄")
        except Exception as history_error:
            print(f"⚠️ 清理條件歷史記錄時發生錯誤: {history_error}")

        # 清理過期的快取記錄
        try:
            delete_cache_sql = text(f"DELETE FROM task_condition_cache WHERE expires_at < '{cutoff_date.isoformat()}'")
            result = session.exec(delete_cache_sql)
            affected_rows = 0
            if hasattr(result, 'rowcount'):
                affected_rows = result.rowcount
            print(f"🧹 清理了 {affected_rows} 個過期的快取記錄")
        except Exception as cache_error:
            print(f"⚠️ 清理快取記錄時發生錯誤: {cache_error}")

        session.commit()
        print("✅ 舊記錄清理完成")

    except Exception as e:
        print(f"❌ 清理舊記錄失敗: {e}")
        session.rollback()


#if __name__ == "__main__":
#    # 測試用的初始化腳本
#    from db_proxy.connection import connection_pool
#    
#    print("🚀 開始初始化任務條件歷史資料表...")
#
#    with connection_pool.get_session() as session:
#        init_task_condition_history_tables(session)
#
#        # 可選：清理舊記錄
#        # cleanup_old_records(session, days_to_keep=7)
#
#    print("🎉 任務條件歷史資料表初始化完成！")
