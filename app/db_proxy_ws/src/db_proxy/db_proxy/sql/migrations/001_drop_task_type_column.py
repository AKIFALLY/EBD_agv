#!/usr/bin/env python3
"""
数据库迁移脚本: 移除 task.type 字段

背景:
- type 字段仅在已弃用的 TAFL WCS 系统中使用
- 当前活跃的 KUKA WCS 系统完全不依赖此字段
- 功能已被 work_id 完全取代

作者: Claude Code
日期: 2025-11-18
"""

from sqlalchemy import text
from db_proxy.connection_pool_manager import ConnectionPoolManager


def upgrade(session):
    """执行迁移：删除 task.type 字段"""
    print("🔄 开始执行迁移: 移除 task.type 字段...")

    # 检查字段是否存在
    check_column_sql = text("""
        SELECT EXISTS (
            SELECT FROM information_schema.columns
            WHERE table_schema = 'public'
            AND table_name = 'task'
            AND column_name = 'type'
        )
    """)

    column_exists = session.exec(check_column_sql).first()[0]

    if column_exists:
        # 删除字段
        drop_column_sql = text("ALTER TABLE task DROP COLUMN type")
        session.exec(drop_column_sql)
        session.commit()
        print("✅ task.type 字段已成功删除")
    else:
        print("⚠️  task.type 字段不存在，跳过删除")


def downgrade(session):
    """回滚迁移：恢复 task.type 字段"""
    print("🔄 开始回滚迁移: 恢复 task.type 字段...")

    # 检查字段是否已经存在
    check_column_sql = text("""
        SELECT EXISTS (
            SELECT FROM information_schema.columns
            WHERE table_schema = 'public'
            AND table_name = 'task'
            AND column_name = 'type'
        )
    """)

    column_exists = session.exec(check_column_sql).first()[0]

    if not column_exists:
        # 恢复字段
        add_column_sql = text("ALTER TABLE task ADD COLUMN type VARCHAR")
        session.exec(add_column_sql)
        session.commit()
        print("✅ task.type 字段已成功恢复")
    else:
        print("⚠️  task.type 字段已存在，跳过恢复")


def main(operation='upgrade'):
    """主函数：执行迁移或回滚"""
    db_url = 'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'

    try:
        print(f"🚀 连接数据库: {db_url}")
        pool = ConnectionPoolManager(db_url, 1)

        with pool.get_session() as session:
            if operation == 'upgrade':
                upgrade(session)
            elif operation == 'downgrade':
                downgrade(session)
            else:
                raise ValueError(f"未知的操作: {operation}")

        pool.shutdown()
        print("🎉 迁移完成！")

    except Exception as e:
        print(f"❌ 迁移失败: {e}")
        raise


if __name__ == "__main__":
    import sys

    # 支持命令行参数: upgrade (默认) 或 downgrade
    operation = sys.argv[1] if len(sys.argv) > 1 else 'upgrade'
    main(operation)
