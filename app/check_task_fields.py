#!/usr/bin/env python3
"""
檢查 Task model 的欄位
"""

import sys
sys.path.append('/app/db_proxy_ws/src/db_proxy')

from db_proxy.models.agvc_task import Task

print("Task fields:")
for name, field in Task.model_fields.items():
    print(f"  {name}: {field.annotation}")

# 檢查實際的資料庫欄位
from sqlalchemy import inspect
from db_proxy.connection_pool_manager import ConnectionPoolManager

pool = ConnectionPoolManager('postgresql://agvc:password@192.168.100.254/agvc')
with pool.get_session() as session:
    # 檢查資料庫中的 task 表格欄位
    from sqlalchemy import text
    result = session.execute(text("""
        SELECT column_name, data_type 
        FROM information_schema.columns 
        WHERE table_name = 'task'
        ORDER BY ordinal_position
    """))
    
    print("\nDatabase task table columns:")
    for row in result:
        print(f"  {row[0]}: {row[1]}")