from sqlalchemy import create_engine, text
from db_proxy.connection_pool_manager import ConnectionPoolManager
from typing import List
from datetime import datetime, timezone
import time
# 你可以改成 PostgreSQL, MySQL, etc.


def dynamic_query(pool: ConnectionPoolManager, table_name: str, columns: List[str], data: List = None, condition: str = None, mode="select"):
    with pool.get_session() as session:
        if mode == "select":
            col_str = ", ".join(columns)
            sql = f"SELECT {col_str} FROM {table_name}"
            if condition:
                sql += f" WHERE {condition}"
            result = session.execute(text(sql)).fetchall()
            return [dict(row._mapping) for row in result]

        elif mode == "insert":
            col_str = ", ".join(columns)
            placeholder = ", ".join([f":{col}" for col in columns])
            sql = f"INSERT INTO {table_name} ({col_str}) VALUES ({placeholder})"
            session.execute(text(sql), dict(zip(columns, data)))
            session.commit()
            return True

        elif mode == "update":
            set_clause = ", ".join([f"{col} = :{col}" for col in columns])
            sql = f"UPDATE {table_name} SET {set_clause}"
            if condition:
                sql += f" WHERE {condition}"
            session.execute(text(sql), dict(zip(columns, data)))
            session.commit()
            return True

        elif mode == "delete":
            sql = f"DELETE FROM {table_name}"
            if condition:
                sql += f" WHERE {condition}"
            session.execute(text(sql))
            session.commit()
            return True

        else:
            raise ValueError(f"Unknown mode: {mode}")




#if __name__ == "__main__":
#   
#    db_url_agvc = 'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'
#
#    # 使用 SQLModel metadata 建立資料表
#    pool_agvc = ConnectionPoolManager(db_url_agvc, 1)
#    import json
#    #dynamic_query(
#    #pool_agvc,
#    #table_name="task",
#    #columns=["id", "work_id", "status_id","room_id", "node_id", "name", "description", "agv_id", "priority", "parameters", "created_at", "updated_at"],
#    #data=[3, 2051701,2,2,1004,"測試測試NAME","測試測試DES",1,2,json.dumps({"a": 1, "b": 2}),"2025-06-09 12:00:00","2025-06-09 12:00:00"],
#    #mode="insert")
#
#
#
#    result = dynamic_query(pool_agvc, "task", columns=["*"], condition= "work_id = 2051701 AND room_id = 2", mode="select")
#    print(result)
#
#
#    #SELECT id, work_id, status_id, room_id, node_id, name, description, agv_id, priority, parameters, created_at, updated_at
#	#FROM public.task;



if __name__ == "__main__":
   
    db_url_agvc = 'postgresql+psycopg2://agvc:password@192.168.100.254/agvc'

    # 使用 SQLModel metadata 建立資料表
    pool_agvc = ConnectionPoolManager(db_url_agvc, 1)
    import json
    dynamic_query(
    pool_agvc,
    table_name="task",
    columns=["id", "work_id", "status_id","room_id", "node_id", "name", "description", "agv_id", "priority", "parameters", "created_at", "updated_at"],
    data=[1, 2000102,2,2,1004,"測試測試NAME","測試測試DES",1,100,json.dumps(""),datetime.now(timezone.utc),datetime.now(timezone.utc)],
    mode="insert")