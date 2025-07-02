from sqlmodel import Session, create_engine
from db_proxy.models.agvc_task import Task  # 👈 你剛剛貼的 Task 模型
from db_proxy.sql.sql_builder import SQLBuilder  # 👈 你提供的 SQLBuilder 類別
from datetime import datetime, timezone


# PostgreSQL 資料庫連線字串（請依實際修改）
DATABASE_URL = "postgresql+psycopg2://agvc:password@192.168.11.166/agvc"

engine = create_engine(DATABASE_URL)

def insert_task():
    # 建立任務內容
    new_task_data = {
        "work_id": 1001,
        "status_id": 2,  # 假設你查到 status_running.id = 2
        "room_id": 2,
        "name": "在房間(room_id=2)Cargo從料架(rack.id=1)取到入口傳送箱(eqp.id=1)放",
        "description": "走廊車Cargo將料架上的鏡框架依序送至入口傳送箱",
        "agv_id": 1,
        "priority": 10,
        "parameters": {"room_id": 2, "rack_id": 1, "eqp_id": 1},
        "created_at": datetime.now(timezone.utc),
        "updated_at": None
    }

    # 建立 insert statement
    stmt = SQLBuilder.insert_stmt(Task, **new_task_data)

    # 執行
    with Session(engine) as session:
        session.exec(stmt)
        session.commit()
        print("✅ 新任務已成功插入")

if __name__ == "__main__":
    insert_task()
