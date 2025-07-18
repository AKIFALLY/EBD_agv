# db/db.py
from datetime import datetime, timezone
from db_proxy.connection_pool_manager import ConnectionPoolManager
from db_proxy.models import Client, Product, Machine, Room, Rack, Task, Work, TaskStatus
from db_proxy.crud.base_crud import BaseCRUD
from db_proxy.crud.location_crud import location_crud
from sqlmodel import SQLModel, Session, select

# 這是資料庫的連線 URL
db_url_agvc = 'postgresql+psycopg2://agvc:password@192.168.100.254/agvc?client_encoding=utf8'

# 初始化 ConnectionPoolManager，無需傳遞 base
connection_pool = ConnectionPoolManager(db_url_agvc)

# 其他模組可以使用 connection_pool.SessionLocal
# SessionLocal = connection_pool.SessionLocal

client_crud = BaseCRUD(Client, id_column="id")
product_crud = BaseCRUD(Product, id_column="id")
machine_crud = BaseCRUD(Machine, id_column="id")
room_crud = BaseCRUD(Room, id_column="id")
rack_crud = BaseCRUD(Rack, id_column="id")
task_crud = BaseCRUD(Task, id_column="id")
work_crud = BaseCRUD(Work, id_column="id")
task_status_crud = BaseCRUD(TaskStatus, id_column="id")


def get_client(client_data) -> dict:
    # client: dict, 包含 clientId, userAgent, op[0,1] 等資訊
    session = connection_pool.get_session()
    try:
        data = dict(client_data)
        field_map = {
            "clientId": "id",
            "userAgent": "user_agent",
            "machineId": "machine_id",
            "createdAt": "created_at",
            "updatedAt": "updated_at"
        }
        for old_key, new_key in field_map.items():
            if old_key in data:
                data[new_key] = data.pop(old_key)
        # 預設值處理
        if not data.get("user_agent"):
            data["user_agent"] = ""
        if not data.get("op"):
            data["op"] = {}
        if not data.get("machine_id"):
            data["machine_id"] = 1
        data["updated_at"] = datetime.now(timezone.utc)

        print(f"🔍 查詢客戶端資料: clientId={data.get('id')}")

        statement = select(Client).where(Client.id == data["id"])
        client_obj = session.exec(statement).first()

        if client_obj is None:
            print(f"⚠️ 客戶端不存在，創建新記錄: clientId={data.get('id')}")
            client_obj = client_crud.create(session, Client(**data))

        # 嘗試不同的方法來獲取資料
        try:
            db_data = client_obj.model_dump()
            print(f"🔍 model_dump() 結果: {db_data}")
        except Exception as e:
            print(f"❌ model_dump() 失敗: {e}")
            # 使用備用方法
            db_data = {
                "id": client_obj.id,
                "user_agent": client_obj.user_agent,
                "machine_id": client_obj.machine_id,
                "op": client_obj.op,
                "created_at": client_obj.created_at,
                "updated_at": client_obj.updated_at
            }
            print(f"🔄 使用備用方法獲取資料: {db_data}")

        # 如果 db_data 仍然為空，手動構建
        if not db_data or not isinstance(db_data, dict):
            print(f"⚠️ db_data 無效，手動構建資料")
            db_data = {
                "id": getattr(client_obj, 'id', None),
                "user_agent": getattr(client_obj, 'user_agent', ''),
                "machine_id": getattr(client_obj, 'machine_id', 1),
                "op": getattr(client_obj, 'op', {}),
                "created_at": getattr(client_obj, 'created_at', None),
                "updated_at": getattr(client_obj, 'updated_at', None)
            }
            print(f"🔧 手動構建的資料: {db_data}")

        # 進行欄位映射
        for old_key, new_key in field_map.items():
            if new_key in db_data:
                db_data[old_key] = db_data.pop(new_key)
                print(f"🔄 欄位映射: {new_key} -> {old_key} = {db_data.get(old_key)}")

        print(f"🔍 最終返回資料: {db_data}")
        return db_data
    except Exception as e:
        print(f"❌ 查詢客戶端資料失敗: {e}")
        raise e
    finally:
        session.close()


def get_or_create_or_update_client(client_data) -> dict:
    # 直接用 BaseCRUD 的 create_or_update
    session = connection_pool.get_session()
    try:
        data = dict(client_data)
        field_map = {
            "clientId": "id",
            "userAgent": "user_agent",
            "machineId": "machine_id",
            "createdAt": "created_at",
            "updatedAt": "updated_at"
        }
        for old_key, new_key in field_map.items():
            if old_key in data:
                data[new_key] = data.pop(old_key)
        # 預設值處理
        if not data.get("user_agent"):
            data["user_agent"] = ""
        if not data.get("op"):
            data["op"] = {}
        if not data.get("machine_id"):
            data["machine_id"] = 1

        data["updated_at"] = datetime.now(timezone.utc)

        print(f"🔧 準備更新客戶端資料: clientId={data.get('id')}, machineId={data.get('machine_id')}")

        db_data = client_crud.create_or_update(
            session, Client(**data)).model_dump()

        print(f"✅ 資料庫更新成功: clientId={db_data.get('id')}, machineId={db_data.get('machine_id')}")

        for old_key, new_key in field_map.items():
            if new_key in db_data:
                db_data[old_key] = db_data.pop(new_key)

        return db_data
    except Exception as e:
        print(f"❌ 資料庫更新失敗: {e}")
        raise e
    finally:
        session.close()


def create_or_update_product(product) -> dict:
    # 直接用 BaseCRUD 的 create_or_update
    with connection_pool.get_session() as session:
        # Check if the product exists using the name field

        data = dict(product)
        field_map = {
            "product": "name",
            "size": "size",
            "process": "process_settings_id",
            "createdAt": "created_at",
            "updatedAt": "updated_at"
        }
        for old_key, new_key in field_map.items():
            if old_key in data:
                data[new_key] = data.pop(old_key)
        data["updated_at"] = datetime.now(timezone.utc)
        data["process_settings_id"] = int(data["process_settings_id"])

        sql = select(Product).where(Product.name == data["name"])
        existing_product = session.exec(sql).first()

        # If the product exists, update it; otherwise, create a new one
        if existing_product:
            db_data = product_crud.update(
                session, existing_product.id, Product(**data)).model_dump()
        else:
            db_data = product_crud.create(
                session, Product(**data)).model_dump()

        for old_key, new_key in field_map.items():
            if new_key in data:
                db_data[old_key] = db_data.pop(new_key)

        return db_data


def product_all() -> list[dict]:
    # 直接用 BaseCRUD 的 get_all
    with connection_pool.get_session() as session:
        products = product_crud.get_all(session)
        return [p.model_dump() for p in products]


def machine_all() -> list[dict]:
    # 直接用 BaseCRUD 的 get_all，並動態添加料架狀態資訊
    with connection_pool.get_session() as session:
        machines = machine_crud.get_all(session)
        result = []

        for machine in machines:
            machine_dict = machine.model_dump()

            # 動態計算停車格是否有料架
            if machine.parking_space_1:
                # 使用 SQLModel 查詢該位置的料架
                left_racks = session.exec(
                    select(Rack).where(Rack.location_id == machine.parking_space_1)
                ).all()
                machine_dict['parking_space_1_has_rack'] = len(left_racks) > 0
            else:
                machine_dict['parking_space_1_has_rack'] = False

            if machine.parking_space_2:
                # 使用 SQLModel 查詢該位置的料架
                right_racks = session.exec(
                    select(Rack).where(Rack.location_id == machine.parking_space_2)
                ).all()
                machine_dict['parking_space_2_has_rack'] = len(right_racks) > 0
            else:
                machine_dict['parking_space_2_has_rack'] = False

            # 添加前端期望的屬性名稱，用於向後相容
            machine_dict['left_parking_space'] = machine.parking_space_1
            machine_dict['right_parking_space'] = machine.parking_space_2

            result.append(machine_dict)

        return result


def room_all() -> list[dict]:
    # 直接用 BaseCRUD 的 get_all
    with connection_pool.get_session() as session:
        rooms = room_crud.get_all(session)
        return [r.model_dump() for r in rooms]


def create_task(task_data: dict) -> dict:
    """創建新任務"""
    with connection_pool.get_session() as session:
        # 創建 Task 對象
        task_obj = Task(**task_data)
        created_task = task_crud.create(session, task_obj)
        return created_task.model_dump()


def work_all() -> list[dict]:
    """獲取所有工作類型"""
    with connection_pool.get_session() as session:
        works = work_crud.get_all(session)
        return [w.model_dump() for w in works]


def task_status_all() -> list[dict]:
    """獲取所有任務狀態"""
    with connection_pool.get_session() as session:
        task_statuses = task_status_crud.get_all(session)
        return [ts.model_dump() for ts in task_statuses]


def get_call_empty_work_id() -> int:
    """獲取叫空車工作類型的 ID，如果不存在則創建"""
    with connection_pool.get_session() as session:
        # 查找叫空車工作類型
        statement = select(Work).where(Work.name == "opui-call-empty")
        work = session.exec(statement).first()

        if work is None:
            # 如果不存在，創建叫空車工作類型
            work_data = {
                "name": "opui-call-empty",
                "description": "作業員從opui請求將空Rack派至[人工作業準備區]",
                "parameters": {"type": "call_empty"}
            }
            work = work_crud.create(session, Work(**work_data))

        return work.id


def get_dispatch_full_work_id() -> int:
    """獲取派滿車工作類型的 ID，如果不存在則創建"""
    with connection_pool.get_session() as session:
        # 查找派滿車工作類型
        statement = select(Work).where(Work.name == "opui-dispatch-full")
        work = session.exec(statement).first()

        if work is None:
            # 如果不存在，創建派滿車工作類型
            work_data = {
                "name": "opui-dispatch-full",
                "description": "作業員從opui請求將Rack派至[系統準備派車區]",
                "parameters": {"type": "dispatch_full"}
            }
            work = work_crud.create(session, Work(**work_data))

        return work.id


def delete_task_by_parking(node_id: int) -> bool:
    """根據停車格 ID 刪除對應的叫空車/派滿車任務，並回傳是否成功"""
    from db_proxy.models import Task
    with connection_pool.get_session() as session:
        # 找到該停車格的未完成叫空車/派滿車任務
        stmt = select(Task).where(
            Task.node_id == node_id,
            Task.status_id == TaskStatus.REQUESTING,
            Task.work_id.in_(
                [get_call_empty_work_id(), get_dispatch_full_work_id()])
        )
        task = session.exec(stmt).first()
        if not task:
            return False
        task_crud.delete(session, task.id)
        return True


def delete_task(task_id: int) -> bool:
    """根據任務 ID 刪除任務，回傳是否成功"""
    with connection_pool.get_session() as session:
        return task_crud.delete(session, task_id)
