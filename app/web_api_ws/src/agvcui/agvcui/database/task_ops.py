# database/task_ops.py
"""
任務相關資料庫操作
"""

from sqlmodel import select, func, or_
from sqlalchemy.orm import selectinload
from db_proxy.models import Task, Work, ModifyLog
from .connection import connection_pool, task_crud, work_crud, task_status_crud
from .utils import fetch_all


def task_all() -> list[dict]:
    """獲取所有任務"""
    return fetch_all(task_crud)


def get_tasks(offset: int = 0, limit: int = 20):
    """獲取任務列表（分頁），使用 ORM 關聯載入 AGV 資料"""
    with connection_pool.get_session() as session:
        # 使用 selectinload 來預載入 AGV 關聯資料
        statement = (
            select(Task)
            .options(selectinload(Task.agv))
            .order_by(Task.created_at.desc())
            .offset(offset)
            .limit(limit)
        )
        return session.exec(statement).all()


def get_tasks_with_hierarchy():
    """獲取所有任務並建立階層結構"""
    with connection_pool.get_session() as session:
        # 獲取所有任務
        statement = (
            select(Task)
            .options(selectinload(Task.agv))
            .order_by(Task.created_at.desc())
        )
        all_tasks = session.exec(statement).all()

        # 建立任務字典以便快速查找
        task_dict = {task.id: task for task in all_tasks}

        # 為每個任務添加子任務列表
        for task in all_tasks:
            task.children = []
            task.parent = None

        # 建立父子關係
        for task in all_tasks:
            if task.parent_task_id and task.parent_task_id in task_dict:
                parent = task_dict[task.parent_task_id]
                parent.children.append(task)
                task.parent = parent

        # 返回根任務（沒有父任務的任務）
        root_tasks = [task for task in all_tasks if task.parent_task_id is None]
        return root_tasks


def get_task_children(task_id: int):
    """獲取指定任務的所有子任務"""
    with connection_pool.get_session() as session:
        statement = (
            select(Task)
            .options(selectinload(Task.agv))
            .where(Task.parent_task_id == task_id)
            .order_by(Task.created_at.asc())
        )
        return session.exec(statement).all()


def get_task_with_relations(task_id: int):
    """獲取任務及其父子關係"""
    with connection_pool.get_session() as session:
        # 獲取目標任務
        task = task_crud.get_by_id(session, task_id)
        if not task:
            return None

        # 獲取父任務
        parent_task = None
        if task.parent_task_id:
            parent_task = task_crud.get_by_id(session, task.parent_task_id)

        # 獲取子任務
        children = get_task_children(task_id)

        return {
            'task': task,
            'parent': parent_task,
            'children': children
        }


def count_tasks():
    """計算任務總數"""
    with connection_pool.get_session() as session:
        statement = select(func.count()).select_from(Task)
        return session.exec(statement).one()


def get_task_by_id(task_id: int):
    """根據 ID 獲取任務"""
    with connection_pool.get_session() as session:
        return task_crud.get_by_id(session, task_id)


def create_task(task_data: dict):
    """創建新任務"""
    with connection_pool.get_session() as session:
        # 創建 Task 對象
        task_obj = Task(**task_data)
        result = task_crud.create(session, task_obj)

        # 🔴 關鍵：標記 task 表已更新，觸發前端即時更新
        # 絕對不可移除！前端 agvc_ui_socket.py 監聽此事件
        ModifyLog.mark(session, "task")
        session.commit()

        return result


def update_task(task_id: int, task_data: dict):
    """更新任務"""
    with connection_pool.get_session() as session:
        # 創建 Task 對象用於更新
        task_obj = Task(**task_data)
        result = task_crud.update(session, task_id, task_obj)

        # 🔴 關鍵：標記 task 表已更新，觸發前端即時更新
        # 絕對不可移除！前端 agvc_ui_socket.py 監聽此事件
        ModifyLog.mark(session, "task")
        session.commit()

        return result


def delete_task(task_id: int):
    """刪除任務"""
    with connection_pool.get_session() as session:
        result = task_crud.delete(session, task_id)

        # 🔴 關鍵：標記 task 表已更新，觸發前端即時更新
        # 絕對不可移除！前端 agvc_ui_socket.py 監聽此事件
        ModifyLog.mark(session, "task")
        session.commit()

        return result


def work_all() -> list[dict]:
    """獲取所有工作類型"""
    return fetch_all(work_crud)


def get_works(offset: int = 0, limit: int = 20, search: str = None,
              sort_by: str = "id", sort_order: str = "desc") -> list[dict]:
    """獲取工作類型列表（分頁、搜尋和排序）"""
    with connection_pool.get_session() as session:
        # 建立基本查詢
        statement = select(Work)

        # 添加搜尋條件
        if search:
            statement = statement.where(
                or_(
                    Work.name.ilike(f"%{search}%"),
                    Work.description.ilike(f"%{search}%")
                )
            )

        # 添加排序
        sort_column = getattr(Work, sort_by, Work.id)
        if sort_order.lower() == "asc":
            statement = statement.order_by(sort_column.asc())
        else:
            statement = statement.order_by(sort_column.desc())

        # 添加分頁
        statement = statement.offset(offset).limit(limit)

        works = session.exec(statement).all()
        return [work.model_dump() for work in works]


def count_works(search: str = None) -> int:
    """獲取工作類型總數"""
    with connection_pool.get_session() as session:
        if search:
            statement = select(func.count(Work.id)).where(
                or_(
                    Work.name.ilike(f"%{search}%"),
                    Work.description.ilike(f"%{search}%")
                )
            )
        else:
            statement = select(func.count(Work.id))

        return session.exec(statement).one()


def get_work_by_id(work_id: int, include_tasks: bool = False):
    """根據 ID 獲取工作類型"""
    with connection_pool.get_session() as session:
        if include_tasks:
            # 使用 eager loading 預先加載 tasks 關係
            from sqlalchemy.orm import selectinload
            statement = select(Work).where(Work.id == work_id).options(selectinload(Work.tasks))
            work = session.exec(statement).first()
            if work:
                # 將對象轉換為字典以避免 detached 問題
                work_dict = work.model_dump()
                # 手動添加 tasks 信息
                work_dict['tasks'] = [task.model_dump()
                                      for task in work.tasks] if work.tasks else []
                return work_dict
            return None
        else:
            work = work_crud.get_by_id(session, work_id)
            if work:
                return work.model_dump()
            return None


def create_work(work_data: dict):
    """創建新工作類型"""
    with connection_pool.get_session() as session:
        work_obj = Work(**work_data)
        return work_crud.create(session, work_obj)


def update_work(work_id: int, work_data: dict):
    """更新工作類型"""
    with connection_pool.get_session() as session:
        return work_crud.update(session, work_id, work_data)


def delete_work(work_id: int) -> bool:
    """刪除工作類型"""
    with connection_pool.get_session() as session:
        return work_crud.delete(session, work_id)


def task_status_all() -> list[dict]:
    """獲取所有任務狀態"""
    return fetch_all(task_status_crud)
