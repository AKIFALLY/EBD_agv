# database/task_ops.py
"""
任務相關資料庫操作
"""

from sqlmodel import select, func
from sqlalchemy.orm import selectinload
from db_proxy.models import Task
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
        return task_crud.create(session, task_obj)


def update_task(task_id: int, task_data: dict):
    """更新任務"""
    with connection_pool.get_session() as session:
        # 創建 Task 對象用於更新
        task_obj = Task(**task_data)
        return task_crud.update(session, task_id, task_obj)


def delete_task(task_id: int):
    """刪除任務"""
    with connection_pool.get_session() as session:
        return task_crud.delete(session, task_id)


def work_all() -> list[dict]:
    """獲取所有工作類型"""
    return fetch_all(work_crud)


def task_status_all() -> list[dict]:
    """獲取所有任務狀態"""
    return fetch_all(task_status_crud)
