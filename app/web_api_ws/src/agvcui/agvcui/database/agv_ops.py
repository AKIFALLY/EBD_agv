# database/agv_ops.py
"""
AGV 相關資料庫操作
"""

from sqlmodel import select, func
from db_proxy.models import AGV
from .connection import connection_pool, agv_crud
from .utils import fetch_all


def agv_all() -> list[dict]:
    """獲取所有 AGV"""
    return fetch_all(agv_crud)


def get_agvs(offset: int = 0, limit: int = 20):
    """獲取 AGV 列表（分頁）"""
    with connection_pool.get_session() as session:
        statement = select(AGV).order_by(AGV.id.desc()).offset(offset).limit(limit)
        return session.exec(statement).all()


def count_agvs():
    """計算 AGV 總數"""
    with connection_pool.get_session() as session:
        statement = select(func.count()).select_from(AGV)
        result = session.exec(statement).one()
        return result


def get_agv_by_id(agv_id: int):
    """根據 ID 獲取 AGV 詳細資訊"""
    with connection_pool.get_session() as session:
        statement = select(AGV).where(AGV.id == agv_id)
        return session.exec(statement).first()


def update_agv(agv_id: int, agv_data: dict):
    """更新 AGV 資訊"""
    with connection_pool.get_session() as session:
        agv = session.exec(select(AGV).where(AGV.id == agv_id)).first()
        if not agv:
            return None

        # 更新允許的欄位
        for field in ['name', 'description', 'model', 'x', 'y', 'heading', 'battery', 'status', 'last_node_id', 'enable']:
            if field in agv_data:
                setattr(agv, field, agv_data[field])

        session.add(agv)
        session.commit()
        session.refresh(agv)
        return agv


def delete_agv(agv_id: int):
    """刪除 AGV"""
    with connection_pool.get_session() as session:
        agv = session.exec(select(AGV).where(AGV.id == agv_id)).first()
        if not agv:
            return False

        session.delete(agv)
        session.commit()
        return True


def create_agv(agv_data: dict):
    """創建新 AGV"""
    with connection_pool.get_session() as session:
        agv = AGV(**agv_data)
        session.add(agv)
        session.commit()
        session.refresh(agv)
        return agv
