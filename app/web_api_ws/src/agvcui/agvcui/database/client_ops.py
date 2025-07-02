# database/client_ops.py
"""
客戶端相關資料庫操作
"""

from datetime import datetime, timezone
from sqlmodel import select, func
from db_proxy.models import Client
from .connection import connection_pool, client_crud
from .utils import fetch_all


def client_all() -> list[dict]:
    """獲取所有客戶端"""
    return fetch_all(client_crud)


def get_clients(offset: int = 0, limit: int = 20):
    """獲取客戶端列表（分頁）"""
    with connection_pool.get_session() as session:
        statement = select(Client).order_by(
            Client.created_at.desc()).offset(offset).limit(limit)
        return session.exec(statement).all()


def count_clients():
    """計算客戶端總數"""
    with connection_pool.get_session() as session:
        statement = select(func.count()).select_from(Client)
        result = session.exec(statement).one()
        return result


def get_client_by_id(client_id: str):
    """根據 ID 獲取客戶端詳細資訊"""
    with connection_pool.get_session() as session:
        statement = select(Client).where(Client.id == client_id)
        return session.exec(statement).first()


def update_client(client_id: str, client_data: dict):
    """更新客戶端資訊"""
    with connection_pool.get_session() as session:
        client = session.exec(select(Client).where(
            Client.id == client_id)).first()
        if not client:
            return None

        # 更新允許的欄位
        if 'machine_id' in client_data:
            client.machine_id = client_data['machine_id']
        if 'user_agent' in client_data:
            client.user_agent = client_data['user_agent']
        if 'op' in client_data:
            client.op = client_data['op']

        client.updated_at = datetime.now(timezone.utc)
        session.add(client)
        session.commit()
        session.refresh(client)
        return client


def reset_client_op_settings(client_id: str):
    """重置客戶端的 OP 設定為預設值"""
    default_op = {
        "left": {
            "productSelected": 0,
            "product": [
                {"name": "", "size": "S", "id": None,
                    "count": 32, "room": 2, "rackId": None},
                {"name": "", "size": "S", "id": None,
                    "count": 32, "room": 2, "rackId": None}
            ]
        },
        "right": {
            "productSelected": 0,
            "product": [
                {"name": "", "size": "S", "id": None,
                    "count": 32, "room": 2, "rackId": None},
                {"name": "", "size": "S", "id": None,
                    "count": 32, "room": 2, "rackId": None}
            ]
        }
    }

    return update_client(client_id, {"op": default_op})


def delete_client(client_id: str):
    """刪除客戶端"""
    with connection_pool.get_session() as session:
        client = session.exec(select(Client).where(
            Client.id == client_id)).first()
        if not client:
            return False

        session.delete(client)
        session.commit()
        return True
