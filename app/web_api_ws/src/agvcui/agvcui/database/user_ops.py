# database/user_ops.py
"""
用戶相關資料庫操作
"""

from sqlmodel import select, func
from db_proxy.models import User
from .connection import connection_pool, user_crud


def get_user_by_username(username: str):
    """根據用戶名獲取用戶"""
    with connection_pool.get_session() as session:
        return user_crud.get_by_username(session, username)


def get_user_by_email(email: str):
    """根據郵箱獲取用戶"""
    with connection_pool.get_session() as session:
        return user_crud.get_by_email(session, email)


def create_user(username: str, password_hash: str, email: str = None,
                full_name: str = None, role: str = "user"):
    """創建新用戶"""
    with connection_pool.get_session() as session:
        return user_crud.create_user(session, username, password_hash, email, full_name, role)


def update_user_last_login(user_id: int):
    """更新用戶最後登入時間"""
    with connection_pool.get_session() as session:
        return user_crud.update_last_login(session, user_id)


def get_users(offset: int = 0, limit: int = 20):
    """獲取用戶列表（分頁）"""
    with connection_pool.get_session() as session:
        statement = select(User).order_by(
            User.created_at.desc()).offset(offset).limit(limit)
        return session.exec(statement).all()


def count_users():
    """計算用戶總數"""
    with connection_pool.get_session() as session:
        statement = select(func.count()).select_from(User)
        return session.exec(statement).one()
