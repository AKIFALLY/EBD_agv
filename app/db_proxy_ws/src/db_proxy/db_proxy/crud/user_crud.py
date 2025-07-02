from typing import Optional, List
from sqlmodel import Session, select
from db_proxy.models import User
from db_proxy.crud.base_crud import BaseCRUD
from datetime import datetime, timezone


class UserCRUD(BaseCRUD):
    def __init__(self):
        super().__init__(User, id_column="id")

    def get_by_username(self, session: Session, username: str) -> Optional[User]:
        """根據用戶名獲取用戶"""
        statement = select(User).where(User.username == username)
        return session.exec(statement).first()

    def get_by_email(self, session: Session, email: str) -> Optional[User]:
        """根據郵箱獲取用戶"""
        statement = select(User).where(User.email == email)
        return session.exec(statement).first()

    def create_user(self, session: Session, username: str, password_hash: str,
                    email: Optional[str] = None, full_name: Optional[str] = None,
                    role: str = "user") -> User:
        """創建新用戶"""
        user = User(
            username=username,
            password_hash=password_hash,
            email=email,
            full_name=full_name,
            role=role,
            is_active=True,
            created_at=datetime.now(timezone.utc)
        )
        session.add(user)
        session.commit()
        session.refresh(user)
        return user

    def update_last_login(self, session: Session, user_id: int) -> Optional[User]:
        """更新用戶最後登入時間"""
        user = self.get_by_id(session, user_id)
        if user:
            user.last_login = datetime.now(timezone.utc)
            user.updated_at = datetime.now(timezone.utc)
            session.add(user)
            session.commit()
            session.refresh(user)
        return user

    def update_password(self, session: Session, user_id: int, new_password_hash: str) -> Optional[User]:
        """更新用戶密碼"""
        user = self.get_by_id(session, user_id)
        if user:
            user.password_hash = new_password_hash
            user.updated_at = datetime.now(timezone.utc)
            session.add(user)
            session.commit()
            session.refresh(user)
        return user

    def activate_user(self, session: Session, user_id: int, is_active: bool = True) -> Optional[User]:
        """啟用或停用用戶"""
        user = self.get_by_id(session, user_id)
        if user:
            user.is_active = is_active
            user.updated_at = datetime.now(timezone.utc)
            session.add(user)
            session.commit()
            session.refresh(user)
        return user

    def get_active_users(self, session: Session, offset: int = 0, limit: int = 100) -> List[User]:
        """獲取活躍用戶列表"""
        statement = select(User).where(
            User.is_active == True).offset(offset).limit(limit)
        return list(session.exec(statement).all())


# 創建全局實例
user_crud = UserCRUD()
