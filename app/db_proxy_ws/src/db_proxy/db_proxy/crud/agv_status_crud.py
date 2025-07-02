"""
AGV 狀態 CRUD 操作
"""
from typing import List, Optional
from sqlmodel import Session, select
from db_proxy.models import AgvStatus
from .base_crud import BaseCRUD


class AgvStatusCRUD(BaseCRUD[AgvStatus]):
    """AGV 狀態 CRUD 操作類別"""
    
    def get_by_name(self, session: Session, name: str) -> Optional[AgvStatus]:
        """根據名稱取得 AGV 狀態"""
        statement = select(AgvStatus).where(AgvStatus.name == name)
        return session.exec(statement).first()
    
    def get_all_active(self, session: Session) -> List[AgvStatus]:
        """取得所有可用的 AGV 狀態"""
        statement = select(AgvStatus).order_by(AgvStatus.id)
        return session.exec(statement).all()
    
    def get_status_by_color(self, session: Session, color: str) -> List[AgvStatus]:
        """根據顏色取得 AGV 狀態"""
        statement = select(AgvStatus).where(AgvStatus.color == color)
        return session.exec(statement).all()


# 創建全域實例
agv_status_crud = AgvStatusCRUD(AgvStatus)
