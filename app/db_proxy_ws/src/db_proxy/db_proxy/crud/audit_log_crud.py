from typing import List, Optional
from datetime import datetime
from sqlmodel import Session, select, func
from db_proxy.models import AuditLog
from db_proxy.crud.base_crud import BaseCRUD


class AuditLogCRUD(BaseCRUD):
    def get_by_session_id(self, session: Session, session_id: str) -> List[AuditLog]:
        """根據會話 ID 獲取審計日誌"""
        statement = select(AuditLog).where(AuditLog.session_id == session_id)
        return session.exec(statement).all()
    
    def get_by_user(self, session: Session, username: str, 
                   start_time: Optional[datetime] = None,
                   end_time: Optional[datetime] = None) -> List[AuditLog]:
        """根據使用者獲取審計日誌"""
        statement = select(AuditLog).where(AuditLog.username == username)
        
        if start_time:
            statement = statement.where(AuditLog.timestamp >= start_time)
        if end_time:
            statement = statement.where(AuditLog.timestamp <= end_time)
            
        return session.exec(statement.order_by(AuditLog.timestamp.desc())).all()
    
    def get_by_operation_type(self, session: Session, operation_type: str,
                             limit: int = 100) -> List[AuditLog]:
        """根據操作類型獲取審計日誌"""
        statement = select(AuditLog).where(
            AuditLog.operation_type == operation_type
        ).order_by(AuditLog.timestamp.desc()).limit(limit)
        return session.exec(statement).all()
    
    def get_by_resource(self, session: Session, resource_type: str,
                       resource_id: Optional[str] = None) -> List[AuditLog]:
        """根據資源類型和 ID 獲取審計日誌"""
        statement = select(AuditLog).where(AuditLog.resource_type == resource_type)
        
        if resource_id:
            statement = statement.where(AuditLog.resource_id == resource_id)
            
        return session.exec(statement.order_by(AuditLog.timestamp.desc())).all()
    
    def get_paginated(self, session: Session, offset: int = 0, limit: int = 20,
                     operation_type: Optional[str] = None,
                     resource_type: Optional[str] = None,
                     username: Optional[str] = None,
                     start_time: Optional[datetime] = None,
                     end_time: Optional[datetime] = None) -> List[AuditLog]:
        """分頁獲取審計日誌"""
        statement = select(AuditLog)
        
        # 添加篩選條件
        if operation_type:
            statement = statement.where(AuditLog.operation_type == operation_type)
        if resource_type:
            statement = statement.where(AuditLog.resource_type == resource_type)
        if username:
            statement = statement.where(AuditLog.username.ilike(f"%{username}%"))
        if start_time:
            statement = statement.where(AuditLog.timestamp >= start_time)
        if end_time:
            statement = statement.where(AuditLog.timestamp <= end_time)
        
        statement = statement.order_by(AuditLog.timestamp.desc()).offset(offset).limit(limit)
        return session.exec(statement).all()
    
    def count_filtered(self, session: Session,
                      operation_type: Optional[str] = None,
                      resource_type: Optional[str] = None,
                      username: Optional[str] = None,
                      start_time: Optional[datetime] = None,
                      end_time: Optional[datetime] = None) -> int:
        """計算篩選後的審計日誌總數"""
        statement = select(func.count()).select_from(AuditLog)
        
        # 添加篩選條件
        if operation_type:
            statement = statement.where(AuditLog.operation_type == operation_type)
        if resource_type:
            statement = statement.where(AuditLog.resource_type == resource_type)
        if username:
            statement = statement.where(AuditLog.username.ilike(f"%{username}%"))
        if start_time:
            statement = statement.where(AuditLog.timestamp >= start_time)
        if end_time:
            statement = statement.where(AuditLog.timestamp <= end_time)
        
        return session.exec(statement).one()


# 創建 CRUD 實例
audit_log_crud = AuditLogCRUD(AuditLog, id_column="id")
