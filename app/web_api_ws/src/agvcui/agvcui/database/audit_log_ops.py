# database/audit_log_ops.py
"""
審計日誌相關資料庫操作
"""

from datetime import datetime
from typing import List, Optional
from sqlmodel import select, func
from db_proxy.models import AuditLog
from db_proxy.crud.audit_log_crud import audit_log_crud
from .connection import connection_pool


def create_audit_log(operation_data: dict) -> bool:
    """創建審計日誌"""
    try:
        with connection_pool.get_session() as session:
            audit_log = AuditLog(**operation_data)
            audit_log_crud.create(session, audit_log)
            return True
    except Exception as e:
        print(f"創建審計日誌失敗: {e}")
        return False


def get_audit_logs(offset: int = 0, limit: int = 20,
                  operation_type: Optional[str] = None,
                  resource_type: Optional[str] = None,
                  username: Optional[str] = None,
                  start_time: Optional[datetime] = None,
                  end_time: Optional[datetime] = None) -> List[AuditLog]:
    """獲取審計日誌（分頁、篩選）"""
    with connection_pool.get_session() as session:
        return audit_log_crud.get_paginated(
            session, offset, limit, operation_type, resource_type,
            username, start_time, end_time
        )


def count_audit_logs(operation_type: Optional[str] = None,
                    resource_type: Optional[str] = None,
                    username: Optional[str] = None,
                    start_time: Optional[datetime] = None,
                    end_time: Optional[datetime] = None) -> int:
    """計算審計日誌總數（含篩選）"""
    with connection_pool.get_session() as session:
        return audit_log_crud.count_filtered(
            session, operation_type, resource_type,
            username, start_time, end_time
        )


def get_user_audit_logs(username: str, start_time: Optional[datetime] = None,
                       end_time: Optional[datetime] = None) -> List[AuditLog]:
    """獲取特定使用者的審計日誌"""
    with connection_pool.get_session() as session:
        return audit_log_crud.get_by_user(session, username, start_time, end_time)


def get_resource_audit_logs(resource_type: str, resource_id: Optional[str] = None) -> List[AuditLog]:
    """獲取特定資源的審計日誌"""
    with connection_pool.get_session() as session:
        return audit_log_crud.get_by_resource(session, resource_type, resource_id)


def get_session_audit_logs(session_id: str) -> List[AuditLog]:
    """獲取特定會話的審計日誌"""
    with connection_pool.get_session() as session:
        return audit_log_crud.get_by_session_id(session, session_id)


def get_operation_type_audit_logs(operation_type: str, limit: int = 100) -> List[AuditLog]:
    """獲取特定操作類型的審計日誌"""
    with connection_pool.get_session() as session:
        return audit_log_crud.get_by_operation_type(session, operation_type, limit)
