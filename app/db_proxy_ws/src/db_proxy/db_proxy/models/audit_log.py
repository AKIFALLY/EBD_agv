from typing import Optional, Dict, Any
from sqlmodel import SQLModel, Field, Index
from datetime import datetime, timezone
from sqlalchemy import Column, DateTime
from sqlalchemy.dialects.postgresql import JSON
from pydantic import ConfigDict


class AuditLog(SQLModel, table=True):
    __tablename__ = "audit_log"
    __table_args__ = (
        Index("idx_timestamp_audit_log", "timestamp"),
        Index("idx_operation_type_audit_log", "operation_type"),
        Index("idx_resource_type_audit_log", "resource_type"),
        Index("idx_username_audit_log", "username"),
        Index("idx_session_id_audit_log", "session_id"),
    )
    
    id: Optional[int] = Field(default=None, primary_key=True)
    operation_id: str = Field(unique=True)  # 前端生成的操作 ID
    session_id: str  # 會話 ID
    timestamp: datetime = Field(
        sa_column=Column(DateTime(timezone=True), nullable=False)
    )
    operation_type: str  # view, create, update, delete, move, assign, control, navigate
    resource_type: str   # task, rack, carrier, agv, equipment, node, map
    resource_id: Optional[str] = None  # 資源 ID
    username: Optional[str] = None
    user_role: Optional[str] = None
    is_logged_in: bool = Field(default=False)
    details: Optional[Dict[str, Any]] = Field(
        default=None, sa_column=Column(JSON)
    )  # JSON 詳細資訊
    user_agent: Optional[str] = None
    url: Optional[str] = None
    created_at: datetime = Field(
        sa_column=Column(DateTime(timezone=True), nullable=False),
        default_factory=lambda: datetime.now(timezone.utc))

    model_config = ConfigDict(from_attributes=True)  # 告訴 Pydantic 這是 ORM 模型
