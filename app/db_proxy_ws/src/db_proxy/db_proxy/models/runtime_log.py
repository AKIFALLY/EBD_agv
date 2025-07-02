from typing import Optional
from sqlmodel import SQLModel, Field, Index
from datetime import datetime, timezone
from sqlalchemy import Column, DateTime
from pydantic import ConfigDict


class RuntimeLog(SQLModel, table=True):
    __tablename__ = "runtime_log"
    __table_args__ = (
        Index("idx_timestamp_runtime_log", "timestamp"),
        Index("idx_level_runtime_log", "level"),
    )
    id: Optional[int] = Field(default=None, primary_key=True)
    timestamp: datetime = Field(  # ✅ 加上時區
        sa_column=Column(DateTime(timezone=True), nullable=False)
    )
    level: int
    name: str
    message: Optional[str] = None
    file: Optional[str] = None
    function: Optional[str] = None
    line: Optional[int] = None
    created_at: datetime = Field(
        sa_column=Column(DateTime(timezone=True), nullable=False),
        default_factory=lambda: datetime.now(timezone.utc))

    model_config = ConfigDict(from_attributes=True)  # 告訴 Pydantic 這是 ORM 模型
