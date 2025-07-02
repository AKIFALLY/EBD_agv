from typing import Optional
from sqlmodel import SQLModel, Field
from datetime import datetime, timezone
from sqlalchemy import Column, DateTime
from pydantic import ConfigDict


class Node(SQLModel, table=True):
    __tablename__ = "node"
    id: Optional[int] = Field(default=None, primary_key=True)
    node_type_id: Optional[int] = None
    name: Optional[str] = None
    description: Optional[str] = None
    x: float
    y: float
    created_at: datetime = Field(
        sa_column=Column(DateTime(timezone=True), nullable=False),
        default_factory=lambda: datetime.now(timezone.utc))
    updated_at: Optional[datetime] = Field(
        default=None,
        sa_column=Column(DateTime(timezone=True), nullable=True))

    model_config = ConfigDict(from_attributes=True)  # 告訴 Pydantic 這是 ORM 模型
