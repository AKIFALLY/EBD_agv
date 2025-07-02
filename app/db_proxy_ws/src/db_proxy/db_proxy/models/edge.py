from typing import Optional
from sqlmodel import SQLModel, Field
from datetime import datetime, timezone
from sqlalchemy import Column, DateTime
from pydantic import ConfigDict


class Edge(SQLModel, table=True):
    __tablename__ = "edge"
    id: Optional[int] = Field(default=None, primary_key=True)
    from_id: Optional[int] = Field(default=None, foreign_key="node.id")
    to_id: Optional[int] = Field(default=None, foreign_key="node.id")
    weight: Optional[float]
    heading: Optional[float]
    name: str
    description: Optional[str] = None
    created_at: datetime = Field(
        sa_column=Column(DateTime(timezone=True), nullable=False),
        default_factory=lambda: datetime.now(timezone.utc))
    updated_at: Optional[datetime] = Field(
        default=None,
        sa_column=Column(DateTime(timezone=True), nullable=True))

    model_config = ConfigDict(from_attributes=True)  # 告訴 Pydantic 這是 ORM 模型
