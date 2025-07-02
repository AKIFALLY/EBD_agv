from typing import Optional, List, Dict, Any
from sqlmodel import SQLModel, Field, Relationship, Field, Index
from datetime import datetime, timezone
from sqlalchemy import Column, DateTime
from sqlalchemy.dialects.postgresql import JSONB
from pydantic import ConfigDict


class KukaNode(SQLModel, table=True):
    __tablename__ = "kuka_node"
    id: Optional[int] = Field(default=None, primary_key=True)
    uuid: Optional[str] = None
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


class KukaEdge(SQLModel, table=True):
    __tablename__ = "kuka_edge"
    id: Optional[int] = Field(default=None, primary_key=True)
    from_id: Optional[int] = Field(default=None, foreign_key="kuka_node.id")
    to_id: Optional[int] = Field(default=None, foreign_key="kuka_node.id")
    weight: Optional[float] = None
    name: str
    description: Optional[str] = None
    created_at: datetime = Field(
        sa_column=Column(DateTime(timezone=True), nullable=False),
        default_factory=lambda: datetime.now(timezone.utc))
    updated_at: Optional[datetime] = Field(
        default=None,
        sa_column=Column(DateTime(timezone=True), nullable=True))

    model_config = ConfigDict(from_attributes=True)  # 告訴 Pydantic 這是 ORM 模型
