from typing import Optional, List, Dict, Any
from sqlmodel import SQLModel, Field, Relationship
from datetime import datetime, timezone
from sqlalchemy import Column, DateTime
from sqlalchemy.dialects.postgresql import JSONB, JSON
from pydantic import ConfigDict


class Work(SQLModel, table=True):
    __tablename__ = "work"
    id: Optional[int] = Field(default=None, primary_key=True)
    name: str
    description: Optional[str] = None

    parameters: Optional[Dict[str, Any]] = Field(
        sa_column=Column(JSON)
    )

    # ORM 關聯：一對多 (One work to many tasks)
    tasks: List["Task"] = Relationship(back_populates="work")

    model_config = ConfigDict(from_attributes=True)  # 告訴 Pydantic 這是 ORM 模型


class TaskStatus(SQLModel, table=True):
    __tablename__ = "task_status"
    id: Optional[int] = Field(default=None, primary_key=True)
    name: str
    description: Optional[str] = None

    model_config = ConfigDict(from_attributes=True)  # 告訴 Pydantic 這是 ORM 模型


class Task(SQLModel, table=True):
    __tablename__ = "task"
    id: Optional[int] = Field(default=None, primary_key=True)
    parent_task_id: Optional[int] = Field(default=None, foreign_key="task.id")
    work_id: Optional[int] = Field(default=None, foreign_key="work.id")
    status_id: Optional[int] = Field(
        default=None, foreign_key="task_status.id")
    room_id: Optional[int] = Field(default=None, foreign_key="room.id")
    node_id: Optional[int] = Field(default=None, foreign_key="node.id")
    name: str
    description: Optional[str] = None
    mission_code: Optional[str] = None  # Kuka 系統的任務代碼
    agv_id: Optional[int] = Field(default=None, foreign_key="agv.id")
    priority: int = Field(default=0)
    parameters: Optional[Dict[str, Any]] = Field(
        sa_column=Column(JSON)
    )
    created_at: datetime = Field(
        sa_column=Column(DateTime(timezone=True), nullable=False),
        default_factory=lambda: datetime.now(timezone.utc))
    updated_at: Optional[datetime] = Field(
        default=None,
        sa_column=Column(DateTime(timezone=True), nullable=True))

    # ORM 關聯：多對一 (Many tasks to one work)
    work: Optional["Work"] = Relationship(back_populates="tasks")

    # ORM 關聯：多對一 (Many tasks to one AGV)
    agv: Optional["AGV"] = Relationship()

    model_config = ConfigDict(from_attributes=True)  # 告訴 Pydantic 這是 ORM 模型
