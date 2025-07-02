from typing import Optional, Dict, Any
from sqlmodel import SQLModel, Field
from datetime import datetime, timezone
from sqlalchemy import Column, DateTime
from sqlalchemy.dialects.postgresql import JSON
from pydantic import ConfigDict


class Client(SQLModel, table=True):
    __tablename__ = "client"
    id: Optional[str] = Field(default=None, primary_key=True)
    machine_id: Optional[int] = Field(default=None, foreign_key="machine.id")
    op: Optional[Dict[str, Any]] = Field(
        default=None, sa_column=Column(JSON))
    user_agent: str
    created_at: datetime = Field(
        sa_column=Column(DateTime(timezone=True), nullable=False),
        default_factory=lambda: datetime.now(timezone.utc))
    updated_at: Optional[datetime] = Field(
        default=None,
        sa_column=Column(DateTime(timezone=True), nullable=True))

    model_config = ConfigDict(from_attributes=True)
