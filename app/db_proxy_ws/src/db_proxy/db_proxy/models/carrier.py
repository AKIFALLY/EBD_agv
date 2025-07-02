from typing import Optional
from sqlmodel import SQLModel, Field
from pydantic import ConfigDict
from datetime import datetime, timezone
from sqlalchemy import Column, DateTime


class Carrier(SQLModel, table=True):
    __tablename__ = "carrier"
    id: Optional[int] = Field(default=None, primary_key=True)
    room_id: Optional[int] = None  # FK room.id 不強綁定
    rack_id: Optional[int] = None  # FK rack.id 不強綁定
    port_id: Optional[int] = None  # FK eqp_port.id 不強綁定
    rack_index: Optional[int] = None
    status_id: Optional[int] = None
    created_at: datetime = Field(
        sa_column=Column(DateTime(timezone=True), nullable=False),
        default_factory=lambda: datetime.now(timezone.utc))
    updated_at: Optional[datetime] = Field(
        default=None,
        sa_column=Column(DateTime(timezone=True), nullable=True))

    model_config = ConfigDict(from_attributes=True)  # 告訴 Pydantic 這是 ORM 模型
