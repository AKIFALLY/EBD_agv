from typing import Optional, List, Dict, Any
from sqlmodel import SQLModel, Field, Relationship
from datetime import datetime, timezone
from sqlalchemy import Column, DateTime
from sqlalchemy.dialects.postgresql import JSONB
from pydantic import ConfigDict


class ProcessSettings(SQLModel, table=True):
    __tablename__ = "process_settings"  # 指定表名

    id: Optional[int] = Field(default=None, primary_key=True)
    soaking_times: int
    description: Optional[str] = None

    # 關聯：每個製程對應多個產品
    products: List["Product"] = Relationship(back_populates="process_setting")

    model_config = ConfigDict(from_attributes=True)  # 告訴 Pydantic 這是 ORM 模型


class Product(SQLModel, table=True):
    __tablename__ = "product"  # 指定表名
    id: Optional[int] = Field(default=None, primary_key=True)
    name: str
    size: str

    # 外鍵：對應到 ProcessSettings 的 process_id
    process_settings_id: int = Field(foreign_key="process_settings.id")
    created_at: datetime = Field(
        sa_column=Column(DateTime(timezone=True), nullable=False),
        default_factory=lambda: datetime.now(timezone.utc))
    updated_at: Optional[datetime] = Field(
        default=None,
        sa_column=Column(DateTime(timezone=True), nullable=True))

    # 關聯：每個產品對應一個製程
    process_setting: Optional["ProcessSettings"] = Relationship(
        back_populates="products")

    model_config = ConfigDict(from_attributes=True)  # 告訴 Pydantic 這是 ORM 模型
