from typing import Optional
from sqlmodel import SQLModel, Field
from pydantic import ConfigDict


class License(SQLModel, table=True):
    __tablename__ = "license"
    id: Optional[int] = Field(default=None, primary_key=True)
    device_id: str
    active: int

    model_config = ConfigDict(from_attributes=True)  # 告訴 Pydantic 這是 ORM 模型
