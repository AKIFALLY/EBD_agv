from typing import Optional
from sqlmodel import SQLModel, Field, Column, JSON
from pydantic import ConfigDict


class License(SQLModel, table=True):
    __tablename__ = "license"
    id: Optional[int] = Field(default=None, primary_key=True)
    device_id: str = Field(index=True, unique=True)
    active: int = Field(default=1)
    device_type: str = Field(default="injection_machine")  # 設備類型
    description: Optional[str] = None  # 描述
    permissions: Optional[dict] = Field(default=None, sa_column=Column(JSON))  # 權限設定

    model_config = ConfigDict(from_attributes=True)  # 告訴 Pydantic 這是 ORM 模型
