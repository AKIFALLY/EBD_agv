from typing import Optional
from sqlmodel import SQLModel, Field
from pydantic import ConfigDict


class AgvStatus(SQLModel, table=True):
    __tablename__ = "agv_status"
    id: Optional[int] = Field(default=None, primary_key=True)
    name: str
    description: Optional[str] = None
    color: Optional[str] = Field(default="is-light")  # Bulma CSS 顏色類別

    model_config = ConfigDict(from_attributes=True)  # 告訴 Pydantic 這是 ORM 模型
