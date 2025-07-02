from typing import Optional
from sqlmodel import SQLModel, Field
from pydantic import ConfigDict


class NodeType(SQLModel, table=True):
    __tablename__ = "node_type"
    id: Optional[int] = Field(default=None, primary_key=True)
    name: str
    description: Optional[str] = None

    model_config = ConfigDict(from_attributes=True)  # 告訴 Pydantic 這是 ORM 模型
