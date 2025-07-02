from typing import Optional
from sqlmodel import SQLModel, Field
from pydantic import ConfigDict


class Machine(SQLModel, table=True):
    __tablename__ = "machine"
    id: Optional[int] = Field(default=None, primary_key=True)
    parking_space_1: Optional[int] = Field(default=None, foreign_key="node.id")
    parking_space_2: Optional[int] = Field(default=None, foreign_key="node.id")
    parking_space_1_status: Optional[int] = Field(default=0)
    parking_space_2_status: Optional[int] = Field(default=0)
    name: str
    description: Optional[str] = None
    enable: int = Field(default=1)

    model_config = ConfigDict(from_attributes=True)  # 告訴 Pydantic 這是 ORM 模型
