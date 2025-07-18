from typing import Optional
from sqlmodel import SQLModel, Field
from pydantic import ConfigDict


class Room(SQLModel, table=True):
    __tablename__ = "room"
    id: Optional[int] = Field(default=None, primary_key=True)
    # 外鍵：對應到 ProcessSettings 的 process_id
    process_settings_id: int = Field(foreign_key="process_settings.id")
    name: str
    description: Optional[str] = None
    enable: int = Field(default=1)
    # 新增房間入口和出口位置ID - 不設外鍵約束避免循環依賴
    enter_location_id: Optional[int] = Field(default=None)
    exit_location_id: Optional[int] = Field(default=None)

    model_config = ConfigDict(from_attributes=True)  # 告訴 Pydantic 這是 ORM 模型
