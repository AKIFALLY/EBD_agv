from typing import Optional
from sqlmodel import SQLModel, Field
from pydantic import ConfigDict


class Rack(SQLModel, table=True):
    __tablename__ = "rack"
    id: Optional[int] = Field(default=None, primary_key=True)
    name: str
    agv_id: Optional[int] = Field(default=None, foreign_key="agv.id")
    location_id: Optional[int] = Field(default=None, foreign_key="location.id")
    product_id: Optional[int] = Field(default=None, foreign_key="product.id")
    is_carry: Optional[int] = None
    is_in_map: Optional[int] = None
    is_docked: Optional[int] = None
    status_id: Optional[int] = Field(
        default=None, foreign_key="rack_status.id")
    direction: int = Field(default=0)

    model_config = ConfigDict(from_attributes=True)  # 告訴 Pydantic 這是 ORM 模型
