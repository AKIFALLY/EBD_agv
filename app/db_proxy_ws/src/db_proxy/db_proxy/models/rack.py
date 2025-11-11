from typing import Optional
from sqlmodel import SQLModel, Field
from pydantic import ConfigDict


class Rack(SQLModel, table=True):
    __tablename__ = "rack"
    id: Optional[int] = Field(default=None, primary_key=True)
    name: str
    room_id: Optional[int] = Field(default=None, foreign_key="room.id")  # 這個rack要搬去哪個房間
    agv_id: Optional[int] = Field(default=None, foreign_key="agv.id")
    location_id: Optional[int] = Field(default=None, foreign_key="location.id")
    product_id: Optional[int] = Field(default=None, foreign_key="product.id")
    is_carry: Optional[int] = None
    is_in_map: Optional[int] = None
    is_docked: Optional[int] = None
    status_id: Optional[int] = Field(
        default=None, foreign_key="rack_status.id")
    carrier_bitmap: str = Field(default="00000000")  # FFFF FFFF 代表架上32格位置 在席狀態
    #FFFF 0000 A面空 B面滿(16個)
    #0000 FFFF B面空 A面滿(16個)
    carrier_enable_bitmap: str = Field(default="00000000")  # FFFF FFFF 代表架上32格位置 都啟用(可拿放)
    #FFFF FFFF S尺寸產品 32格都可以拿/放
    #0F0F 0F0F L尺寸產品 A面 1,2,3,4 | 9,10,11,12 B面 17,18,19,20 | 25,26,27,28
    direction: int = Field(default=0)

    model_config = ConfigDict(from_attributes=True)  # 告訴 Pydantic 這是 ORM 模型
