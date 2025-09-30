from typing import Optional, List, Dict, Any, ClassVar
from sqlmodel import SQLModel, Field, Relationship
from datetime import datetime, timezone
from sqlalchemy import Column
from sqlalchemy.dialects.postgresql import JSONB
from pydantic import ConfigDict


class LocationStatus(SQLModel, table=True):
    __tablename__ = "location_status"
    id: Optional[int] = Field(default=None, primary_key=True)
    name: str
    description: Optional[str] = None

    # 位置狀態常數定義
    UNKNOWN: ClassVar[int] = 1          # 未知狀態
    UNOCCUPIED: ClassVar[int] = 2       # 未佔用 (空位沒有被使用)
    OCCUPIED: ClassVar[int] = 3         # 佔用 (已經有停放的料架)

    # 狀態碼對應的中文描述
    STATUS_DESCRIPTIONS: ClassVar[Dict[int, str]] = {
        1: "未知狀態",
        2: "未佔用",
        3: "佔用"
    }

    # 狀態碼對應的英文名稱
    STATUS_NAMES: ClassVar[Dict[int, str]] = {
        1: "UNKNOWN",
        2: "UNOCCUPIED",
        3: "OCCUPIED"
    }

    # 狀態碼對應的詳細說明
    STATUS_DETAILS: ClassVar[Dict[int, str]] = {
        1: "未知狀態",
        2: "空位沒有被使用",
        3: "已經有停放的料架"
    }

    @classmethod
    def get_description(cls, status_code: int) -> str:
        """取得狀態碼對應的中文描述"""
        return cls.STATUS_DESCRIPTIONS.get(status_code, "未知狀態")

    @classmethod
    def get_name(cls, status_code: int) -> str:
        """取得狀態碼對應的英文名稱"""
        return cls.STATUS_NAMES.get(status_code, "UNKNOWN")

    @classmethod
    def get_detail(cls, status_code: int) -> str:
        """取得狀態碼對應的詳細說明"""
        return cls.STATUS_DETAILS.get(status_code, "未知狀態")

    @classmethod
    def is_valid_status(cls, status_code: int) -> bool:
        """檢查狀態碼是否有效"""
        return status_code in cls.STATUS_DESCRIPTIONS

    model_config = ConfigDict(from_attributes=True)  # 告訴 Pydantic 這是 ORM 模型


class Location(SQLModel, table=True):
    __tablename__ = "location"
    id: Optional[int] = Field(default=None, primary_key=True)
    location_status_id: Optional[int] = Field(
        default=LocationStatus.UNOCCUPIED,  # 預設為未佔用
        foreign_key="location_status.id")
    # 不設外鍵約束避免與 room 表的循環依賴
    room_id: Optional[int] = Field(default=None)
    node_id: Optional[int] = Field(default=None, foreign_key="node.id")  # 關聯到 node 表
    name: str
    description: Optional[str] = None
    # tafl_wcs 需要的欄位
    type: Optional[str] = Field(default="enter_or_exit")  # 位置類型
    rack_id: Optional[int] = Field(default=None, foreign_key="rack.id")  # 關聯的架台 ID
    
    # 關聯關係（用於 JOIN 查詢）
    # location_status: Optional["LocationStatus"] = Relationship()
    # node: Optional["Node"] = Relationship()
    # rack: Optional["Rack"] = Relationship()

    model_config = ConfigDict(from_attributes=True)  # 告訴 Pydantic 這是 ORM 模型
