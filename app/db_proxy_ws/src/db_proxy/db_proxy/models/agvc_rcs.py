from typing import Optional, List, Dict, Any
from sqlmodel import SQLModel, Field, Relationship
from datetime import datetime, timezone
from sqlalchemy import Column, DateTime
from sqlalchemy.dialects.postgresql import JSON, JSONB
from pydantic import ConfigDict


class AGV(SQLModel, table=True):
    __tablename__ = "agv"
    id: Optional[int] = Field(default=None, primary_key=True)
#    external_id: Optional[str] = None
    name: str
    description: Optional[str] = None
    model: str  # K400 ,Cargo ,Loader ,Unloader
    x: float
    y: float
    heading: float
    battery: Optional[float] = None
    last_node_id: Optional[int] = None
    enable: int = Field(default=1)
    status_id: Optional[int] = Field(default=None, foreign_key="agv_status.id")
    agv_status_json: Optional[Dict[str, Any]] = Field(
        default=None,
        sa_column=Column(JSON),
        description="AGV 完整狀態資料（來自 /agv/status topic）"
    )

    contexts: List["AGVContext"] = Relationship(
        back_populates="agv")  # 設定 relationship可以簡寫 join

    traffic_zones: List["TrafficZone"] = Relationship(
        back_populates="owner_agv")

    model_config = ConfigDict(from_attributes=True)  # 告訴 Pydantic 這是 ORM 模型


class AGVContext(SQLModel, table=True):
    __tablename__ = "agv_context"
    id: Optional[int] = Field(default=None, primary_key=True)
    agv_id: int = Field(foreign_key="agv.id")
    context: str
    current_state: str
    last_state: str
    updated_at: datetime = Field(
        sa_column=Column(DateTime(timezone=True), nullable=False),
        default_factory=lambda: datetime.now(timezone.utc))
    agv: Optional["AGV"] = Relationship(
        back_populates="contexts")  # 設定 relationship可以簡寫 join

    model_config = ConfigDict(from_attributes=True)  # 告訴 Pydantic 這是 ORM 模型


class TrafficZone(SQLModel, table=True):
    __tablename__ = "traffic_zone"
    id: Optional[int] = Field(default=None, primary_key=True)
    owner_agv_id: Optional[int] = Field(default=None, foreign_key="agv.id")
    name: str
    description: Optional[str] = None
    points: str  # 多邊形點的座標 (以 JSON 字串儲存)
    status: str = Field(default="free")  # 狀態 (free 或 controlled)
    enable: bool = Field(default=False)

    owner_agv: Optional["AGV"] = Relationship(back_populates="traffic_zones")

    model_config = ConfigDict(from_attributes=True)  # 告訴 Pydantic 這是 ORM 模型
