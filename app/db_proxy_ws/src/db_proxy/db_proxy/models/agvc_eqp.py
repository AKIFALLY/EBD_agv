from typing import Optional, List, Dict, Any
from sqlmodel import SQLModel, Field, Relationship
from datetime import datetime, timezone
from sqlalchemy import Column
from sqlalchemy.dialects.postgresql import JSONB
from pydantic import ConfigDict


class Eqp(SQLModel, table=True):
    __tablename__ = "eqp"
    id: Optional[int] = Field(default=None, primary_key=True)
    location_id: Optional[int] = None #設備所在地,可以是None
    name: str
    description: Optional[str] = None

    # Relationships
    signals: List["EqpSignal"] = Relationship(back_populates="eqp")
    ports: List["EqpPort"] = Relationship(back_populates="eqp")

    model_config = ConfigDict(from_attributes=True)  # 告訴 Pydantic 這是 ORM 模型


class EqpPort(SQLModel, table=True):
    __tablename__ = "eqp_port"
    id: Optional[int] = Field(default=None, primary_key=True)
    eqp_id: int = Field(foreign_key="eqp.id")
    name: str
    description: Optional[str] = None

    eqp: Optional["Eqp"] = Relationship(back_populates="ports")

    model_config = ConfigDict(from_attributes=True)  # 告訴 Pydantic 這是 ORM 模型


class EqpSignal(SQLModel, table=True):
    __tablename__ = "eqp_signal"
    id: Optional[int] = Field(default=None, primary_key=True)
    eqp_id: int = Field(foreign_key="eqp.id")
    eqp_port_id: Optional[int] = Field(default=None, foreign_key="eqp_port.id")
    name: str
    description: Optional[str] = None
    value: str
    type_of_value: str
    dm_address: Optional[str] = None

    eqp: Optional["Eqp"] = Relationship(back_populates="signals")

    model_config = ConfigDict(from_attributes=True)  # 告訴 Pydantic 這是 ORM 模型
