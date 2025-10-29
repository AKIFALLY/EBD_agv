from typing import Optional, Dict, Any
from sqlmodel import SQLModel, Field
from datetime import datetime, timezone
from sqlalchemy import Column, DateTime, JSON
from pydantic import ConfigDict
from enum import Enum


class NodeTypeEnum(str, Enum):
    NONE = "NONE"
    REST_AREA = "REST_AREA"
    CHARGING_AREA = "CHARGING_AREA"
    TRANSPORT_POINT = "TRANSPORT_POINT"


class PGVEnum(str, Enum):
    FRONT = "前"
    REAR = "後"


class ActionModeEnum(str, Enum):
    VECTOR = "向量"
    OPEN_DOOR = "開門"
    CLOSE_DOOR = "關門"


class Node(SQLModel, table=True):
    __tablename__ = "node"

    id: Optional[int] = Field(default=None, primary_key=True)

    # 像素座標 (px 單位，用於地圖顯示)
    x: float = Field(description="X座標(px)")
    y: float = Field(description="Y座標(px)")
    theta: float = Field(description="角度(Θ)")

    # 原始座標 (mm 單位，匯入時的原始值)
    x_mm: Optional[float] = Field(default=None, description="X座標(mm)")
    y_mm: Optional[float] = Field(default=None, description="Y座標(mm)")

    # 節點屬性
    node_type_id: Optional[int] = Field(default=None, description="節點類型ID")
    name: Optional[str] = Field(default=None, description="節點名稱")
    description: Optional[str] = Field(default=None, description="節點描述")
    pgv: Optional[str] = Field(default=None, description="PGV設定")

    # 5個群組設定 (JSON格式儲存)
    group_1: Optional[Dict[str, Any]] = Field(
        default=None,
        sa_column=Column(JSON),
        description="群組1設定"
    )
    group_2: Optional[Dict[str, Any]] = Field(
        default=None,
        sa_column=Column(JSON),
        description="群組2設定"
    )
    group_3: Optional[Dict[str, Any]] = Field(
        default=None,
        sa_column=Column(JSON),
        description="群組3設定"
    )
    group_4: Optional[Dict[str, Any]] = Field(
        default=None,
        sa_column=Column(JSON),
        description="群組4設定"
    )
    group_5: Optional[Dict[str, Any]] = Field(
        default=None,
        sa_column=Column(JSON),
        description="群組5設定"
    )

    # 系統欄位
    created_at: datetime = Field(
        sa_column=Column(DateTime(timezone=True), nullable=False),
        default_factory=lambda: datetime.now(timezone.utc)
    )
    updated_at: Optional[datetime] = Field(
        default=None,
        sa_column=Column(DateTime(timezone=True), nullable=True)
    )

    model_config = ConfigDict(from_attributes=True)  # 告訴 Pydantic 這是 ORM 模型

    def set_group_config(self, group_number: int, movable_point: float, action_mode: ActionModeEnum,
                        speed_setting: float, vector_angle: float, area_protection: float):
        """設定群組配置"""
        group_data = {
            "可移動點": movable_point,
            "動作模式": action_mode.value,
            "速度設定": speed_setting,
            "向量角度": vector_angle,
            "區域防護": area_protection
        }

        if group_number == 1:
            self.group_1 = group_data
        elif group_number == 2:
            self.group_2 = group_data
        elif group_number == 3:
            self.group_3 = group_data
        elif group_number == 4:
            self.group_4 = group_data
        elif group_number == 5:
            self.group_5 = group_data
        else:
            raise ValueError("群組編號必須是1-5之間")

    def get_group_config(self, group_number: int) -> Optional[Dict[str, Any]]:
        """取得群組配置"""
        if group_number == 1:
            return self.group_1
        elif group_number == 2:
            return self.group_2
        elif group_number == 3:
            return self.group_3
        elif group_number == 4:
            return self.group_4
        elif group_number == 5:
            return self.group_5
        else:
            raise ValueError("群組編號必須是1-5之間")
