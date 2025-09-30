from typing import Optional, List, Dict, Any, ClassVar
from sqlmodel import SQLModel, Field, Relationship
from datetime import datetime, timezone
from sqlalchemy import Column, DateTime
from sqlalchemy.dialects.postgresql import JSONB, JSON
from pydantic import ConfigDict
import json


class Work(SQLModel, table=True):
    __tablename__ = "work"
    id: Optional[int] = Field(default=None, primary_key=True)
    work_code: Optional[str] = None  # Flow WCS 需要的 work_code 欄位
    name: str
    description: Optional[str] = None

    parameters: Optional[Dict[str, Any]] = Field(
        sa_column=Column(JSON(none_as_null=True))
    )

    # ORM 關聯：一對多 (One work to many tasks)
    tasks: List["Task"] = Relationship(back_populates="work")

    model_config = ConfigDict(from_attributes=True)  # 告訴 Pydantic 這是 ORM 模型


class TaskStatus(SQLModel, table=True):
    __tablename__ = "task_status"
    id: Optional[int] = Field(default=None, primary_key=True)
    name: str
    description: Optional[str] = None

    # 任務狀態常數定義
    # 主要狀態
    REQUESTING: ClassVar[int] = 0          # 請求中 (UI-請求執行任務)
    PENDING: ClassVar[int] = 1             # 待處理 (WCS-任務已接受，待處理)
    READY_TO_EXECUTE: ClassVar[int] = 2    # 待執行 (RCS-任務已派發，待執行)
    EXECUTING: ClassVar[int] = 3           # 執行中 (AGV-任務正在執行)
    COMPLETED: ClassVar[int] = 4           # 已完成 (AGV-任務已完成)
    CANCELLING: ClassVar[int] = 5          # 取消中 (任務取消)
    ERROR: ClassVar[int] = 6               # 錯誤 (錯誤)

    # 取消相關狀態
    WCS_CANCELLING: ClassVar[int] = 51     # WCS-取消中 (WCS-任務取消中，待處理)
    RCS_CANCELLING: ClassVar[int] = 52     # RCS-取消中 (RCS-任務取消中，取消中)
    AGV_CANCELLING: ClassVar[int] = 53     # AGV-取消中 (AGV-取消完成)
    CANCELLED: ClassVar[int] = 54          # 已取消 (任務已取消)

    # 狀態碼對應的中文描述
    STATUS_DESCRIPTIONS: ClassVar[Dict[int, str]] = {
        0: "請求中",
        1: "待處理",
        2: "待執行",
        3: "執行中",
        4: "已完成",
        5: "取消中",
        6: "錯誤",
        51: "WCS-取消中",
        52: "RCS-取消中",
        53: "AGV-取消中",
        54: "已取消"
    }

    # 狀態碼對應的英文名稱
    STATUS_NAMES: ClassVar[Dict[int, str]] = {
        0: "REQUESTING",
        1: "PENDING",
        2: "READY_TO_EXECUTE",
        3: "EXECUTING",
        4: "COMPLETED",
        5: "CANCELLING",
        6: "ERROR",
        51: "WCS_CANCELLING",
        52: "RCS_CANCELLING",
        53: "AGV_CANCELLING",
        54: "CANCELLED"
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
    def is_valid_status(cls, status_code: int) -> bool:
        """檢查狀態碼是否有效"""
        return status_code in cls.STATUS_DESCRIPTIONS

    model_config = ConfigDict(from_attributes=True)  # 告訴 Pydantic 這是 ORM 模型


class Task(SQLModel, table=True):
    __tablename__ = "task"
    id: Optional[int] = Field(default=None, primary_key=True)
    task_id: Optional[str] = None  # Flow WCS 需要的 task_id 欄位
    type: Optional[str] = None  # Flow WCS 需要的 type 欄位
    parent_task_id: Optional[int] = Field(default=None, foreign_key="task.id")
    work_id: Optional[int] = Field(default=None, foreign_key="work.id")
    status_id: Optional[int] = Field(
        default=None, foreign_key="task_status.id")
    room_id: Optional[int] = Field(default=None, foreign_key="room.id")
    node_id: Optional[int] = Field(default=None, foreign_key="node.id")
    location_id: Optional[int] = None  # Flow WCS 需要的 location_id 欄位
    rack_id: Optional[int] = None  # Flow WCS 需要的 rack_id 欄位
    name: str
    description: Optional[str] = None
    mission_code: Optional[str] = None  # Kuka 系統的任務代碼
    agv_id: Optional[int] = Field(default=None, foreign_key="agv.id")
    priority: int = Field(default=0)
    parameters: Optional[Dict[str, Any]] = Field(
        sa_column=Column(JSON(none_as_null=True))
    )
    created_at: datetime = Field(
        sa_column=Column(DateTime(timezone=True), nullable=False),
        default_factory=lambda: datetime.now(timezone.utc))
    updated_at: Optional[datetime] = Field(
        sa_column=Column(DateTime(timezone=True), nullable=True))

    # ORM 關聯：多對一 (Many tasks to one work)
    work: Optional["Work"] = Relationship(back_populates="tasks")

    # ORM 關聯：多對一 (Many tasks to one AGV)
    agv: Optional["AGV"] = Relationship()

    model_config = ConfigDict(from_attributes=True)  # 告訴 Pydantic 這是 ORM 模型
    
    def generate_task_id(self, work_code: str = None) -> str:
        """生成 task_id (for tafl_wcs compatibility)"""
        from datetime import datetime
        if work_code:
            return f"TASK_{work_code}_{datetime.now().strftime('%Y%m%d%H%M%S')}"
        else:
            return f"TASK_{self.id}_{datetime.now().strftime('%Y%m%d%H%M%S')}"

