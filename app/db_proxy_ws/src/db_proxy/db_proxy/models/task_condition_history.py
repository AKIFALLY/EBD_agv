"""
任務條件狀態資料表模型
包含 TaskCondition、TaskConditionHistory 和 TaskConditionCache
"""

from sqlmodel import SQLModel, Field, Column, JSON, Text
from datetime import datetime, timezone
from typing import Optional, Dict, Any
from enum import Enum


class ConditionType(str, Enum):
    """條件類型枚舉"""
    AREA_STATUS = "area_status"
    TASK_DUPLICATE = "task_duplicate"
    POSITION_OCCUPIED = "position_occupied"
    TASK_STATUS = "task_status"
    CARRIER_STATUS = "carrier_status"
    AGV_CONTEXT = "agv_context"
    TASK_CONDITION = "task_condition"


class TaskCondition(SQLModel, table=True):
    """
    任務條件表 - 簡化版本

    用於儲存任務條件和結果的基本資料表
    符合用戶要求的簡化結構：id, conditions, results
    """
    __tablename__ = "task_condition"

    id: Optional[int] = Field(default=None, primary_key=True, description="主鍵")
    conditions: str = Field(sa_column=Column(Text), description="條件內容（TEXT 格式）")
    results: Optional[Dict[str, Any]] = Field(default=None, sa_column=Column(JSON), description="結果資料（JSONB 格式）")
    description: Optional[str] = Field(default=None, max_length=500, description="條件描述")

    class Config:
        json_encoders = {
            datetime: lambda v: v.isoformat()
        }


class TaskConditionHistory(SQLModel, table=True):
    """
    任務條件歷史表

    存放各種狀態判斷的歷史結果，提供基本的快取功能
    """
    __tablename__ = "task_condition_history"

    id: Optional[int] = Field(default=None, primary_key=True)

    # 條件識別
    condition_type: ConditionType = Field(description="條件類型")
    condition_name: str = Field(max_length=100, description="條件名稱", index=True)
    condition_key: str = Field(max_length=200, description="條件唯一鍵值", index=True)

    # 判斷結果
    is_satisfied: bool = Field(description="條件是否滿足")
    result_data: Dict[str, Any] = Field(default_factory=dict, sa_column=Column(JSON), description="結果資料")

    # 時間戳記
    created_at: datetime = Field(default_factory=lambda: datetime.now(timezone.utc), description="創建時間")
    updated_at: datetime = Field(default_factory=lambda: datetime.now(timezone.utc), description="更新時間", index=True)
    expires_at: datetime = Field(description="過期時間", index=True)

    # 錯誤資訊
    error_message: Optional[str] = Field(default=None, max_length=500, description="錯誤訊息")

    class Config:
        json_encoders = {
            datetime: lambda v: v.isoformat()
        }


class TaskConditionCache(SQLModel, table=True):
    """
    任務條件快取表

    提供高效能的條件查詢快取
    """
    __tablename__ = "task_condition_cache"

    id: Optional[int] = Field(default=None, primary_key=True)

    # 快取鍵值
    cache_key: str = Field(max_length=200, unique=True, description="快取鍵值", index=True)
    cache_group: str = Field(max_length=50, description="快取群組", index=True)
    description: Optional[str] = Field(default=None, max_length=200, description="中文敘述")

    # 快取內容
    condition_result: Dict[str, Any] = Field(sa_column=Column(JSON), description="條件結果")

    # 快取管理
    created_at: datetime = Field(default_factory=lambda: datetime.now(timezone.utc), description="創建時間")
    expires_at: datetime = Field(description="過期時間", index=True)
    hit_count: int = Field(default=0, description="命中次數")
    last_hit_at: Optional[datetime] = Field(default=None, description="最後命中時間")

    class Config:
        json_encoders = {
            datetime: lambda v: v.isoformat()
        }
