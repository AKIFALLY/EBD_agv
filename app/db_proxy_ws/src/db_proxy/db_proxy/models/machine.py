from typing import Optional, ClassVar, Dict
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

    # 停車格狀態常數定義
    PARKING_AVAILABLE: ClassVar[int] = 0        # 可用 - 停車格空閒，可以叫車
    PARKING_TASK_ACTIVE: ClassVar[int] = 1      # 任務進行中 - 已叫車，等待AGV送達
    PARKING_TASK_COMPLETED: ClassVar[int] = 2   # 任務完成 - 車輛已送達，等待確認取貨

    # 停車格狀態描述
    PARKING_STATUS_DESCRIPTIONS: ClassVar[Dict[int, str]] = {
        0: "可用",
        1: "任務進行中",
        2: "任務完成"
    }

    # 停車格狀態詳細資訊
    PARKING_STATUS_INFO: ClassVar[Dict[int, Dict[str, str]]] = {
        0: {
            'name': '可用',
            'description': '停車格空閒，可以叫車',
            'color': 'is-light'
        },
        1: {
            'name': '任務進行中',
            'description': '已叫車，等待AGV送達',
            'color': 'is-warning'
        },
        2: {
            'name': '任務完成',
            'description': '車輛已送達，等待確認取貨',
            'color': 'is-success'
        }
    }

    @classmethod
    def get_parking_status_info(cls, status_id: int) -> Dict[str, str]:
        """根據狀態 ID 獲取停車格狀態資訊"""
        return cls.PARKING_STATUS_INFO.get(status_id, {
            'name': f'未知({status_id})',
            'description': '未知狀態',
            'color': 'is-light'
        })

    @classmethod
    def get_parking_status_name(cls, status_id: int) -> str:
        """根據狀態 ID 獲取停車格狀態名稱"""
        return cls.get_parking_status_info(status_id)['name']

    @classmethod
    def is_parking_available(cls, status_id: int) -> bool:
        """檢查停車格是否可用"""
        return status_id == cls.PARKING_AVAILABLE

    @classmethod
    def is_parking_task_active(cls, status_id: int) -> bool:
        """檢查停車格是否有進行中的任務"""
        return status_id == cls.PARKING_TASK_ACTIVE

    @classmethod
    def is_parking_task_completed(cls, status_id: int) -> bool:
        """檢查停車格任務是否已完成"""
        return status_id == cls.PARKING_TASK_COMPLETED

    def is_parking_space_1_available(self) -> bool:
        """檢查停車格1是否可用"""
        return self.is_parking_available(self.parking_space_1_status or 0)

    def is_parking_space_2_available(self) -> bool:
        """檢查停車格2是否可用"""
        return self.is_parking_available(self.parking_space_2_status or 0)

    def get_parking_space_1_status_name(self) -> str:
        """取得停車格1的狀態名稱"""
        return self.get_parking_status_name(self.parking_space_1_status or 0)

    def get_parking_space_2_status_name(self) -> str:
        """取得停車格2的狀態名稱"""
        return self.get_parking_status_name(self.parking_space_2_status or 0)

    def get_available_parking_spaces(self) -> list:
        """取得可用的停車格列表"""
        available_spaces = []
        if self.is_parking_space_1_available():
            available_spaces.append(1)
        if self.is_parking_space_2_available():
            available_spaces.append(2)
        return available_spaces

    model_config = ConfigDict(from_attributes=True)  # 告訴 Pydantic 這是 ORM 模型
