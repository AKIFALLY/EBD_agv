from typing import Optional
from sqlmodel import Session
from db_proxy.crud.base_crud import BaseCRUD
from db_proxy.models.room import Room


class RoomCRUD(BaseCRUD):
    """Room 專用的 CRUD 類別，提供額外的便利方法"""

    def get_enter_location_id(self, session: Session, room_id: int) -> Optional[int]:
        """
        取得房間的入口位置ID

        Args:
            session: 資料庫 session
            room_id: 房間ID

        Returns:
            入口位置ID，如果房間不存在則回傳 None
        """
        room = self.get_by_id(session, room_id)
        return room.enter_location_id if room else None

    def get_exit_location_id(self, session: Session, room_id: int) -> Optional[int]:
        """
        取得房間的出口位置ID

        Args:
            session: 資料庫 session
            room_id: 房間ID

        Returns:
            出口位置ID，如果房間不存在則回傳 None
        """
        room = self.get_by_id(session, room_id)
        return room.exit_location_id if room else None

    def get_process_settings_id(self, session: Session, room_id: int) -> Optional[int]:
        """
        取得房間的製程設定ID

        Args:
            session: 資料庫 session
            room_id: 房間ID

        Returns:
            製程設定ID，如果房間不存在則回傳 None
        """
        room = self.get_by_id(session, room_id)
        return room.process_settings_id if room else None

    def get_room_locations(self, session: Session, room_id: int) -> Optional[dict]:
        """
        取得房間的入口和出口位置ID

        Args:
            session: 資料庫 session
            room_id: 房間ID

        Returns:
            包含入口和出口位置ID的字典，格式：
            {"enter_location_id": int, "exit_location_id": int}
            如果房間不存在則回傳 None
        """
        room = self.get_by_id(session, room_id)
        if room:
            return {
                "enter_location_id": room.enter_location_id,
                "exit_location_id": room.exit_location_id
            }
        return None

    def is_room_enabled(self, session: Session, room_id: int) -> bool:
        """
        檢查房間是否啟用

        Args:
            session: 資料庫 session
            room_id: 房間ID

        Returns:
            True 如果房間啟用，False 如果房間停用或不存在
        """
        room = self.get_by_id(session, room_id)
        return room.enable == 1 if room else False


# 創建 Room CRUD 實例
room_crud = RoomCRUD(Room, id_column="id")
