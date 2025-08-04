# db_proxy/services/room_service.py
"""
房間相關的通用業務邏輯服務
"""

from typing import Optional, List, Dict
from sqlmodel import Session
from db_proxy.models import Room
from db_proxy.crud.base_crud import BaseCRUD


class RoomService:
    """房間相關的通用業務邏輯"""
    
    def __init__(self):
        self.crud = BaseCRUD(Room, id_column="id")
    
    def get_process_settings_id(self, room_id: int, session: Session) -> Optional[int]:
        """
        取得房間的製程設定ID
        
        Args:
            room_id: 房間ID
            session: 資料庫 session
            
        Returns:
            製程設定ID，如果房間不存在則回傳 None
        """
        room = self.crud.get_by_id(session, room_id)
        return room.process_settings_id if room else None
    
    def get_enter_location_id(self, room_id: int, session: Session) -> Optional[int]:
        """
        取得房間的入口位置ID（使用硬編碼規則）
        
        Args:
            room_id: 房間ID
            session: 資料庫 session
            
        Returns:
            入口位置ID，格式：room_id * 10000 + 1，如果房間不存在則回傳 None
        """
        room = self.crud.get_by_id(session, room_id)
        return room_id * 10000 + 1 if room else None
    
    def get_exit_location_id(self, room_id: int, session: Session) -> Optional[int]:
        """
        取得房間的出口位置ID（使用硬編碼規則）
        
        Args:
            room_id: 房間ID
            session: 資料庫 session
            
        Returns:
            出口位置ID，格式：room_id * 10000 + 2，如果房間不存在則回傳 None
        """
        room = self.crud.get_by_id(session, room_id)
        return room_id * 10000 + 2 if room else None
    
    def get_room_locations(self, room_id: int, session: Session) -> Optional[Dict[str, int]]:
        """
        取得房間的入口和出口位置ID（使用硬編碼規則）
        
        Args:
            room_id: 房間ID
            session: 資料庫 session
            
        Returns:
            包含入口和出口位置ID的字典，格式：
            {"enter_location_id": int, "exit_location_id": int}
            如果房間不存在則回傳 None
        """
        room = self.crud.get_by_id(session, room_id)
        if room:
            return {
                "enter_location_id": room_id * 10000 + 1,
                "exit_location_id": room_id * 10000 + 2
            }
        return None
    
    def is_room_enabled(self, room_id: int, session: Session) -> bool:
        """
        檢查房間是否啟用
        
        Args:
            room_id: 房間ID
            session: 資料庫 session
            
        Returns:
            True 如果房間啟用，False 如果房間停用或不存在
        """
        room = self.crud.get_by_id(session, room_id)
        return room.enable == 1 if room else False
    
    def get_room_by_name(self, name: str, session: Session) -> Optional[Room]:
        """
        根據名稱查詢房間
        
        Args:
            name: 房間名稱
            session: 資料庫 session
            
        Returns:
            Room 物件或 None
        """
        return self.crud.get_by_field(session, "name", name)
    
    def get_rooms_by_process_settings_id(self, process_settings_id: int, session: Session) -> List[Room]:
        """
        根據製程設定ID查詢房間列表
        
        Args:
            process_settings_id: 製程設定ID
            session: 資料庫 session
            
        Returns:
            房間列表
        """
        from sqlmodel import select
        stmt = select(Room).where(Room.process_settings_id == process_settings_id)
        return session.exec(stmt).all()
    
    def get_enabled_rooms(self, session: Session) -> List[Room]:
        """
        取得所有啟用的房間
        
        Args:
            session: 資料庫 session
            
        Returns:
            啟用的房間列表
        """
        from sqlmodel import select
        stmt = select(Room).where(Room.enable == 1)
        return session.exec(stmt).all()
    
    def get_all_rooms(self, session: Session) -> List[Room]:
        """
        取得所有房間列表
        
        Args:
            session: 資料庫 session
            
        Returns:
            所有房間列表
        """
        return self.crud.get_all(session)
    
    def update_room_status(self, room_id: int, enable: int, session: Session) -> Optional[Room]:
        """
        更新房間啟用狀態
        
        Args:
            room_id: 房間ID
            enable: 啟用狀態 (1=啟用, 0=停用)
            session: 資料庫 session
            
        Returns:
            更新後的房間物件，如果房間不存在則回傳 None
        """
        room = self.crud.get_by_id(session, room_id)
        if room:
            room.enable = enable
            return self.crud.update(session, room_id, room)
        return None
