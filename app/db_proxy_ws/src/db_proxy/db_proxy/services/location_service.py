# db_proxy/services/location_service.py
"""
位置相關的通用業務邏輯服務
"""

from typing import Optional, List
from sqlmodel import Session, select
from db_proxy.models import Location, LocationStatus
from db_proxy.crud.base_crud import BaseCRUD


class LocationService:
    """位置相關的通用業務邏輯"""
    
    def __init__(self):
        self.crud = BaseCRUD(Location, id_column="id")
    
    def get_by_node_id(self, node_id: int, session: Session) -> Optional[Location]:
        """
        根據 node_id 查詢位置
        
        Args:
            node_id: 節點ID
            session: 資料庫 session
            
        Returns:
            Location 物件或 None（如果找不到對應的位置）
        """
        stmt = select(Location).where(Location.node_id == node_id)
        return session.exec(stmt).first()
    
    def get_by_room_id(self, room_id: int, session: Session) -> List[Location]:
        """
        根據房間ID查詢位置列表
        
        Args:
            room_id: 房間ID
            session: 資料庫 session
            
        Returns:
            位置列表
        """
        stmt = select(Location).where(Location.room_id == room_id)
        return session.exec(stmt).all()
    
    def is_location_available(self, location: Location) -> bool:
        """
        判斷位置是否可用
        
        Args:
            location: 位置物件
            
        Returns:
            True 如果位置可用，False 如果被佔用
        """
        return location.location_status_id == LocationStatus.UNOCCUPIED
    
    def is_location_occupied(self, location: Location) -> bool:
        """
        判斷位置是否被佔用
        
        Args:
            location: 位置物件
            
        Returns:
            True 如果位置被佔用，False 如果可用
        """
        return location.location_status_id == LocationStatus.OCCUPIED
    
    def get_available_locations(self, session: Session) -> List[Location]:
        """
        取得所有可用的位置
        
        Args:
            session: 資料庫 session
            
        Returns:
            可用位置列表
        """
        stmt = select(Location).where(Location.location_status_id == LocationStatus.UNOCCUPIED)
        return session.exec(stmt).all()
    
    def get_occupied_locations(self, session: Session) -> List[Location]:
        """
        取得所有被佔用的位置
        
        Args:
            session: 資料庫 session
            
        Returns:
            被佔用位置列表
        """
        stmt = select(Location).where(Location.location_status_id == LocationStatus.OCCUPIED)
        return session.exec(stmt).all()
    
    def update_location_status(self, location_id: int, status_id: int, session: Session) -> Optional[Location]:
        """
        更新位置狀態
        
        Args:
            location_id: 位置ID
            status_id: 新的狀態ID
            session: 資料庫 session
            
        Returns:
            更新後的位置物件，如果位置不存在則回傳 None
        """
        location = self.crud.get_by_id(session, location_id)
        if location:
            location.location_status_id = status_id
            return self.crud.update(session, location_id, location)
        return None
    
    def get_location_by_name(self, name: str, session: Session) -> Optional[Location]:
        """
        根據名稱查詢位置
        
        Args:
            name: 位置名稱
            session: 資料庫 session
            
        Returns:
            Location 物件或 None
        """
        return self.crud.get_by_field(session, "name", name)
    
    def get_all_locations(self, session: Session) -> List[Location]:
        """
        取得所有位置列表
        
        Args:
            session: 資料庫 session
            
        Returns:
            所有位置列表
        """
        return self.crud.get_all(session)
