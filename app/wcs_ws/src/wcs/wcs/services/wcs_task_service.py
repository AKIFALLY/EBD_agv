# wcs/services/wcs_task_service.py
"""
WCS 特定的任務業務邏輯服務
"""

from typing import Optional
from sqlmodel import Session
from db_proxy.services import ProductService, LocationService, RoomService
from db_proxy.models import Location


class WcsTaskService:
    """WCS 特定的任務業務邏輯"""
    
    def __init__(self, db_pool, logger):
        self.db_pool = db_pool
        self.logger = logger
        
        # 使用 db_proxy 的通用 Service
        self.product_service = ProductService()
        self.location_service = LocationService()
        self.room_service = RoomService()
    
    def get_product_process_settings(self, product_id: int, session=None) -> Optional[int]:
        """
        根據 product_id 查詢資料庫中的 Product 表，取得對應的 process_settings_id
        支援 session 管理的業務邏輯層方法
        
        Args:
            product_id: 產品 ID
            session: 資料庫 session，如果為 None 則使用連線池取得新的 session
            
        Returns:
            process_settings_id 或 None（如果找不到產品）
        """
        if session is None:
            with self.db_pool.get_session() as session:
                return self._get_product_process_settings_impl(product_id, session)
        else:
            return self._get_product_process_settings_impl(product_id, session)
    
    def _get_product_process_settings_impl(self, product_id: int, session) -> Optional[int]:
        """實際執行產品查詢的內部方法"""
        try:
            process_settings_id = self.product_service.get_process_settings_id(product_id, session)
            if process_settings_id:
                self.logger.debug(f"找到產品 {product_id}，process_settings_id: {process_settings_id}")
                return process_settings_id
            else:
                self.logger.warning(f"找不到產品 ID: {product_id}")
                return None
        except Exception as e:
            self.logger.error(f"查詢產品 {product_id} 時發生錯誤: {e}")
            return None
    
    def get_location_by_node(self, node_id: int, session) -> Optional[Location]:
        """
        根據 node_id 查詢資料庫中的 Location 表，取得對應的 Location 物件
        WCS 特定的位置查詢邏輯，包含日誌記錄
        
        Args:
            node_id: 節點 ID
            session: 資料庫 session
            
        Returns:
            Location 物件或 None（如果找不到對應的位置）
        """
        try:
            location = self.location_service.get_by_node_id(node_id, session)
            
            if location:
                self.logger.debug(f"找到節點 {node_id} 對應的位置: {location.name} (ID: {location.id})")
                return location
            else:
                self.logger.warning(f"找不到節點 ID {node_id} 對應的位置")
                return None
        except Exception as e:
            self.logger.error(f"查詢節點 {node_id} 對應位置時發生錯誤: {e}")
            return None
    
    def location_is_available(self, location: Location) -> bool:
        """
        判斷 Location 是否可用
        WCS 特定的位置可用性檢查邏輯
        
        Args:
            location: Location 物件
            
        Returns:
            True 如果位置可用，False 如果不可用
        """
        return self.location_service.is_location_available(location)
    
    def validate_dispatch_task(self, rack_id: int, room_id: int, target_node_id: int, session) -> tuple[bool, str]:
        """
        驗證派車任務的條件
        WCS 特定的派車驗證邏輯
        
        Args:
            rack_id: 料架ID
            room_id: 房間ID
            target_node_id: 目標節點ID
            session: 資料庫 session
            
        Returns:
            (是否通過驗證, 錯誤訊息)
        """
        try:
            # 檢查房間是否啟用
            if not self.room_service.is_room_enabled(room_id, session):
                return False, f"房間 {room_id} 未啟用"
            
            # 檢查目標位置是否存在且可用
            location = self.get_location_by_node(target_node_id, session)
            if not location:
                return False, f"找不到節點 {target_node_id} 對應的位置"
            
            if not self.location_is_available(location):
                return False, f"目標位置 {location.name} 不可用"
            
            return True, "驗證通過"
            
        except Exception as e:
            self.logger.error(f"驗證派車任務時發生錯誤: {e}")
            return False, f"驗證過程發生錯誤: {str(e)}"
    
    def validate_product_room_compatibility(self, product_id: int, room_id: int, session) -> tuple[bool, str]:
        """
        驗證產品與房間的製程相容性
        WCS 特定的相容性檢查邏輯
        
        Args:
            product_id: 產品ID
            room_id: 房間ID
            session: 資料庫 session
            
        Returns:
            (是否相容, 訊息)
        """
        try:
            product_process_id = self.product_service.get_process_settings_id(product_id, session)
            if product_process_id is None:
                return False, f"找不到產品 {product_id} 的製程設定"
            
            room_process_id = self.room_service.get_process_settings_id(room_id, session)
            if room_process_id is None:
                return False, f"找不到房間 {room_id} 的製程設定"
            
            if product_process_id != room_process_id:
                return False, f"產品製程 {product_process_id} 與房間製程 {room_process_id} 不相容"
            
            return True, "製程相容"
            
        except Exception as e:
            self.logger.error(f"檢查製程相容性時發生錯誤: {e}")
            return False, f"檢查過程發生錯誤: {str(e)}"
