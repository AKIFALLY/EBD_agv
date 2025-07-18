# db_proxy/services/product_service.py
"""
產品相關的通用業務邏輯服務
"""

from typing import Optional, List, Dict, Any
from sqlmodel import Session, select
from db_proxy.models import Product
from db_proxy.crud.base_crud import BaseCRUD


class ProductService:
    """產品相關的通用業務邏輯"""
    
    def __init__(self):
        self.crud = BaseCRUD(Product, id_column="id")
    
    def get_process_settings_id(self, product_id: int, session: Session) -> Optional[int]:
        """
        取得產品的製程設定ID
        
        Args:
            product_id: 產品ID
            session: 資料庫 session
            
        Returns:
            製程設定ID，如果產品不存在則回傳 None
        """
        product = self.crud.get_by_id(session, product_id)
        return product.process_settings_id if product else None
    
    def get_by_name(self, product_name: str, session: Session) -> Optional[Product]:
        """
        根據產品名稱查詢產品
        
        Args:
            product_name: 產品名稱
            session: 資料庫 session
            
        Returns:
            Product 物件，如果找不到則回傳 None
        """
        return self.crud.get_by_field(session, "name", product_name)
    
    def is_product_exists(self, product_name: str, session: Session) -> bool:
        """
        檢查產品名稱是否已存在
        
        Args:
            product_name: 產品名稱
            session: 資料庫 session
            
        Returns:
            True 如果產品存在，False 如果不存在
        """
        product = self.get_by_name(product_name, session)
        return product is not None
    
    def get_products_by_process_settings_id(self, process_settings_id: int, session: Session) -> List[Product]:
        """
        根據製程設定ID查詢產品列表
        
        Args:
            process_settings_id: 製程設定ID
            session: 資料庫 session
            
        Returns:
            產品列表
        """
        stmt = select(Product).where(Product.process_settings_id == process_settings_id)
        return session.exec(stmt).all()
    
    def get_products_by_size(self, size: str, session: Session) -> List[Product]:
        """
        根據尺寸查詢產品列表
        
        Args:
            size: 產品尺寸
            session: 資料庫 session
            
        Returns:
            產品列表
        """
        stmt = select(Product).where(Product.size == size)
        return session.exec(stmt).all()
    
    def get_product_info(self, product_id: int, session: Session) -> Optional[Dict[str, Any]]:
        """
        取得產品完整資訊
        
        Args:
            product_id: 產品ID
            session: 資料庫 session
            
        Returns:
            產品資訊字典，如果產品不存在則回傳 None
        """
        product = self.crud.get_by_id(session, product_id)
        if product:
            return {
                "id": product.id,
                "name": product.name,
                "size": product.size,
                "process_settings_id": product.process_settings_id,
                "created_at": product.created_at,
                "updated_at": product.updated_at
            }
        return None
    
    def get_all_products(self, session: Session) -> List[Product]:
        """
        取得所有產品列表
        
        Args:
            session: 資料庫 session
            
        Returns:
            所有產品列表
        """
        return self.crud.get_all(session)
    
    def create_product(self, product_data: Dict[str, Any], session: Session) -> Product:
        """
        創建新產品
        
        Args:
            product_data: 產品資料字典
            session: 資料庫 session
            
        Returns:
            創建的產品物件
        """
        product = Product(
            name=product_data['name'],
            size=product_data['size'],
            process_settings_id=product_data['process_settings_id']
        )
        return self.crud.create(session, product)
    
    def update_product(self, product_id: int, product_data: Dict[str, Any], session: Session) -> Optional[Product]:
        """
        更新產品資訊
        
        Args:
            product_id: 產品ID
            product_data: 更新的產品資料
            session: 資料庫 session
            
        Returns:
            更新後的產品物件，如果產品不存在則回傳 None
        """
        product = Product(
            id=product_id,
            name=product_data.get('name'),
            size=product_data.get('size'),
            process_settings_id=product_data.get('process_settings_id')
        )
        return self.crud.update(session, product_id, product)
    
    def delete_product(self, product_id: int, session: Session) -> bool:
        """
        刪除產品
        
        Args:
            product_id: 產品ID
            session: 資料庫 session
            
        Returns:
            True 如果刪除成功，False 如果產品不存在
        """
        return self.crud.delete(session, product_id)
