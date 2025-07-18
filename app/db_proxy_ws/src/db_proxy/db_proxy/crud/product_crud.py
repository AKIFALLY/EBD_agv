from typing import Optional, List
from sqlmodel import Session, select
from db_proxy.models import Product
from db_proxy.crud.base_crud import BaseCRUD


class ProductCRUD(BaseCRUD):
    """Product 專用的 CRUD 類別，提供額外的便利方法"""

    def get_process_settings_id(self, session: Session, product_id: int) -> Optional[int]:
        """
        取得產品的製程設定ID

        Args:
            session: 資料庫 session
            product_id: 產品ID

        Returns:
            製程設定ID，如果產品不存在則回傳 None
        """
        product = self.get_by_id(session, product_id)
        return product.process_settings_id if product else None

    def get_product_by_name(self, session: Session, product_name: str) -> Optional[Product]:
        """
        根據產品名稱查詢產品

        Args:
            session: 資料庫 session
            product_name: 產品名稱

        Returns:
            Product 物件，如果找不到則回傳 None
        """
        stmt = select(Product).where(Product.name == product_name)
        return session.exec(stmt).first()

    def get_products_by_process_settings_id(self, session: Session, process_settings_id: int) -> List[Product]:
        """
        根據製程設定ID查詢所有相關產品

        Args:
            session: 資料庫 session
            process_settings_id: 製程設定ID

        Returns:
            產品列表
        """
        stmt = select(Product).where(Product.process_settings_id == process_settings_id)
        return list(session.exec(stmt).all())

    def get_products_by_size(self, session: Session, size: str) -> List[Product]:
        """
        根據產品尺寸查詢產品

        Args:
            session: 資料庫 session
            size: 產品尺寸 (如 "S", "L")

        Returns:
            產品列表
        """
        stmt = select(Product).where(Product.size == size)
        return list(session.exec(stmt).all())

    def get_product_info(self, session: Session, product_id: int) -> Optional[dict]:
        """
        取得產品的完整資訊

        Args:
            session: 資料庫 session
            product_id: 產品ID

        Returns:
            包含產品完整資訊的字典，格式：
            {
                "id": int,
                "name": str,
                "size": str,
                "process_settings_id": int,
                "created_at": datetime,
                "updated_at": datetime
            }
            如果產品不存在則回傳 None
        """
        product = self.get_by_id(session, product_id)
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

    def is_product_exists(self, session: Session, product_name: str) -> bool:
        """
        檢查產品名稱是否已存在

        Args:
            session: 資料庫 session
            product_name: 產品名稱

        Returns:
            True 如果產品存在，False 如果不存在
        """
        product = self.get_product_by_name(session, product_name)
        return product is not None


# 創建 Product CRUD 實例
product_crud = ProductCRUD(Product, id_column="id")
