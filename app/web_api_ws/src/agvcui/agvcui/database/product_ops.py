# database/product_ops.py
"""
產品相關資料庫操作
"""

from datetime import datetime, timezone
from sqlmodel import select, func
from db_proxy.models import Product
from .connection import connection_pool, product_crud, process_settings_crud
from .utils import fetch_all


def product_all() -> list[dict]:
    """獲取所有產品"""
    return fetch_all(product_crud)


def get_products(offset: int = 0, limit: int = 20, filters: dict = None, sort_by: str = "id"):
    """獲取產品列表（分頁、篩選、排序）"""
    with connection_pool.get_session() as session:
        statement = select(Product)

        # 應用篩選條件
        if filters:
            if 'search' in filters and filters['search']:
                search_term = f"%{filters['search']}%"
                statement = statement.where(Product.name.ilike(search_term))

            if 'size' in filters and filters['size']:
                statement = statement.where(Product.size == filters['size'])

        # 應用排序
        if sort_by == "name":
            statement = statement.order_by(Product.name)
        elif sort_by == "created_at":
            statement = statement.order_by(Product.created_at.desc())
        elif sort_by == "updated_at":
            statement = statement.order_by(Product.updated_at.desc())
        else:  # 默認按 id 排序
            statement = statement.order_by(Product.id.desc())

        statement = statement.offset(offset).limit(limit)
        return session.exec(statement).all()


def count_products(filters: dict = None):
    """計算產品總數（含篩選）"""
    with connection_pool.get_session() as session:
        statement = select(func.count()).select_from(Product)

        # 應用篩選條件
        if filters:
            if 'search' in filters and filters['search']:
                search_term = f"%{filters['search']}%"
                statement = statement.where(Product.name.ilike(search_term))

            if 'size' in filters and filters['size']:
                statement = statement.where(Product.size == filters['size'])

        result = session.exec(statement).one()
        return result


def get_product_by_id(product_id: int):
    """根據 ID 獲取產品"""
    with connection_pool.get_session() as session:
        return product_crud.get_by_id(session, product_id)


def create_product(product_data: dict):
    """創建新產品"""
    with connection_pool.get_session() as session:
        now = datetime.now(timezone.utc)

        # 創建 Product 對象
        product_obj = Product(
            name=product_data['name'],
            size=product_data['size'],
            process_settings_id=product_data['process_settings_id'],
            created_at=now,
            updated_at=now
        )
        return product_crud.create(session, product_obj)


def update_product(product_id: int, product_data: dict):
    """更新產品"""
    with connection_pool.get_session() as session:
        now = datetime.now(timezone.utc)

        # 創建 Product 對象用於更新
        product_obj = Product(
            id=product_id,
            name=product_data['name'],
            size=product_data['size'],
            process_settings_id=product_data['process_settings_id'],
            updated_at=now
        )
        return product_crud.update(session, product_id, product_obj)


def delete_product(product_id: int):
    """刪除產品"""
    with connection_pool.get_session() as session:
        return product_crud.delete(session, product_id)


def get_all_products():
    """獲取所有產品"""
    with connection_pool.get_session() as session:
        return product_crud.get_all(session)


def get_all_process_settings():
    """獲取所有製程設置"""
    with connection_pool.get_session() as session:
        return process_settings_crud.get_all(session)
