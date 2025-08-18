"""
09. 產品初始化資料
依賴：製程設置
"""

from db_proxy.models import Product
from ..db_install import insert_data_if_not_exists_name


def initialize_products(session):
    """初始化產品資料"""
    print("📦 初始化產品資料...")
    
    default_products = [
        {"id":1,"name": "ABC12345", "size": "S", "process_settings_id": 1},
        {"id":2,"name": "DEF67890", "size": "L", "process_settings_id": 2},
        {"id":3,"name": "ABC54321", "size": "S", "process_settings_id": 1},
        {"id":4,"name": "DEF09876", "size": "L", "process_settings_id": 2}
    ]
    
    insert_data_if_not_exists_name(session, default_products, Product)
    print("✅ 產品資料初始化完成")
