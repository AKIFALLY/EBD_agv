"""
11. 貨架初始化資料
依賴：位置、AGV、產品、貨架狀態
"""

from db_proxy.models import Rack
from ..db_install import insert_data_if_not_exists_name


def initialize_racks(session):
    """初始化貨架資料"""
    print("🗄️ 初始化貨架資料...")
    
    default_rack = [
        {"id": 1, "name": "001", "location_id": 51, "agv_id": None, "is_carry": 0, "product_id": 1, "status_id": 1, "direction": 0, "is_docked": 1, "carrier_bitmap": "00000000", "carrier_enable_bitmap": "FFFFFFFF"},
        {"id": 2, "name": "002", "location_id": 52, "agv_id": None, "is_carry": 0, "product_id": 1, "status_id": 1, "direction": 0, "carrier_bitmap": "00000000", "carrier_enable_bitmap": "FFFFFFFF"},
        {"id": 3, "name": "003", "location_id": 53, "agv_id": None, "is_carry": 0, "product_id": 1, "status_id": 1, "direction": 180, "carrier_bitmap": "00000000", "carrier_enable_bitmap": "FFFFFFFF"},
        {"id": 4, "name": "004", "location_id": 54, "agv_id": None, "is_carry": 0, "product_id": 2, "status_id": 1, "direction": 180, "carrier_bitmap": "00000000", "carrier_enable_bitmap": "0F0F0F0F"},
        {"id": 5, "name": "005", "location_id": 55, "agv_id": None, "is_carry": 0, "product_id": 2, "status_id": 1, "direction": 180, "carrier_bitmap": "00000000", "carrier_enable_bitmap": "0F0F0F0F"}
    ]
    
    insert_data_if_not_exists_name(session, default_rack, Rack)
    print("✅ 貨架資料初始化完成")
