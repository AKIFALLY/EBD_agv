"""
載具狀態初始化資料
"""

from db_proxy.models import CarrierStatus
from db_proxy.sql.db_install import insert_data_if_not_exists_name_and_not_exists_id


def initialize_carrier_status(session):
    """初始化載具狀態資料"""
    print("📋 初始化載具狀態資料...")
    
    carrier_status_data = [
        {
            "id": 1,
            "name": "空閒",
            "description": "載具空閒，可以使用",
            "color": "is-success"
        },
        {
            "id": 2,
            "name": "使用中",
            "description": "載具正在使用中",
            "color": "is-warning"
        },
        {
            "id": 3,
            "name": "故障",
            "description": "載具發生故障",
            "color": "is-danger"
        },
        {
            "id": 4,
            "name": "待處理",
            "description": "載具等待處理",
            "color": "is-info"
        },
        {
            "id": 5,
            "name": "處理中",
            "description": "載具正在處理製程",
            "color": "is-primary"
        },
        {
            "id": 6,
            "name": "NG",
            "description": "載具處理結果不良",
            "color": "is-dark"
        },
        {
            "id": 7,
            "name": "維護中",
            "description": "載具正在維護",
            "color": "is-light"
        },
        {
            "id": 8,
            "name": "已完成",
            "description": "載具處理完成",
            "color": "is-link"
        }
    ]
    
    insert_data_if_not_exists_name_and_not_exists_id(
        session, carrier_status_data, CarrierStatus
    )
    
    print(f"   ✅ 載具狀態資料: {len(carrier_status_data)} 筆")
