"""
02. 位置狀態初始化資料
無相依性
"""

from db_proxy.models import LocationStatus
from ..db_install import insert_data_if_not_exists_name


def initialize_location_status(session):
    """初始化位置狀態資料"""
    print("📍 初始化位置狀態資料...")
    
    default_location_status = [
        {"id": 1, "name": "未知狀態", "description": "未知狀態"},
        {"id": 2, "name": "未佔用", "description": "空位沒有被使用"},
        {"id": 3, "name": "佔用", "description": "已經有停放的料架"},
        {"id": 4, "name": "任務佔用中", "description": "有任務正在使用此位置"},
    ]
    
    insert_data_if_not_exists_name(session, default_location_status, LocationStatus)
    print("✅ 位置狀態資料初始化完成")
