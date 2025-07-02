"""
07. 房間初始化資料
依賴：製程設置
"""

from db_proxy.models import Room
from ..db_install import insert_data_if_not_exists_name


def initialize_rooms(session):
    """初始化房間資料"""
    print("🏠 初始化房間資料...")
    
    default_rooms = [
        {"id": 1, "process_settings_id": 1, "name": "Room1",
            "description": "第一間房間", "enable": 1},
        {"id": 2, "process_settings_id": 1, "name": "Room2",
            "description": "第二間房間", "enable": 1},
        {"id": 3, "process_settings_id": 1, "name": "Room3",
            "description": "第三間房間", "enable": 0},
        {"id": 4, "process_settings_id": 1, "name": "Room4",
            "description": "第四間房間", "enable": 0},
        {"id": 5, "process_settings_id": 1, "name": "Room5",
            "description": "第五間房間", "enable": 0},
    ]
    
    insert_data_if_not_exists_name(session, default_rooms, Room)
    print("✅ 房間資料初始化完成")
