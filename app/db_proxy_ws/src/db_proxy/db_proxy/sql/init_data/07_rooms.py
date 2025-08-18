"""
07. 房間初始化資料
依賴：製程設置
"""

from db_proxy.models import Room, ProcessSettings
from sqlmodel import select
from ..db_install import insert_data_if_not_exists_name


def initialize_rooms(session):
    """初始化房間資料"""
    print("🏠 初始化房間資料...")
    
    # 獲取第一個 process_settings 的 ID (soaking_times=1 的那個)
    process_setting = session.exec(
        select(ProcessSettings).where(ProcessSettings.soaking_times == 1)
    ).first()
    
    if not process_setting:
        print("⚠️ 警告：找不到 process_settings 資料，跳過房間初始化")
        return
    
    process_settings_id = process_setting.id
    print(f"📌 使用 process_settings_id: {process_settings_id}")
    
    default_rooms = [
        {"id": 1, "process_settings_id": process_settings_id, "name": "Room1",
            "description": "第一間房間", "enable": 1},
        {"id": 2, "process_settings_id": process_settings_id, "name": "Room2",
            "description": "第二間房間", "enable": 1},
        {"id": 3, "process_settings_id": process_settings_id, "name": "Room3",
            "description": "第三間房間", "enable": 0},
        {"id": 4, "process_settings_id": process_settings_id, "name": "Room4",
            "description": "第四間房間", "enable": 0},
        {"id": 5, "process_settings_id": process_settings_id, "name": "Room5",
            "description": "第五間房間", "enable": 0},
    ]
    
    insert_data_if_not_exists_name(session, default_rooms, Room)
    print("✅ 房間資料初始化完成")
