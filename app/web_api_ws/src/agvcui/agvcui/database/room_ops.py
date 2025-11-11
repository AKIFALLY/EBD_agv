# database/room_ops.py
"""
房間相關資料庫操作
"""

from sqlmodel import select, func
from db_proxy.models import Room
from .connection import connection_pool, room_crud, process_settings_crud


def get_rooms(offset: int = 0, limit: int = 20):
    """獲取房間列表（分頁）"""
    with connection_pool.get_session() as session:
        statement = select(Room).order_by(Room.id).offset(offset).limit(limit)
        return session.exec(statement).all()


def count_rooms():
    """計算房間總數"""
    with connection_pool.get_session() as session:
        statement = select(func.count()).select_from(Room)
        result = session.exec(statement).one()
        return result


def get_room_by_id(room_id: int):
    """根據 ID 獲取房間"""
    with connection_pool.get_session() as session:
        return room_crud.get_by_id(session, room_id)


def update_room(room_id: int, room_data: dict):
    """更新房間"""
    with connection_pool.get_session() as session:
        # 獲取現有房間
        room = room_crud.get_by_id(session, room_id)
        if not room:
            return None

        # 更新欄位
        if 'process_settings_id' in room_data:
            room.process_settings_id = room_data['process_settings_id']
        if 'enable' in room_data:
            room.enable = room_data['enable']

        # 執行更新
        session.add(room)
        session.commit()
        session.refresh(room)
        return room


def get_all_process_settings():
    """獲取所有製程設置"""
    with connection_pool.get_session() as session:
        return process_settings_crud.get_all(session)


def get_room_locations(room_id: int):
    """獲取房間的入口和出口位置ID"""
    with connection_pool.get_session() as session:
        return room_crud.get_room_locations(session, room_id)
