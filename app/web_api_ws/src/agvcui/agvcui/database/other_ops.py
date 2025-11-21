# database/other_ops.py
"""
其他資料庫操作（機器、房間、節點、AGV等）
"""

from .connection import (
    connection_pool, machine_crud, room_crud, node_crud, edge_crud,
    kuka_node_crud, kuka_edge_crud, node_type_crud, agv_crud,
    modify_log_crud, eqp_signal_crud, location_crud
)
from .utils import fetch_all


# 機器相關
def machine_all() -> list[dict]:
    """獲取所有機器"""
    return fetch_all(machine_crud)


def get_all_machines():
    """獲取所有機器列表"""
    return fetch_all(machine_crud)


# 房間相關
def room_all() -> list[dict]:
    """獲取所有房間"""
    return fetch_all(room_crud)


def get_all_rooms():
    """獲取所有房間列表"""
    return fetch_all(room_crud)


# 信號相關
def signal_all() -> list[dict]:
    """獲取所有信號"""
    return fetch_all(eqp_signal_crud)


# 節點相關
def node_all() -> list[dict]:
    """獲取所有節點"""
    return fetch_all(node_crud)


def edge_all() -> list[dict]:
    """獲取所有邊"""
    return fetch_all(edge_crud)


def kuka_node_all() -> list[dict]:
    """獲取所有 Kuka 節點"""
    return fetch_all(kuka_node_crud)


def kuka_edge_all() -> list[dict]:
    """獲取所有 Kuka 邊"""
    return fetch_all(kuka_edge_crud)


def node_type_all() -> list[dict]:
    """獲取所有節點類型"""
    return fetch_all(node_type_crud)


# AGV 相關
def agv_all() -> list[dict]:
    """獲取所有 AGV"""
    return fetch_all(agv_crud)


def get_all_agvs():
    """獲取所有 AGV"""
    with connection_pool.get_session() as session:
        return agv_crud.get_all(session)


# AGV 狀態相關
def agv_status_all() -> list[dict]:
    """獲取所有 AGV 狀態"""
    from db_proxy.crud.agv_status_crud import agv_status_crud
    with connection_pool.get_session() as session:
        agv_statuses = agv_status_crud.get_all(session)
        return [{"id": status.id, "name": status.name, "description": status.description, "color": status.color}
                for status in agv_statuses]


def get_all_agv_statuses():
    """獲取所有 AGV 狀態物件"""
    from db_proxy.crud.agv_status_crud import agv_status_crud
    with connection_pool.get_session() as session:
        return agv_status_crud.get_all(session)


# 位置相關
def get_all_locations():
    """獲取所有位置"""
    with connection_pool.get_session() as session:
        from db_proxy.models import Location
        from sqlmodel import select
        statement = select(Location).order_by(Location.name)
        locations = session.exec(statement).all()
        return [{"id": loc.id, "name": loc.name, "description": loc.description} for loc in locations]


# 修改日誌相關
def modify_log_all() -> list[dict]:
    """獲取所有修改日誌"""
    return fetch_all(modify_log_crud)


def modify_log_all_objects():
    """取得所有 ModifyLog 物件"""
    with connection_pool.get_session() as session:
        return modify_log_crud.get_all(session)


# 交管區相關
def traffic_zone_all() -> list[dict]:
    """獲取所有交管區及其占用者 AGV 名稱"""
    from db_proxy.crud.traffic_crud import traffic_zone_curd
    with connection_pool.get_session() as session:
        traffic_zones = traffic_zone_curd.get_all(session)
        result = []

        for zone in traffic_zones:
            zone_dict = {
                'id': zone.id,
                'status': zone.status,
                'owner_agv_id': zone.owner_agv_id,
                'enable': zone.enable,
                'owner_agv_name': None
            }

            # 如果有 owner_agv_id，查詢對應的 AGV 名稱
            if zone.owner_agv_id:
                agv = agv_crud.get_by_id(session, zone.owner_agv_id)
                if agv:
                    zone_dict['owner_agv_name'] = agv.name

            result.append(zone_dict)

        return result
