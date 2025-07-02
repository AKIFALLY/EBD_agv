# database/rack_ops.py
"""
貨架相關資料庫操作
"""

from sqlmodel import select, func
from db_proxy.models import Rack, Carrier, RackStatus
from .connection import connection_pool, rack_crud, product_crud, carrier_crud
from .utils import fetch_all


def rack_all() -> list[dict]:
    """獲取所有貨架（含產品和載具統計）"""
    racks = fetch_all(rack_crud)
    products = fetch_all(product_crud)
    product_map = {p["id"]: p for p in products}

    # 預先抓出所有 carriers 並依 rack_id 分組
    all_carriers = fetch_all(carrier_crud)
    carrier_group = {}
    for c in all_carriers:
        carrier_group.setdefault(c["rack_id"], []).append(c)

    for rack in racks:
        product = product_map.get(rack["product_id"])
        if product:
            rack["size"] = product["size"]
            rack["product_name"] = product["name"]
            rack["total"] = 32 if product["size"] == 'S' else 16
            rack["count"] = len(carrier_group.get(rack["id"], []))
        else:
            rack["size"] = ''
            rack["product_name"] = ''
            rack["total"] = 0
            rack["count"] = 0

    return racks


def get_racks(offset: int = 0, limit: int = 20):
    """獲取貨架列表（分頁）"""
    with connection_pool.get_session() as session:
        statement = select(Rack).order_by(
            Rack.id.desc()).offset(offset).limit(limit)
        return session.exec(statement).all()


def count_racks():
    """計算貨架總數"""
    with connection_pool.get_session() as session:
        statement = select(func.count()).select_from(Rack)
        result = session.exec(statement).one()
        return result


def count_carriers_by_rack(rack_id: int) -> int:
    """計算特定貨架的載具數量"""
    with connection_pool.get_session() as session:
        statement = select(func.count()).select_from(
            Carrier).where(Carrier.rack_id == rack_id)
        return session.exec(statement).one()


def get_rack_by_id(rack_id: int):
    """根據 ID 獲取貨架"""
    with connection_pool.get_session() as session:
        return rack_crud.get_by_id(session, rack_id)


def create_rack(rack_data: dict):
    """創建新貨架"""
    with connection_pool.get_session() as session:
        # 創建 Rack 對象，明確設置 id=None 讓資料庫自動生成
        rack_obj = Rack(
            id=None,  # 明確設置為 None
            name=rack_data['name'],
            agv_id=rack_data.get('agv_id'),
            location_id=rack_data.get('location_id'),
            product_id=rack_data.get('product_id'),
            is_carry=rack_data.get('is_carry', 0),
            is_in_map=rack_data.get('is_in_map', 0),
            is_docked=rack_data.get('is_docked', 0),
            status_id=rack_data.get('status_id', 1),
            direction=rack_data.get('direction', 0)
        )
        return rack_crud.create(session, rack_obj)


def update_rack(rack_id: int, rack_data: dict):
    """更新貨架"""
    with connection_pool.get_session() as session:
        # 創建 Rack 對象用於更新
        rack_obj = Rack(
            id=rack_id,
            name=rack_data['name'],
            agv_id=rack_data.get('agv_id'),
            location_id=rack_data.get('location_id'),
            product_id=rack_data.get('product_id'),
            is_carry=rack_data.get('is_carry', 0),
            is_in_map=rack_data.get('is_in_map', 0),
            is_docked=rack_data.get('is_docked', 0),
            status_id=rack_data.get('status_id', 1),
            direction=rack_data.get('direction', 0)
        )
        return rack_crud.update(session, rack_id, rack_obj)


def delete_rack(rack_id: int):
    """刪除貨架"""
    with connection_pool.get_session() as session:
        return rack_crud.delete(session, rack_id)


def get_all_racks():
    """獲取所有貨架列表"""
    return fetch_all(rack_crud)


def get_all_rack_statuses():
    """獲取所有貨架狀態"""
    with connection_pool.get_session() as session:
        # 創建 RackStatus 的 CRUD
        from db_proxy.crud.base_crud import BaseCRUD
        rack_status_crud = BaseCRUD(RackStatus, id_column="id")
        return rack_status_crud.get_all(session)


def get_rack_grid_info(rack_id: int, product_size: str = 'S'):
    """獲取貨架格位資訊"""
    with connection_pool.get_session() as session:
        # 獲取該貨架上的所有載具
        carriers = session.exec(
            select(Carrier).where(Carrier.rack_id == rack_id)
        ).all()

        # 根據產品尺寸決定格位配置
        if product_size == 'S':
            # S產品: 32格 (A面16格 + B面16格，每面4x4)
            total_slots = 32
            side_slots = 16
            rows = 4
            cols = 4
        else:
            # L產品: 16格 (A面8格 + B面8格，每面2x4)
            total_slots = 16
            side_slots = 8
            rows = 2
            cols = 4

        # 創建格位佔用狀態
        occupied_slots = {
            carrier.rack_index: carrier for carrier in carriers if carrier.rack_index}

        return {
            'total_slots': total_slots,
            'side_slots': side_slots,
            'rows': rows,
            'cols': cols,
            'occupied_slots': occupied_slots,
            'carriers': carriers
        }
