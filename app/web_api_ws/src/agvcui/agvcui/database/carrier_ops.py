# database/carrier_ops.py
"""
載具相關資料庫操作
"""

from sqlmodel import select, func
from db_proxy.models import Carrier, Rack, EqpPort, Eqp
from .connection import connection_pool, carrier_crud, carrier_status_crud
from .utils import fetch_all


def carrier_all() -> list[dict]:
    """獲取所有載具"""
    return fetch_all(carrier_crud)


def get_carriers(offset: int = 0, limit: int = 20):
    """獲取載具列表（分頁）"""
    with connection_pool.get_session() as session:
        statement = select(Carrier).order_by(
            Carrier.id.desc()).offset(offset).limit(limit)
        return session.exec(statement).all()


def count_carriers():
    """計算載具總數"""
    with connection_pool.get_session() as session:
        statement = select(func.count()).select_from(Carrier)
        result = session.exec(statement).one()
        return result


def get_carriers_grouped():
    """獲取按房間和貨架分組的載具資料"""
    with connection_pool.get_session() as session:
        # 獲取在貨架上的載具，包含產品資訊
        from db_proxy.models import Product
        rack_carriers_stmt = select(
            Carrier,
            Rack.name.label('rack_name'),
            Rack.product_id.label('product_id'),
            Product.size.label('product_size')
        ).join(
            Rack, Carrier.rack_id == Rack.id, isouter=True
        ).join(
            Product, Rack.product_id == Product.id, isouter=True
        ).where(Carrier.rack_id.is_not(None))

        rack_carriers = session.exec(rack_carriers_stmt).all()

        # 按貨架分組處理
        rack_groups = {}
        for carrier, rack_name, product_id, product_size in rack_carriers:
            rack_id = carrier.rack_id
            if rack_id not in rack_groups:
                rack_groups[rack_id] = {
                    'rack_id': rack_id,
                    'rack_name': rack_name,
                    'product_id': product_id,
                    'product_size': product_size or 'S',  # 預設為 S
                    'carriers': []
                }
            rack_groups[rack_id]['carriers'].append((carrier, rack_name))

        # 獲取房間內的載具 - 根據設備的 location_id 判斷房間
        # 先獲取所有在設備端口的載具
        port_carriers_stmt = select(
            Carrier,
            EqpPort.name.label('port_name'),
            Eqp.name.label('eqp_name'),
            Eqp.id.label('eqp_id'),
            Eqp.location_id.label('location_id')
        ).join(
            EqpPort, Carrier.port_id == EqpPort.id
        ).join(
            Eqp, EqpPort.eqp_id == Eqp.id
        ).where(Carrier.port_id.is_not(None))

        port_carriers = session.exec(port_carriers_stmt).all()

        # 按房間分組設備和載具
        room_groups = {}
        for carrier, port_name, eqp_name, eqp_id, location_id in port_carriers:
            # 根據設備的 location_id 判斷房間
            # 1xx -> 房間1, 2xx -> 房間2, 3xx -> 房間3, 4xx -> 房間4, 5xx -> 房間5
            room_number = None
            if location_id:
                # 200-299 -> 2, 100-199 -> 1, etc.
                room_number = location_id // 100
                if room_number < 1 or room_number > 5:
                    room_number = None  # 只處理房間1-5

            if room_number:
                if room_number not in room_groups:
                    room_groups[room_number] = {
                        'room_number': room_number,
                        'room_name': f'房間 {room_number}',
                        'equipment': {}
                    }

                # 按設備分組
                if eqp_id not in room_groups[room_number]['equipment']:
                    room_groups[room_number]['equipment'][eqp_id] = {
                        'eqp_id': eqp_id,
                        'eqp_name': eqp_name,
                        'location_id': location_id,
                        'ports': {}
                    }

                # 按端口分組載具
                port_id = carrier.port_id
                if port_id not in room_groups[room_number]['equipment'][eqp_id]['ports']:
                    room_groups[room_number]['equipment'][eqp_id]['ports'][port_id] = {
                        'port_id': port_id,
                        'port_name': port_name,
                        'carriers': []
                    }

                room_groups[room_number]['equipment'][eqp_id]['ports'][port_id]['carriers'].append(
                    carrier)

        # 轉換為列表格式
        room_groups_list = []
        for room_number in sorted(room_groups.keys()):
            room_data = room_groups[room_number]
            room_data['equipment'] = list(room_data['equipment'].values())
            for equipment in room_data['equipment']:
                equipment['ports'] = list(equipment['ports'].values())
            room_groups_list.append(room_data)

        # 獲取未分配的載具
        unassigned_carriers_stmt = select(Carrier).where(
            Carrier.rack_id.is_(None),
            Carrier.port_id.is_(None)
        )

        unassigned_carriers = session.exec(unassigned_carriers_stmt).all()

        return {
            'rack_groups': list(rack_groups.values()),
            'room_groups': room_groups_list,
            'unassigned_carriers': unassigned_carriers
        }


def get_carrier_status_list():
    """獲取所有載具狀態"""
    with connection_pool.get_session() as session:
        return carrier_status_crud.get_all(session)


def get_carrier_by_id(carrier_id: int):
    """根據 ID 獲取載具詳細資訊"""
    with connection_pool.get_session() as session:
        statement = select(Carrier).where(Carrier.id == carrier_id)
        return session.exec(statement).first()


def update_carrier(carrier_id: int, carrier_data: dict):
    """更新載具資訊"""
    with connection_pool.get_session() as session:
        carrier = session.exec(select(Carrier).where(
            Carrier.id == carrier_id)).first()
        if not carrier:
            return None

        # 更新允許的欄位
        for field in ['room_id', 'rack_id', 'port_id', 'rack_index', 'status_id']:
            if field in carrier_data:
                setattr(carrier, field, carrier_data[field])

        session.add(carrier)
        session.commit()
        session.refresh(carrier)
        return carrier


def delete_carrier(carrier_id: int):
    """刪除載具"""
    with connection_pool.get_session() as session:
        carrier = session.exec(select(Carrier).where(
            Carrier.id == carrier_id)).first()
        if not carrier:
            return False

        session.delete(carrier)
        session.commit()
        return True


def create_carrier(carrier_data: dict):
    """創建新載具"""
    with connection_pool.get_session() as session:
        carrier = Carrier(**carrier_data)
        session.add(carrier)
        session.commit()
        session.refresh(carrier)
        return carrier
