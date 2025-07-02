# database/equipment_ops.py
"""
設備相關資料庫操作
"""

from sqlmodel import select, func
from sqlalchemy.orm import selectinload
from db_proxy.models import Eqp, EqpPort, EqpSignal
from .connection import connection_pool, eqp_crud, eqp_port_crud, eqp_signal_crud


def get_eqps(offset: int = 0, limit: int = 20):
    """獲取設備列表（分頁）"""
    with connection_pool.get_session() as session:
        statement = (
            select(Eqp)
            .options(selectinload(Eqp.ports))  # 顯式載入 ports 關聯
            .order_by(Eqp.id.desc())
            .offset(offset)
            .limit(limit)
        )
        return session.exec(statement).all()


def count_eqps():
    """計算設備總數"""
    with connection_pool.get_session() as session:
        statement = select(func.count()).select_from(Eqp)
        result = session.exec(statement).one()
        return result


def get_signals(offset: int = 0, limit: int = 20):
    """獲取信號列表（分頁）"""
    with connection_pool.get_session() as session:
        statement = select(EqpSignal).order_by(
            EqpSignal.id.desc()).offset(offset).limit(limit)
        return session.exec(statement).all()


def count_signals():
    """計算信號總數"""
    with connection_pool.get_session() as session:
        statement = select(func.count()).select_from(EqpSignal)
        return session.exec(statement).one()


def get_signals_by_eqp_id(eqp_id: int, offset: int = 0, limit: int = 20):
    """根據設備ID獲取信號列表"""
    with connection_pool.get_session() as session:
        statement = select(EqpSignal).where(EqpSignal.eqp_id == eqp_id).order_by(
            EqpSignal.id.desc()).offset(offset).limit(limit)
        return session.exec(statement).all()


def count_signals_by_eqp_id(eqp_id: int):
    """計算特定設備的信號數量"""
    with connection_pool.get_session() as session:
        statement = select(func.count()).select_from(
            EqpSignal).where(EqpSignal.eqp_id == eqp_id)
        return session.exec(statement).one()


def get_eqps_with_signal_counts():
    """獲取所有設備及其信號數量"""
    with connection_pool.get_session() as session:
        # 獲取所有設備
        eqps_statement = select(Eqp).order_by(Eqp.name)
        eqps = session.exec(eqps_statement).all()

        # 為每個設備計算信號數量
        eqps_with_counts = []
        for eqp in eqps:
            signal_count = count_signals_by_eqp_id(eqp.id)
            eqps_with_counts.append({
                "id": eqp.id,
                "name": eqp.name,
                "description": eqp.description,
                "signal_count": signal_count
            })

        return eqps_with_counts


def get_eqp_by_id(eqp_id: int):
    """根據 ID 獲取設備"""
    with connection_pool.get_session() as session:
        return eqp_crud.get_by_id(session, eqp_id)


def create_eqp(eqp_data: dict):
    """創建新設備"""
    with connection_pool.get_session() as session:
        # 創建 Eqp 對象
        eqp_obj = Eqp(**eqp_data)
        return eqp_crud.create(session, eqp_obj)


def update_eqp(eqp_id: int, eqp_data: dict):
    """更新設備"""
    with connection_pool.get_session() as session:
        # 創建 Eqp 對象用於更新
        eqp_obj = Eqp(**eqp_data)
        return eqp_crud.update(session, eqp_id, eqp_obj)


def delete_eqp(eqp_id: int):
    """刪除設備"""
    with connection_pool.get_session() as session:
        return eqp_crud.delete(session, eqp_id)


def get_eqp_ports_by_eqp_id(eqp_id: int):
    """根據設備ID獲取端口列表"""
    with connection_pool.get_session() as session:
        statement = select(EqpPort).where(EqpPort.eqp_id == eqp_id)
        return session.exec(statement).all()


def create_eqp_port(port_data: dict):
    """創建新端口"""
    with connection_pool.get_session() as session:
        port_obj = EqpPort(**port_data)
        return eqp_port_crud.create(session, port_obj)


def delete_eqp_port(port_id: int):
    """刪除端口"""
    with connection_pool.get_session() as session:
        return eqp_port_crud.delete(session, port_id)


def check_port_has_signals(port_id: int) -> bool:
    """檢查端口是否被 signal 引用"""
    with connection_pool.get_session() as session:
        statement = select(EqpSignal).where(EqpSignal.eqp_port_id == port_id)
        signals = session.exec(statement).all()
        return len(signals) > 0


def update_eqp_port(port_id: int, port_data: dict):
    """更新端口"""
    with connection_pool.get_session() as session:
        port_obj = EqpPort(**port_data)
        return eqp_port_crud.update(session, port_id, port_obj)


def get_all_eqp_ports():
    """獲取所有設備端口列表"""
    with connection_pool.get_session() as session:
        statement = select(EqpPort, Eqp.name.label('eqp_name')).join(
            Eqp, EqpPort.eqp_id == Eqp.id, isouter=True
        )
        results = session.exec(statement).all()

        # 將結果轉換為包含設備名稱的格式
        ports = []
        for port, eqp_name in results:
            port_dict = port.model_dump()
            port_dict['eqp'] = {'name': eqp_name} if eqp_name else None
            ports.append(type('Port', (), port_dict)())

        return ports


def get_complete_device(device_id: int) -> dict:
    """獲取完整的設備信息，包含 eqp + ports + signals"""
    with connection_pool.get_session() as session:
        # 獲取設備基本信息
        device = get_eqp_by_id(device_id)
        if not device:
            return None

        # 獲取設備的端口
        ports = get_eqp_ports_by_eqp_id(device_id)

        # 獲取設備的信號
        statement = select(EqpSignal).where(EqpSignal.eqp_id == device_id)
        signals = session.exec(statement).all()

        # 組織端口和信號的關聯
        ports_data = []
        for port in ports:
            # 獲取該端口的信號
            port_signals = [s for s in signals if s.eqp_port_id == port.id]
            ports_data.append({
                "id": port.id,
                "name": port.name,
                "description": port.description,
                "signals": [
                    {
                        "id": signal.id,
                        "name": signal.name,
                        "description": signal.description,
                        "value": signal.value,
                        "type_of_value": signal.type_of_value,
                        "dm_address": signal.dm_address
                    } for signal in port_signals
                ]
            })

        # 獲取不屬於任何端口的信號
        device_signals = [s for s in signals if s.eqp_port_id is None]

        return {
            "id": device.id,
            "name": device.name,
            "description": device.description,
            "location_id": device.location_id,
            "ports": ports_data,
            "device_signals": [
                {
                    "id": signal.id,
                    "name": signal.name,
                    "description": signal.description,
                    "value": signal.value,
                    "type_of_value": signal.type_of_value,
                    "dm_address": signal.dm_address
                } for signal in device_signals
            ]
        }


def create_complete_device(device_data: dict) -> dict:
    """創建完整的設備，包含端口和信號"""
    with connection_pool.get_session() as session:
        try:
            # 創建設備基本信息
            eqp_data = {
                "name": device_data["name"],
                "description": device_data.get("description"),
                "location_id": device_data.get("location_id")
            }
            new_device = create_eqp(eqp_data)

            # 創建端口和信號
            if "ports" in device_data:
                for port_data in device_data["ports"]:
                    # 創建端口
                    port_obj_data = {
                        "eqp_id": new_device.id,
                        "name": port_data["name"],
                        "description": port_data.get("description", f"{device_data['name']} {port_data['name']}")
                    }
                    new_port = create_eqp_port(port_obj_data)

                    # 創建端口的信號
                    if "signals" in port_data:
                        for signal_data in port_data["signals"]:
                            signal_obj_data = {
                                "eqp_id": new_device.id,
                                "eqp_port_id": new_port.id,
                                "name": signal_data["name"],
                                "description": signal_data.get("description"),
                                "value": signal_data.get("value", ""),
                                "type_of_value": signal_data.get("type_of_value", "string"),
                                "dm_address": signal_data.get("dm_address")
                            }
                            signal_obj = EqpSignal(**signal_obj_data)
                            eqp_signal_crud.create(session, signal_obj)

            # 創建設備級別的信號（不屬於任何端口）
            if "device_signals" in device_data:
                for signal_data in device_data["device_signals"]:
                    signal_obj_data = {
                        "eqp_id": new_device.id,
                        "eqp_port_id": None,
                        "name": signal_data["name"],
                        "description": signal_data.get("description"),
                        "value": signal_data.get("value", ""),
                        "type_of_value": signal_data.get("type_of_value", "string"),
                        "dm_address": signal_data.get("dm_address")
                    }
                    signal_obj = EqpSignal(**signal_obj_data)
                    eqp_signal_crud.create(session, signal_obj)

            return get_complete_device(new_device.id)

        except Exception as e:
            session.rollback()
            raise e


def update_complete_device(device_id: int, device_data: dict) -> dict:
    """更新完整的設備，包含端口和信號"""
    with connection_pool.get_session() as session:
        try:
            # 更新設備基本信息
            eqp_data = {
                "name": device_data["name"],
                "description": device_data.get("description"),
                "location_id": device_data.get("location_id")
            }
            success = update_eqp(device_id, eqp_data)
            if not success:
                raise Exception("設備不存在")

            # 獲取現有的端口和信號
            current_ports = get_eqp_ports_by_eqp_id(device_id)
            current_signals_stmt = select(EqpSignal).where(
                EqpSignal.eqp_id == device_id)
            current_signals = session.exec(current_signals_stmt).all()

            # 刪除所有現有的信號（因為它們會重新創建）
            for signal in current_signals:
                eqp_signal_crud.delete(session, signal.id)

            # 刪除所有現有的端口（現在沒有信號引用了）
            for port in current_ports:
                delete_eqp_port(port.id)

            # 重新創建端口和信號
            if "ports" in device_data:
                for port_data in device_data["ports"]:
                    # 創建端口
                    port_obj_data = {
                        "eqp_id": device_id,
                        "name": port_data["name"],
                        "description": port_data.get("description", f"{device_data['name']} {port_data['name']}")
                    }
                    new_port = create_eqp_port(port_obj_data)

                    # 創建端口的信號
                    if "signals" in port_data:
                        for signal_data in port_data["signals"]:
                            signal_obj_data = {
                                "eqp_id": device_id,
                                "eqp_port_id": new_port.id,
                                "name": signal_data["name"],
                                "description": signal_data.get("description"),
                                "value": signal_data.get("value", ""),
                                "type_of_value": signal_data.get("type_of_value", "string"),
                                "dm_address": signal_data.get("dm_address")
                            }
                            signal_obj = EqpSignal(**signal_obj_data)
                            eqp_signal_crud.create(session, signal_obj)

            # 重新創建設備級別的信號
            if "device_signals" in device_data:
                for signal_data in device_data["device_signals"]:
                    signal_obj_data = {
                        "eqp_id": device_id,
                        "eqp_port_id": None,
                        "name": signal_data["name"],
                        "description": signal_data.get("description"),
                        "value": signal_data.get("value", ""),
                        "type_of_value": signal_data.get("type_of_value", "string"),
                        "dm_address": signal_data.get("dm_address")
                    }
                    signal_obj = EqpSignal(**signal_obj_data)
                    eqp_signal_crud.create(session, signal_obj)

            return get_complete_device(device_id)

        except Exception as e:
            session.rollback()
            raise e


def delete_complete_device(device_id: int) -> bool:
    """刪除完整的設備，包含所有端口和信號"""
    with connection_pool.get_session() as session:
        try:
            # 先刪除所有信號
            signals_stmt = select(EqpSignal).where(
                EqpSignal.eqp_id == device_id)
            signals = session.exec(signals_stmt).all()
            for signal in signals:
                eqp_signal_crud.delete(session, signal.id)

            # 再刪除所有端口
            ports = get_eqp_ports_by_eqp_id(device_id)
            for port in ports:
                delete_eqp_port(port.id)

            # 最後刪除設備
            success = delete_eqp(device_id)
            return success

        except Exception as e:
            session.rollback()
            raise e


def get_all_available_ports() -> list[dict]:
    """獲取資料庫中所有可用的端口"""
    with connection_pool.get_session() as session:
        statement = select(EqpPort).order_by(EqpPort.name)
        ports = session.exec(statement).all()
        return [{"id": port.id, "name": port.name, "description": port.description, "eqp_id": port.eqp_id} for port in ports]


def get_available_ports_for_eqp(eqp_id: int) -> list[dict]:
    """獲取特定設備可用的端口列表"""
    with connection_pool.get_session() as session:
        statement = select(EqpPort).where(
            EqpPort.eqp_id == eqp_id).order_by(EqpPort.name)
        ports = session.exec(statement).all()
        return [{"id": port.id, "name": port.name, "description": port.description} for port in ports]


def get_ports_not_assigned_to_any_eqp() -> list[dict]:
    """獲取尚未分配給任何設備的端口（如果有的話）"""
    with connection_pool.get_session() as session:
        # 獲取所有端口，按名稱分組，找出沒有被任何設備使用的端口名稱
        statement = select(
            EqpPort.name, EqpPort.description).distinct().order_by(EqpPort.name)
        all_ports = session.exec(statement).all()

        # 轉換為字典格式，這些是可以被新設備使用的端口模板
        unique_ports = {}
        for port in all_ports:
            if port.name not in unique_ports:
                unique_ports[port.name] = {
                    "name": port.name,
                    "description": port.description or f"端口 {port.name}"
                }

        return list(unique_ports.values())
