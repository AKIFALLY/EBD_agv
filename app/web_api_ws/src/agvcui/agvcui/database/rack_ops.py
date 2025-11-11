# database/rack_ops.py
"""
貨架相關資料庫操作
"""

import logging
from sqlmodel import select, func
from db_proxy.models import Rack, RackStatus
from .connection import connection_pool, rack_crud, product_crud
from .utils import fetch_all

# KUKA 同步服務
from agvcui.services.kuka_sync_service import KukaContainerSyncService

# Logger
logger = logging.getLogger(__name__)


def rack_all() -> list[dict]:
    """獲取所有貨架（含產品和載具統計）"""
    racks = fetch_all(rack_crud)
    products = fetch_all(product_crud)
    product_map = {p["id"]: p for p in products}

    for rack in racks:
        product = product_map.get(rack["product_id"])
        if product:
            rack["size"] = product["size"]
            rack["product_name"] = product["name"]
            rack["total"] = 32 if product["size"] == 'S' else 16
            # 使用 carrier_bitmap 計算載具數量
            carrier_bitmap = rack.get("carrier_bitmap", "00000000")
            rack["count"] = bin(int(carrier_bitmap, 16)).count('1') if carrier_bitmap else 0
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
    """
    更新貨架（整合 KUKA Container 同步）

    自動同步到 KUKA Fleet Manager：
    - is_in_map: 0→1 時調用 container_in（入場）
    - is_in_map: 1→0 時調用 container_out（出場）
    - location_id 變更時調用 update_container（位置更新）

    Args:
        rack_id: Rack ID
        rack_data: Rack 更新資料

    Returns:
        Rack: 更新後的 Rack 物件
    """
    with connection_pool.get_session() as session:
        # 1. 取得更新前的 rack 資料（用於比對變更）
        old_rack = rack_crud.get_by_id(session, rack_id)

        # 2. 創建 Rack 對象用於更新
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

        # 3. 執行資料庫更新
        updated_rack = rack_crud.update(session, rack_id, rack_obj)

        # 4. 同步到 KUKA Fleet Manager（錯誤不影響資料庫更新）
        try:
            kuka_sync = KukaContainerSyncService()
            sync_result = kuka_sync.sync_rack_to_kuka(old_rack, updated_rack, session)

            # 記錄同步結果
            if sync_result["success"]:
                action = sync_result.get("action", "unknown")
                if action != "skip":
                    logger.info(
                        f"✅ KUKA 同步成功: Rack(id={rack_id}, name={rack_data['name']}) | "
                        f"Action: {action}"
                    )
            else:
                logger.warning(
                    f"⚠️ KUKA 同步失敗 (本地更新已完成): Rack(id={rack_id}) | "
                    f"Error: {sync_result.get('error', 'Unknown')}"
                )
        except Exception as e:
            # KUKA 同步失敗不影響資料庫更新
            logger.error(
                f"❌ KUKA 同步異常 (本地更新已完成): Rack(id={rack_id}) | "
                f"Error: {str(e)}",
                exc_info=True
            )

        return updated_rack


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
    """獲取貨架格位資訊（基於 carrier_bitmap）"""
    with connection_pool.get_session() as session:
        rack = rack_crud.get_by_id(session, rack_id)

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

        # 從 carrier_bitmap 創建格位佔用狀態
        occupied_slots = {}
        if rack and rack.carrier_bitmap:
            bitmap_int = int(rack.carrier_bitmap, 16)
            for slot_index in range(1, total_slots + 1):
                bit_position = slot_index - 1
                is_occupied = bool(bitmap_int & (1 << bit_position))
                if is_occupied:
                    occupied_slots[slot_index] = {"rack_index": slot_index, "occupied": True}

        return {
            'total_slots': total_slots,
            'side_slots': side_slots,
            'rows': rows,
            'cols': cols,
            'occupied_slots': occupied_slots
        }


# ============================================================================
# Carrier Bitmap 操作函數
# ============================================================================

def parse_bitmap(bitmap_str: str) -> int:
    """
    將16進制bitmap字串轉換為整數

    Args:
        bitmap_str: 16進制字串 (如 "FFFFFFFF", "00000000")

    Returns:
        int: bitmap 整數值
    """
    try:
        return int(bitmap_str, 16) if bitmap_str else 0
    except ValueError:
        logger.warning(f"Invalid bitmap string: {bitmap_str}, returning 0")
        return 0


def bitmap_to_str(bitmap_int: int) -> str:
    """
    將整數轉換為8位16進制bitmap字串

    Args:
        bitmap_int: bitmap 整數值

    Returns:
        str: 8位16進制字串 (大寫)
    """
    return f"{bitmap_int:08X}"


def is_slot_occupied(carrier_bitmap: str, slot_index: int) -> bool:
    """
    檢查指定格位是否有貨

    Args:
        carrier_bitmap: carrier bitmap 字串
        slot_index: 格位索引 (1-32)

    Returns:
        bool: True=有貨, False=空
    """
    if not 1 <= slot_index <= 32:
        return False
    bitmap_int = parse_bitmap(carrier_bitmap)
    return bool(bitmap_int & (1 << (slot_index - 1)))


def is_slot_enabled(enable_bitmap: str, slot_index: int) -> bool:
    """
    檢查指定格位是否啟用

    Args:
        enable_bitmap: enable bitmap 字串
        slot_index: 格位索引 (1-32)

    Returns:
        bool: True=啟用, False=禁用
    """
    if not 1 <= slot_index <= 32:
        return False
    bitmap_int = parse_bitmap(enable_bitmap)
    return bool(bitmap_int & (1 << (slot_index - 1)))


def count_occupied_slots(carrier_bitmap: str) -> int:
    """
    統計有貨格位數量

    Args:
        carrier_bitmap: carrier bitmap 字串

    Returns:
        int: 有貨格位數量
    """
    bitmap_int = parse_bitmap(carrier_bitmap)
    return bin(bitmap_int).count('1')


def count_enabled_slots(enable_bitmap: str) -> int:
    """
    統計啟用格位數量

    Args:
        enable_bitmap: enable bitmap 字串

    Returns:
        int: 啟用格位數量
    """
    bitmap_int = parse_bitmap(enable_bitmap)
    return bin(bitmap_int).count('1')


def get_occupied_slot_list(carrier_bitmap: str, max_slots: int = 32) -> list[int]:
    """
    獲取所有有貨格位的索引列表

    Args:
        carrier_bitmap: carrier bitmap 字串
        max_slots: 最大格位數 (S產品=32, L產品=16)

    Returns:
        list[int]: 有貨格位索引列表 (如 [1, 3, 5])
    """
    bitmap_int = parse_bitmap(carrier_bitmap)
    return [i + 1 for i in range(max_slots) if bitmap_int & (1 << i)]


def get_empty_slot_list(carrier_bitmap: str, enable_bitmap: str, max_slots: int = 32) -> list[int]:
    """
    獲取所有空且啟用的格位索引列表

    Args:
        carrier_bitmap: carrier bitmap 字串
        enable_bitmap: enable bitmap 字串
        max_slots: 最大格位數 (S產品=32, L產品=16)

    Returns:
        list[int]: 空且啟用格位索引列表
    """
    carrier_int = parse_bitmap(carrier_bitmap)
    enable_int = parse_bitmap(enable_bitmap)

    return [
        i + 1 for i in range(max_slots)
        if (not (carrier_int & (1 << i))) and (enable_int & (1 << i))
    ]


def parse_rack_bitmap_status(rack: Rack, product_size: str = 'S') -> dict:
    """
    解析 Rack 的 bitmap 狀態為結構化資料

    Args:
        rack: Rack 物件
        product_size: 產品尺寸 ('S' or 'L')

    Returns:
        dict: 包含每個格位的詳細狀態
        {
            1: {"occupied": True, "enabled": True},
            2: {"occupied": False, "enabled": True},
            ...
        }
    """
    # 永遠返回 32 格位的狀態（不管產品尺寸）
    carrier_bitmap = rack.carrier_bitmap or "00000000"
    enable_bitmap = rack.carrier_enable_bitmap or "00000000"

    result = {}
    for slot_index in range(1, 33):  # 永遠 1-32
        result[slot_index] = {
            "occupied": is_slot_occupied(carrier_bitmap, slot_index),
            "enabled": is_slot_enabled(enable_bitmap, slot_index)
        }

    return result


def set_slot_occupied(rack_id: int, slot_index: int) -> bool:
    """
    設置指定格位為有貨

    Args:
        rack_id: Rack ID
        slot_index: 格位索引 (1-32)

    Returns:
        bool: 操作是否成功
    """
    if not 1 <= slot_index <= 32:
        logger.error(f"Invalid slot_index: {slot_index}, must be 1-32")
        return False

    with connection_pool.get_session() as session:
        rack = rack_crud.get_by_id(session, rack_id)
        if not rack:
            logger.error(f"Rack not found: {rack_id}")
            return False

        # 設置 bit
        bitmap_int = parse_bitmap(rack.carrier_bitmap)
        bitmap_int |= (1 << (slot_index - 1))
        rack.carrier_bitmap = bitmap_to_str(bitmap_int)

        session.add(rack)
        session.commit()

        logger.info(f"Set rack {rack_id} slot {slot_index} to occupied")
        return True


def set_slot_empty(rack_id: int, slot_index: int) -> bool:
    """
    設置指定格位為空

    Args:
        rack_id: Rack ID
        slot_index: 格位索引 (1-32)

    Returns:
        bool: 操作是否成功
    """
    if not 1 <= slot_index <= 32:
        logger.error(f"Invalid slot_index: {slot_index}, must be 1-32")
        return False

    with connection_pool.get_session() as session:
        rack = rack_crud.get_by_id(session, rack_id)
        if not rack:
            logger.error(f"Rack not found: {rack_id}")
            return False

        # 清除 bit
        bitmap_int = parse_bitmap(rack.carrier_bitmap)
        bitmap_int &= ~(1 << (slot_index - 1))
        rack.carrier_bitmap = bitmap_to_str(bitmap_int)

        session.add(rack)
        session.commit()

        logger.info(f"Set rack {rack_id} slot {slot_index} to empty")
        return True


def toggle_slot_enabled(rack_id: int, slot_index: int) -> bool:
    """
    切換指定格位的啟用狀態

    Args:
        rack_id: Rack ID
        slot_index: 格位索引 (1-32)

    Returns:
        bool: 操作是否成功
    """
    if not 1 <= slot_index <= 32:
        logger.error(f"Invalid slot_index: {slot_index}, must be 1-32")
        return False

    with connection_pool.get_session() as session:
        rack = rack_crud.get_by_id(session, rack_id)
        if not rack:
            logger.error(f"Rack not found: {rack_id}")
            return False

        # 切換 bit
        bitmap_int = parse_bitmap(rack.carrier_enable_bitmap)
        bitmap_int ^= (1 << (slot_index - 1))
        rack.carrier_enable_bitmap = bitmap_to_str(bitmap_int)

        session.add(rack)
        session.commit()

        is_enabled = bool(bitmap_int & (1 << (slot_index - 1)))
        logger.info(f"Toggled rack {rack_id} slot {slot_index} to {'enabled' if is_enabled else 'disabled'}")
        return True


def clear_all_slots(rack_id: int) -> bool:
    """
    清空所有格位 (設置為無貨)

    Args:
        rack_id: Rack ID

    Returns:
        bool: 操作是否成功
    """
    with connection_pool.get_session() as session:
        rack = rack_crud.get_by_id(session, rack_id)
        if not rack:
            logger.error(f"Rack not found: {rack_id}")
            return False

        rack.carrier_bitmap = "00000000"
        session.add(rack)
        session.commit()

        logger.info(f"Cleared all slots for rack {rack_id}")
        return True


def enable_all_slots(rack_id: int, product_size: str = 'S') -> bool:
    """
    啟用所有格位

    Args:
        rack_id: Rack ID
        product_size: 產品尺寸 ('S'=32格全啟用, 'L'=特定16格啟用)

    Returns:
        bool: 操作是否成功
    """
    with connection_pool.get_session() as session:
        rack = rack_crud.get_by_id(session, rack_id)
        if not rack:
            logger.error(f"Rack not found: {rack_id}")
            return False

        if product_size == 'S':
            # S產品: 32格全啟用 = FFFFFFFF (所有 bit 為 1)
            rack.carrier_enable_bitmap = "FFFFFFFF"
        else:  # L產品
            # L產品特定位置:
            # A面: 1,2,3,4,9,10,11,12 (bits 0-3, 8-11)
            # B面: 17,18,19,20,25,26,27,28 (bits 16-19, 24-27)
            # 二進制: 0000 1111 0000 1111 0000 1111 0000 1111
            # 十六進制: 0F0F0F0F
            rack.carrier_enable_bitmap = "0F0F0F0F"

        session.add(rack)
        session.commit()

        logger.info(f"Enabled all slots for rack {rack_id} (product_size={product_size})")
        return True


def disable_all_slots(rack_id: int) -> bool:
    """
    禁用所有格位

    Args:
        rack_id: Rack ID

    Returns:
        bool: 操作是否成功
    """
    with connection_pool.get_session() as session:
        rack = rack_crud.get_by_id(session, rack_id)
        if not rack:
            logger.error(f"Rack not found: {rack_id}")
            return False

        rack.carrier_enable_bitmap = "00000000"
        session.add(rack)
        session.commit()

        logger.info(f"Disabled all slots for rack {rack_id}")
        return True


def get_rack_with_bitmap_status(rack_id: int) -> dict | None:
    """
    獲取 Rack 資料及其 bitmap 狀態解析

    Args:
        rack_id: Rack ID

    Returns:
        dict: Rack 資料 + bitmap_status
        {
            "id": 1,
            "name": "001",
            "carrier_bitmap": "FFFFFFFF",
            "carrier_enable_bitmap": "FFFFFFFF",
            "product_size": "S",
            "bitmap_status": {
                1: {"occupied": True, "enabled": True},
                ...
            },
            "stats": {
                "occupied_count": 10,
                "enabled_count": 32,
                "empty_enabled_count": 22
            }
        }
    """
    with connection_pool.get_session() as session:
        rack = rack_crud.get_by_id(session, rack_id)
        if not rack:
            logger.error(f"Rack not found: {rack_id}")
            return None

        # 獲取產品尺寸
        product = product_crud.get_by_id(session, rack.product_id) if rack.product_id else None
        product_size = product.size if product else 'S'

        # 永遠使用 32 格位（不管產品尺寸）
        max_slots = 32

        # 解析 bitmap 狀態
        bitmap_status = parse_rack_bitmap_status(rack, product_size)

        # 統計資料
        carrier_bitmap = rack.carrier_bitmap or "00000000"
        enable_bitmap = rack.carrier_enable_bitmap or "00000000"
        occupied_count = count_occupied_slots(carrier_bitmap)
        enabled_count = count_enabled_slots(enable_bitmap)
        empty_enabled = get_empty_slot_list(carrier_bitmap, enable_bitmap, max_slots)

        return {
            "id": rack.id,
            "name": rack.name,
            "carrier_bitmap": carrier_bitmap,
            "carrier_enable_bitmap": enable_bitmap,
            "product_size": product_size,
            "product_name": product.name if product else None,
            "max_slots": max_slots,
            "bitmap_status": bitmap_status,
            "stats": {
                "occupied_count": occupied_count,
                "enabled_count": enabled_count,
                "empty_enabled_count": len(empty_enabled),
                "empty_enabled_slots": empty_enabled
            }
        }
