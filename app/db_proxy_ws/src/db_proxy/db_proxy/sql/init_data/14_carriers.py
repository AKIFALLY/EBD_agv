"""
14. 載具初始化資料
依賴：貨架
"""

from db_proxy.models import Carrier
from sqlmodel import select


def initialize_carriers(session):
    """初始化載具資料"""
    print("📦 初始化載具資料...")

    # 測試用的 Carrier 資料，PK 為 id（自動產生），FK 包含 room_id、rack_id、port_id，其他欄位如 rack_index 與 status 可選擇性設定
    default_test_carrier = [
        {"room_id": 2, "rack_id": 123, "rack_index": 17, "port_id": 2021, "status_id": 5},
        {"room_id": 2, "rack_id": 123, "rack_index": 18, "port_id": 2022, "status_id": 5},
        {"room_id": 2, "rack_id": 123, "rack_index": 19, "port_id": 2023, "status_id": 5},
        {"room_id": 2, "rack_id": 123, "rack_index": 20, "port_id": 2024, "status_id": 5},
        {"room_id": 2, "rack_id": 123, "rack_index": 21, "port_id": 2061, "status_id": 4},
        {"room_id": 2, "rack_id": 123, "rack_index": 22, "port_id": 2062, "status_id": 4},
        {"room_id": 2, "rack_id": 123, "rack_index": 23, "port_id": 2063, "status_id": 4},
        {"room_id": 2, "rack_id": 123, "rack_index": 24, "port_id": 2064, "status_id": 4},
        {"room_id": 2, "rack_id": 123, "rack_index": 25, "port_id": 2065, "status_id": 4},
        {"room_id": 2, "rack_id": 123, "rack_index": 28, "port_id": 2066, "status_id": 4},
        {"room_id": 2, "rack_id": 123, "rack_index": 29, "port_id": 2067, "status_id": 4},
        {"room_id": 2, "rack_id": 123, "rack_index": 32, "port_id": 2068, "status_id": 4},
    ]

    # 檢查是否已存在預設資料，如果不存在則插入
    for data in default_test_carrier:
        exists = session.exec(select(Carrier).where(
            Carrier.rack_id == data["rack_id"],
            Carrier.rack_index == data["rack_index"]
        )).first()

        if not exists:
            session.add(Carrier(**data))

    session.commit()
    print("✅ 載具資料初始化完成")
