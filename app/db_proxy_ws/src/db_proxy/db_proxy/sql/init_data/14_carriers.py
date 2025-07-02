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
        {"rack_id": 123, "rack_index": 17},
        {"rack_id": 123, "rack_index": 18},
        {"rack_id": 123, "rack_index": 19},
        {"rack_id": 123, "rack_index": 20},
        {"rack_id": 123, "rack_index": 21},
        {"rack_id": 123, "rack_index": 22},
        {"rack_id": 123, "rack_index": 23},
        {"rack_id": 123, "rack_index": 24},
        {"rack_id": 123, "rack_index": 25},
        {"rack_id": 123, "rack_index": 28},
        {"rack_id": 123, "rack_index": 29},
        {"rack_id": 123, "rack_index": 32},
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
