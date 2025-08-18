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
        
        # Flow WCS 測試用 carriers
        # SWC001 (rack_id: 101) - 滿載 rack 用於 full_rack_to_manual_area flow
        # A面 (rack_index 1-16) 全部完成，B面 (rack_index 17-32) 全部完成 = 滿載狀態
        {"room_id": 2, "rack_id": 101, "rack_index": 1, "port_id": 2021, "status_id": 5},   # A面完成
        {"room_id": 2, "rack_id": 101, "rack_index": 2, "port_id": 2022, "status_id": 5},   # A面完成
        {"room_id": 2, "rack_id": 101, "rack_index": 3, "port_id": 2023, "status_id": 5},   # A面完成
        {"room_id": 2, "rack_id": 101, "rack_index": 4, "port_id": 2024, "status_id": 5},   # A面完成
        {"room_id": 2, "rack_id": 101, "rack_index": 17, "port_id": 2061, "status_id": 5},  # B面完成
        {"room_id": 2, "rack_id": 101, "rack_index": 18, "port_id": 2062, "status_id": 5},  # B面完成
        {"room_id": 2, "rack_id": 101, "rack_index": 19, "port_id": 2063, "status_id": 5},  # B面完成
        {"room_id": 2, "rack_id": 101, "rack_index": 20, "port_id": 2064, "status_id": 5},  # B面完成
        
        # SWC002 (rack_id: 102) - A面完成、B面有工作 用於 rack_rotation_inlet flow
        # A面完成，B面待作業
        {"room_id": 1, "rack_id": 102, "rack_index": 1, "port_id": 1011, "status_id": 5},   # A面完成
        {"room_id": 1, "rack_id": 102, "rack_index": 2, "port_id": 1012, "status_id": 5},   # A面完成
        {"room_id": 1, "rack_id": 102, "rack_index": 17, "port_id": 1013, "status_id": 2},  # B面待作業
        {"room_id": 1, "rack_id": 102, "rack_index": 18, "port_id": 1014, "status_id": 2},  # B面待作業
        
        # SWC003 (rack_id: 103) - B面完成、A面有工作 用於 rack_rotation_exit flow  
        # B面完成，A面待作業
        {"room_id": 1, "rack_id": 103, "rack_index": 17, "port_id": 1021, "status_id": 5},  # B面完成
        {"room_id": 1, "rack_id": 103, "rack_index": 18, "port_id": 1022, "status_id": 5},  # B面完成
        {"room_id": 1, "rack_id": 103, "rack_index": 1, "port_id": 1023, "status_id": 2},   # A面待作業
        {"room_id": 1, "rack_id": 103, "rack_index": 2, "port_id": 1024, "status_id": 2},   # A面待作業
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
